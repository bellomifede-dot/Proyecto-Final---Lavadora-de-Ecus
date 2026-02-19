#!/usr/bin/env python3
"""
Servidor ECU integrado (con color por tipo y etiqueta en esquina superior derecha):
 - Detección ArUco (WORLD) y filtrado
 - Detección YOLO (OBB/boxes) solo para identificar el tipo de ECU (on-demand / periodic)
 - GET en el servidor TCP dispara una corrida YOLO (espera con timeout) y devuelve tipo+pose
 - En la ventana live se muestra la etiqueta de la clase (Ecu-1 / Ecu-2...) en la esquina superior derecha,
   con color distinto por tipo. La ventana se abre en tamaño mayor (configurable).

"""
import cv2
import numpy as np
import math
import socket
import json
import threading
import time
import sys
import os
from ultralytics import YOLO
from collections import deque, Counter

# -----------------------
# CONFIG 
markerLength = 31.0      # valor por compatibilidad para visualización (mm)
target_id = 0            # id del marcador sobre la ECU (objetivo)

# marcadores fijos (world <- marker) en las mismas unidades (mm)
known_markers_world = {
    10: np.array([[1,0,0,51.0],[0,1,0,0.0],[0,0,1,0.0],[0,0,0,1.0]], dtype=np.float64),
    11: np.array([[1,0,0,0.0],[0,1,0,0.0],[0,0,1,0.0],[0,0,0,1.0]], dtype=np.float64),
    12: np.array([[1,0,0,0.0],[0,1,0,-51.0],[0,0,1,0.0],[0,0,0,1.0]], dtype=np.float64),
}

# MARKER_SIZES: tamaño físico (mm) asociado a cada id de marcador
MARKER_SIZES = {
    0: 31.0,
    10: 38.0,
    11: 38.0,
    12: 38.0,
}
DEFAULT_MARKER_LENGTH = 38.0

def get_marker_corners_local_for_length(length_mm):
    L = length_mm / 2.0
    return np.array([
        [-L,  L, 0.0],
        [ L,  L, 0.0],
        [ L, -L, 0.0],
        [-L, -L, 0.0]
    ], dtype=np.float32)

# Suavizado anisotrópico: alpha por eje (X, Y, Z) -- 
alpha_xyz = np.array([0.03, 0.03, 0.01], dtype=np.float64)
alpha_rot = 0.08    # SLERP factor para rotación 

# Fast convergence / instant init settings 
INSTANT_INIT_ON_LOCK = True          # si True, al enganchar un marcador actualiza pos_world_f y quat_world_f instantáneamente
FAST_CONVERGENCE_FRAMES = 8          # si INSTANT_INIT_ON_LOCK False, usar blending rápido estos frames
FAST_ALPHA_XYZ = np.array([0.5, 0.5, 0.3], dtype=np.float64)  # alpha rápidos por eje
FAST_ALPHA_ROT = 0.6                 # SLERP rápido
# Esto permite que la pose se acomode en pocos frames tras detectar un nuevo marcador.

# Opción: forzar Z a un valor de referencia 
USE_Z_REF = False
Z_REF = 0.0  # mm, ajustar si USE_Z_REF = True
Z_REF_BLEND = 0.05  # cuánto mezclar hacia Z_REF: 0.05 = 5%

# YOLO model path
MODEL_PATH = "best3.pt"   

# Offsets por tipo de ECU (en mm)
OFFSETS = {
    "Ecu-1": np.array([0, 0, 0.0], dtype=np.float64),
    "Ecu-2": np.array([0, 0, 0.0], dtype=np.float64),
    "Ecu-3": np.array([0, 0, 0.0], dtype=np.float64),
}

# Server settings
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000

# Ventana: tamaño preferido
WINDOW_NAME = "POSE ECU (WORLD) + TYPE (colored)"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# -----------------------
# YOLO on-demand / matching / locking / ejes mundo
YOLO_RUN_MODE = "on_demand"   # "on_demand" o "periodic"
YOLO_PERIOD = 15              # frames si periodic
YOLO_RUN_EVERY_N_FRAMES = YOLO_PERIOD
MATCH_MAX_DIST_PIX = 200      # umbral máximo (pixels) para asociar bbox->marker
LOCKED_MAX_MISSES = 30        # frames tolerados sin ver el marker antes de liberar lock
WORLD_AXIS_SCALE = 10.0       # mm: largo de ejes del mundo que se dibujan

# Nuevos parámetros para disparos 
YOLO_TRIGGER_FRAMES = 15     # cuántos frames ejecutar YOLO tras un trigger ('p' o GET)
YOLO_WAIT_TIMEOUT = 30.0      # segundos que esperará el servidor GET (timeout)
# -----------------------

# -----------------------
# UTILIDADES (rvec/euler/quaternion/SLERP) 
def rvec_to_euler(rvec):
    R, _ = cv2.Rodrigues(rvec)
    yaw   = np.degrees(np.arctan2(R[1,0], R[0,0]))
    pitch = np.degrees(np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2)))
    roll  = np.degrees(np.arctan2(R[2,1], R[2,2]))
    return yaw, pitch, roll

def rvectvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3,3]  = tvec.reshape(3,)
    return T

def rotmat_to_quat_full(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        i = np.argmax(np.diag(R))
        j, k = (i+1)%3, (i+2)%3
        s = math.sqrt(1 + R[i,i] - R[j,j] - R[k,k]) * 2
        q = np.zeros(4, dtype=np.float64)
        q[i+1] = 0.25 * s
        q[0]   = (R[k,j] - R[j,k]) / s
        q[j+1] = (R[j,i] + R[i,j]) / s
        q[k+1] = (R[k,i] + R[i,k]) / s
        return q
    return np.array([w,x,y,z], dtype=np.float64)

def quat_to_rotmat(q):
    q = q / np.linalg.norm(q)
    w,x,y,z = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)
    return R

def slerp(q1, q2, a):
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        q = q1 + a*(q2 - q1)
        return q / np.linalg.norm(q)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * a
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q1) + (s1 * q2)

# -----------------------
# Carga calibración intrínseca
CALIB_FILE = "calibracion_camara.npz"
if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"Calib file not found: {CALIB_FILE}")
data = np.load(CALIB_FILE)
K = data["K"]
dist = data["dist"]

# Detector ArUco 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# -----------------------
# Globals protegidos por lock (para server)
latest_label = None                # string: última etiqueta YOLO
latest_pos_world_f = None          # 3-vector filtrado (world)
latest_R_world_obj_f = None        # 3x3 rot filtrada (world <- object)
latest_yaw = None
lock = threading.Lock()
stop_flag = False

# -----------------------
# Inicializo modelo YOLO (puede tardar al cargar)
try:
    model = YOLO(MODEL_PATH)
    print("Modelo YOLO cargado:", MODEL_PATH)
    try:
        print("Clases YOLO:", model.names)
    except Exception:
        pass
except Exception as e:
    print("Error cargando modelo YOLO:", e)
    model = None

# -----------------------
# COLOR MAP: vinculá la clase detectada (lowercase) con un color BGR
COLOR_MAP = {
    "ecu-1": (0, 200, 0),      # verde (B,G,R)
    "ecu-2": (0, 200, 200),    # amarillo/cian parecido (BGR)
    "ecu-3": (0, 0, 200),      # rojo
}
DEFAULT_COLOR = (255, 255, 255)  # blanco si no está el tipo en el map

# -----------------------
# Sincronización entre hilo server <-> bucle principal para disparar YOLO
force_yolo_frames = 0               # contador: si >0, ejecutar YOLO en cada frame y decrementar
yolo_done_event = threading.Event() # set() cuando la corrida solicitada finalizó (para el GET)
# -----------------------

# -----------------------
# TCP server: responde GET con tipo+pose (aplica OFFSETS en frame del objeto)
# al recibir GET, dispara una corrida YOLO (si hay modelo) y espera hasta YOLO_WAIT_TIMEOUT
def get_ecu_pose_and_type():
    """
    Devuelve dict o None. Usa latest_pos_world_f, latest_R_world_obj_f, latest_label y latest_yaw.
    Nota: el OFFSET solo se rota respecto del eje Z (yaw).
    """
    with lock:
        if latest_pos_world_f is None or latest_R_world_obj_f is None or latest_label is None:
            return None
        pos = latest_pos_world_f.copy()
        R = latest_R_world_obj_f.copy()
        label = latest_label
        lyaw = latest_yaw  # capturamos latest_yaw bajo el lock

    if label not in OFFSETS:
        return {"error": f"offset no definido para {label}"}
    OFFSET = np.asarray(OFFSETS[label], dtype=float).reshape(3,)  # asegurar vector 3

    # Determinar yaw en grados 
    if lyaw is not None:
        yaw_deg = float(lyaw)
    else:
        # atan2(R[1,0], R[0,0]) da el yaw en radianes; convertimos a grados y normalizamos 0-360
        yaw_deg = float((math.degrees(math.atan2(R[1, 0], R[0, 0])) + 360) % 360)

    # Construir la rotación alrededor del eje Z usando yaw (en radianes)
    yaw_rad = math.radians(yaw_deg)
    cz = math.cos(yaw_rad)
    sz = math.sin(yaw_rad)
    Rz = np.array([
        [cz, -sz, 0.0],
        [sz,  cz, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=float)

    # Aplicar solo Rz al OFFSET (no usamos la matriz R completa para rotar el offset)
    ecu_pos = pos + Rz @ OFFSET

    return {
        "type": label,
        "X": float(ecu_pos[0]),
        "Y": float(ecu_pos[1]),
        "Z": float(ecu_pos[2]),
        "Yaw": float(yaw_deg)
    }


def tcp_server():
    global stop_flag, force_yolo_frames, yolo_done_event
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(1)
    server.settimeout(1.0)  # para poder salir limpiamente con stop_flag
    print(f"Servidor TCP escuchando en {SERVER_HOST}:{SERVER_PORT}")
    try:
        while not stop_flag:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print("Cliente conectado:", addr)
            conn.settimeout(1.0)
            try:
                while True:
                    try:
                        data_recv = conn.recv(1024)
                    except socket.timeout:
                        if stop_flag:
                            break
                        continue
                    if not data_recv:
                        break
                    cmd_raw = data_recv.decode(errors='ignore').strip()
                    cmd = cmd_raw.upper()
                    if cmd == "GET":
                        # Disparar YOLO y esperar a que termine (o timeout)
                        if model is not None:
                            # clear previous event and request YOLO frames
                            yolo_done_event.clear()
                            with lock:
                                # pedir al menos YOLO_TRIGGER_FRAMES 
                                force_yolo_frames = max(force_yolo_frames, YOLO_TRIGGER_FRAMES)
                            # esperar al resultado (o timeout)
                            yolo_done_event.wait(timeout=YOLO_WAIT_TIMEOUT)

                        # después de esto intentamos devolver el último pose disponible 
                        res = get_ecu_pose_and_type()

                        # Si no hay detección, aplicamos fallback con OFFSETS["Ecu-3"] sólo si tenemos pose filtrada
                        if res is None:
                            with lock:
                                if latest_pos_world_f is not None and latest_R_world_obj_f is not None:
                                    pos = latest_pos_world_f.copy()
                                    R = latest_R_world_obj_f.copy()
                                    lyaw = latest_yaw
                                    # calcular yaw si es necesario
                                    if lyaw is not None:
                                        yaw_deg = float(lyaw)
                                    else:
                                        yaw_deg = float((math.degrees(math.atan2(R[1, 0], R[0, 0])) + 360) % 360)
                                    yaw_rad = math.radians(yaw_deg)
                                    cz = math.cos(yaw_rad)
                                    sz = math.sin(yaw_rad)
                                    Rz = np.array([
                                        [cz, -sz, 0.0],
                                        [sz,  cz, 0.0],
                                        [0.0, 0.0, 1.0]
                                    ], dtype=float)
                                    # aplicar offset de Ecu-3
                                    OFFSET = np.asarray(OFFSETS.get("Ecu-3", np.array([0.0,0.0,0.0])), dtype=float).reshape(3,)
                                    ecu_pos = pos + Rz @ OFFSET
                                    payload = {
                                        "type": "Ecu-3",
                                        "X": float(ecu_pos[0]),
                                        "Y": float(ecu_pos[1]),
                                        "Z": float(ecu_pos[2]),
                                        "Yaw": float(yaw_deg)
                                    }
                                    try:
                                        conn.sendall(json.dumps(payload).encode())
                                        print("DEBUG SERVER: sent fallback Ecu-3 response to client:", payload)
                                    except Exception as e:
                                        print("DEBUG SERVER: error sending fallback response:", e)
                                    continue
                                else:
                                    # no pose filtrada disponible -> respondemos no detection
                                    payload = {"error": "no detection"}
                                    try:
                                        conn.sendall(json.dumps(payload).encode())
                                        print("DEBUG SERVER: sent response to client:", payload)
                                    except Exception as e:
                                        print("DEBUG SERVER: error sending response:", e)
                                    continue

                        # Si llegamos acá, res no es None: enviamos la pose normal
                        try:
                            conn.sendall(json.dumps(res).encode())
                            print("DEBUG SERVER: sent response to client:", res)
                        except Exception as e:
                            print("DEBUG SERVER: error sending response:", e)

                    elif cmd == "EXIT":
                        conn.sendall(json.dumps({"status":"bye"}).encode())
                        break
                    else:
                        conn.sendall(json.dumps({"error":"unknown command"}).encode())
            except Exception as e:
                print("Cliente error:", e)
            finally:
                conn.close()
                print("Cliente desconectado.")
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        print("Servidor TCP cerrado.")

# -----------------------
# MAIN: bucle principal con ArUco
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("No se pudo abrir la cámara")

# Abrir ventana redimensionable y fijar tamaño inicial
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

# Lanzar servidor TCP en hilo separado
server_thread = threading.Thread(target=tcp_server, daemon=True)
server_thread.start()

print("Iniciando. Presiona ESC para salir. `p`=YOLO snapshot, `o`=toggle periodic, `s`=save snapshot")

# Estados de filtro 
pos_world_f = None
quat_world_f = None
T_cam_world = None
T_world_cam = None
cam_pose_valid = False
cam_rvec_last = None
cam_tvec_last = None

# -----------------------
# Parámetros y estados para YOLO robusto
LABEL_HISTORY_LEN = 7        # cuántas últimas etiquetas guardar
LABEL_VOTE_MIN = 2           # mínimo votos para aceptar una etiqueta por mayoría
NO_DETECT_MAX = 12           # cuántos frames sin detección antes de limpiar label
MIN_CONF_ACCEPT = 0.10       # umbral mínimo de confianza para considerar una detección

label_history = deque(maxlen=LABEL_HISTORY_LEN)
conf_history = deque(maxlen=LABEL_HISTORY_LEN)
no_detect_count = 0
last_bbox = None  # para visualización bbox

# -----------------------
# Estados para locking del marker target 
locked_target_found = False
locked_centroid = None
locked_marker_corners = None  # 4x2 array de corners en imagen (float32)
locked_misses = 0

# track previous lock state to detect transitions
prev_locked_target_found = False
# fast convergence counter (si usamos blending rápido)
fast_conv_counter = 0

# -----------------------
# Otros estados YOLO run control
frame_count = 0
force_yolo_now = False
periodic_mode = (YOLO_RUN_MODE == "periodic")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    key = cv2.waitKey(1) & 0xFF

    # Manejo de teclas
    if key == 27:  # ESC
        break
    elif key == ord('p'):
        # Forzar corrida YOLO sobre los próximos N frames 
        with lock:
            force_yolo_frames = max(force_yolo_frames, YOLO_TRIGGER_FRAMES)
        force_yolo_now = True
    elif key == ord('o'):
        # Toggle periodic/on_demand
        periodic_mode = not periodic_mode
        YOLO_RUN_MODE = "periodic" if periodic_mode else "on_demand"
        print("YOLO_RUN_MODE ->", YOLO_RUN_MODE)
    elif key == ord('s'):
        # guardar snapshot
        fname = f"snapshot_{int(time.time())}.png"
        cv2.imwrite(fname, frame)
        print("Guardado snapshot:", fname)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    cam_pose_valid = False
    T_cam_world = None
    T_world_cam = None

    # Lista de candidatos marcadores target detectados (centroides e indices)
    target_candidates = []  # list of tuples: (centroid (2,), corners_raw (4,2))

    if ids is not None and len(ids) > 0:
        # Construir correspondencias WORLD <-> IMG usando marcadores fijos
        global_objpts = []
        global_imgpts = []

        for c, mid in zip(corners, ids.flatten()):
            mid = int(mid)
            imgp = c.reshape(-1,2)

            # usar tamaño adecuado para este marker al calcular puntos 3D locales
            length_for_mid = MARKER_SIZES.get(mid, DEFAULT_MARKER_LENGTH)
            local_corners = get_marker_corners_local_for_length(length_for_mid)

            if mid in known_markers_world:
                T_w_m = known_markers_world[mid]
                for corner_local, imgpt in zip(local_corners, imgp):
                    p_local_h = np.hstack([corner_local, 1.0]).reshape(4,1)
                    p_world_h = T_w_m @ p_local_h
                    p_world = p_world_h[:3,0].astype(np.float64)
                    global_objpts.append(p_world)
                    global_imgpts.append(imgpt.astype(np.float64))

            # recoger candidatos target para matching espacial
            if mid == target_id:
                centroid = np.mean(c.reshape(-1,2), axis=0)
                target_candidates.append((centroid, c.reshape(-1,2).astype(np.float32)))

                # debug tamaño aparente en px
                xs = c.reshape(-1,2)[:,0]; ys = c.reshape(-1,2)[:,1]
                w_px = max(xs) - min(xs)
                h_px = max(ys) - min(ys)
                # imprime tamaño aparente para debugging (opcional)
                # print(f"DEBUG: detected target marker id={mid} size px ~ w={w_px:.1f} h={h_px:.1f}")

        if len(global_objpts) >= 3:
            objpts_np = np.array(global_objpts, dtype=np.float64).reshape(-1,3)
            imgpts_np = np.array(global_imgpts, dtype=np.float64).reshape(-1,2)

            # estimar pose de la CAMARA en WORLD: X_cam = R_cam * X_world + t_cam
            ok_cam, rvec_cam, tvec_cam = cv2.solvePnP(objpts_np, imgpts_np, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok_cam:
                cam_rvec_last = rvec_cam.copy()
                cam_tvec_last = tvec_cam.copy()
                T_cam_world = rvectvec_to_T(rvec_cam, tvec_cam)  # camera <- world
                # invertimos: T_world_cam (world <- camera)
                R_cw = T_cam_world[:3,:3]; t_cw = T_cam_world[:3,3]
                R_wc = R_cw.T
                t_wc = -R_wc @ t_cw
                T_world_cam = np.eye(4, dtype=np.float64)
                T_world_cam[:3,:3] = R_wc
                T_world_cam[:3,3]  = t_wc
                cam_pose_valid = True

    # Decide si corremos YOLO en este frame
    do_yolo = False
    # 1) si se pidió por GET/tecla ('force_yolo_frames' > 0)
    # introducimos yolo_requested local para notificar sólo si esta corrida fue solicitada
    yolo_requested = False
    with lock:
        if force_yolo_frames > 0:
            do_yolo = True
            force_yolo_frames -= 1
            yolo_requested = True
    # 2) si estamos en modo periodic
    if (not do_yolo) and periodic_mode and (frame_count % YOLO_RUN_EVERY_N_FRAMES == 0):
        do_yolo = True

    label_this_frame = None
    best_conf = 0.0
    best_label = None
    best_bbox = None
    bbox_center = None

    if do_yolo and model is not None:
        # Ejecutar YOLO sobre este frame (RGB)
        try:
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model.predict(img_rgb, conf=MIN_CONF_ACCEPT, imgsz=640, iou=0.45, verbose=False)
            # Tomar la mejor detección (por confianza)
            for r in results:
                # boxes
                if hasattr(r, "boxes") and r.boxes is not None and len(r.boxes) > 0:
                    for box in r.boxes:
                        try:
                            cls_t = getattr(box, "cls", None)
                            conf_t = getattr(box, "conf", None)
                            if cls_t is None or conf_t is None:
                                continue
                            cls_id = int(cls_t.cpu().numpy()[0]) if hasattr(cls_t, "cpu") else int(cls_t)
                            conf_v = float(conf_t.cpu().numpy()[0]) if hasattr(conf_t, "cpu") else float(conf_t)
                            bbox_t = None
                            if hasattr(box, "xyxy"):
                                try:
                                    bbox_t = box.xyxy.cpu().numpy()[0].astype(int)
                                except Exception:
                                    bbox_t = None
                            if conf_v > best_conf:
                                best_conf = conf_v
                                best_label = model.names[cls_id] if cls_id in model.names else str(cls_id)
                                best_bbox = bbox_t
                        except Exception:
                            continue

                # obb (si está)
                if hasattr(r, "obb") and r.obb is not None and len(r.obb) > 0:
                    for obb in r.obb:
                        try:
                            cls_t = getattr(obb, "cls", None)
                            conf_t = getattr(obb, "conf", None) or getattr(obb, "score", None)
                            if cls_t is None or conf_t is None:
                                continue
                            cls_id = int(cls_t.cpu().numpy()[0]) if hasattr(cls_t, "cpu") else int(cls_t)
                            conf_v = float(conf_t.cpu().numpy()[0]) if hasattr(conf_t, "cpu") else float(conf_t)
                            if conf_v > best_conf:
                                best_conf = conf_v
                                best_label = model.names[cls_id] if cls_id in model.names else str(cls_id)
                                best_bbox = None
                        except Exception:
                            continue
        except Exception as e:
            print("YOLO error:", e)
            best_label = None
            best_conf = 0.0
            best_bbox = None
        finally:
            # --- Fin de la corrida YOLO en este frame ---
            # Si esta corrida fue solicitada (por GET o por 'p') avisamos al server-thread
            if yolo_requested:
                try:
                    yolo_done_event.set()
                except Exception:
                    pass

            # Si la corrida fue solicitada (tecla 'p' o GET) y NO hubo detección pero HAY marcador,
            # hacemos fallback: etiquetamos temporalmente como "Ecu-3" para que la GUI muestre
            # y la pose filtrada (solvePnP) se calcule y quede disponible para el server.
            if yolo_requested and (best_label is None or best_conf < MIN_CONF_ACCEPT) and (locked_marker_corners is not None):
                with lock:
                    latest_label = "Ecu-3"
                print("DEBUG: yolo_requested but no detection; fallback label -> Ecu-3 (marker present).")

            # reset del flag de tecla 
            force_yolo_now = False

    # Si encontramos una detección "buena" en este frame:
    if best_label is not None and best_conf >= MIN_CONF_ACCEPT:
        label_this_frame = best_label
        last_bbox = best_bbox
        no_detect_count = 0
        label_history.append(label_this_frame)
        conf_history.append(best_conf)
        # Calcular centro bbox si existe
        if best_bbox is not None:
            x1, y1, x2, y2 = best_bbox
            bbox_center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0])
        else:
            bbox_center = None
    else:
        # si no corrimos YOLO (o no detectó), aumentar contador
        no_detect_count += 1
        if no_detect_count > NO_DETECT_MAX:
            label_history.clear()
            conf_history.clear()
            # NO borramos latest_label si hay un marker locked: el label debe quedarse hasta que el marcador desaparezca
            with lock:
                if not locked_target_found:
                    latest_label = None
            last_bbox = None

    # Determinar etiqueta "actual" por voto de historial
    with lock:
        if len(label_history) > 0:
            cnt = Counter(label_history)
            most_common_label, votes = cnt.most_common(1)[0]
            if votes >= LABEL_VOTE_MIN:
                latest_label = most_common_label
        label_copy = latest_label

    # ----------------- Matching marker <-> YOLO bbox y locking -----------------
    # 1) Si YOLO detectó bbox y hay candidatos de marker (id==target_id), elegir el más cercano
    matched_marker_updated = False
    if bbox_center is not None and len(target_candidates) > 0:
        # encontrar candidato con mínima distancia pixel
        best_dist = float("inf")
        best_corners = None
        best_centroid = None
        for centroid, corners_raw in target_candidates:
            d = np.linalg.norm(centroid - bbox_center)
            if d < best_dist:
                best_dist = d
                best_corners = corners_raw.copy()
                best_centroid = centroid.copy()
        if best_dist <= MATCH_MAX_DIST_PIX:
            # lock al marker que mejor coincide con bbox
            locked_marker_corners = best_corners.copy()
            locked_centroid = best_centroid.copy()
            locked_target_found = True
            locked_misses = 0
            matched_marker_updated = True
            # actualizo la label global con lo detectado
            with lock:
                latest_label = label_this_frame
        else:
            # no hay correspondencia confiable bbox->marker 
            matched_marker_updated = False

    # 2) Si no corrió YOLO o no matched, pero ya había un lock, intentar re-identificarlo entre candidatos:
    if not matched_marker_updated and locked_target_found:
        # Buscar el candidato actual más cercano al locked_centroid 
        if len(target_candidates) > 0:
            best_dist = float("inf")
            best_corners = None
            best_centroid = None
            for centroid, corners_raw in target_candidates:
                d = np.linalg.norm(centroid - locked_centroid)
                if d < best_dist:
                    best_dist = d
                    best_corners = corners_raw.copy()
                    best_centroid = centroid.copy()
            # Si encontramos un candidate "cercano", actualizamos
            if best_dist < MATCH_MAX_DIST_PIX:
                locked_marker_corners = best_corners.copy()
                locked_centroid = best_centroid.copy()
                locked_misses = 0
            else:
                # no lo encontramos en este frame
                locked_misses += 1
                if locked_misses > LOCKED_MAX_MISSES:
                    # liberar lock y también quitamos la etiqueta 
                    locked_target_found = False
                    locked_centroid = None
                    locked_marker_corners = None
                    locked_misses = 0
                    with lock:
                        latest_label = None
        else:
            # no hay candidatos en la detección actual
            locked_misses += 1
            if locked_misses > LOCKED_MAX_MISSES:
                locked_target_found = False
                locked_centroid = None
                locked_marker_corners = None
                locked_misses = 0
                with lock:
                    latest_label = None
    # 3) Si no había lock y no hubo una match (pero hay candidatos), opcionalmente nos podemos enganchar al primero
    if (not locked_target_found) and (len(target_candidates) > 0) and (not matched_marker_updated):
        # Enganchamos al primer candidato por defecto 
        first_centroid, first_corners = target_candidates[0]
        locked_marker_corners = first_corners.copy()
        locked_centroid = first_centroid.copy()
        locked_target_found = True
        locked_misses = 0
        

    # detectar transición de lock 
    lock_just_acquired = (not prev_locked_target_found) and locked_target_found
    if lock_just_acquired:
        # si se acaba de adquirir un lock, preparamos fase rápida o inicialización instantánea
        fast_conv_counter = FAST_CONVERGENCE_FRAMES
        if INSTANT_INIT_ON_LOCK:
            # Si tenemos ya pose del objeto (podemos calcularla inmediatamente), lo aplicamos de golpe.
            # Pero para ello debemos calcular rvec/tvec con los corners actuales:
            try:
                imgp_obj = locked_marker_corners.astype(np.float32)
                # usar tamaño del marker objetivo
                marker_len_for_target = MARKER_SIZES.get(target_id, DEFAULT_MARKER_LENGTH)
                local_corners_target = get_marker_corners_local_for_length(marker_len_for_target)
                ok_obj_tmp, rvec_obj_tmp, tvec_obj_tmp = cv2.solvePnP(local_corners_target, imgp_obj, K, dist,
                                                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok_obj_tmp:
                    # transform a world si tenemos T_world_cam
                    T_cam_obj_tmp = rvectvec_to_T(rvec_obj_tmp, tvec_obj_tmp)
                    if cam_pose_valid and T_world_cam is not None:
                        T_world_obj_tmp = T_world_cam @ T_cam_obj_tmp
                        pos_world_tmp = T_world_obj_tmp[:3,3].astype(np.float64)
                        R_world_obj_tmp = T_world_obj_tmp[:3,:3].astype(np.float64)
                        q_new_tmp = rotmat_to_quat_full(R_world_obj_tmp)
                        # inicializamos filtros inmediatamente
                        pos_world_f = pos_world_tmp.copy()
                        quat_world_f = q_new_tmp.copy()
                        # actualizar variables globales también
                        R_world_obj_f = quat_to_rotmat(quat_world_f)
                        with lock:
                            latest_pos_world_f = pos_world_f.copy()
                            latest_R_world_obj_f = R_world_obj_f.copy()
                        # preparo yaw
                        rvec_dbg, _ = cv2.Rodrigues(R_world_obj_f)
                        yaw_dbg, _, _ = rvec_to_euler(rvec_dbg)
                        with lock:
                            latest_yaw = float((yaw_dbg + 360) % 360)
                        # DEBUG print
                        print("DEBUG: instant init lock -> latest_pos_world_f set to", latest_pos_world_f, "yaw", latest_yaw)
            except Exception as e:
                # si algo falla no pasa nada: seguimos con la convergencia rápida en el paso normal
                print("DEBUG: instant init exception:", e)

    # ----------------- Ahora usar locked_marker_corners para el solvePnP del OBJ -------
    if locked_marker_corners is not None:
        imgp_obj = locked_marker_corners.astype(np.float32)

        # usar tamaño del marker objetivo
        marker_len_for_target = MARKER_SIZES.get(target_id, DEFAULT_MARKER_LENGTH)
        local_corners_target = get_marker_corners_local_for_length(marker_len_for_target)

        # pose del OBJ en CAMERA 
        ok_obj, rvec_obj, tvec_obj = cv2.solvePnP(local_corners_target, imgp_obj, K, dist,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok_obj:
            T_cam_obj = rvectvec_to_T(rvec_obj, tvec_obj)  # camera <- object
            pos_cam = T_cam_obj[:3,3].copy()

            if cam_pose_valid and T_world_cam is not None:
                # Transformar a WORLD: T_world_obj = T_world_cam @ T_cam_obj
                T_world_obj = T_world_cam @ T_cam_obj
                pos_world = T_world_obj[:3,3].astype(np.float64)
                R_world_obj = T_world_obj[:3,:3].astype(np.float64)

                # --- OPCIONAL: forzar Z 
                if USE_Z_REF:
                    if pos_world_f is None:
                        pos_world[2] = (1.0 - Z_REF_BLEND) * pos_world[2] + Z_REF_BLEND * Z_REF
                    else:
                        pos_world[2] = 0.95 * pos_world_f[2] + Z_REF_BLEND * Z_REF

                # Suavizado: aplico fast convergence si corresponde
                q_new = rotmat_to_quat_full(R_world_obj)  # [w,x,y,z]

                if pos_world_f is None:
                    # si no había filtro inicializado (caso improbable si usamos instant init), inicializo directo
                    pos_world_f = pos_world.copy()
                    quat_world_f = q_new.copy()
                else:
                    # elegimos alpha según si estamos en fase rápida o normal
                    if fast_conv_counter > 0:
                        # uso alphas rápidos
                        pos_world_f = FAST_ALPHA_XYZ * pos_world + (1.0 - FAST_ALPHA_XYZ) * pos_world_f
                        quat_world_f = slerp(quat_world_f, q_new, FAST_ALPHA_ROT)
                        fast_conv_counter -= 1
                    else:
                        # comportamiento normal
                        pos_world_f = alpha_xyz * pos_world + (1.0 - alpha_xyz) * pos_world_f
                        quat_world_f = slerp(quat_world_f, q_new, alpha_rot)
                    quat_world_f = quat_world_f / np.linalg.norm(quat_world_f)

                # Matriz filtrada
                R_world_obj_f = quat_to_rotmat(quat_world_f)
                T_world_obj_f = np.eye(4, dtype=np.float64)
                T_world_obj_f[:3,:3] = R_world_obj_f
                T_world_obj_f[:3,3]  = pos_world_f

                # Mostrar pose filtrada WORLD 
                Xw, Yw, Zw = pos_world_f
                rvec_dbg, _ = cv2.Rodrigues(R_world_obj_f)
                yaw_dbg, pitch_dbg, roll_dbg = rvec_to_euler(rvec_dbg)
                yaw_dbg = float((yaw_dbg + 360) % 360)

                cv2.putText(frame, f"OBJ world X:{Xw:.1f} Y:{Yw:.1f} Z:{Zw:.1f}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(frame, f"Yaw:{yaw_dbg:.1f} Pitch:{pitch_dbg:.1f} Roll:{roll_dbg:.1f}", (10,60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                # Actualizo variables globales para el server (con lock)
                with lock:
                    latest_pos_world_f = pos_world_f.copy()
                    latest_R_world_obj_f = R_world_obj_f.copy()
                    latest_yaw = float((yaw_dbg + 360) % 360)
                    # PRINT de DEBUG: mostrar la pose que se va a devolver al cliente
                    #print(f"DEBUG SERVER: latest_label={latest_label} -> pos_world_f = [{latest_pos_world_f[0]:.2f}, {latest_pos_world_f[1]:.2f}, {latest_pos_world_f[2]:.2f}], yaw={latest_yaw:.2f}")

                # Dibujar ejes del objeto filtrado (transformarlos WORLD -> CAM y proyectar)
                if T_cam_world is not None:
                    R_cam = T_cam_world[:3,:3]
                    t_cam = T_cam_world[:3,3]
                    axis_len = MARKER_SIZES.get(target_id, DEFAULT_MARKER_LENGTH)  # usar tamaño del marker para visualizar
                    origin_world = pos_world_f
                    x_world = origin_world + R_world_obj_f[:,0] * axis_len
                    y_world = origin_world + R_world_obj_f[:,1] * axis_len
                    z_world = origin_world + R_world_obj_f[:,2] * axis_len
                    pts_world = np.vstack([origin_world, x_world, y_world, z_world]).astype(np.float32)

                    # world -> cam (mediante rvec_cam/tvec_cam)
                    if cam_rvec_last is not None and cam_tvec_last is not None:
                        pts2d, _ = cv2.projectPoints(pts_world.reshape(-1,3), cam_rvec_last, cam_tvec_last, K, dist)
                        p0 = tuple(pts2d[0].ravel().astype(int))
                        pX = tuple(pts2d[1].ravel().astype(int))
                        pY = tuple(pts2d[2].ravel().astype(int))
                        pZ = tuple(pts2d[3].ravel().astype(int))
                        cv2.line(frame, p0, pX, (0,0,255), 2)  # X red
                        cv2.line(frame, p0, pY, (0,255,0), 2)  # Y green
                        cv2.line(frame, p0, pZ, (255,0,0), 2)  # Z blue

            else:
                # No tenemos pose WORLD válida: mostramos pose en cámara sin filtrado
                Xc, Yc, Zc = pos_cam
                cv2.putText(frame, f"OBJ cam X:{Xc:.1f} Y:{Yc:.1f} Z:{Zc:.1f}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    # ----------------- DIBUJAR EJE MUNDO (terna) -----------------
    if cam_pose_valid and cam_rvec_last is not None and cam_tvec_last is not None:
        origin_world = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        x_world = np.array([WORLD_AXIS_SCALE, 0.0, 0.0], dtype=np.float32)
        y_world = np.array([0.0, WORLD_AXIS_SCALE, 0.0], dtype=np.float32)
        z_world = np.array([0.0, 0.0, WORLD_AXIS_SCALE], dtype=np.float32)
        pts_world = np.vstack([origin_world, x_world, y_world, z_world]).astype(np.float32)
        pts2d, _ = cv2.projectPoints(pts_world.reshape(-1,3), cam_rvec_last, cam_tvec_last, K, dist)
        p0 = tuple(pts2d[0].ravel().astype(int))
        pX = tuple(pts2d[1].ravel().astype(int))
        pY = tuple(pts2d[2].ravel().astype(int))
        pZ = tuple(pts2d[3].ravel().astype(int))
        cv2.line(frame, p0, pX, (0,0,255), 2)  # X rojo
        cv2.line(frame, p0, pY, (0,255,0), 2)  # Y verde
        cv2.line(frame, p0, pZ, (255,0,0), 2)  # Z azul
        cv2.putText(frame, "WORLD frame", (p0[0]+5, p0[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

    # --------- AQUI: dibujamos la etiqueta en esquina superior derecha ----------
    color = DEFAULT_COLOR
    if label_copy is not None:
        display_label = label_copy
        key = label_copy.lower().replace(" ", "-")
        color = COLOR_MAP.get(key, DEFAULT_COLOR)
        if last_bbox is not None:
            try:
                x1, y1, x2, y2 = last_bbox
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            except Exception:
                pass
        text = display_label
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.9
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        margin = 12
        tx = frame.shape[1] - tw - margin
        ty = margin + th
        rect_tl = (tx - 6, ty - th - 6)
        rect_br = (tx + tw + 6, ty + 6)
        cv2.rectangle(frame, rect_tl, rect_br, (30,30,30), -1)
        cv2.putText(frame, text, (tx, ty), font, scale, color, thickness, cv2.LINE_AA)
    else:
        info = f"YOLO mode: {YOLO_RUN_MODE} (press 'p' to run)"
        cv2.putText(frame, info, (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

    # Mostrar también estado del marker seleccionado
    if locked_marker_corners is not None:
        c = locked_marker_corners.reshape(-1,2).astype(int)
        cv2.polylines(frame, [c], True, (0,255,255), 2)
        cc = tuple(np.mean(c, axis=0).astype(int))
        cv2.circle(frame, cc, 4, (0,255,255), -1)
        cv2.putText(frame, f"Locked marker id={target_id}", (cc[0]+8, cc[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    # update prev lock flag for next iteration
    prev_locked_target_found = locked_target_found

    # Mostrar frame 
    cv2.imshow(WINDOW_NAME, frame)

# cleanup
stop_flag = True
time.sleep(0.2)
cap.release()
cv2.destroyAllWindows()
print("Finalizado.")
