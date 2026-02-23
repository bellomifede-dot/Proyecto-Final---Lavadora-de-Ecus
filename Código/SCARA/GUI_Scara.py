#!/usr/bin/env python3
# interfaz_robot_real.py
import time
import threading
import serial
import serial.tools.list_ports
import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox
import os
import json
import pygame
import re
import socket
# Importar módulos usuarios (asegurate están en la misma carpeta)
import motion_model
import scara_kinematics
from motion_model import RobotConfig, MotionModel
import numpy as np
import math   # añadir si no está en los imports

# --------------------------
# Config
# --------------------------
BAUDRATE = 115200
TIMEOUT = 0.2
POINTS_FILE = "puntos_guardados.json"

def normalize_and_invert_yaw(yaw_deg):
    """
    Normaliza e invierte el yaw para la convención de tu robot.
    """
    yaw = float(yaw_deg) % 360.0
    if yaw > 180.0:
        yaw = yaw - 360.0
    result = yaw
    if result > 180.0:
        result -= 360.0
    if result < -180.0:
        result += 360.0
    return float(result)

def parse_float_from_labelz(label_text):
    """Extrae el primer número válido de un texto como 'Z: 0 p | 0.00'."""
    match = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", label_text)
    return float(match[0]) if match else 0.0

def parse_float_from_label(label_text):
    """Extrae el último número válido de un texto como 'X: 123.45 mm'."""
    match = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", label_text)
    return float(match[-1]) if match else 0.0
    
def parse_float_from_label2(label_text):
    """Extrae el segundo número (usado para obtener pasos) de un texto tipo 'J1: 123 p | 12.34°'."""
    match = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", label_text)
    return float(match[1]) if match and len(match) > 1 else (float(match[0]) if match else 0.0)

def obtener_coordenadas(host="127.0.0.1", port=5000, timeout=60):
    """📡 Obtiene la pose desde el servidor TCP."""
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(timeout)
        client.connect((host, port))
        client.sendall(b"GET")
        data = client.recv(4096).decode().strip()
        client.sendall(b"EXIT")
        client.close()

        if not data:
            print("DEBUG cliente: no data recibida")
            return None

        pose = json.loads(data)
        pose = {k.lower(): v for k, v in pose.items()}
        if "error" in pose:
            print("DEBUG cliente: server error:", pose["error"])
            return None
        return pose

    except Exception as e:
        print("DEBUG cliente exception:", e)
        return None

# Matriz de transformación homogénea [4x4]
T_CAM_TO_ROBOT = np.array([
    [1, 0, 0, 498.5],
    [0, 1, 0, 4.5],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

def transformar_a_robot(x, y, z, T=T_CAM_TO_ROBOT):
    punto_cam = np.array([x, y, z, 1]).reshape(4, 1)
    punto_robot = T @ punto_cam
    Xr, Yr, Zr = punto_robot[:3, 0]
    return float(Xr), float(Yr), float(Zr)

# --------------------------
# Clase de interfaz serial
# --------------------------
class RobotSerialInterface:
    def __init__(self):
        self.ser = None
        self.port = None
        self.alive = False
        self.listener_thread = None

    def detect_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if any(k in desc for k in ["arduino", "ch340", "usb serial", "uno", "mega", "nano", "leonardo"]) \
               or "ch340" in hwid or "usb vid" in hwid:
                return p.device
        return None

    def connect(self, explicit_port=None):
        port = explicit_port or self.detect_port()
        if not port:
            raise RuntimeError("No se detectó puerto Arduino. Conectar y reintentar.")
        self.port = port
        self.ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
        time.sleep(1.5)
        self.alive = True
        self.listener_thread = threading.Thread(target=self._listener, daemon=True)
        self.listener_thread.start()
        return True

    def close(self):
        self.alive = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def send(self, text):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Puerto serial no conectado")
        self.ser.write((text + "\n").encode())
        return

    def _listener(self):
        while self.alive and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        if GUI.singleton:
                            GUI.singleton.append_log(f"Arduino: {line}")
            except Exception:
                pass
            time.sleep(0.02)

# --------------------------
# Manejo de puntos guardados
# --------------------------
class PointManager:
    def __init__(self, filename=POINTS_FILE):
        self.filename = filename
        self.points = self.load_points()

    def load_points(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                return {}
        return {}

    def save_points(self):
        try:
            with open(self.filename, "w", encoding="utf-8") as f:
                json.dump(self.points, f, indent=2)
        except Exception as e:
            print("Error guardando puntos:", e)

    def add_point(self, name, x, y, z, phi_abs):
        self.points[name] = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "phi_abs": float(phi_abs)
        }
        self.save_points()
        
    def get_points(self):
        return self.points

    def get_point(self, name):
        return self.points.get(name)

    def delete_point(self, name):
        if name in self.points:
            del self.points[name]
            self.save_points()

# --------------------------
# GUI
# --------------------------
class GUI:
    singleton = None  # para que el serial listener pueda acceder

    def __init__(self, root):
        GUI.singleton = self
        self.root = root
        self.root.title("Interfaz Robot Real - SCARA")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.serial = RobotSerialInterface()
        self.unit_mode = tk.StringVar(value="human")  # "human" (deg/mm) or "steps"
        self.point_manager = PointManager()
        self._build_ui()
        self.update_status_loop()
        self.robot_config = RobotConfig("robot_config.json")
        self.motion = MotionModel(self.robot_config)


    def _build_ui(self):
        # Layout
        self.root.geometry("1100x700")
        self.root.grid_columnconfigure(1, weight=1)
        left_container = ctk.CTkFrame(self.root, width=300)
        left_container.grid(row=0, column=0, sticky="nsw", padx=10, pady=10)
        left = left_container
        right = ctk.CTkFrame(self.root)
        right.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=2)

        ctk.CTkLabel(left, text="Selector Unidades:", anchor="w").pack(padx=10, pady=(10,0))
        self.unit_switch = ctk.CTkSegmentedButton(left, values=["Pasos","Grados/mm"], command=self.unit_changed)
        self.unit_switch.set("Grados/mm")
        self.unit_switch.pack(padx=10, pady=5, fill="x")

        # status positions
        ctk.CTkLabel(left, text="Pos Art (p / mm):", anchor="w").pack(padx=10, pady=(10,0))
        self.j1_var = tk.StringVar(value="J1: --")
        self.j2_var = tk.StringVar(value="J2: --")
        self.z_var  = tk.StringVar(value="Z: --")
        self.j6_var = tk.StringVar(value="J6: --")
        for v in (self.j1_var, self.j2_var, self.z_var, self.j6_var):
            ctk.CTkLabel(left, textvariable=v, anchor="w").pack(padx=15)

        ctk.CTkLabel(left, text="Pos Cartesianas:", anchor="w").pack(padx=10, pady=(10,0))
        self.x_var = tk.StringVar(value="X: --")
        self.y_var = tk.StringVar(value="Y: --")
        self.z_cart_var = tk.StringVar(value="Z: --")
        self.phi_var = tk.StringVar(value="φ_abs: --")
        ctk.CTkLabel(left, textvariable=self.x_var).pack(padx=15)
        ctk.CTkLabel(left, textvariable=self.y_var).pack(padx=15)
        ctk.CTkLabel(left, textvariable=self.z_cart_var).pack(padx=15)
        ctk.CTkLabel(left, textvariable=self.phi_var).pack(padx=15)    

        # Gestión de Puntos Guardados
        ctk.CTkLabel(left, text="Gestión de Puntos", font=("Segoe UI", 12, "bold")).pack(pady=(5,0))

        self.point_name_entry = ctk.CTkEntry(left, placeholder_text="Nombre del punto")
        self.point_name_entry.pack(fill="x", padx=10, pady=5)

        btn_frame = ctk.CTkFrame(left)
        btn_frame.pack(fill="x", padx=10, pady=(0,5))
        ctk.CTkButton(btn_frame, text="Guardar punto actual", command=self.save_point_button).pack(side="left", expand=True, padx=3)
        ctk.CTkButton(btn_frame, text="Cargar puntos", command=self.refresh_points_list).pack(side="left", expand=True, padx=3)
        ctk.CTkButton(btn_frame, text="Obtener ECU", fg_color="#3A7EBF", command=lambda: threading.Thread(target=self.obtener_ecu_y_guardar, daemon=True).start()).pack(side="left", expand=True, padx=3)

        self.points_list = tk.Listbox(left, height=12)
        self.points_list.pack(fill="both", padx=10, pady=5)
        
        btn_action = ctk.CTkFrame(left)
        btn_action.pack(fill="x", padx=10, pady=(0,10))
        ctk.CTkButton(btn_action, text="Mover a punto seleccionado", fg_color="#107C41", command=self.go_to_point).pack(side="left", expand=True, padx=3)
        ctk.CTkButton(btn_action, text="Borrar punto", fg_color="#D33E2A", command=self.delete_point_button).pack(side="left", expand=True, padx=3)

        # --- Right: Controls ---
        right_combined = ctk.CTkFrame(right)
        right_combined.pack(fill="both", expand=True, padx=10, pady=10)

        # Jogging Manual
        jog_frame = ctk.CTkFrame(right_combined)
        jog_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        ctk.CTkLabel(jog_frame, text="Jogging Manual", font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=6, pady=(5,10))
        ctk.CTkLabel(jog_frame, text="P").grid(row=2, column=0, padx=5, sticky="w")
        self.jog_value = ctk.CTkEntry(jog_frame, placeholder_text="steps/deg/mm", width=60)
        self.jog_value.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(jog_frame, text="V").grid(row=2, column=3, padx=5, sticky="w")
        self.jog_vmax = ctk.CTkEntry(jog_frame, placeholder_text="V general", width=60)
        self.jog_vmax.grid(row=2, column=4, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(jog_frame, text="A").grid(row=2, column=5, padx=5, sticky="w")
        self.jog_amax = ctk.CTkEntry(jog_frame, placeholder_text="A general", width=60)
        self.jog_amax.grid(row=2, column=6, padx=5, pady=5, sticky="ew")

        axes = ["J1", "J2", "Z", "J6"]
        for i, axis in enumerate(axes):
            ctk.CTkLabel(jog_frame, text=f"{axis}:").grid(row=3+i, column=0, padx=5, sticky="e")
            btn_m = ctk.CTkButton(jog_frame, text="-", width=40, command=lambda a=axis: self.jog_axis(a, -1))
            btn_p = ctk.CTkButton(jog_frame, text="+", width=40, command=lambda a=axis: self.jog_axis(a, +1))
            btn_m.grid(row=3+i, column=1, padx=2)
            btn_p.grid(row=3+i, column=2, padx=2)
            ctk.CTkLabel(jog_frame, text=f"V{axis}:").grid(row=3+i, column=3)
            setattr(self, f"{axis}_vmax", ctk.CTkEntry(jog_frame, placeholder_text="v", width=60))
            getattr(self, f"{axis}_vmax").grid(row=3+i, column=4)
            ctk.CTkLabel(jog_frame, text=f"A{axis}:").grid(row=3+i, column=5)
            setattr(self, f"{axis}_amax", ctk.CTkEntry(jog_frame, placeholder_text="a", width=60))
            getattr(self, f"{axis}_amax").grid(row=3+i, column=6)

        # Conexión y Control
        control_frame = ctk.CTkFrame(right_combined)
        control_frame.pack(side="left", fill="y", padx=10, pady=10)

        ctk.CTkLabel(control_frame, text="Conexión y Control", font=("Segoe UI", 16, "bold")).pack(pady=(10,5))
        self.port_entry = ctk.CTkEntry(control_frame, placeholder_text="Puerto COM (opcional)")
        self.port_entry.pack(fill="x", padx=10, pady=5)
        self.connect_button = ctk.CTkButton(control_frame, text="Conectar", command=self.connect_pressed)
        self.connect_button.pack(fill="x", padx=10, pady=5)
        self.enable_button = ctk.CTkButton(control_frame, text="Enable ON", fg_color="#107C41", command=self.toggle_enable)
        self.enable_button.pack(fill="x", padx=10, pady=5)
        self.home_button = ctk.CTkButton(control_frame, text="HOME", command=self.home_pressed, fg_color="orange")
        self.home_button.pack(fill="x", padx=10, pady=5)
        self.stop_button = ctk.CTkButton(control_frame, text="STOP", command=self.stop_pressed, fg_color="#D33E2A")
        self.stop_button.pack(fill="x", padx=10, pady=5)
        self.ps4_button = ctk.CTkButton(control_frame, text="Control PS4", fg_color="#4040FF", command=self.open_joystick_window)
        self.ps4_button.pack(fill="x", padx=10, pady=5)
        self.gripper_button = ctk.CTkButton(control_frame, text="Abrir Gripper", fg_color="#0078D7", command=self.toggle_gripper)
        self.gripper_button.pack(fill="x", padx=10, pady=5)

        # ---------- Controles para movimiento según ECU (UI defaults) ----------
        ctk.CTkLabel(control_frame, text="Mov. según ECU:", anchor="w").pack(padx=10, pady=(8,0))

        ecu_frame = ctk.CTkFrame(control_frame)
        ecu_frame.pack(fill="x", padx=10, pady=(4,6))

        # ΔZ para Ecu-1
        ctk.CTkLabel(ecu_frame, text="Ecu-1 ΔZ:").grid(row=0, column=0, sticky="w", padx=(0,4))
        self.ecu1_entry = ctk.CTkEntry(ecu_frame, width=60)
        self.ecu1_entry.grid(row=0, column=1, padx=(0,6))
        self.ecu1_entry.insert(0, "-120")

        # ΔZ para Ecu-2
        ctk.CTkLabel(ecu_frame, text="Ecu-2 ΔZ:").grid(row=0, column=2, sticky="w", padx=(6,4))
        self.ecu2_entry = ctk.CTkEntry(ecu_frame, width=60)
        self.ecu2_entry.grid(row=0, column=3, padx=(0,6))
        self.ecu2_entry.insert(0, "-30")

        # V y A para el comando
        ctk.CTkLabel(ecu_frame, text="V:").grid(row=1, column=0, sticky="w", padx=(0,4), pady=(6,0))
        self.ecu_v_entry = ctk.CTkEntry(ecu_frame, width=60)
        self.ecu_v_entry.grid(row=1, column=1, pady=(6,0))
        self.ecu_v_entry.insert(0, "100")

        ctk.CTkLabel(ecu_frame, text="A:").grid(row=1, column=2, sticky="w", padx=(6,4), pady=(6,0))
        self.ecu_a_entry = ctk.CTkEntry(ecu_frame, width=60)
        self.ecu_a_entry.grid(row=1, column=3, pady=(6,0))
        self.ecu_a_entry.insert(0, "3000")

        # Button left for compatibility (optional)
        ctk.CTkButton(control_frame, text="Mover según ECU (botón)", fg_color="#1C7ED6",
                      command=self.move_z_based_on_ecu).pack(fill="x", padx=10, pady=(6,8))

        control2_frame = ctk.CTkFrame(right_combined)
        control2_frame.pack(side="left", fill="y", padx=10, pady=10)

        self.status_label = ctk.CTkLabel(control2_frame, text="Estado: Desconectado")
        self.status_label.pack(pady=10)

        ctk.CTkLabel(control2_frame, text="Log Arduino:").pack(pady=(10,0))
        self.log_box = tk.Text(control2_frame, height=10, width=45, bg="#1F1F1F", fg="white")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=(5,10))

        # Terminal Manual
        terminal_frame = ctk.CTkFrame(right)
        terminal_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(terminal_frame, text="Terminal Manual", font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=5, pady=(5,2))

        term_inner = ctk.CTkFrame(terminal_frame)
        term_inner.pack(fill="x", padx=5, pady=(0,5))

        self.term_entry = ctk.CTkEntry(term_inner, placeholder_text="Ejemplo: moveJ J1 1000 800 400")
        self.term_entry.pack(side="left", fill="x", expand=True, padx=(0,5))
        self.term_entry.bind("<Return>", self.send_terminal_command)
        ctk.CTkButton(term_inner, text="Enviar", fg_color="#107C41", command=self.send_terminal_command).pack(side="right")

        # Panel de Rutinas (scripts)
        script_frame = ctk.CTkFrame(right)
        script_frame.pack(fill="both", expand=True, padx=10, pady=10)
        ctk.CTkLabel(script_frame, text="Rutinas (Script)", font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=5, pady=(5,2))

        self.script_box = tk.Text(script_frame, height=12, width=70, bg="#1F1F1F", fg="white", insertbackground="white")
        self.script_box.pack(fill="both", expand=True, padx=5, pady=(0,5))

        script_buttons = ctk.CTkFrame(script_frame)
        script_buttons.pack(fill="x", pady=(5, 5))

        ctk.CTkButton(script_buttons, text="▶ Ejecutar rutina", fg_color="#107C41", command=self.run_script_threaded).pack(side="left", padx=5)
        ctk.CTkButton(script_buttons, text="💾 Guardar rutina", command=self.save_script_to_file).pack(side="left", padx=5)
        ctk.CTkButton(script_buttons, text="📂 Cargar rutina", command=self.load_script_from_file).pack(side="left", padx=5)
        ctk.CTkButton(script_buttons, text="🧹 Limpiar", fg_color="#D33E2A", command=self.clear_script_box).pack(side="left", padx=5)
        ctk.CTkButton(script_buttons, text="🔁 Loop ON/OFF", fg_color="#1C7ED6", command=self.toggle_loop).pack(side="left", padx=5)
        ctk.CTkButton(script_buttons, text="⛔ Stop", fg_color="#D33E2A", command=self.stop_script).pack(side="left", padx=5)
     
    # -------------------------
    # Helper: append log
    # -------------------------
    def append_log(self, text):
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")

    # -------------------------
    # Connection actions
    # -------------------------
    def connect_pressed(self):
        explicit = self.port_entry.get().strip()
        try:
            if explicit == "":
                self.append_log("Intentando detectar Arduino...")
                self.serial.connect()
            else:
                self.serial.connect(explicit_port=explicit)
            self.status_label.configure(text=f"Estado: Conectado ({self.serial.port})")
            self.append_log(f"Conectado a {self.serial.port}")
        except Exception as e:
            messagebox.showerror("Error conexión", str(e))
            self.append_log(f"Error conectar: {e}")

    def toggle_enable(self):
        current = self.enable_button.cget("text")
        if "ON" in current:
            try:
                self.serial.send("enable on")
                self.enable_button.configure(text="Enable OFF", fg_color="#777777")
            except Exception as e:
                self.append_log(f"Error: {e}")
        else:
            try:
                self.serial.send("enable off")
                self.enable_button.configure(text="Enable ON", fg_color="#107C41")
            except Exception as e:
                self.append_log(f"Error: {e}")

    def stop_pressed(self):
        try:
            self.serial.send("s")
        except Exception as e:
            self.append_log(f"Error: {e}")

    def home_pressed(self):
        try:
            self.serial.send("home")
        except Exception as e:
            self.append_log(f"Error: {e}")

    def unit_changed(self, val):
        if val == "Pasos":
            self.unit_mode.set("steps")
        else:
            self.unit_mode.set("human")
        self.append_log(f"Modo unidades: {self.unit_mode.get()}")

    # -------------------------
    # Jog
    # -------------------------
    def jog_axis(self, axis, direction):
        val = self.jog_value.get().strip()
        if val == "":
            messagebox.showwarning("Jog", "Ingrese distancia (pasos o grados/mm).")
            return
        try:
            v = float(val)
        except Exception:
            messagebox.showwarning("Jog", "Valor inválido")
            return

        if self.unit_mode.get() == "steps":
            steps = int(v) * direction
        else:
            if axis in ["J1","J2","J6"]:
                steps = int(self.motion.units_to_steps(axis, v * direction))
            elif axis == "Z":
                steps = int(self.motion.units_to_steps("Z", v * direction))
            else:
                steps = int(v * direction)

        vmax = getattr(self, f"{axis}_vmax").get().strip()
        amax = getattr(self, f"{axis}_amax").get().strip()
        if vmax == "":
            vmax = self.jog_vmax.get().strip() or "1000"
        if amax == "":
            amax = self.jog_amax.get().strip() or "3000"

        cmd = f"moveJ {axis} {steps} {int(float(vmax))} {int(float(amax))}"
        try:
            self.serial.send(cmd)
        except Exception as e:
            self.append_log(str(e))

    # -------------------------
    # Status update loop
    # -------------------------
    def update_status_loop(self):
        try:
            text = self.log_box.get("1.0", "end").strip().splitlines()
            for line in reversed(text[-30:]):
                ln = line.lower()
                if "pos" in ln or "j1" in ln:
                    parts = ln.replace(":", " ").replace(",", " ").split()
                    d = {}
                    for i, w in enumerate(parts):
                        if w.upper() in ["J1", "J2", "Z", "J6"]:
                            try:
                                d[w.upper()] = int(parts[i+1])
                            except:
                                pass
                    if d:
                        try:
                            j1_p = d.get("J1", 0)
                            j2_p = d.get("J2", 0)
                            z_p  = d.get("Z", 0)
                            j6_p = d.get("J6", 0)

                            j1_deg = self.motion.steps_to_units("J1", j1_p)
                            j2_deg = self.motion.steps_to_units("J2", j2_p)
                            z_mm   = self.motion.steps_to_units("Z",  z_p)
                            j6_deg = self.motion.steps_to_units("J6", j6_p)

                            self.j1_var.set(f"J1: {j1_p} p | {j1_deg:.2f}°")
                            self.j2_var.set(f"J2: {j2_p} p | {j2_deg:.2f}°")
                            self.z_var.set( f"Z: {z_p} p | {z_mm:.2f} mm")
                            self.j6_var.set(f"J6: {j6_p} p | {j6_deg:.2f}°")

                            x, y, _, phi_abs = scara_kinematics.forward_kinematics(j1_deg, j2_deg,z_mm,j6_deg)

                            self.x_var.set(f"X: {x:.2f} mm")
                            self.y_var.set(f"Y: {y:.2f} mm")
                            self.z_cart_var.set(f"Z: {z_mm:.2f} mm")
                            self.phi_var.set(f"φ_abs: {phi_abs:.2f}°")
                        except Exception as e:
                            self.append_log(f"⚠ Error parsing posición: {e}")
                        break
        except Exception:
            pass

        self.root.after(200, self.update_status_loop)

    # --------------------------
    # Gestión de puntos
    # --------------------------
    def save_point_button(self):
        name = self.point_name_entry.get().strip()
        if not name:
            messagebox.showwarning("Guardar Punto", "Ingrese un nombre para el punto.")
            return
    
        try:
            x = parse_float_from_label(self.x_var.get())
            y = parse_float_from_label(self.y_var.get())
            z = parse_float_from_label(self.z_cart_var.get())
            phi_abs = parse_float_from_label(self.phi_var.get())

            self.point_manager.add_point(name, x, y, z, phi_abs)
            self.append_log(f"Punto guardado: {name} ({x:.2f}, {y:.2f}, {z:.2f}, φ={phi_abs:.2f})")
            self.refresh_points_list()
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar punto: {e}")

    def refresh_points_list(self):
        self.points_list.delete(0, "end")
        pts = self.point_manager.get_points()
        for name, coords in pts.items():
            tipo = coords.get("tipo", "")
            display_tipo = f", tipo={tipo}" if tipo else ""
            self.points_list.insert(
                "end",
                f"{name}: x={coords['x']:.2f}, y={coords['y']:.2f}, z={coords['z']:.2f}, φ={coords['phi_abs']:.2f}{display_tipo}"
            )
    
    def go_to_point(self):
        sel = self.points_list.curselection()
        if not sel:
            messagebox.showwarning("Mover", "Seleccione un punto.")
            return
    
        text = self.points_list.get(sel)
        name = text.split(":")[0].strip()
        coords = self.point_manager.get_point(name)
        if not coords:
            messagebox.showerror("Error", "Punto no encontrado.")
            return

        x, y, z, phi_abs = coords["x"], coords["y"], coords["z"], coords["phi_abs"]
        cmd_type = "moveJ"
        
        try:
            ik_solutions = scara_kinematics.inverse_kinematics(x, y, z,phi_abs)
            if not ik_solutions:
                raise ValueError("No se encontraron soluciones IK válidas")
            ik = ik_solutions[0]
        except Exception as e:
            messagebox.showerror("Error IK", str(e))
            return

        j1_pas = parse_float_from_label2(self.j1_var.get())
        j2_pas = parse_float_from_label2(self.j2_var.get())
        z_pas  = parse_float_from_labelz(self.z_var.get())
        j6_pas = parse_float_from_label2(self.j6_var.get())

        tokens = []
        for eje, valor in {
            "J1": ik["theta1"],
            "J2": ik["theta2"],
            "Z":  ik["d"],
            "J6": ik["phi_m"]
        }.items():
            if eje in ["J1", "J2", "J6"]:
                steps_target = int(self.motion.units_to_steps(eje, valor))
            else:
                steps_target = int(self.motion.units_to_steps("Z", valor))

            steps_current = int(
                j1_pas if eje == "J1" else
                j2_pas if eje == "J2" else
                z_pas if eje == "Z" else
                j6_pas
            )
            
            V = 500 
            a = 3000 
            delta = steps_target - steps_current
            tokens += [eje, str(delta), str(V), str(a)]

        cmd = cmd_type + " " + " ".join(tokens)
        try:
            self.serial.send(cmd)
        except Exception as e:
            self.append_log(f"Error moviendo a {name}: {e}")


    def delete_point_button(self):
        sel = self.points_list.curselection()
        if not sel:
            messagebox.showwarning("Borrar", "Seleccione un punto para borrar.")
            return
        text = self.points_list.get(sel)
        name = text.split(":")[0].strip()
        if messagebox.askyesno("Confirmar", f"¿Borrar punto '{name}'?"):
            self.point_manager.delete_point(name)
            self.refresh_points_list()
            self.append_log(f"Punto '{name}' eliminado.")

    # ============================================
    # Terminal de comandos
    # ============================================
    def send_terminal_command(self, event=None):
        cmd = self.term_entry.get().strip()
        if cmd == "":
            return
        try:
            self.serial.send(cmd)
            self.append_log(f" Enviado: {cmd}")
            self.term_entry.delete(0, "end")
        except Exception as e:
            self.append_log(f" Error enviando comando: {e}")
            
    def send_to_esp(self, text, host='127.0.0.1', port=60001):
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect((host, port))
            client.sendall((text + "\n").encode('utf-8'))
            response = client.recv(1024).decode('utf-8').strip()
            client.close()
            self.append_log(f"📡 ESP Envío: '{text}'")
            return True
        except ConnectionRefusedError:
            self.append_log("❌ Error: esp_comm.py no está corriendo o el puerto 60001 está cerrado.")
            return False
        except Exception as e:
            self.append_log(f"❌ Error comunicando con ESP: {e}")
            return False

    # ==========================================================
    # FUNCIONES DEL PANEL DE RUTINAS CON LOOP Y GOTO
    # ==========================================================
    def run_script_threaded(self):
        t = threading.Thread(target=self.run_script)
        t.daemon = True
        t.start()

    def run_script(self):
        lines = self.script_box.get("1.0", "end").strip().splitlines()
        if not lines:
            messagebox.showinfo("Rutina", "No hay comandos para ejecutar.")
            return

        self._stop_script = False
        self.append_log(" Iniciando ejecución de rutina...")

        idx = 0
        n_lines = len(lines)

        def execute_simple_cmd(cmd):
            try:
                self.serial.send(cmd)
                self.append_log(f" Enviado: {cmd}")
                self.root.update()
                time.sleep(0.1)
                return True
            except Exception as e:
                self.append_log(f" Error al enviar '{cmd}': {e}")
                return False

        while True:
            idx = 0
            while idx < n_lines:
                if self._stop_script:
                    self.append_log(" Rutina detenida por el usuario.")
                    return

                line = lines[idx].strip()
                idx += 1

                if not line or line.startswith("#"):
                    continue

                lower = line.lower()

                # Comando nuevo: moveecu
                # Sintaxis:
                # 1) moveecu
                # 2) moveecu <dz1> <dz2> <V> <A>
                # 3) moveecu override <Ecu-1|Ecu-2> <dz> <V> <A>
                if lower.startswith("moveecu"):
                    parts = line.split()
                    try:
                        if len(parts) == 1:
                            # usar UI/defaults
                            self.move_z_based_on_ecu()
                        elif len(parts) == 5 and parts[1] not in ("override",):
                            # moveecu dz1 dz2 V A
                            dz1 = float(parts[1])
                            dz2 = float(parts[2])
                            V = int(float(parts[3]))
                            A = int(float(parts[4]))
                            self._moveecu_with_params(dz1, dz2, V, A, override_type=None)
                        elif len(parts) >= 6 and parts[1].lower() == "override":
                            # moveecu override Ecu-1 -110 100 3000
                            override_type = parts[2]
                            dz = float(parts[3])
                            V = int(float(parts[4]))
                            A = int(float(parts[5]))
                            # map dz to dz1 or dz2 depending override_type
                            if "1" in override_type or "ecu-1" in override_type.lower() or "ecu1" in override_type.lower():
                                dz1 = dz
                                dz2 = float(self.ecu2_entry.get() or "-30")
                            else:
                                dz2 = dz
                                dz1 = float(self.ecu1_entry.get() or "-120")
                            self._moveecu_with_params(dz1, dz2, V, A, override_type=override_type)
                        else:
                            self.append_log(" Sintaxis inválida para moveecu. Usa: moveecu | moveecu dz1 dz2 V A | moveecu override Ecu-1 dz V A")
                    except Exception as e:
                        self.append_log(f" Error procesando moveecu: {e}")
                    continue

                # ---> BORRAR PUNTO: delpoint <nombre>
                if lower.startswith("delpoint ") or lower.startswith("rmpt "):
                    parts = line.split(maxsplit=1)
                    if len(parts) < 2 or not parts[1].strip():
                        self.append_log(" ⚠️ Sintaxis inválida en delpoint (use: delpoint <nombre>)")
                        continue
                    name = parts[1].strip()
                    if self.point_manager.get_point(name):
                        self.point_manager.delete_point(name)
                        self.refresh_points_list()
                        self.append_log(f" Punto '{name}' eliminado (comando delpoint).")
                    else:
                        self.append_log(f" Punto '{name}' no existe.")
                    continue

                # ---> BLOQUE UNTIL: until <nombre_punto> { ... }
                if lower.startswith("until "):
                    try:
                        rest = line[len("until "):].strip()
                        if "{" in rest:
                            parts = rest.split("{", 1)
                            point_name = parts[0].strip()
                            block_first = parts[1]
                            block_lines = []
                            if "}" in block_first:
                                inner = block_first.split("}", 1)[0]
                                for s in [l.strip() for l in inner.split(";") if l.strip()]:
                                    block_lines.append(s)
                            else:
                                if block_first.strip():
                                    block_lines.append(block_first.strip())
                                while idx < n_lines:
                                    next_line = lines[idx].strip()
                                    idx += 1
                                    if "}" in next_line:
                                        before = next_line.split("}", 1)[0].strip()
                                        if before:
                                            block_lines.append(before)
                                        break
                                    block_lines.append(next_line)
                        else:
                            parts = rest.split(None, 1)
                            if not parts:
                                self.append_log(" ⚠️ Sintaxis inválida en until (no se encontró nombre).")
                                continue
                            point_name = parts[0].strip()
                            found_open = False
                            block_lines = []
                            while idx < n_lines:
                                l = lines[idx].strip()
                                idx += 1
                                if "{" in l:
                                    found_open = True
                                    after = l.split("{", 1)[1]
                                    if "}" in after:
                                        inner = after.split("}", 1)[0]
                                        for s in [t.strip() for t in inner.split(";") if t.strip()]:
                                            block_lines.append(s)
                                        break
                                    else:
                                        if after.strip():
                                            block_lines.append(after.strip())
                                        while idx < n_lines:
                                            nl = lines[idx].strip()
                                            idx += 1
                                            if "}" in nl:
                                                before = nl.split("}", 1)[0].strip()
                                                if before:
                                                    block_lines.append(before)
                                                break
                                            block_lines.append(nl)
                            if not found_open and not block_lines:
                                self.append_log(" ⚠️ Bloque 'until' sin '{' de apertura encontrado. Ignorando.")
                                continue

                        if not point_name:
                            self.append_log(" ⚠️ Nombre de punto vacío en until.")
                            continue

                        point_name = point_name.strip()
                        self.append_log(f" 🔁 Ejecutando bloque UNTIL '{point_name}' hasta que exista el punto...")

                        if self.point_manager.get_point(point_name):
                            self.append_log(f" ✅ Punto '{point_name}' ya existe. Saltando bloque UNTIL.")
                            continue

                        while not self.point_manager.get_point(point_name):
                            if self._stop_script:
                                self.append_log(" Rutina detenida por el usuario dentro de UNTIL.")
                                return

                            for inner_cmd in block_lines:
                                inner_cmd = inner_cmd.strip()
                                if not inner_cmd or inner_cmd.startswith("#"):
                                    continue

                                low_inner = inner_cmd.lower()

                                if low_inner.startswith("goto"):
                                    parts = inner_cmd.split()
                                    if len(parts) < 2:
                                        self.append_log(" Sintaxis inválida en goto dentro de UNTIL (use: goto nombre_punto)")
                                        continue
                                    point_name_g = parts[1]
                                    point = self.point_manager.get_point(point_name_g)
                                    if not point:
                                        self.append_log(f" Punto '{point_name_g}' no encontrado en JSON.")
                                        continue
                                    try:
                                        x, y, z = point["x"], point["y"], point["z"]
                                        phi_abs = point.get("phi_abs", 0.0)
                                        ik_solutions = scara_kinematics.inverse_kinematics(x, y, z, phi_abs)
                                        if not ik_solutions:
                                            raise ValueError("No se encontraron soluciones IK válidas")
                                        ik = ik_solutions[0]
                                        j1_pas = parse_float_from_label2(self.j1_var.get())
                                        j2_pas = parse_float_from_label2(self.j2_var.get())
                                        z_pas  = parse_float_from_labelz(self.z_var.get())
                                        j6_pas = parse_float_from_label2(self.j6_var.get())
                                        tokens = []
                                        V = 1000
                                        a = 3000
                                        for eje, valor in {
                                            "J1": ik["theta1"],
                                            "J2": ik["theta2"],
                                            "Z": ik["d"],
                                            "J6": ik["phi_m"]
                                        }.items():
                                            if eje in ["J1", "J2", "J6"]:
                                                steps_target = int(self.motion.units_to_steps(eje, valor))
                                            else:
                                                steps_target = int(self.motion.units_to_steps("Z", valor))
                                            steps_current = int(
                                                j1_pas if eje == "J1" else
                                                j2_pas if eje == "J2" else
                                                z_pas if eje == "Z" else
                                                j6_pas
                                            )
                                            delta = steps_target - steps_current
                                            tokens += [eje, str(delta), str(V), str(a)]
                                        cmd_full = "moveJ " + " ".join(tokens)
                                        self.serial.send(cmd_full)
                                        self.append_log(f" Goto (UNTIL) {point_name_g} → {cmd_full}")
                                    except Exception as e:
                                        self.append_log(f" Error en goto (UNTIL) {point_name_g}: {e}")
                                    continue

                                if low_inner.startswith("wait"):
                                    try:
                                        t = float(inner_cmd.split()[1])
                                        self.append_log(f" Esperando {t} segundos... (UNTIL block)")
                                        self.root.update()
                                        time.sleep(t)
                                    except Exception:
                                        self.append_log(" Comando wait inválido en UNTIL.")
                                    continue

                                if low_inner.startswith("getecu"):
                                    self.append_log("🔍 Ejecutando comando GETECU desde UNTIL...")
                                    threading.Thread(target=self.obtener_ecu_y_guardar, daemon=True).start()
                                    time.sleep(0.5)
                                    continue

                                if low_inner.startswith("moveecu"):
                                    # reutilizar la misma lógica que arriba (simple despacho)
                                    try:
                                        # llamar recursivamente al parser simplificado: crear una mini lista
                                        saved = [inner_cmd]
                                        # ejecutar en contexto: usar la misma lógica que en la rutina principal
                                        # llamamos a run_script de forma simple no recursiva: replicar parse breve
                                        parts = inner_cmd.split()
                                        if len(parts) == 1:
                                            self.move_z_based_on_ecu()
                                        elif len(parts) == 5 and parts[1] not in ("override",):
                                            dz1 = float(parts[1]); dz2 = float(parts[2]); V = int(float(parts[3])); A = int(float(parts[4]))
                                            self._moveecu_with_params(dz1, dz2, V, A, override_type=None)
                                        elif len(parts) >= 6 and parts[1].lower() == "override":
                                            override_type = parts[2]; dz = float(parts[3]); V = int(float(parts[4])); A = int(float(parts[5]))
                                            if "1" in override_type or "ecu-1" in override_type.lower() or "ecu1" in override_type.lower():
                                                dz1 = dz; dz2 = float(self.ecu2_entry.get() or "-30")
                                            else:
                                                dz2 = dz; dz1 = float(self.ecu1_entry.get() or "-120")
                                            self._moveecu_with_params(dz1, dz2, V, A, override_type=override_type)
                                        else:
                                            self.append_log(" Sintaxis inválida para moveecu dentro de UNTIL.")
                                    except Exception as e:
                                        self.append_log(f" Error moveecu en UNTIL: {e}")
                                    continue

                                if low_inner.startswith("sendesp "):
                                    payload = inner_cmd[8:].strip()
                                    if payload:
                                        self.send_to_esp(payload)
                                    else:
                                        self.append_log("⚠️ Sintaxis inválida en sendesp (use: sendesp <texto>)")
                                    time.sleep(0.1)
                                    continue

                                if low_inner.startswith("delpoint ") or low_inner.startswith("rmpt "):
                                    parts = inner_cmd.split(maxsplit=1)
                                    if len(parts) < 2 or not parts[1].strip():
                                        self.append_log(" ⚠️ Sintaxis inválida en delpoint dentro de UNTIL.")
                                        continue
                                    name = parts[1].strip()
                                    if self.point_manager.get_point(name):
                                        self.point_manager.delete_point(name)
                                        self.refresh_points_list()
                                        self.append_log(f" Punto '{name}' eliminado (delpoint dentro de UNTIL).")
                                    else:
                                        self.append_log(f" Punto '{name}' no existe (delpoint dentro de UNTIL).")
                                    continue

                                execute_simple_cmd(inner_cmd)

                            time.sleep(0.3)

                        self.append_log(f" ✅ UNTIL terminado: punto '{point_name}' ahora existe.")
                        continue

                    except Exception as e:
                        self.append_log(f" ⚠️ Error procesando UNTIL: {e}")
                        continue

                # --- Comando de espera ---
                if lower.startswith("wait"):
                    try:
                        t = float(line.split()[1])
                        self.append_log(f" Esperando {t} segundos...")
                        self.root.update()
                        time.sleep(t)
                    except Exception:
                        self.append_log(" Comando wait inválido.")
                    continue

                # --- GET ECU desde rutina ---
                if lower.startswith("getecu"):
                    self.append_log("🔍 Ejecutando comando GETECU desde rutina...")
                    threading.Thread(target=self.obtener_ecu_y_guardar, daemon=True).start()
                    time.sleep(0.5)
                    continue

                # ---> NUEVO: Comando para enviar texto al ESP32 <---
                if lower.startswith("sendesp "):
                    payload = line[8:].strip()
                    if payload:
                        self.send_to_esp(payload)
                    else:
                        self.append_log("⚠️ Sintaxis inválida en sendesp (use: sendesp <texto>)")
                    time.sleep(0.1)
                    continue
                
                # --- GOTO a punto guardado ---
                if lower.startswith("goto"):
                    parts = lower.split()
                    if len(parts) < 2:
                        self.append_log(" Sintaxis inválida en goto (use: goto nombre_punto)")
                        continue

                    point_name = parts[1]
                    point = self.point_manager.get_point(point_name)

                    if not point:
                        self.append_log(f" Punto '{point_name}' no encontrado en JSON.")
                        continue

                    try:
                        x = point["x"]
                        y = point["y"]
                        z = point["z"]
                        phi_abs = point.get("phi_abs", 0.0)

                        ik_solutions = scara_kinematics.inverse_kinematics(x, y, z, phi_abs)
                        if not ik_solutions:
                            raise ValueError("No se encontraron soluciones IK válidas")
                        ik = ik_solutions[0]

                        j1_pas = parse_float_from_label2(self.j1_var.get())
                        j2_pas = parse_float_from_label2(self.j2_var.get())
                        z_pas  = parse_float_from_labelz(self.z_var.get())
                        j6_pas = parse_float_from_label2(self.j6_var.get())

                        tokens = []
                        V = 700
                        a = 3000

                        for eje, valor in {
                            "J1": ik["theta1"],
                            "J2": ik["theta2"],
                            "Z":  ik["d"],
                            "J6": ik["phi_m"]
                        }.items():

                            if eje in ["J1", "J2", "J6"]:
                                steps_target = int(self.motion.units_to_steps(eje, valor))
                            else:
                                steps_target = int(self.motion.units_to_steps("Z", valor))

                            steps_current = int(
                                j1_pas if eje == "J1" else
                                j2_pas if eje == "J2" else
                                z_pas  if eje == "Z"  else
                                j6_pas
                            )

                            delta = steps_target - steps_current

                            tokens += [eje, str(delta), str(V), str(a)]

                        cmd_full = "moveJ " + " ".join(tokens)

                        self.serial.send(cmd_full)
                        self.append_log(f" Goto {point_name} → {cmd_full}")

                    except Exception as e:
                        self.append_log(f" Error en goto {point_name}: {e}")

                    time.sleep(0.2)
                    continue

                # --- Comandos normales (envío a serial) ---
                if not execute_simple_cmd(line):
                    break

            # fin while lines

            if not getattr(self, "_loop_script", False):
                break
            self.append_log(" Reejecutando rutina en loop...")

        self.append_log(" Rutina completada.")

    def toggle_loop(self):
        self._loop_script = not getattr(self, "_loop_script", False)
        status = "ON" if self._loop_script else "OFF"
        self.append_log(f" Loop {status}")

    def stop_script(self):
        self._stop_script = True
        self._loop_script = False
        self.append_log(" Deteniendo rutina...")

    def save_script_to_file(self):
        from tkinter import filedialog
        content = self.script_box.get("1.0", "end").strip()
        if not content:
            messagebox.showinfo("Guardar rutina", "No hay contenido para guardar.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Archivos de texto", "*.txt")],
            title="Guardar rutina como..."
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(content)
            self.append_log(f" Rutina guardada en: {file_path}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar el archivo: {e}")

    def load_script_from_file(self):
        from tkinter import filedialog
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("Archivos de texto", "*.txt")],
            title="Cargar rutina"
        )
        if not file_path:
            return

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
            self.script_box.delete("1.0", "end")
            self.script_box.insert("1.0", content)
            self.append_log(f" Rutina cargada desde: {file_path}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo cargar el archivo: {e}")

    def clear_script_box(self):
        self.script_box.delete("1.0", "end")
        self.append_log(" Rutina limpiada.")

    # ============================================
    # Gripper
    # ============================================
    def toggle_gripper(self):
        current = self.gripper_button.cget("text")
        try:
            if "Abrir" in current:
                self.serial.send("gripper open")
                self.gripper_button.configure(text="Cerrar Gripper", fg_color="#D33E2A")
                self.append_log(" Gripper abierto")
            else:
                self.serial.send("gripper close")
                self.gripper_button.configure(text="Abrir Gripper", fg_color="#1C7ED6")
                self.append_log(" Gripper cerrado")
        except Exception as e:
            self.append_log(f" Error controlando gripper: {e}")

    # ============================================
    # Joystick PS4
    # ============================================
    def open_joystick_window(self):
        top = ctk.CTkToplevel(self.root)
        top.title("Joystick PS4 - Teach Pendant")
        top.geometry("400x350")

        ctk.CTkLabel(top, text="Control PS4 conectado ", font=("Segoe UI", 16, "bold")).pack(pady=10)

        self.joystick_speed = tk.StringVar(value="Velocidad: 800 pasos/s")
        self.joystick_accel = tk.StringVar(value="Aceleración: 400 pasos/s²")
        ctk.CTkLabel(top, textvariable=self.joystick_speed).pack(pady=5)
        ctk.CTkLabel(top, textvariable=self.joystick_accel).pack(pady=5)

        self.joystick_status = tk.StringVar(value="Esperando entrada...")
        ctk.CTkLabel(top, textvariable=self.joystick_status, wraplength=350).pack(pady=10)

        ctk.CTkButton(top, text="Desconectar", fg_color="#D33E2A",command=lambda: setattr(self, "joystick_running", False)).pack(pady=20)

        self.joystick_running = True
        threading.Thread(target=self.joystick_loop, daemon=True).start()


    def joystick_loop(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.append_log("No se detectó ningún joystick PS4 conectado.")
            return

        js = pygame.joystick.Joystick(0)
        js.init()
        self.append_log(f"Joystick detectado: {js.get_name()}")

        vmax, amax = 800, 1000
        gripper_open = False

        self.update_joystick_status(vmax, amax)

        while getattr(self, "joystick_running", False):
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    btn = event.button
                    if btn == 0:
                        self.save_point_button()
                    elif btn == 1:
                        self.serial.send("s")
                        self.append_log(" STOP (PS4)")
                    elif btn == 2:
                        cmd = "gripper close" if gripper_open else "gripper open"
                        self.serial.send(cmd)
                        gripper_open = not gripper_open
                        self.append_log(f" Gripper: {'cerrado' if gripper_open else 'abierto'}")
                    elif btn == 3:
                        self.serial.send("home")
                    elif btn == 9:
                        self.move_axis("J1", -1, vmax, amax)
                    elif btn == 10:
                        self.move_axis("J1", +1, vmax, amax)
                    elif btn == 11:
                        vmax += 100
                        self.append_log(f"⬆️ Velocidad + : {vmax}")
                    elif btn == 12:
                        vmax = max(100, vmax - 100)
                        self.append_log(f"⬇️ Velocidad - : {vmax}")
                    elif btn == 13:
                        amax = max(50, amax - 50)
                        self.append_log(f"⬅️ Aceleración - : {amax}")
                    elif btn == 14:
                        amax += 50
                        self.append_log(f"➡️ Aceleración + : {amax}")

                elif event.type == pygame.JOYAXISMOTION:
                    if event.axis == 1:
                        if abs(event.value) > 0.3:
                            direction = 1 if event.value < 0 else -1
                            self.move_axis("Z", direction, vmax, amax)
                    elif event.axis == 2:
                        if abs(event.value) > 0.3:
                            direction = -1 if event.value < 0 else 1
                            self.move_axis("J6", direction, vmax, amax)
                    elif event.axis == 4:
                        if event.value > 0.3:
                            self.move_axis("J2", -1, vmax, amax)
                    elif event.axis == 5:
                        if event.value > 0.3:
                            self.move_axis("J2", +1, vmax, amax)

            self.update_joystick_status(vmax, amax)
            time.sleep(0.05)

        pygame.quit()
        self.append_log(" Joystick desconectado.")

    def move_axis(self, axis, direction, vmax, amax):
        steps = 200 * direction
        cmd = f"moveJ {axis} {steps} {vmax} {amax}"
        try:
            self.serial.send(cmd)
            self.append_log(f"➡️ {cmd}")
        except Exception as e:
            self.append_log(f" Error joystick: {e}")

    def update_joystick_status(self, vmax, amax):
        self.joystick_speed.set(f"Velocidad: {vmax} pasos/s")
        self.joystick_accel.set(f"Aceleración: {amax} pasos/s²")

    # ============================================
    # Camara: obtener ECU y guardar tipo en p_ecu
    # ============================================
    def obtener_ecu_y_guardar(self, reintentos=5, delay=15):
        import re
        self.append_log("📷 Buscando ECU...")

        for intento in range(reintentos):
            pose = obtener_coordenadas()
            if not pose:
                self.append_log(f"⚠️ No se detectó ninguna ECU visible. Reintentando ({intento+1}/{reintentos})...")
                time.sleep(delay)
                continue

            try:
                x_cam = float(pose.get("x", 0))
                y_cam = float(pose.get("y", 0))
                z_cam = float(pose.get("z", 0))
                phi_raw = float(pose.get("yaw", 0))
            except Exception as e:
                self.append_log(f"⚠️ Error al leer valores de pose: {e}. Reintentando...")
                time.sleep(delay)
                continue

            phi_abs = normalize_and_invert_yaw(phi_raw)

            ecu_type_raw = None
            for key in ("type", "label", "class", "ecu_type", "detected_class", "name"):
                if key in pose and pose.get(key) is not None:
                    ecu_type_raw = str(pose.get(key))
                    break

            if not ecu_type_raw or ecu_type_raw.strip() == "":
                self.append_log(f"⚠️ Tipo de ECU no identificado en la detección. Reintentando ({intento+1}/{reintentos})...")
                time.sleep(delay)
                continue

            ecu_type = ecu_type_raw.lower().strip()
            norm = re.sub(r'[^a-z0-9]', '', ecu_type)

            z_final = None
            if re.search(r'\b3\b', ecu_type) or 'ecu3' in norm:
                self.append_log(f"🔁 Se detectó '{ecu_type_raw}' (Ecu-3). Volviendo a pedir coordenadas ({intento+1}/{reintentos})...")
                time.sleep(delay)
                continue
            elif re.search(r'\b1\b', ecu_type) or 'ecu1' in norm:
                z_final = 70
                tipo_usado = "Ecu-1"
            elif re.search(r'\b2\b', ecu_type) or 'ecu2' in norm:
                z_final = 60
                tipo_usado = "Ecu-2"
            else:
                self.append_log(f"⚠️ Tipo de ECU ambiguo/no reconocido: '{ecu_type_raw}'. Reintentando ({intento+1}/{reintentos})...")
                time.sleep(delay)
                continue

            x, y, z = transformar_a_robot(x_cam, y_cam, z_cam)
            z = z_final

            # Guardar o sobrescribir en JSON (añadir campo tipo)
            self.point_manager.add_point("p_ecu", x, y, z, phi_abs)
            try:
                self.point_manager.points["p_ecu"]["tipo"] = tipo_usado
                self.point_manager.save_points()
            except Exception:
                pass

            self.append_log(f"✅ {tipo_usado} detectada y guardada: p_ecu ({x:.1f}, {y:.1f}, {z:.1f}, φ={phi_abs:.1f})")
            self.refresh_points_list()
            return

        self.append_log("❌ No se pudo detectar una ECU válida (Ecu-1 o Ecu-2) tras varios intentos.")

    # ==========================================================
    # Movimiento según ECU (función reutilizable)
    # ==========================================================
    def move_z_based_on_ecu(self, override_type=None):
        pt = self.point_manager.get_point("p_ecu")
        if not pt:
            messagebox.showwarning("Mover ECU", "No existe 'p_ecu'. Ejecute GETECU primero.")
            return

        tipo = str(pt.get("tipo", "")).lower()

        try:
            dz1 = float(self.ecu1_entry.get())
        except Exception:
            dz1 = -120.0
        try:
            dz2 = float(self.ecu2_entry.get())
        except Exception:
            dz2 = -30.0
        try:
            V = int(float(self.ecu_v_entry.get()))
        except Exception:
            V = 100
        try:
            A = int(float(self.ecu_a_entry.get()))
        except Exception:
            A = 3000

        if override_type:
            tipo = override_type.lower()

        if "1" in tipo or "ecu-1" in tipo or "ecu1" in tipo:
            dz = dz1
        elif "2" in tipo or "ecu-2" in tipo or "ecu2" in tipo:
            dz = dz2
        else:
            dz = dz2

        cmd = f"moveJ Z {dz} {V} {A}"
        try:
            self.serial.send(cmd)
            self.append_log(f"Mov. ECU ({pt.get('tipo','?')}) -> {cmd}")
        except Exception as e:
            self.append_log(f"Error al enviar mov ECU: {e}")
            messagebox.showerror("Error", f"No se pudo enviar comando: {e}")

    def _moveecu_with_params(self, dz1, dz2, V, A, override_type=None):
        """
        Función auxiliar que recibe parámetros numéricos y decide qué dz aplicar según el tipo guardado
        o el override_type.
        """
        pt = self.point_manager.get_point("p_ecu")
        if not pt:
            self.append_log(" No existe 'p_ecu'. Ejecuta getecu antes de moveecu.")
            return

        tipo = str(pt.get("tipo", "")).lower()
        if override_type:
            tipo = str(override_type).lower()

        if "1" in tipo or "ecu-1" in tipo or "ecu1" in tipo:
            dz = dz1
        elif "2" in tipo or "ecu-2" in tipo or "ecu2" in tipo:
            dz = dz2
        else:
            dz = dz2

        cmd = f"moveJ Z {dz} {V} {A}"
        try:
            self.serial.send(cmd)
            self.append_log(f"Mov. ECU ({pt.get('tipo','?')}) -> {cmd}")
        except Exception as e:
            self.append_log(f"Error al enviar mov ECU: {e}")

# --------------------------
# Main
# --------------------------
if __name__ == "__main__":
    root = ctk.CTk()
    gui = GUI(root)
    root.mainloop()