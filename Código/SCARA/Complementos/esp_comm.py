# esp_comm.py
import socket
import threading
import json
import time

# --- CONFIGURACIÓN ---
HOST = '0.0.0.0'
PORT = 60000          # puerto donde se conecta el ESP32
CONTROL_HOST = '127.0.0.1'
CONTROL_PORT = 60001  # puerto para que la GUI u otros procesos locales se conecten

# Variables compartidas
esp_conn = None
esp_addr = None
esp_lock = threading.Lock()

def process_received_data(line):
    """Parsea el JSON o texto recibido desde el ESP32/Mega"""
    try:
        data = json.loads(line)
        if "type" in data and data["type"] == "status":
            # mensaje de estado del ESP32
            pass
        elif "from_mega" in data:
            print(f"[MEGA] -> {data['from_mega']}")
        else:
            print(f"[JSON DESCONOCIDO] -> {data}")
    except json.JSONDecodeError:
        print(f"[RAW] -> {line}")

def handle_esp_client(conn, addr):
    """Hilo para recibir y procesar datos desde el ESP32"""
    global esp_conn, esp_addr
    print(f"\n[+] ¡ESP32 conectado desde {addr}!")
    with esp_lock:
        esp_conn = conn
        esp_addr = addr

    buffer = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                print(f"\n[-] ESP32 en {addr} se ha desconectado.")
                break
            buffer += data.decode('utf-8', errors='ignore')
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line:
                    process_received_data(line)
    except ConnectionResetError:
        print(f"\n[-] Conexión reseteada por el ESP32.")
    except Exception as e:
        print(f"\n[!] Error en la conexión ESP32: {e}")
    finally:
        with esp_lock:
            if esp_conn is conn:
                esp_conn = None
                esp_addr = None
        try:
            conn.close()
        except:
            pass

def control_client_handler(conn, addr):
    """
    Hilo para manejar una conexión de control desde la GUI (o cualquier cliente local).
    Lectura línea por línea; lo que reciba lo reenvía al ESP32 si está conectado.
    También responde con mensajes de confirmación/estado.
    """
    print(f"[+] Control client conectado desde {addr}")
    try:
        buffer = ""
        while True:
            data = conn.recv(1024)
            if not data:
                break
            buffer += data.decode('utf-8', errors='ignore')
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue

                # Si el cliente pide estado:
                if line.lower() in ('status', 'estado', 'info'):
                    with esp_lock:
                        if esp_conn:
                            resp = json.dumps({"status":"ok","esp_connected":True,"esp_addr":str(esp_addr)})
                        else:
                            resp = json.dumps({"status":"ok","esp_connected":False})
                    conn.sendall((resp + "\n").encode('utf-8'))
                    continue

                # Forward al ESP32
                with esp_lock:
                    if esp_conn:
                        try:
                            # enviamos tal cual lo recibimos; el ESP32 debería aceptar JSON o texto
                            esp_conn.sendall((line + "\n").encode('utf-8'))
                            # confirmación al cliente de control
                            conn.sendall((json.dumps({"sent": True, "payload": line}) + "\n").encode('utf-8'))
                        except Exception as e:
                            conn.sendall((json.dumps({"sent": False, "error": str(e)}) + "\n").encode('utf-8'))
                    else:
                        conn.sendall((json.dumps({"sent": False, "error": "No ESP32 conectado"}) + "\n").encode('utf-8'))
    except Exception as e:
        print(f"[!] Error control client {addr}: {e}")
    finally:
        try:
            conn.close()
        except:
            pass
        print(f"[-] Control client desconectado {addr}")

def control_server_thread():
    """Servidor de control (acepta conexiones locales de la GUI)"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((CONTROL_HOST, CONTROL_PORT))
    srv.listen(5)
    print(f"[*] Control server escuchando en {CONTROL_HOST}:{CONTROL_PORT} (GUI -> esp_comm)")
    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(target=control_client_handler, args=(conn, addr), daemon=True)
            t.start()
    except Exception as e:
        print(f"[!] Control server error: {e}")
    finally:
        try:
            srv.close()
        except:
            pass

def main():
    # Lanza el servidor de control en un hilo
    t_ctrl = threading.Thread(target=control_server_thread, daemon=True)
    t_ctrl.start()

    # Servidor principal para ESP32
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((HOST, PORT))
        server.listen(1)
        print(f"[*] Servidor TCP iniciado.")
        print(f"[*] Escuchando en el puerto {PORT} para ESP32...")
        print("[*] Esperando a que el ESP32 se conecte...\n")
    except Exception as e:
        print(f"[!] Error al iniciar servidor (¿El puerto está en uso?): {e}")
        return

    # Acepta la conexión del ESP32 (bloqueante)
    try:
        while True:
            conn, addr = server.accept()
            # manejar en hilo y volver a esperar conexiones ESP32
            th = threading.Thread(target=handle_esp_client, args=(conn, addr), daemon=True)
            th.start()

            # Console: mantener la interfaz actual para enviar comandos manualmente
            print("=== COMANDOS DISPONIBLES (consola local) ===")
            print("- Escribe 'led_on' o 'led_off' para controlar el LED del ESP32.")
            print("- Escribe cualquier otra cosa para enviarla por Serial2 al MEGA/ESP32.")
            print("- Escribe 'salir' para cerrar el programa.")
            print("============================================\n")

            # Bucle de consola para enviar comandos manuales (seguirá vivo mientras el server esté corriendo)
            while True:
                try:
                    comando = input("")
                except (EOFError, KeyboardInterrupt):
                    comando = "salir"

                if comando.lower() == 'salir':
                    print("Saliendo por consola... cerrando servidor.")
                    raise KeyboardInterrupt()

                # Si no hay ESP conectado, avisar
                with esp_lock:
                    if not esp_conn:
                        print("[!] No hay ESP32 conectado. No se puede enviar.")
                        continue
                    # Preparar mensaje
                    if comando in ['led_on', 'led_off']:
                        mensaje = json.dumps({"esp_cmd": comando})
                    else:
                        mensaje = comando
                    payload = mensaje + "\n"
                    try:
                        esp_conn.sendall(payload.encode('utf-8'))
                    except Exception as e:
                        print(f"[!] Error enviando al ESP32: {e}")
    except KeyboardInterrupt:
        print("\nSaliendo (KeyboardInterrupt).")
    except Exception as e:
        print(f"[!] Excepción principal: {e}")
    finally:
        try:
            server.close()
        except:
            pass

if __name__ == "__main__":
    main()