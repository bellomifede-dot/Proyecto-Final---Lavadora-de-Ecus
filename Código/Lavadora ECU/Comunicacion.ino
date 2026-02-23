#include <WiFi.h>
#include <ArduinoJson.h>

// ---------------- CONFIG ----------------
const char* STA_SSID = "MOVISTAR1234"; // tu SSID
const char* STA_PASS = "44876581";           // tu pass

const char* PC_SERVER_IP = "192.168.1.87";   // IP de la PC con la GUI (cliente mode)
const uint16_t PC_SERVER_PORT = 60000;       // puerto del server en la PC

const char* AP_SSID = "ESP32-Box";
const char* AP_PASS = "12345678";

const uint16_t LOCAL_SERVER_PORT = 60000;    // puerto en modo servidor fallback

// UART hacia MEGA (Serial2)
const int MEGA_RX_PIN = 16; // RX2 (a TX del Mega)
const int MEGA_TX_PIN = 17; // TX2 (a RX del Mega)
const unsigned long MEGA_BAUD = 115200; // baud con Mega — cambialo si tu Mega usa otro

const int RELAY_PIN = 26; // ejemplo (opcional)
const int LED_PIN = 2;

WiFiClient pcClient;        // si actuamos como cliente hacia la PC
WiFiServer localServer(LOCAL_SERVER_PORT);
WiFiClient localClient;     // si actuamos como server fallback and accept incoming client

String recvBuffer = "";
String megaRecvBuffer = "";

unsigned long lastStatusMs = 0;
const unsigned long STATUS_INTERVAL = 5000;

bool runningAsClient = false;
bool runningAsServer = false;

void debugPrint(const String &s) {
  Serial.println(s);
}

// Envío de JSON (compacto) al cliente conectado (PC)
void sendJsonToClient(WiFiClient &c, const JsonDocument &doc) {
  if (!c || !c.connected()) return;
  String out;
  serializeJson(doc, out);
  out += "\n";
  c.print(out);
}

// Intenta conectar como TCP client a la PC
bool tryConnectToPc(const char* pc_ip, uint16_t pc_port) {
  if (strlen(pc_ip) == 0) return false;
  debugPrint(String("Intentando conectar a PC ") + pc_ip + ":" + String(pc_port) + " ...");
  if (pcClient && pcClient.connected()) pcClient.stop();
  if (pcClient.connect(pc_ip, pc_port)) {
    debugPrint("Conectado al servidor PC como CLIENT.");
    runningAsClient = true;
    return true;
  }
  debugPrint("No se pudo conectar como client.");
  runningAsClient = false;
  return false;
}

// Forward de una línea (sin newline) desde PC hacia MEGA (Serial2)
// Modificación: normaliza la línea (quita CR/LF finales) y siempre envía EXACTAMENTE '\n' al final.
void forwardPcLineToMega(const String &line) {
  if (line.length() == 0) return;

  // make a copy we can modify
  String s = line;

  // quitar cualquier '\r' o '\n' al final (por si acaso)
  while (s.length() > 0 && (s.charAt(s.length()-1) == '\n' || s.charAt(s.length()-1) == '\r')) {
    s = s.substring(0, s.length() - 1);
  }

  // Enviar siempre con un '\n' (solo LF)
  Serial2.print(s);
  Serial2.print("\n");
  Serial2.flush(); // opcional: esperar que el buffer de salida se vacíe

  // opcional log local (por USB)
  Serial.printf("→ MEGA << %s\n", s.c_str());
}

// Forward de una línea desde MEGA hacia PC (envía JSON {"from_mega": "<line>"}\n)
void forwardMegaLineToPc(const String &line) {
  if (line.length() == 0) return;
  DynamicJsonDocument doc(512);
  doc["from_mega"] = line;
  // enviar al cliente conectado (preferir pcClient si estamos como cliente)
  if (runningAsClient && pcClient && pcClient.connected()) {
    sendJsonToClient(pcClient, doc);
  } else if (runningAsServer && localClient && localClient.connected()) {
    sendJsonToClient(localClient, doc);
  } else {
    // no hay cliente: opcionalmente imprimir por Serial (debug)
    Serial.printf("Mega -> (no client): %s\n", line.c_str());
  }
  Serial.printf("MEGA -> PC JSON sent: %s\n", line.c_str());
}

void setup() {
  Serial.begin(115200);
  // inicializar UART con Mega
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("\n=== ESP32 bridge PC <-> MEGA starting ===");

  bool sta_ok = false;
  if (strlen(STA_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    unsigned long start = millis();
    Serial.printf("Conectando a WiFi '%s' ...\n", STA_SSID);
    while (millis() - start < 10000) {
      if (WiFi.status() == WL_CONNECTED) { sta_ok = true; break; }
      delay(200);
      Serial.print(".");
    }
    Serial.println();
  }

  if (sta_ok) {
    Serial.print("WiFi STA OK, IP: "); Serial.println(WiFi.localIP());
    // intentar conectar como cliente TCP al PC
    if (tryConnectToPc(PC_SERVER_IP, PC_SERVER_PORT)) {
      runningAsClient = true;
    } else {
      runningAsClient = false;
    }
  } else {
    Serial.println("No STA: pasar a AP fallback.");
    WiFi.mode(WIFI_AP);
    if (strlen(AP_PASS) >= 8) WiFi.softAP(AP_SSID, AP_PASS);
    else WiFi.softAP(AP_SSID);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
    runningAsClient = false;
  }

  // si no pudimos conectar como cliente, arrancamos servidor local para que GUI pueda conectarse
  if (!runningAsClient) {
    localServer.begin();
    localServer.setNoDelay(true);
    runningAsServer = true;
    Serial.printf("Servidor TCP LOCAL escuchando en puerto %u (fallback)\n", LOCAL_SERVER_PORT);
  } else {
    runningAsServer = false;
  }

  lastStatusMs = millis();
}

// Helper: procesa una línea recibida desde TCP (PC). Por defecto la reenviamos a MEGA.
// Si querés interceptar comandos para el ESP, podés detectarlos aquí (JSON con "esp_cmd") y manejarlos localmente.
void handleLineFromPcAndForwardToMega(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Intentar parsear como JSON para comandos ESP locales (opcional)
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, s);
  if (!err && doc.containsKey("esp_cmd")) {
    // Ejemplo: {"esp_cmd":"led_on"}
    String cmd = doc["esp_cmd"].as<String>();
    Serial.printf("Comando ESP local: %s\n", cmd.c_str());
    if (cmd == "led_on") digitalWrite(LED_PIN, HIGH);
    else if (cmd == "led_off") digitalWrite(LED_PIN, LOW);
    // no lo reenviamos al Mega si es comando local
    return;
  }

  // otherwise: forward raw to mega
  forwardPcLineToMega(s);
}

// Lee Serial2 (Mega). Construye líneas y las reenvía al PC
void handleSerial2FromMega() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      String line = megaRecvBuffer;
      megaRecvBuffer = "";
      line.trim();
      if (line.length() > 0) {
        forwardMegaLineToPc(line);
      }
    } else if (c != '\r') {
      megaRecvBuffer += c;
      if (megaRecvBuffer.length() > 8192) megaRecvBuffer = megaRecvBuffer.substring(megaRecvBuffer.length()-4096);
    }
  }
}

void loop() {
  // 1) si estamos como cliente TCP hacia la PC
  if (runningAsClient) {
    if (!pcClient || !pcClient.connected()) {
      runningAsClient = false;
      pcClient.stop();
      Serial.println("Cliente a PC desconectado. Intento reconectar en 2s...");
      delay(2000);
      if (tryConnectToPc(PC_SERVER_IP, PC_SERVER_PORT)) {
        runningAsClient = true;
      } else {
        runningAsClient = false;
        localServer.begin();
        runningAsServer = true;
      }
      return;
    }

    // leer del servidor PC: líneas newline-delimited → forward a Mega
    while (pcClient.available()) {
      char c = (char)pcClient.read();
      if (c == '\n') {
        String line = recvBuffer;
        recvBuffer = "";
        handleLineFromPcAndForwardToMega(line);
      } else if (c != '\r') {
        recvBuffer += c;
        if (recvBuffer.length() > 8192) recvBuffer = recvBuffer.substring(recvBuffer.length()-4096);
      }
    }

    // leer del Mega y forward a PC
    handleSerial2FromMega();

    // status periodic
    if (millis() - lastStatusMs > STATUS_INTERVAL) {
      lastStatusMs = millis();
      DynamicJsonDocument doc(256);
      doc["device"] = "esp32";
      doc["type"] = "status";
      doc["uptime_ms"] = (long)millis();
      doc["ip"] = WiFi.localIP().toString();
      sendJsonToClient(pcClient, doc);
    }
    delay(2);
    return;
  }

  // 2) modo servidor fallback (acepta conexión de la GUI)
  if (runningAsServer) {
    // aceptar cliente si hay
    if (!localClient || !localClient.connected()) {
      if (localServer.hasClient()) {
        if (localClient && localClient.connected()) localClient.stop();
        localClient = localServer.available();
        Serial.print("Cliente GUI conectado desde: "); Serial.println(localClient.remoteIP());
      } else {
        // no hay cliente: intentar reconectar como cliente a PC si tenemos STA
        if (WiFi.status() == WL_CONNECTED && strlen(PC_SERVER_IP) > 0) {
          if (tryConnectToPc(PC_SERVER_IP, PC_SERVER_PORT)) {
            runningAsClient = true; runningAsServer = false;
            if (localClient && localClient.connected()) localClient.stop();
            localServer.stop();
            return;
          }
        }
        delay(50);
        return;
      }
    }

    // manejar lectura desde GUI (localClient)
    while (localClient && localClient.available()) {
      char c = (char)localClient.read();
      if (c == '\n') {
        String line = recvBuffer;
        recvBuffer = "";
        handleLineFromPcAndForwardToMega(line);
      } else if (c != '\r') {
        recvBuffer += c;
        if (recvBuffer.length() > 8192) recvBuffer = recvBuffer.substring(recvBuffer.length()-4096);
      }
    }

    // leer del Mega y reenvía a GUI si existe
    handleSerial2FromMega();

    // status periodic
    if (millis() - lastStatusMs > STATUS_INTERVAL) {
      lastStatusMs = millis();
      DynamicJsonDocument doc(256);
      doc["device"] = "esp32";
      doc["type"] = "status";
      doc["uptime_ms"] = (long)millis();
      doc["ip"] = WiFi.localIP().toString();
      if (localClient && localClient.connected()) sendJsonToClient(localClient, doc);
    }
    delay(2);
    return;
  }

  delay(50);
}