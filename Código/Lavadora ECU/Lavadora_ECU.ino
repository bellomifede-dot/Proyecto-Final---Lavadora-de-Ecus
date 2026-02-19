#include <Servo.h> // Librería para controlar servos con Arduino

// ================== Pines RAMPS 1.4 ==================
// Motor A (Lavadora)
#define ENA 66
#define IN1 63
#define IN2 65

// Motor B (Inyector)
#define ENB 64
#define IN3 44
#define IN4 59

// Motores NEMA eje X e Y
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

// Endstops
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15

// Servos
#define SERVO1_PIN 11
#define SERVO2_PIN 6
#define SERVO3_PIN 5
#define SERVO4_PIN 4

// ================== Variables ==================
const float LEADSCREW_PITCH = 8.0;
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPPING = 1;

bool activarInyectorFlag = false;
int stepDelay = 800;
bool homingDone = false;

Servo gripper1;
Servo gripper2;
Servo gripper3;
Servo gripper4;

// =====================================================
// ===============  COMUNICACIÓN CON ROBOT  ============
// =====================================================
String rxRobot = "";

// Envía un mensaje al robot
void enviarRobot(String msg) {
    Serial2.println(msg);
    Serial.print(">> Enviado al ROBOT: ");
    Serial.println(msg);
}

// Procesa comandos entrantes
void procesarMensajeRobot(String msg) {
    msg.trim();
    Serial.print("<< Robot dice: ");
    Serial.println(msg);

    // ===== ECU PLACED =====
    if (msg == "ECU_PLACED") {
        Serial.println(" ECU colocada → cerrando grippers...");
        cerrarGrippers();
        delay(500);
        enviarRobot("BOX_CLOSED");
    }

    // ===== WAIT_OK =====
    else if (msg == "WAIT_OK") {
        Serial.println(" Robot esperando confirmación...");
        rutinaCompleta();
        enviarRobot("CLEAN_DONE");
    }

    // ===== GET_ECU =====
    else if (msg == "GET_ECU") {
        Serial.println(" Robot viene a retirar la ECU...");
        abrirGrippers();
        delay(300);
        
    }
}
// Lee mensajes desde Serial2
void leerRobot() {
    while (Serial2.available()) {
        char c = Serial2.read();

        if (c == '\n') {
            procesarMensajeRobot(rxRobot);
            rxRobot = "";
        } else {
            rxRobot += c;
        }
    }
}
// =====================================================
// ================ FIN COMUNICACIÓN ====================
// =====================================================


void setup() {

    // --- Motores DC ---
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // --- Motores NEMA ---
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(X_ENABLE_PIN, OUTPUT);

    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);

    // Endstops
    pinMode(Y_MIN_PIN, INPUT_PULLUP);
    pinMode(Y_MAX_PIN, INPUT_PULLUP);

    digitalWrite(X_ENABLE_PIN, LOW);
    digitalWrite(Y_ENABLE_PIN, LOW);

    // --- Servos ---
    gripper1.attach(SERVO1_PIN);
    gripper2.attach(SERVO2_PIN);
    gripper3.attach(SERVO3_PIN);
    gripper4.attach(SERVO4_PIN);

    Serial.begin(115200);
    Serial2.begin(115200);  // <<< NUEVO: COMUNICACIÓN CON ROBOT

    Serial.println("Iniciando sistema...");

    homingServos();
    hacerHoming();
    delay(500);

    moverAmbosMotores_mm(60, HIGH);
    delay(500);
    
    
}

void loop() {
    leerRobot();   // <<< NUEVO: escucha al robot TODO EL TIEMPO
    leerSimulacion();  // <<< NUEVO: comandos desde PC por USB
    //if (activarInyectorFlag) {
        //Inyector();    // ahora se ejecuta en el loop PRINCIPAL
    //}

}



// ================== FUNCIONES DE CONTROL ==================
void Inyector(){
    moverMotorB(150, true);
    activarInyector();
    delayMicroseconds(10000);
    desactivarInyector();
    delay(50);
}

// Motor A
void moverMotorA(int velocidad, bool adelante) {
    analogWrite(ENA, velocidad);
    digitalWrite(IN1, adelante ? HIGH : LOW);
    digitalWrite(IN2, adelante ? LOW : HIGH);
}

// Motor B
void moverMotorB(int velocidad, bool adelante) {
    analogWrite(ENB, velocidad);
    digitalWrite(IN3, adelante ? HIGH : LOW);
    digitalWrite(IN4, adelante ? LOW : HIGH);
}

// Inyector
void activarInyector() {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void desactivarInyector() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void pararMotores() {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

// Grippers homing
void homingServos() {
    Serial.println("Homing de grippers...");
    gripper1.write(0);
    gripper3.write(0);
    delay(1000);
    gripper2.write(0);
    gripper4.write(170);
    delay(1000);
}

// Homing eje Y
void hacerHoming() {
    digitalWrite(X_DIR_PIN, LOW);
    digitalWrite(Y_DIR_PIN, LOW);
    Serial.println("Haciendo homing...");

    while (digitalRead(Y_MIN_PIN) == HIGH) {
        darPaso();
    }

    Serial.println("Homing completado");
}

// Conversión mm → pasos
long mmToSteps(float distancia_mm) {
    float vueltas = distancia_mm / LEADSCREW_PITCH;
    long pasos = vueltas * MOTOR_STEPS_PER_REV * MICROSTEPPING;
    return pasos;
}

// Movimiento NEMA
void moverAmbosMotores_mm(float distancia_mm, bool direccion) {
    long pasos = mmToSteps(distancia_mm);
    moverAmbosMotores(pasos, direccion);
}

void moverAmbosMotores(long pasos, bool direccion) {
    digitalWrite(X_DIR_PIN, direccion);
    digitalWrite(Y_DIR_PIN, direccion);

    for (long i = 0; i < pasos; i++) {

        if (digitalRead(Y_MAX_PIN) == LOW) {
            Serial.println("Y_MAX detectado, deteniendo motores");
            break;
        }

        darPaso();
    }
}

void darPaso() {
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);

    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
}

// Grippers
void cerrarGrippers() {
    Serial.println("Cerrando grippers...");
    gripper1.write(155);
    gripper3.write(155);
}

void abrirGrippers() {
    Serial.println("Abriendo grippers...");
    gripper1.write(0);
    gripper3.write(0);
}

// ======================================================
// ================   RUTINA COMPLETA   =================
// ======================================================

// Tiempo (ms) que dura la inyección
unsigned long tiempoInyeccion = 5000;   // <-- ajustalo

void rutinaCompleta() {

    Serial.println("=== INICIO RUTINA COMPLETA ===");

    //activarInyectorFlag = true;  // <<< ACTIVA

    // ---------- 1) Inyectores primera fase ----------
    Serial.println("Activando inyectores (fase 1)...");
    unsigned long inicio = millis();
    while (millis() - inicio < tiempoInyeccion) {
        Inyector(); 
    }
    analogWrite(ENB, 0);


    //activarInyectorFlag = false;   // <<< DESACTIVA
    moverAmbosMotores_mm(50, HIGH);

    // ---------- 2) Rotar servos 2 y 4 en conjunto ----------
    Serial.println("Girando servos 2 y 4...");
    girarServos24(170);   // Ángulo de giro → AJUSTAR SI QUERÉS
    delay(500);

    // ---------- 3) Segunda fase de inyectores ----------
    //activarInyectorFlag = true;  // <<< ACTIVA
    Serial.println("Activando inyectores (fase 2)...");
    inicio = millis();
    while (millis() - inicio < tiempoInyeccion) {
        Inyector(); 
    }
    analogWrite(ENB, 0);

    girarServos24(0);   // Ángulo de giro → AJUSTAR SI QUERÉS
    delay(500);

    // ---------- 4) Bajar hasta endstop MIN ----------
    Serial.println("Bajando hasta Y_MIN...");
    subirHastaMax();

    // ---------- 5) Activar motor A ----------
    Serial.println("Motor A funcionando...");
    moverMotorA(150, true);   // velocidad 150 adelante
    delay(4000);              // tiempo lavado → ajustar
    pararMotores();

    // ---------- 6) Subir hasta endstop MAX ----------
    Serial.println("Subiendo hasta Y_MAX...");
    bajarHastaMin();

    // ---------- 7) Posición segura ----------
    moverAmbosMotores_mm(60, HIGH);

    Serial.println("=== RUTINA COMPLETA FINALIZADA ===");
}

void girarServos24(int anguloFinal) {

    int s2 = gripper2.read();
    int s4 = gripper4.read();

    int paso2 = (anguloFinal > s2) ? 1 : -1;   // servo 2 normal
    int paso4 = -paso2;                       // servo 4 invertido

    int target4 = 170 - anguloFinal;          // siempre contrario

    while (s2 != anguloFinal || s4 != target4) {

        if (s2 != anguloFinal) {
            s2 += paso2;
            gripper2.write(s2);
        }

        if (s4 != target4) {
            s4 += paso4;
            gripper4.write(s4);
        }

        delay(10);
    }
}


void bajarHastaMin() {
    digitalWrite(Y_DIR_PIN, LOW);
    digitalWrite(X_DIR_PIN, LOW);

    while (digitalRead(Y_MIN_PIN) == HIGH) {
        darPaso();
    }
}

void subirHastaMax() {
    digitalWrite(Y_DIR_PIN, HIGH);
    digitalWrite(X_DIR_PIN, HIGH);

    while (digitalRead(Y_MAX_PIN) == HIGH) {
        darPaso();
    }
}




String rxSim = "";

void leerSimulacion() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            procesarMensajeRobot(rxSim);  // <<< usamos la MISMA lógica
            rxSim = "";
        } else {
            rxSim += c;
        }
    }
}
