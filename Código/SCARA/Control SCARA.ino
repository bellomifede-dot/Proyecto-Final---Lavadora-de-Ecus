#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h> 

// ============================
// 📌 Pines RAMPS y endstops
// ============================
#define X_STEP_PIN 54
#define X_DIR_PIN  55
#define X_EN_PIN   38
#define X_MIN_PIN   3

#define Y_STEP_PIN 60
#define Y_DIR_PIN  61
#define Y_EN_PIN   56
#define Y_MIN_PIN  14

#define Z_STEP_PIN 46
#define Z_DIR_PIN  48
#define Z_EN_PIN   62
#define Z_MIN_PIN   18

#define E0_STEP_PIN 26
#define E0_DIR_PIN  28
#define E0_EN_PIN   24
#define E0_MIN_PIN  19

#define GRIPPER_PIN 11   // pin digital de salida para el gripper

// ============================
// 📌 Límites de software (pasos) 
// ============================
#define J1_MIN -1204
#define J1_MAX  1204

#define J2_MIN -1354
#define J2_MAX  1354

#define Z_MIN  0
#define Z_MAX  4900
#define J6_MIN -10000
#define J6_MAX 10000

// ============================
// 📌 Home positions (valor que queremos que tenga cada eje cuando toca el endstop)
// ============================
#define J1_HOME_POS -1204
#define J2_HOME_POS  1354
#define Z_HOME_POS   0
#define J6_HOME_POS  0

// ============================
// 📌 Objetos motores
// ============================
AccelStepper stepperJ1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperJ2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ (AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperJ6(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN);
MultiStepper steppers; // lo dejamos por compatibilidad si lo querés usar después

Servo gripperServo;

// ============================
// 📌 Variables globales
// ============================
volatile bool stopRequested = false;
bool motorsEnabled = true;
bool gripperOpen = true;

bool movementActive = false; // <-- bandera: hay movimiento en curso?

#define SERVO_CLOSE_ANGLE -10  
#define SERVO_OPEN_ANGLE  180 

// Guardamos el estado "inverted" inicial (para poder restaurarlo)
bool j1_inverted = true;   // TU configuración original tenía J1 invertido
bool j2_inverted = false;  // J2 no estaba invertido originalmente (según tu código)

/* ---------------------------
   Función para energizar/desenergizar motores
   --------------------------- */
void enableMotors(bool state) {
  motorsEnabled = state;
  digitalWrite(X_EN_PIN, state ? LOW : HIGH);
  digitalWrite(Y_EN_PIN, state ? LOW : HIGH);
  digitalWrite(Z_EN_PIN, state ? LOW : HIGH);
  digitalWrite(E0_EN_PIN, state ? LOW : HIGH);

  if (state)
    Serial.println(" Motores ENERGIZADOS (enable on)");
  else
    Serial.println(" Motores DESENERGIZADOS (enable off)");
}

// ============================
// 📌 Setup
// ============================
void setup() {
  Serial.begin(115200);

  // Aplicamos la inversión inicial conocida (no la cambiamos aquí salvo que quieras)
  stepperJ1.setPinsInverted(j1_inverted, false, false);
  stepperJ2.setPinsInverted(j2_inverted, false, false);
  // otros ejes por defecto:
  stepperZ.setPinsInverted(false, false, false);
  stepperJ6.setPinsInverted(false, false, false);
  
  pinMode(X_EN_PIN, OUTPUT);
  pinMode(Y_EN_PIN, OUTPUT);
  pinMode(Z_EN_PIN, OUTPUT);
  pinMode(E0_EN_PIN, OUTPUT);

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(E0_MIN_PIN, INPUT_PULLUP);
  
  gripperServo.attach(GRIPPER_PIN); 
  gripperServo.write(SERVO_OPEN_ANGLE);
  gripperOpen = true;

  enableMotors(true); // activar motores al iniciar

  stepperJ1.setMaxSpeed(1000); stepperJ1.setAcceleration(200);
  stepperJ2.setMaxSpeed(1000); stepperJ2.setAcceleration(200);
  stepperZ.setMaxSpeed(800);   stepperZ.setAcceleration(150);
  stepperJ6.setMaxSpeed(1200); stepperJ6.setAcceleration(300);

  steppers.addStepper(stepperJ1);
  steppers.addStepper(stepperJ2);
  steppers.addStepper(stepperZ);
  steppers.addStepper(stepperJ6);

  Serial.println("Listo! Comandos:");
  Serial.println("home -> homing de todos los ejes (J6 no se mueve, se pone a 0)");
  Serial.println("moveJ J1 pasos vmax amax ...");
  Serial.println("moveL J1 target J2 target ...");
  Serial.println("enable on/off -> energiza o desenergiza motores");
  Serial.println("s o Enter vacío -> STOP global");
}

// ============================
// 📌 STOP global
// ============================
void stopAll() {
  stopRequested = true;

  stepperJ1.stop(); stepperJ2.stop(); stepperZ.stop(); stepperJ6.stop();

  stepperJ1.setCurrentPosition(stepperJ1.currentPosition());
  stepperJ2.setCurrentPosition(stepperJ2.currentPosition());
  stepperZ.setCurrentPosition(stepperZ.currentPosition());
  stepperJ6.setCurrentPosition(stepperJ6.currentPosition());

  long pos[4] = {stepperJ1.currentPosition(), stepperJ2.currentPosition(),
                 stepperZ.currentPosition(), stepperJ6.currentPosition()};
  steppers.moveTo(pos);

  movementActive = false; // evitar envíos al quedar en STOP
  Serial.println(" STOP GLOBAL ACTIVADO!");
}

// ============================
// 📌 Homing interrumpible (ahora: inversion temporal por eje + setCurrentPosition correcto)
// - motorId: 1=>J1, 2=>J2, 3=>Z, 6=>J6 (solo para distinguir inversión por defecto)
// - homingPosition: valor de steps que queremos asignar cuando toca el endstop
// - tempInvert: si true, invertimos temporalmente la inversión de pines para que vaya en el sentido deseado
// ============================
bool homeAxis(AccelStepper &motor, int endstopPin, int homingSpeed, const char* ejeName, int motorId, long homingPosition, bool tempInvert) {
  Serial.print(" Homing "); Serial.print(ejeName);
  Serial.print(" con velocidad "); Serial.println(homingSpeed);

  // Determinar inversión original conocida según el eje (para poder restaurarla)
  bool originalInvert = false;
  if (motorId == 1) originalInvert = j1_inverted;
  else if (motorId == 2) originalInvert = j2_inverted;
  else originalInvert = false;

  // Si pedimos inversión temporal, la aplicamos (flip)
  if (tempInvert) {
    motor.setPinsInverted(!originalInvert, false, false);
    Serial.print("  Inversión temporal aplicada a ");
    Serial.println(ejeName);
  }

  motor.setMaxSpeed(abs(homingSpeed));
  motor.setSpeed(homingSpeed);

  // Bucle de avance hasta el endstop (bloqueante mientras hace homing del eje)
  while (digitalRead(endstopPin) == HIGH) {
    // Permitir recibir "s" durante homing
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "s") {
        stopRequested = true;
        Serial.println(" STOP recibido durante homing!");
        // Restaurar inversión antes de salir
        if (tempInvert) motor.setPinsInverted(originalInvert, false, false);
        return false;
      }
    }

    if (stopRequested) {
      if (tempInvert) motor.setPinsInverted(originalInvert, false, false);
      return false;
    }
    motor.runSpeed();
  }

  delay(200); // antirrebote

  // Seteamos SOLO la posición del motor que hizo homing al valor deseado
  motor.setCurrentPosition(homingPosition);
  Serial.print("  Posicion setada de ");
  Serial.print(ejeName);
  Serial.print(" a ");
  Serial.println(homingPosition);

  // Restaurar inversión original si hicimos cambio temporal
  if (tempInvert) {
    motor.setPinsInverted(originalInvert, false, false);
    Serial.print("  Inversión restaurada en ");
    Serial.println(ejeName);
  }

  Serial.print(" Homing completado: ");
  Serial.println(ejeName);
  return true;
}

// ============================
// 📌 Verificar límites  (FIX: comparar con MIN y MAX correctamente)
// ============================
bool checkLimits(String eje, long target) {
  if (eje == "J1" && (target < J1_MIN || target > J1_MAX)) return false;
  if (eje == "J2" && (target < J2_MIN || target > J2_MAX)) return false;
  if (eje == "Z"  && (target < Z_MIN  || target > Z_MAX))  return false;
  if (eje == "J6" && (target < J6_MIN || target > J6_MAX)) return false;
  return true;
}

// ============================
// 📌 Procesar comandos
// ============================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) { stopAll(); return; }

  if (cmd == "s") { stopAll(); return; }

  if (cmd.startsWith("enable")) {
    if (cmd.endsWith("on")) enableMotors(true);
    else if (cmd.endsWith("off")) enableMotors(false);
    else Serial.println("Uso: enable on/off");
    return;
  }

  // --- Homing completo ---
  if (cmd == "home") {
    stopRequested = false;
    Serial.println(" Iniciando homing eje por eje");

    // IMPORTANTE: aquí indicamos tempInvert = true para J1 y J2
    // (esto invierte temporalmente la dirección de pines solo durante homing)
    // homingSpeed lo ponemos negativo si el endstop MIN se activa y el motor debe moverse hacia negativo.
    if (!homeAxis(stepperJ1, X_MIN_PIN, -200, "J1", 1, J1_HOME_POS, false)) return;
    if (!homeAxis(stepperJ2, Y_MIN_PIN, -200, "J2", 2, J2_HOME_POS, true)) return;
    if (!homeAxis(stepperZ,  Z_MIN_PIN, -200, "Z", 3, Z_HOME_POS, false)) return;

    // J6 NO se mueve durante homing — lo ponemos a su posición conocida:
    stepperJ6.setCurrentPosition(J6_HOME_POS);
    Serial.println(" Homing: J6 pos set to 0 (sin movimiento)");

    Serial.println("🏁 Homing completado!");
    // enviar posición AHORA que terminó el homing completo
    enviarPosicion();
    return;
  }

  // --- MoveJ ---
  if (cmd.startsWith("moveJ")) {
    stopRequested = false;
    String tokens[20];
    int tokenCount = 0;

    while (cmd.length() > 0 && tokenCount < 20) {
      int idx = cmd.indexOf(' ');
      if (idx == -1) { tokens[tokenCount++] = cmd; break; }
      tokens[tokenCount++] = cmd.substring(0, idx);
      cmd = cmd.substring(idx + 1);
      cmd.trim();
    }

    for (int i = 1; i < tokenCount; i += 4) {
      String eje = tokens[i];
      long steps = tokens[i+1].toInt();
      long vmax  = tokens[i+2].toInt();
      long amax  = tokens[i+3].toInt();

      AccelStepper *motor = nullptr;
      if (eje == "J1") motor = &stepperJ1;
      else if (eje == "J2") motor = &stepperJ2;
      else if (eje == "Z")  motor = &stepperZ;
      else if (eje == "J6") motor = &stepperJ6;

      if (motor != nullptr) {
        long target = motor->currentPosition() + steps;
        if (!checkLimits(eje, target)) {
          Serial.print("ERROR: Movimiento fuera de rango en ");
          Serial.println(eje);
          continue;
        }
        motor->setMaxSpeed(vmax);
        motor->setAcceleration(amax);
        motor->move(steps);           // non-blocking
        movementActive = true;        // marcamos que hay movimiento en curso
        Serial.print("MoveJ "); Serial.print(eje);
        Serial.print(" a "); Serial.println(target);
      }
    }
    return;
  }

  // --- MoveL (no bloqueante) ---
  if (cmd.startsWith("moveL")) {
    long positions[4] = {stepperJ1.currentPosition(), stepperJ2.currentPosition(),
                         stepperZ.currentPosition(), stepperJ6.currentPosition()};

    String tokens[20];
    int tokenCount = 0;
    while (cmd.length() > 0 && tokenCount < 20) {
      int idx = cmd.indexOf(' ');
      if (idx == -1) { tokens[tokenCount++] = cmd; break; }
      tokens[tokenCount++] = cmd.substring(0, idx);
      cmd = cmd.substring(idx + 1);
      cmd.trim();
    }

    for (int i = 1; i < tokenCount; i += 2) {
      String eje = tokens[i];
      long target = tokens[i+1].toInt();
      if (!checkLimits(eje, target)) {
        Serial.print("ERROR: Movimiento fuera de rango en ");
        Serial.println(eje);
        return;
      }
      if (eje == "J1") positions[0] = target;
      else if (eje == "J2") positions[1] = target;
      else if (eje == "Z")  positions[2] = target;
      else if (eje == "J6") positions[3] = target;
    }

    // Asignamos los targets a cada stepper (no bloqueante)
    stepperJ1.moveTo(positions[0]);
    stepperJ2.moveTo(positions[1]);
    stepperZ.moveTo(positions[2]);
    stepperJ6.moveTo(positions[3]);

    movementActive = true; // marca que hay movimiento en curso
    Serial.println("MoveL iniciado (no bloqueante)");
    return;
  }

  // --- Control del gripper ---
  if (cmd.startsWith("gripper")) {
    if (cmd.endsWith("open")) {
      gripperServo.write(SERVO_OPEN_ANGLE);
      gripperOpen = true;
      Serial.println(" Gripper ABIERTO");
    } else if (cmd.endsWith("close")) {
      gripperServo.write(SERVO_CLOSE_ANGLE);
      gripperOpen = false;
      Serial.println(" Gripper CERRADO");
    } else {
      Serial.println("Uso: gripper open / gripper close");
    }
    return;
  }

  Serial.println("Comando no reconocido.");
}

void enviarPosicion() {
  long j1_steps = stepperJ1.currentPosition();
  long j2_steps = stepperJ2.currentPosition();
  long z_steps = stepperZ.currentPosition();
  long j6_steps = stepperJ6.currentPosition();
  Serial.print("POS J1 ");
  Serial.print(j1_steps);  
  Serial.print(" J2 ");
  Serial.print(j2_steps);
  Serial.print(" Z ");
  Serial.print(z_steps);
  Serial.print(" J6 ");
  Serial.println(j6_steps);
}

// ============================
// 📌 Loop principal
// ============================
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  if (!stopRequested && motorsEnabled) {
    // Estos .run() son los que hacen avanzar los motores en background
    stepperJ1.run();
    stepperJ2.run();
    stepperZ.run();
    stepperJ6.run();
  }

  // Si hay movimiento en curso, detectamos cuando TODOS los ejes llegaron (distanceToGo()==0)
  if (movementActive) {
    if ( stepperJ1.distanceToGo() == 0 &&
         stepperJ2.distanceToGo() == 0 &&
         stepperZ.distanceToGo()  == 0 &&
         stepperJ6.distanceToGo() == 0 ) {

      movementActive = false;
      Serial.println("Movimiento finalizado");
      enviarPosicion(); // enviamos posición SOLO al finalizar
    }
  }
}