#include <AccelStepper.h>
#include <MultiStepper.h>

// ============================================================
// RAMPS 1.4 - Pin assi
// ============================================================
#define X_STEP_PIN    54
#define X_DIR_PIN     55
#define X_ENABLE_PIN  38

#define Y_STEP_PIN    60
#define Y_DIR_PIN     61
#define Y_ENABLE_PIN  56

#define Z_STEP_PIN    46
#define Z_DIR_PIN     48
#define Z_ENABLE_PIN  62

#define E0_STEP_PIN    26
#define E0_DIR_PIN     28
#define E0_ENABLE_PIN  24

#define E1_STEP_PIN    36
#define E1_DIR_PIN     34
#define E1_ENABLE_PIN  30

// ============================================================
// COSTANTI EBA17 - Rapporto riduzione 38.4:1 - Microstep 1/8
// ============================================================
#define MICROSTEP        8
#define STEP_PER_REV     200
#define STEP_PER_REV_MS  (STEP_PER_REV * MICROSTEP)
#define GEAR_RATIO       38.4
#define STEPS_OUTPUT_REV (long)(STEP_PER_REV_MS * GEAR_RATIO)  // 61440 step = 360°

#define TARGET_RPM_OUTPUT  4   // ridotto per test con braccio montato
#define STEPS_PER_SEC      (STEPS_OUTPUT_REV * TARGET_RPM_OUTPUT / 60.0)
#define ACCEL              (STEPS_PER_SEC * 0.2)

// Conversione gradi → step uscita
#define DEG_TO_STEPS(deg) ((long)((deg) * STEPS_OUTPUT_REV / 360.0))

// ============================================================
// ISTANZE
// ============================================================
AccelStepper stepperX (AccelStepper::DRIVER, X_STEP_PIN,  X_DIR_PIN);
AccelStepper stepperY (AccelStepper::DRIVER, Y_STEP_PIN,  Y_DIR_PIN);
AccelStepper stepperZ (AccelStepper::DRIVER, Z_STEP_PIN,  Z_DIR_PIN);
AccelStepper stepperE0(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN);
AccelStepper stepperE1(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);

// ============================================================
// STRUTTURA per mappare nome asse → stepper + enable pin
// ============================================================
struct Axis {
  AccelStepper* stepper;
  int enablePin;
  const char* name;
};

Axis axes[] = {
  { &stepperX,  X_ENABLE_PIN,  "X"  },
  { &stepperY,  Y_ENABLE_PIN,  "Y"  },
  { &stepperZ,  Z_ENABLE_PIN,  "Z"  },
  { &stepperE0, E0_ENABLE_PIN, "E0" },
  { &stepperE1, E1_ENABLE_PIN, "E1" },
};

// ============================================================
// HELPER - muove un asse di N gradi (con segno)
// ============================================================
void moveByDegrees(Axis &ax, float degrees) {
  long steps = DEG_TO_STEPS(degrees);
  Serial.print(ax.name);
  Serial.print(" → ");
  Serial.print(degrees);
  Serial.print("° (");
  Serial.print(steps);
  Serial.println(" step)");

  digitalWrite(ax.enablePin, LOW);
  ax.stepper->moveTo(ax.stepper->currentPosition() + steps);
  while (ax.stepper->distanceToGo() != 0) ax.stepper->run();
  digitalWrite(ax.enablePin, HIGH);

  Serial.print(ax.name);
  Serial.print(" fine. Posizione accumulata: ");
  Serial.print(ax.stepper->currentPosition() * 360.0 / STEPS_OUTPUT_REV);
  Serial.println("°");
}

// ============================================================
// PARSER comando Serial: formato "X+45" / "Y-90" / "E0+30"
// ============================================================
void parseAndExecute(String cmd) {
  cmd.trim();
  if (cmd.length() < 3) { Serial.println("Comando non valido."); return; }

  // Trova il + o - che separa asse da angolo
  int signPos = -1;
  for (int i = 1; i < (int)cmd.length(); i++) {
    if (cmd[i] == '+' || cmd[i] == '-') { signPos = i; break; }
  }
  if (signPos == -1) { Serial.println("Manca segno + o -"); return; }

  String axisName = cmd.substring(0, signPos);
  axisName.toUpperCase();
  float degrees = cmd.substring(signPos).toFloat();

  // Cerca l'asse corrispondente
  for (int i = 0; i < 5; i++) {
    if (axisName == axes[i].name) {
      moveByDegrees(axes[i], degrees);
      return;
    }
  }
  Serial.print("Asse non trovato: "); Serial.println(axisName);
}

// ============================================================
// HELPER - stampa posizione attuale di tutti gli assi
// ============================================================
void printPositions() {
  Serial.println("----- Posizioni attuali -----");
  for (int i = 0; i < 5; i++) {
    Serial.print(axes[i].name);
    Serial.print(": ");
    Serial.print(axes[i].stepper->currentPosition() * 360.0 / STEPS_OUTPUT_REV);
    Serial.println("°");
  }
  Serial.println("-----------------------------");
}

// ============================================================
// HELPER - azzera posizione di tutti gli assi (solo software)
// ============================================================
void resetZero() {
  for (int i = 0; i < 5; i++) axes[i].stepper->setCurrentPosition(0);
  Serial.println("Zero resettato su tutti gli assi.");
}

void setup() {
  Serial.begin(9600);

  pinMode(X_ENABLE_PIN,  OUTPUT); digitalWrite(X_ENABLE_PIN,  HIGH);
  pinMode(Y_ENABLE_PIN,  OUTPUT); digitalWrite(Y_ENABLE_PIN,  HIGH);
  pinMode(Z_ENABLE_PIN,  OUTPUT); digitalWrite(Z_ENABLE_PIN,  HIGH);
  pinMode(E0_ENABLE_PIN, OUTPUT); digitalWrite(E0_ENABLE_PIN, HIGH);
  pinMode(E1_ENABLE_PIN, OUTPUT); digitalWrite(E1_ENABLE_PIN, HIGH);

  stepperX.setMaxSpeed(STEPS_PER_SEC);  stepperX.setAcceleration(ACCEL);
  stepperY.setMaxSpeed(STEPS_PER_SEC);  stepperY.setAcceleration(ACCEL);
  stepperZ.setMaxSpeed(STEPS_PER_SEC);  stepperZ.setAcceleration(ACCEL);
  stepperE0.setMaxSpeed(STEPS_PER_SEC); stepperE0.setAcceleration(ACCEL);
  stepperE1.setMaxSpeed(STEPS_PER_SEC); stepperE1.setAcceleration(ACCEL);

  Serial.println("===== Controllo Angolare EBA17 =====");
  Serial.println("Formato comando: ASSE+gradi o ASSE-gradi");
  Serial.println("Esempi:");
  Serial.println("  X+45    → ruota X di +45°");
  Serial.println("  Y-90    → ruota Y di -90°");
  Serial.println("  E0+180  → ruota E0 di +180°");
  Serial.println("Comandi speciali:");
  Serial.println("  p → stampa posizioni attuali");
  Serial.println("  r → reset zero tutti gli assi");
  Serial.println("====================================");
}

String inputBuffer = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {

        // Comandi speciali a singolo carattere
        if (inputBuffer == "p" || inputBuffer == "P") {
          printPositions();
        } else if (inputBuffer == "r" || inputBuffer == "R") {
          resetZero();
        } else {
          parseAndExecute(inputBuffer);
        }
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}
