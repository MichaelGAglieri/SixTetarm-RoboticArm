#include <AccelStepper.h>

// ============================================================
// FIRMWARE BRACCIO ROBOTICO - 5 ASSI
// Hardware: Arduino Mega + RAMPS 1.4 + 5x A4988 + 5x NEMA 17
// Riduttore: EBA17S 38.4:1 su ogni giunto
// Comunicazione: Serial USB ← ROS 2 Bridge
// Formato comando ricevuto: "J1:500 J2:-200 J3:1500 J4:800 J5:300\n"
// ============================================================

// ============================================================
// SEZIONE 1: PINOUT RAMPS 1.4
// Ogni asse ha 3 pin: STEP, DIR, ENABLE
// ENABLE: LOW = driver attivo, HIGH = driver disattivo
// ============================================================
#define X_STEP_PIN   54  // J1 - Base
#define X_DIR_PIN    55
#define X_EN_PIN     38

#define Y_STEP_PIN   60  // J2 - Spalla
#define Y_DIR_PIN    61
#define Y_EN_PIN     56

#define Z_STEP_PIN   46  // J3 - Gomito
#define Z_DIR_PIN    48
#define Z_EN_PIN     62

#define E0_STEP_PIN  26  // J4 - Polso Pitch
#define E0_DIR_PIN   28
#define E0_EN_PIN    24

#define E1_STEP_PIN  36  // J5 - Polso Roll
#define E1_DIR_PIN   34
#define E1_EN_PIN    30

// ============================================================
// SEZIONE 2: COSTANTI CINEMATICHE
// Microstep: 1/8 (compromesso fluido/velocità)
// → rimuovi 1 jumper, lascia MS1+MS2 ON sotto ogni driver
// Passi per giro motore: 200 × 8 = 1600
// Passi per giro uscita: 1600 × 38.4 = 61440
// Passi per grado:       61440 / 360 = 170.67 step/°
// ============================================================
#define MICROSTEP         8
#define STEP_PER_REV      200
#define STEP_PER_REV_MS   (STEP_PER_REV * MICROSTEP)          // 1600
#define GEAR_RATIO        38.4
#define STEPS_OUTPUT_REV  (long)(STEP_PER_REV_MS * GEAR_RATIO) // 61440
#define STEPS_PER_DEGREE  (STEPS_OUTPUT_REV / 360.0)           // 170.67 step/°

// ============================================================
// SEZIONE 3: VELOCITÀ E ACCELERAZIONE
// MAX_SPEED: step/sec massimi per asse
// Con 1/8 microstep il limite pratico A4988 è ~8000 step/sec
// Corrispondente a ~7.8 RPM sull'uscita
// ACCELERATION: rampa morbida per proteggere ingranaggi PETG
// Aumenta MAX_SPEED con cautela dopo i test fisici
// ============================================================
#define MAX_SPEED     5000   // step/sec (~4.9 RPM uscita) — sicuro per PETG
#define ACCELERATION  1500   // step/sec² — rampa ~3 sec per raggiungere MAX_SPEED

// ============================================================
// SEZIONE 4: SOFT LIMITS
// Definisce i range angolari sicuri per ogni giunto in STEP
// Formula: gradi × STEPS_PER_DEGREE = step
// ⚠️ DA RIEMPIRE dopo i test fisici con gli EBA17S montati!
// Esempio: 90° × 170.67 = 15360 step
// ============================================================
struct JointLimits {
  long minSteps;  // limite minimo in step (negativo = senso antiorario)
  long maxSteps;  // limite massimo in step (positivo = senso orario)
};

JointLimits limits[5] = {
  // { minSteps,  maxSteps }  // Giunto — note
  {  -STEPS_OUTPUT_REV,  STEPS_OUTPUT_REV  },  // J1 Base:        ±360° (rotazione libera)
  {  -15360,              15360             },  // J2 Spalla:      ±90° — ⚠️ DA MISURARE
  {  -15360,              15360             },  // J3 Gomito:      ±90° — ⚠️ DA MISURARE
  {  -10240,              10240             },  // J4 Polso Pitch: ±60° — ⚠️ DA MISURARE
  {  -15360,              15360             },  // J5 Polso Roll:  ±90° — ⚠️ DA MISURARE
};

// ============================================================
// SEZIONE 5: ISTANZE ACCELSTEPPER
// AccelStepper::DRIVER = modalità step+dir
// Il driver A4988 gestisce internamente la sequenza delle bobine
// ============================================================
AccelStepper joint1(AccelStepper::DRIVER, X_STEP_PIN,  X_DIR_PIN);
AccelStepper joint2(AccelStepper::DRIVER, Y_STEP_PIN,  Y_DIR_PIN);
AccelStepper joint3(AccelStepper::DRIVER, Z_STEP_PIN,  Z_DIR_PIN);
AccelStepper joint4(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN);
AccelStepper joint5(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);

// Array di puntatori per gestire tutti i joint in loop
// Permette di iterare sui 5 motori senza ripetere codice
AccelStepper* joints[5] = { &joint1, &joint2, &joint3, &joint4, &joint5 };

// Pin ENABLE per ogni asse (stesso ordine dell'array joints)
const int enablePins[5] = { X_EN_PIN, Y_EN_PIN, Z_EN_PIN, E0_EN_PIN, E1_EN_PIN };

// ============================================================
// SEZIONE 6: VARIABILI DI STATO
// Tiene traccia se c'è movimento in corso su almeno un asse
// Usato per abilitare/disabilitare i driver automaticamente
// ============================================================
bool motorsEnabled = false;  // true = driver attivi, false = driver spenti

// ============================================================
// FUNZIONE: enableMotors()
// Abilita tutti e 5 i driver contemporaneamente (ENABLE = LOW)
// Da chiamare PRIMA di ogni movimento
// ============================================================
void enableMotors() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(enablePins[i], LOW);
  }
  motorsEnabled = true;
}

// ============================================================
// FUNZIONE: disableMotors()
// Disabilita tutti i driver (ENABLE = HIGH)
// Da chiamare quando tutti i motori sono fermi
// Riduce calore su driver e bobine durante le pause
// ============================================================
void disableMotors() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(enablePins[i], HIGH);
  }
  motorsEnabled = false;
}

// ============================================================
// FUNZIONE: clampToLimits()
// Applica i soft limits: se il target supera il range sicuro,
// lo forza al valore limite invece di bloccare o danneggiare
// Parametri: joint = indice (0-4), targetSteps = posizione richiesta
// Ritorna: posizione corretta entro i limiti
// ============================================================
long clampToLimits(int joint, long targetSteps) {
  return constrain(targetSteps, limits[joint].minSteps, limits[joint].maxSteps);
}

// ============================================================
// FUNZIONE: degreesToSteps()
// Converte gradi (da ROS 2, in float) in step interi
// ROS 2 manda angoli in radianti → il bridge li converte in gradi
// prima di inviarli via seriale
// ============================================================
long degreesToSteps(float degrees) {
  return (long)(degrees * STEPS_PER_DEGREE);
}

// ============================================================
// FUNZIONE: parseAndMove()
// Analizza la stringa seriale ricevuta da ROS 2 e imposta
// la posizione target per ogni giunto
// Formato atteso: "J1:500 J2:-200 J3:1500 J4:800 J5:300\n"
// I valori sono già in STEP (conversione fatta nel bridge ROS 2)
// ============================================================
void parseAndMove(String command) {
  // Array temporaneo per i target step dei 5 giunti
  long targetSteps[5] = {0, 0, 0, 0, 0};

  // Estrae il valore numerico per ogni giunto dalla stringa
  // Cerca il pattern "J1:", "J2:", ecc. e legge il numero dopo
  for (int i = 0; i < 5; i++) {
    String token = "J" + String(i + 1) + ":";
    int idx = command.indexOf(token);
    if (idx != -1) {
      // Prende la sottostringa dopo "J1:" fino allo spazio o fine stringa
      int start = idx + token.length();
      int end = command.indexOf(' ', start);
      String valueStr = (end == -1) ? command.substring(start)
                                    : command.substring(start, end);
      targetSteps[i] = valueStr.toInt();
    }
  }

  // Abilita i driver prima di muovere
  enableMotors();

  // Applica soft limits e imposta il target per ogni giunto
  for (int i = 0; i < 5; i++) {
    long safeTarget = clampToLimits(i, targetSteps[i]);

    // Avvisa se il target è stato clampato (utile per debug)
    if (safeTarget != targetSteps[i]) {
      Serial.print("⚠️ J"); Serial.print(i + 1);
      Serial.print(" clampato da "); Serial.print(targetSteps[i]);
      Serial.print(" a "); Serial.println(safeTarget);
    }

    joints[i]->moveTo(safeTarget);
  }
}

// ============================================================
// FUNZIONE: allStopped()
// Controlla se tutti e 5 i motori hanno raggiunto la posizione
// Ritorna true solo se distanceToGo() == 0 per tutti
// ============================================================
bool allStopped() {
  for (int i = 0; i < 5; i++) {
    if (joints[i]->distanceToGo() != 0) return false;
  }
  return true;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);  // 115200 baud per comunicazione veloce con ROS 2

  // Configura tutti i pin ENABLE come output e disabilita i driver
  for (int i = 0; i < 5; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH);  // disabilitato all'avvio
  }

  // Configura velocità e accelerazione per ogni giunto
  for (int i = 0; i < 5; i++) {
    joints[i]->setMaxSpeed(MAX_SPEED);
    joints[i]->setAcceleration(ACCELERATION);
  }

  // Messaggio di avvio sul Serial Monitor
  Serial.println("===== BRACCIO ROBOTICO PRONTO =====");
  Serial.println("Formato comando: J1:steps J2:steps J3:steps J4:steps J5:steps");
  Serial.print("Steps/giro uscita: "); Serial.println(STEPS_OUTPUT_REV);
  Serial.print("Steps/grado: "); Serial.println(STEPS_PER_DEGREE);
  Serial.println("===================================");
}

// ============================================================
// LOOP PRINCIPALE
// 1. Legge dalla seriale se arriva un comando da ROS 2
// 2. Esegue un passo su ogni motore che deve ancora muoversi
// 3. Quando tutti i motori sono fermi, disabilita i driver
// IMPORTANTE: non usare delay() qui — blocca AccelStepper
// ============================================================
void loop() {

  // --- Lettura seriale ---
  // readStringUntil('\n') legge fino al carattere newline
  // Non blocca il loop perché controlla Serial.available() prima
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // rimuove spazi e \r residui
    if (command.length() > 0) {
      Serial.print("Comando ricevuto: "); Serial.println(command);
      parseAndMove(command);
    }
  }

  // --- Esecuzione movimento ---
  // run() esegue al massimo 1 step per chiamata se è il momento giusto
  // Va chiamato il più frequentemente possibile → mai usare delay() qui
  if (motorsEnabled) {
    for (int i = 0; i < 5; i++) {
      joints[i]->run();
    }

    // Quando tutti i motori hanno raggiunto il target → disabilita i driver
    if (allStopped()) {
      disableMotors();
      Serial.println("✅ Tutti i motori in posizione.");
    }
  }
}