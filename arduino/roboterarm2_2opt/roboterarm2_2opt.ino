#include <Servo.h>
#include <math.h>
#include <TMCStepper.h>

Servo servo150;   // Schulter
Servo servo60;    // Ellenbogen
Servo equalizer;  // Equalizer
Servo wrist;      // Handgelenkrotation
Servo gripper;    // Greifer

// Drehplatte: NEMA 17 mit TMC2209
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int EN_PIN   = 4;

#define TMC_UART_PIN 6        // RX und TX auf demselben Pin
#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b11
const long TMC_BAUD = 19200;
const int MOTOR_CURRENT = 900; // mA

TMC2209Stepper driver(TMC_UART_PIN, TMC_UART_PIN, R_SENSE, DRIVER_ADDRESS);

const int MICROSTEPS = 16;
const long STEPS_PER_REV = 200L * MICROSTEPS;

const float MIN_BASE_ANGLE = -135.0;   // Drehplatte darf nie hinten durchdrehen
const float MAX_BASE_ANGLE =  225.0;

const long RAMP_STEPS = 400;
const unsigned int RAMP_EXTRA_DELAY = 1500;
const unsigned int BASE_STEP_DELAY = 300;

// Home-Positionen
const float HOME_BASE_ANGLE = 0.0;
const int HOME_SHOULDER  = 82;
const int HOME_ELBOW     = 171;
const int HOME_EQUALIZER = 170;
const int PARK_EQUALIZER = 80;
const int HOME_WRIST     = 90;

const int GRIPPER_OPEN   = 122;
const int GRIPPER_CLOSED = 55;

const int SYNC_MS_MOVETO = 25;
const int SYNC_MS_LINEAR = 15;

// Geometrie in cm
const float L  = 22.5;         // Oberarm = Unterarm
const float LH = 11.5;
const float GH = 16.5;
const float H  = GH - LH;

const float BASE_X = 50.0;     // Drehachse in Weltkoordinaten
const float BASE_Y = 50.0;
const float SHOULDER_BACK = 5.0;   // Schulterachse hinter der Drehachse
const float GRIPPER_SIDE  = 2.0;   // seitlicher Greiferabstand

const float FIRST_BLOCK_Z = 2.5;
const float STACK_HEIGHT_STEP = 5.0;
const float HOVER_Z = 10.0;

// Ablage: Index = Farb-ID vom Pi (0 = rot, 1 = gruen, 2 = blau)
const float DROP_X = 50.0;
const float DROP_Y[3] = {14.0, 24.0, 34.0};
int stackCount[3] = {0, 0, 0};

const float ORIENT_HOME = -1000.0;   // Handgelenk in Ausgangslage lassen
const float DROP_ORIENTATION = ORIENT_HOME;

long currentBaseSteps = 0;
int currentShoulder  = HOME_SHOULDER;
int currentElbow     = HOME_ELBOW;
int currentEqualizer = PARK_EQUALIZER;
int currentWrist     = HOME_WRIST;
int currentGripper   = GRIPPER_OPEN;

float q1, q2, q3, q4, q5;

// wird gesetzt, wenn ein Ziel nicht erreichbar ist
bool ikFehler = false;


// Winkel in den erlaubten Bereich -135 .. 225 Grad bringen
float normalizeAngle(float a) {
  while (a >= MAX_BASE_ANGLE) a -= 360.0;
  while (a <  MIN_BASE_ANGLE) a += 360.0;
  return a;
}

void MoveBaseTo(float targetAngle) {
  long targetSteps = lround(normalizeAngle(targetAngle) / 360.0 * STEPS_PER_REV);
  long diff = targetSteps - currentBaseSteps;
  long steps = labs(diff);
  digitalWrite(DIR_PIN, diff >= 0 ? HIGH : LOW);

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);

    // Rampe: am Anfang und am Ende langsamer
    long rampPos = min(i, steps - 1 - i);
    unsigned int extra = 0;
    if (rampPos < RAMP_STEPS) {
      extra = (unsigned int)((long)RAMP_EXTRA_DELAY * (RAMP_STEPS - rampPos) / RAMP_STEPS);
    }
    delayMicroseconds(BASE_STEP_DELAY + extra);
  }
  currentBaseSteps = targetSteps;
}

// Ein Servo Grad für Grad zum Ziel bewegen
void MoveServo(Servo &s, int &currentPos, int target, int msPerStep) {
  target = constrain(target, 0, 180);
  while (currentPos != target) {
    currentPos += (target > currentPos) ? 1 : -1;
    s.write(currentPos);
    delay(msPerStep);
  }
}

// Alle vier Armservos gleichzeitig bewegen, sodass alle zur selben Zeit ankommen
void MoveArmSync(float tShoulder, float tElbow, float tEq, float tWrist, int msPerStep) {
  int target[4] = {
    constrain((int)round(tShoulder), 0, 180),
    constrain((int)round(tElbow),    0, 180),
    constrain((int)round(tEq),       0, 180),
    constrain((int)round(tWrist),    0, 180)
  };
  int start[4] = { currentShoulder, currentElbow, currentEqualizer, currentWrist };

  int maxDiff = 0;
  for (int i = 0; i < 4; i++) maxDiff = max(maxDiff, abs(target[i] - start[i]));

  for (int k = 1; k <= maxDiff; k++) {
    float t = (float)k / maxDiff;
    servo150.write( (int)round(start[0] + (target[0] - start[0]) * t) );
    servo60.write(  (int)round(start[1] + (target[1] - start[1]) * t) );
    equalizer.write((int)round(start[2] + (target[2] - start[2]) * t) );
    wrist.write(    (int)round(start[3] + (target[3] - start[3]) * t) );
    delay(msPerStep);
  }

  currentShoulder  = target[0];
  currentElbow     = target[1];
  currentEqualizer = target[2];
  currentWrist     = target[3];
}

void MoveGripper(int target) {
  MoveServo(gripper, currentGripper, target, 20);
  delay(700);
}

// INVERSE KINEMATIK: Weltkoordinaten (cm) -> Gelenkwinkel q1-q5
void calculateIK(float targetX, float targetY, float targetZ, float orientation) {
  ikFehler = false;

  float dx = targetX - BASE_X;
  float dy = targetY - BASE_Y;
  float baseR = sqrt(dx * dx + dy * dy);

  if (baseR <= GRIPPER_SIDE) {
    Serial.println(F("FEHLER: Ziel zu nah an der Drehachse!"));
    ikFehler = true;
    return;
  }

  float a = sqrt(baseR * baseR - GRIPPER_SIDE * GRIPPER_SIDE) + SHOULDER_BACK; // horizontal Schulter -> Handgelenk
  float s = targetZ - H;                                                       // vertikal Schulter -> Handgelenk
  float f = sqrt(s * s + a * a);                                               // direkte Distanz

  // zwei Glieder der Länge L reichen höchstens 2L weit
  if (f >= 2.0 * L) {
    Serial.println(F("FEHLER: Ziel ausser Reichweite!"));
    ikFehler = true;
    return;
  }

  q1 = normalizeAngle(degrees(atan2(dy, dx) - asin(GRIPPER_SIDE / baseR)));

  float alpha = degrees(atan2(s, a));          // negativ unter Schulterhöhe
  float beta  = degrees(acos(f / (2.0 * L)));

  q2 = HOME_SHOULDER + 90 - alpha - beta;
  q3 = degrees(acos((2.0 * L * L - f * f) / (2.0 * L * L))) - 9;
  q4 = HOME_EQUALIZER - q2 - (HOME_ELBOW - HOME_SHOULDER) + q3 - 7;

  if (orientation == ORIENT_HOME) {
    q5 = HOME_WRIST;
  } else {
    // Drehung der Basis ausgleichen, damit der Greifer zur Blockkante passt
    float rel = fmod(q1, 90.0);
    if (rel < 0) rel += 90.0;
    q5 = 90.0 + rel - orientation;
    // ausserhalb der Servorange: um 90 Grad verschieben (Quadrat ist symmetrisch)
    while (q5 > 180.0) q5 -= 90.0;
    while (q5 < 0.0)   q5 += 90.0;
  }
}

// Senkrecht in kleinen Schritten von startZ nach targetZ fahren
void MoveLinearZ(float targetX, float targetY, float startZ, float targetZ, float orientation, float stepCm = 0.5) {
  float zDiff = targetZ - startZ;
  int steps = max(1, (int)(fabs(zDiff) / stepCm));
  float zStep = zDiff / steps;

  for (int i = 1; i <= steps; i++) {
    float currentZ = (i == steps) ? targetZ : startZ + i * zStep;
    calculateIK(targetX, targetY, currentZ, orientation);
    if (ikFehler) return;
    MoveArmSync(q2, q3, q4, q5, SYNC_MS_LINEAR);
  }
}

void MoveTo(float targetX, float targetY, float targetZ, float orientation) {
  calculateIK(targetX, targetY, targetZ, orientation);
  if (ikFehler) return;

  MoveBaseTo(q1);
  delay(300);
  MoveArmSync(q2, q3, q4, q5, SYNC_MS_MOVETO);
  delay(300);
}

void MoveHome() {
  MoveServo(servo150,  currentShoulder,  HOME_SHOULDER,  20);  delay(400);
  MoveServo(servo60,   currentElbow,     HOME_ELBOW,     20);  delay(400);
  MoveServo(equalizer, currentEqualizer, PARK_EQUALIZER, 100); delay(400);
  MoveServo(wrist,     currentWrist,     HOME_WRIST,     20);  delay(400);
  MoveBaseTo(HOME_BASE_ANGLE);
  delay(600);
}

// Ablauf: Greifer auf -> über Block -> runter -> greifen -> hoch -> Home
// -> über Ablage -> auf Stapelhöhe runter -> loslassen -> hoch -> Home
// Bei einem IK-Fehler: zurück nach Home, ikFehler bleibt gesetzt.
void sortBlock(int farbe, float blockX, float blockY, float blockOrientation) {
  float dropY = DROP_Y[farbe];
  float dropZ = FIRST_BLOCK_Z + stackCount[farbe] * STACK_HEIGHT_STEP + 2.0;

  MoveGripper(GRIPPER_OPEN);

  MoveTo(blockX, blockY, HOVER_Z, blockOrientation);
  if (ikFehler) { MoveHome(); return; }
  delay(200);

  MoveLinearZ(blockX, blockY, HOVER_Z, FIRST_BLOCK_Z, blockOrientation);
  if (ikFehler) { MoveHome(); return; }
  delay(200);

  MoveGripper(GRIPPER_CLOSED);

  MoveLinearZ(blockX, blockY, FIRST_BLOCK_Z, HOVER_Z, blockOrientation);
  if (ikFehler) { MoveHome(); return; }
  MoveHome();

  MoveTo(DROP_X, dropY, HOVER_Z, DROP_ORIENTATION);
  if (ikFehler) { MoveHome(); return; }

  MoveLinearZ(DROP_X, dropY, HOVER_Z, dropZ, DROP_ORIENTATION);
  if (ikFehler) { MoveHome(); return; }

  MoveGripper(GRIPPER_OPEN);
  stackCount[farbe]++;

  MoveLinearZ(DROP_X, dropY, dropZ, HOVER_Z, DROP_ORIENTATION);
  MoveHome();
}


int parseNumbers(const String &text, float values[], int maxCount) {
  char buf[64];
  text.toCharArray(buf, sizeof(buf));

  int count = 0;
  char *p = buf;

  while (*p != '\0' && count < maxCount) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (*p == '\0') break;

    char *end;
    double v = strtod(p, &end);
    if (end == p) return -1;   // keine gültige Zahl
    values[count++] = v;
    p = end;
  }
  return count;
}

// Befehl vom Pi: Farbe,X,Y,Orientierung, Antwort DONE oder ERROR
void handleBlock(const String &line) {
  float v[4];
  int n = parseNumbers(line, v, 4);

  if (n != 4 || v[0] < 0 || v[0] > 2) {
    Serial.println(F("FEHLER: Falsches Datenformat."));
    ikFehler = true;
  } else {
    Serial.print(F("BLOCK: Farbe ")); Serial.print((int)v[0]);
    Serial.print(F(" | X ")); Serial.print(v[1]);
    Serial.print(F(" | Y ")); Serial.print(v[2]);
    Serial.print(F(" | W ")); Serial.println(v[3]);
    sortBlock((int)v[0], v[1], v[2], v[3]);
  }

  // Alles verwerfen, was während der Bewegung angekommen ist.
  while (Serial.available() > 0) Serial.read();

  Serial.println(ikFehler ? F("ERROR") : F("DONE"));
}

bool setupDriver() {
  driver.begin();
  driver.beginSerial(TMC_BAUD);

  driver.toff(5);
  driver.I_scale_analog(false);
  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.microsteps(MICROSTEPS);
  driver.rms_current(MOTOR_CURRENT);
  driver.ihold(16);
  driver.iholddelay(8);
  driver.TPOWERDOWN(20);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);

  return driver.test_connection() == 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // Treiber aktiv

  if (!setupDriver()) {
    Serial.println(F("TMC antwortet nicht!"));
    while (true);
  }

  servo150.write(HOME_SHOULDER);
  servo60.write(HOME_ELBOW);
  equalizer.write(PARK_EQUALIZER);
  wrist.write(HOME_WRIST);
  gripper.write(GRIPPER_OPEN);

  delay(2000);

  servo150.attach(5);
  servo60.attach(7);
  equalizer.attach(9);
  wrist.attach(11);
  gripper.attach(13);
  delay(2000);

  servo150.write(HOME_SHOULDER);
  servo60.write(HOME_ELBOW);
  equalizer.write(PARK_EQUALIZER);
  wrist.write(HOME_WRIST);
  gripper.write(GRIPPER_OPEN);

  Serial.println(F("ROBOTER BEREIT"));
}

void loop() {
  if (Serial.available() == 0) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  handleBlock(line);
}
