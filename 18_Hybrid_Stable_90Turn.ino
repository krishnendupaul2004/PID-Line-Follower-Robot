// ============================================================
// ULTRA SMOOTH + SHARP 90° TURN DETECTION
// Arduino Nano + 5 Analog Sensors
// ============================================================

// ===== MOTOR PINS =====
#define STBY  4
#define PWMA  5
#define PWMB  6
#define AIN1  7
#define AIN2  8
#define BIN1  9
#define BIN2  10

#define LEFT_REVERSE  1
#define RIGHT_REVERSE 0

// ===== SENSOR PINS =====
int s[5] = {A0, A1, A2, A3, A6};

// ===== CALIBRATION =====
#define SENSOR_BLACK 147
#define SENSOR_WHITE 989
int threshold = 560;

// ===== WEIGHTS =====
int weight[5] = {-120, -60, 0, 60, 120};

// ===== PID =====
float Kp = 0;
float Kd = 0;

int error            = 0;
int lastError        = 0;
int PIDvalue         = 0;
int active           = 0;

// ===== FILTERS =====
float smoothError      = 0;
float smoothDerivative = 0;

// ===== STATE =====
bool lineLost        = false;
int  lastKnownError  = 0;

// ============================================================
// READ LINE
// ============================================================
int readLine() {

  long weightedSum = 0;
  long sensorSum   = 0;
  active = 0;

  for (int i = 0; i < 5; i++) {
    int raw = analogRead(s[i]);
    int val = constrain(map(raw, SENSOR_WHITE, SENSOR_BLACK, 0, 1000), 0, 1000);
    weightedSum += (long)val * weight[i];
    sensorSum   += val;
    if (raw < threshold) active++;
  }

  if (sensorSum < 80) {
    lineLost = true;
    return lastKnownError;
  }

  lineLost       = false;
  lastKnownError = weightedSum / sensorSum;
  return lastKnownError;
}

// ============================================================
// MOTOR
// ============================================================
void setMotor(int left, int right) {

  if (LEFT_REVERSE)  left  = -left;
  if (RIGHT_REVERSE) right = -right;

  if (left >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else           { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); left = -left; }

  if (right >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); right = -right; }

  analogWrite(PWMB, constrain(left,  0, 255));
  analogWrite(PWMA, constrain(right, 0, 255));
}

// ============================================================
// DETECT SHARP TURN
// Returns:  1 = right turn
//          -1 = left turn
//           0 = no sharp turn
// ============================================================

int detectSharpTurn() {

  int s0 = analogRead(A0);
  int s1 = analogRead(A1);
  int s2 = analogRead(A2);
  int s3 = analogRead(A3);
  int s4 = analogRead(A6);

  if (s0 > threshold &&
      s1 > threshold &&
      s2 < threshold &&
      s3 < threshold &&
      s4 < threshold)
    return 1;

  if (s0 < threshold &&
      s1 < threshold &&
      s2 < threshold &&
      s3 > threshold &&
      s4 > threshold)
    return -1;

  return 0;
}


// ============================================================
// EXECUTE SHARP TURN
// ============================================================

void doSharpTurn(int dir) {

  int turnSpeed = 180;

  if (dir > 0) setMotor(turnSpeed, -turnSpeed);
  else         setMotor(-turnSpeed, turnSpeed);

  delay(120);

  unsigned long start = millis();

  while (millis() - start < 700) {

    if (analogRead(A2) < threshold) {

      setMotor(140, 140);
      delay(40);

      smoothError = 0;
      smoothDerivative = 0;
      lastError = 0;

      return;
    }

    if (dir > 0) setMotor(turnSpeed, -turnSpeed);
    else         setMotor(-turnSpeed, turnSpeed);
  }

  smoothError = 0;
  smoothDerivative = 0;
  lastError = 0;
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  int rawError = readLine();

  // ==========================================================
  // SHARP TURN DETECTION — priority over PID
  // ==========================================================
  int turnDir = detectSharpTurn();

  if (turnDir != 0 && active >= 3) {
    doSharpTurn(turnDir);
    return;
  }

  // ==========================================================
  // TURN MEMORY
  // ==========================================================
  if      (rawError > 0) lastKnownError =  1;
  else if (rawError < 0) lastKnownError = -1;

  // ==========================================================
  // LINE LOST
  // ==========================================================
  if (lineLost) {
    smoothDerivative = 0;
    smoothError      = 0;
    if (lastKnownError > 0) setMotor(210, -110);
    else                    setMotor(-110, 210);
    return;
  }

  // ==========================================================
  // ERROR FILTER
  // ==========================================================
  smoothError = smoothError * 0.65 + rawError * 0.35;
  error = (int)smoothError;

  if (abs(error) < 45) error = 0;

  // ==========================================================
  // DERIVATIVE FILTER
  // ==========================================================
  int rawDerivative = error - lastError;
  smoothDerivative  = smoothDerivative * 0.70 + rawDerivative * 0.30;
  int derivative    = (int)smoothDerivative;

  lastError = error;

  // ==========================================================
  // DYNAMIC Kp / Kd
  // ==========================================================
  if      (active >= 4) { Kp = 2.0; Kd = 5.0;  }
  else if (active == 3) { Kp = 3.0; Kd = 6.5;  }
  else if (active == 2) { Kp = 4.0; Kd = 8.0;  }
  else                  { Kp = 5.5; Kd = 10.0; }

  // ==========================================================
  // PID
  // ==========================================================
  PIDvalue = (Kp * error) + (Kd * derivative);
  PIDvalue = constrain(PIDvalue, -180, 180);

  // ==========================================================
  // BASE SPEED
  // ==========================================================
  int base;
  if      (abs(error) < 35)  base = 250;
  else if (abs(error) < 100) base = 230;
  else if (abs(error) < 180) base = 180;
  else                        base = 140;

  // ==========================================================
  // CORRECTION SCALE
  // ==========================================================
  float correction;
  if      (abs(error) < 40)  correction = PIDvalue * 0.025;
  else if (abs(error) < 150) correction = PIDvalue * 0.07;
  else                        correction = PIDvalue * 0.16;

  // ==========================================================
  // MOTOR OUTPUT
  // ==========================================================
  int left  = constrain(base + correction + 4, 0, 255);
  int right = constrain(base - correction,     0, 255);

  setMotor(left, right);
}