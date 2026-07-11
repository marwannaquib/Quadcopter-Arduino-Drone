#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

MPU6050 mpu(Wire);

Servo esc1, esc2, esc3, esc4;

// ESC PWM pins on Mega
#define M1_FL 2
#define M2_FR 3
#define M3_RL 11
#define M4_RR 12

// PID
#define GAIN              2.0
#define MAX_CORRECTION    200

// Movement
#define THROTTLE_STEP     100
#define TILT_AMOUNT       30

// ESC values
#define ESC_OFF           700
#define MIN_THROTTLE      1025

// Timing
#define BT_UPDATE_INTERVAL 100

// Flight variables
int baseThrottle = MIN_THROTTLE;
int pitchCommand = 0;
int rollCommand  = 0;
int yawCommand   = 0;

bool isArmed = false;
unsigned long lastBTUpdate = 0;

void killMotors() {
  esc1.writeMicroseconds(ESC_OFF);
  esc2.writeMicroseconds(ESC_OFF);
  esc3.writeMicroseconds(ESC_OFF);
  esc4.writeMicroseconds(ESC_OFF);
  baseThrottle = MIN_THROTTLE;
  pitchCommand = 0;
  rollCommand  = 0;
  yawCommand   = 0;
}

void log(String msg) {
  Serial1.println(msg);
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Wire.begin();

  esc1.attach(M1_FL);
  esc2.attach(M2_FR);
  esc3.attach(M3_RL);
  esc4.attach(M4_RR);

  esc1.writeMicroseconds(ESC_OFF);
  esc2.writeMicroseconds(ESC_OFF);
  esc3.writeMicroseconds(ESC_OFF);
  esc4.writeMicroseconds(ESC_OFF);
  delay(2000);

  log("Motors OFF — disarmed");

  byte status = mpu.begin();
  while (status != 0) {
    log("MPU6050 not found! Check wiring.");
    delay(500);
  }
  log("MPU6050 connected");

  log("Calibrating... do not move!");
  delay(2000);
  mpu.calcOffsets();
  log("Calibration done!");

  log("=====================================");
  log("DISARMED — send ARM to fly");
  log("ARM    = arm motors");
  log("DISARM = disarm motors");
  log("UP     = increase throttle");
  log("DOWN   = decrease throttle");
  log("F / B  = forward / backward");
  log("L / R  = left / right");
  log("YL / YR = rotate left / right");
  log("CENTER = hover in place");
  log("STOP   = emergency stop");
  log("=====================================");
}

void loop() {

  // Read Bluetooth commands
  if (Serial1.available()) {
    String cmd = "";
    long startTime = millis();

    while (millis() - startTime < 100) {
      if (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') break;
        cmd += c;
      }
    }

    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() > 0) {

      // ARM
      if (cmd == "ARM") {
        if (!isArmed) {
          isArmed = true;
          esc1.writeMicroseconds(MIN_THROTTLE);
          esc2.writeMicroseconds(MIN_THROTTLE);
          esc3.writeMicroseconds(MIN_THROTTLE);
          esc4.writeMicroseconds(MIN_THROTTLE);
          delay(500);
          log("ARMED — tap UP to increase throttle");
        } else {
          log("Already armed");
        }
      }

      // DISARM
      else if (cmd == "DISARM") {
        isArmed = false;
        killMotors();
        log("DISARMED — motors off");
      }

      // EMERGENCY STOP
      else if (cmd == "STOP") {
        isArmed = false;
        killMotors();
        log("EMERGENCY STOP — motors off");
      }

      // Block all commands if disarmed
      else if (!isArmed) {
        log("Not armed — send ARM first");
      }

      // Flight commands
      else {

        // THROTTLE
        if (cmd == "UP") {
          baseThrottle = constrain(
            baseThrottle + THROTTLE_STEP,
            MIN_THROTTLE, 2000);
          log("UP → THR:" + String(baseThrottle));
        }
        else if (cmd == "DOWN") {
          baseThrottle = constrain(
            baseThrottle - THROTTLE_STEP,
            MIN_THROTTLE, 2000);
          log("DOWN → THR:" + String(baseThrottle));
        }

        // PITCH
        else if (cmd == "F") {
          pitchCommand = TILT_AMOUNT;
          log("FORWARD");
        }
        else if (cmd == "B") {
          pitchCommand = -TILT_AMOUNT;
          log("BACKWARD");
        }

        // ROLL
        else if (cmd == "L") {
          rollCommand = -TILT_AMOUNT;
          log("LEFT");
        }
        else if (cmd == "R") {
          rollCommand = TILT_AMOUNT;
          log("RIGHT");
        }

        // YAW
        else if (cmd == "YL") {
          yawCommand = -TILT_AMOUNT;
          log("ROTATE LEFT");
        }
        else if (cmd == "YR") {
          yawCommand = TILT_AMOUNT;
          log("ROTATE RIGHT");
        }

        // HOVER
        else if (cmd == "CENTER") {
          pitchCommand = 0;
          rollCommand  = 0;
          yawCommand   = 0;
          log("CENTERED — hovering");
        }

        else {
          log("Unknown: " + cmd);
        }
      }
    }
  }

  // Motor control
  if (isArmed) {
    mpu.update();
    float pitch = mpu.getAngleX();
    float roll  = mpu.getAngleY();

    float pitchCorr = constrain(
      (pitchCommand - pitch) * GAIN,
      -MAX_CORRECTION, MAX_CORRECTION);
    float rollCorr = constrain(
      (rollCommand - roll) * GAIN,
      -MAX_CORRECTION, MAX_CORRECTION);

    int m1 = baseThrottle + pitchCorr + rollCorr - yawCommand;
    int m2 = baseThrottle + pitchCorr - rollCorr + yawCommand;
    int m3 = baseThrottle - pitchCorr + rollCorr + yawCommand;
    int m4 = baseThrottle - pitchCorr - rollCorr - yawCommand;

    m1 = constrain(m1, MIN_THROTTLE, 2000);
    m2 = constrain(m2, MIN_THROTTLE, 2000);
    m3 = constrain(m3, MIN_THROTTLE, 2000);
    m4 = constrain(m4, MIN_THROTTLE, 2000);

    esc1.writeMicroseconds(m1);
    esc2.writeMicroseconds(m2);
    esc3.writeMicroseconds(m3);
    esc4.writeMicroseconds(m4);

    if (millis() - lastBTUpdate > BT_UPDATE_INTERVAL) {
      String data =
        "ARMED" +
        String(" P:") + String(pitch, 1) +
        String(" R:") + String(roll, 1) +
        String(" THR:") + String(baseThrottle) +
        String(" M1:") + String(m1) +
        String(" M2:") + String(m2) +
        String(" M3:") + String(m3) +
        String(" M4:") + String(m4);
      Serial1.println(data);
      lastBTUpdate = millis();
    }

  } else {
    killMotors();
    if (millis() - lastBTUpdate > BT_UPDATE_INTERVAL) {
      Serial1.println("DISARMED — send ARM to start");
      lastBTUpdate = millis();
    }
  }

  delay(20);
}
