#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Front Swing Arms
Servo frontLeft;
Servo frontRight;

// Rear Swing Arms
Servo rearLeft;
Servo rearRight;

// Pin Assignments
const int pinFrontLeft = 26; 
const int pinFrontRight = 27; 
const int pinRearLeft = 14; 
const int pinRearRight = 25; 

// --- KINEMATIC CALIBRATION ---
const int fixedStance = 60; 
float pitchScale = 4.4; 
float rollScale  = 3.5; 

// GLOBAL CALIBRATION VARIABLES
float globalPitchOffset = 0.0;
float globalRollOffset  = 0.0;

// --- PID CONTROL VARIABLES ---
float targetPitch = 0.0;
float targetRoll = 0.0;

float activePitch = 0.0;
float activeRoll = 0.0;

float lastPitchError = 0.0;
float lastRollError = 0.0;

// TUNING PARAMETERS 
float Kp = 0.1;  // Speed of correction
float Kd = 0.20;  // Shock absorber / Dampener

unsigned long previousMillis = 0;
const int controlInterval = 50; 

void setup() {
  Serial.begin(115200);

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050! Check wiring.");
    while (1) { delay(10); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  frontLeft.setPeriodHertz(50); 
  frontRight.setPeriodHertz(50); 
  rearLeft.setPeriodHertz(50); 
  rearRight.setPeriodHertz(50); 
  
  frontLeft.attach(pinFrontLeft, 500, 2400); 
  frontRight.attach(pinFrontRight, 500, 2400); 
  rearLeft.attach(pinRearLeft, 500, 2400); 
  rearRight.attach(pinRearRight, 500, 2400); 

  // ==========================================
  // BOOT SEQUENCE STEP 1: GO TO HOME (p0 r0)
  // ==========================================
  Serial.println("\n--- INITIALIZING HARDWARE ---");
  Serial.println("Moving chassis to default 60-degree stance...");
  
  frontLeft.write(fixedStance);
  frontRight.write(180 - fixedStance);
  rearLeft.write(180 - fixedStance);
  rearRight.write(fixedStance);

  // ==========================================
  // BOOT SEQUENCE STEP 2: SETTLE
  // ==========================================
  Serial.println("Waiting 3 seconds for physical wobble to stop...");
  delay(3000); 

  // ==========================================
  // BOOT SEQUENCE STEP 3: CALIBRATE
  // ==========================================
  calibrateSensor();
  
  // ==========================================
  // BOOT SEQUENCE STEP 4: ENGAGE PID (Handled in loop)
  // ==========================================
  Serial.println("\n--- ACTIVE LEVELING ENGAGED ---");
  Serial.println("Try picking up one end of the chassis!");
}

void calibrateSensor() {
  Serial.println("\n--- CALIBRATING MPU6050 ---");
  Serial.println("Keep the chassis perfectly still on a flat surface...");
  
  float pitchSum = 0;
  float rollSum = 0;
  int sampleCount = 100; 

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    pitchSum += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    rollSum  += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    delay(10); 
  }

  globalPitchOffset = pitchSum / sampleCount;
  globalRollOffset  = rollSum / sampleCount;
  
  Serial.print("Locked Pitch Offset: "); Serial.println(globalPitchOffset);
  Serial.print("Locked Roll Offset: ");  Serial.println(globalRollOffset);
}

void loop() {
  // SERIAL OVERRIDES (Change the target away from 0 dynamically)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); input.toLowerCase(); 
    if (input.length() > 0) {
      int pIndex = input.indexOf('p');
      if (pIndex >= 0) {
        int spaceIndex = input.indexOf(' ', pIndex);
        if (spaceIndex == -1) spaceIndex = input.length();
        targetPitch = input.substring(pIndex + 1, spaceIndex).toFloat();
      }
      int rIndex = input.indexOf('r');
      if (rIndex >= 0) {
        int spaceIndex = input.indexOf(' ', rIndex);
        if (spaceIndex == -1) spaceIndex = input.length();
        targetRoll = input.substring(rIndex + 1, spaceIndex).toFloat();
      }
    }
  }

  // ACTIVE LEVELING FEEDBACK LOOP (20Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= controlInterval) {
    previousMillis = currentMillis;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float rawRoll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

    float actualPitch = -(rawPitch - globalPitchOffset);
    float actualRoll  = -(rawRoll - globalRollOffset);

    float pitchError = targetPitch - actualPitch;
    float rollError  = targetRoll - actualRoll;

    float pitchDerivative = pitchError - lastPitchError;
    float rollDerivative  = rollError - lastRollError;

    // Apply PID Math
    activePitch += (pitchError * Kp) + (pitchDerivative * Kd);
    activeRoll  += (rollError * Kp)  + (rollDerivative * Kd);

    lastPitchError = pitchError;
    lastRollError  = rollError;

    // Convert active commands into scaled servo targets
    int servoPitch = activePitch * pitchScale;
    int servoRoll  = activeRoll * rollScale;

    int frontLeftTarget  = fixedStance + servoPitch + servoRoll;
    int rearLeftTarget   = fixedStance - servoPitch + servoRoll;
    int frontRightTarget = fixedStance + servoPitch - servoRoll;
    int rearRightTarget  = fixedStance - servoPitch - servoRoll;

    // Constrain hardware bounds to prevent accumulator wind-up
    activePitch = constrain(activePitch, -25, 25);
    activeRoll  = constrain(activeRoll, -25, 25);
    
    frontLeftTarget  = constrain(frontLeftTarget, 0, 180);
    rearLeftTarget   = constrain(rearLeftTarget, 0, 180);
    frontRightTarget = constrain(frontRightTarget, 0, 180);
    rearRightTarget  = constrain(rearRightTarget, 0, 180);

    // Invert hardware mountings
    int invFrontRight = 180 - frontRightTarget;
    int invRearLeft   = 180 - rearLeftTarget;
    
    frontLeft.write(frontLeftTarget);    
    frontRight.write(invFrontRight);       
    rearLeft.write(invRearLeft);           
    rearRight.write(rearRightTarget);      

    // TELEMETRY TO SERIAL PLOTTER
    Serial.print("Target_Pitch:"); Serial.print(targetPitch); Serial.print(",");
    Serial.print("Actual_Pitch:"); Serial.println(actualPitch);
  }
}