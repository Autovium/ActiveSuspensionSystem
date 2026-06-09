#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

// --- HARDWARE OBJECTS ---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- MOTOR PINS (L298N) ---
const int ENA = 14; 
const int IN1 = 27;
const int IN2 = 26;

// --- SENSOR FILTER ---
float filteredDistance = 0;
const float alpha = 0.4; 

// --- TARGET VARIABLES ---
float setpoint = 0.0; 
const float DEADBAND = 5.0; // Increased to 5mm to prevent violent oscillation

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  Serial.println("Initializing VL53L0X...");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X!"));
    while(1);
  }
  
  lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  
  // Prime the filter
  for(int i = 0; i < 20; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      float raw = measure.RangeMilliMeter;
      if (filteredDistance == 0) filteredDistance = raw;
      filteredDistance = (alpha * raw) + ((1.0 - alpha) * filteredDistance);
    }
    delay(30);
  }

  // Lock the target distance
  setpoint = filteredDistance;
  Serial.print(">>> TARGET LOCKED AT: ");
  Serial.print(setpoint);
  Serial.println(" mm <<<");
  delay(1000); 
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); 

  if (measure.RangeStatus != 4) {  
    float rawDistance = measure.RangeMilliMeter;
    filteredDistance = (alpha * rawDistance) + ((1.0 - alpha) * filteredDistance);
    
    // Calculate how far off we are
    float error = filteredDistance - setpoint; 

    // --- BANG-BANG MOTOR CONTROL ---
    if (abs(error) <= DEADBAND) {
      // In the safe zone -> STOP completely
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      
    } else if (error > 0) {
      // Too far away -> FULL SPEED FORWARD
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 255); 
      
    } else {
      // Too close -> FULL SPEED BACKWARD
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 255); 
    }

    Serial.print("Target:");
    Serial.print(setpoint);
    Serial.print(", Current:");
    Serial.println(filteredDistance);
    
  } else {
    // Sensor failed/out of range -> Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
    
  delay(30); 
}