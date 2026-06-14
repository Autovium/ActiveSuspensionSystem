#include "mpu_sensor.h"
#include "globals.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
bool mpu_ready = false;

void setupMPU() {
  tcaSelect(4);
  if (!mpu.begin()) { Serial.println("Error: MPU6050 (Port 4) Failed!"); } 
  else { 
    Serial.println("Success: MPU6050 ready."); 
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu_ready = true; 
  }
}

void readMPU() {
  if (mpu_ready) {
    tcaSelect(4);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    Serial.println("\n--- MPU6050 Data ---");
    Serial.print("Accel (m/s^2) -> X: "); Serial.print(a.acceleration.x); 
    Serial.print("  Y: "); Serial.print(a.acceleration.y); 
    Serial.print("  Z: "); Serial.println(a.acceleration.z);
    
    Serial.print("Gyro (rad/s)  -> X: "); Serial.print(g.gyro.x); 
    Serial.print("  Y: "); Serial.print(g.gyro.y); 
    Serial.print("  Z: "); Serial.println(g.gyro.z);
  } else {
    Serial.println("\nMPU6050: OFFLINE");
  }
}