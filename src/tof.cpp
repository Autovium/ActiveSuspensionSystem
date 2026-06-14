#include "tof.h"
#include "globals.h"
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X sensorFrontLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorBackRight = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorBackLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorFrontRight = Adafruit_VL53L0X();

bool fl_ready = false;
bool br_ready = false;
bool bl_ready = false;
bool fr_ready = false;

void setupTOF() {
  tcaSelect(0);
  if (!sensorFrontLeft.begin()) { Serial.println("Error: TOF Front Left (0) Failed!"); } 
  else { Serial.println("Success: TOF Front Left ready."); fl_ready = true; }

  tcaSelect(1);
  if (!sensorBackRight.begin()) { Serial.println("Error: TOF Back Right (1) Failed!"); } 
  else { Serial.println("Success: TOF Back Right ready."); br_ready = true; }

  tcaSelect(2);
  if (!sensorBackLeft.begin()) { Serial.println("Error: TOF Back Left (2) Failed!"); } 
  else { Serial.println("Success: TOF Back Left ready."); bl_ready = true; }

  tcaSelect(3);
  if (!sensorFrontRight.begin()) { Serial.println("Error: TOF Front Right (3) Failed!"); } 
  else { Serial.println("Success: TOF Front Right ready."); fr_ready = true; }
}

void readTOF() {
  VL53L0X_RangingMeasurementData_t measure;
  
  if (fl_ready) {
    tcaSelect(0);
    sensorFrontLeft.rangingTest(&measure, false);
    Serial.print("Front Left:  ");
    Serial.println(measure.RangeStatus != 4 ? String(measure.RangeMilliMeter) + " mm" : "Out of range");
  } else { Serial.println("Front Left:  OFFLINE"); }

  if (fr_ready) {
    tcaSelect(3); 
    sensorFrontRight.rangingTest(&measure, false);
    Serial.print("Front Right: ");
    Serial.println(measure.RangeStatus != 4 ? String(measure.RangeMilliMeter) + " mm" : "Out of range");
  } else { Serial.println("Front Right: OFFLINE"); }

  if (bl_ready) {
    tcaSelect(2); 
    sensorBackLeft.rangingTest(&measure, false);
    Serial.print("Back Left:   ");
    Serial.println(measure.RangeStatus != 4 ? String(measure.RangeMilliMeter) + " mm" : "Out of range");
  } else { Serial.println("Back Left:   OFFLINE"); }

  if (br_ready) {
    tcaSelect(1); 
    sensorBackRight.rangingTest(&measure, false);
    Serial.print("Back Right:  ");
    Serial.println(measure.RangeStatus != 4 ? String(measure.RangeMilliMeter) + " mm" : "Out of range");
  } else { Serial.println("Back Right:  OFFLINE"); }
}