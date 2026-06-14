#include <Arduino.h>
#include "servo_control.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(100); } // Wait for serial port to connect
  
  setupServos();
  Serial.println("SYSTEM READY");
}
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    handleServoCommand(input);
    Serial.println("Command Received: " + input);
  }
}