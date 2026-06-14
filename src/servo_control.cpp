#include "servo_control.h"
#include <ESP32Servo.h>

Servo fl, fr, bl, br;

void setupServos() {
    // Standard ESP32Servo setup
    fl.attach(12);
    fr.attach(42);
    bl.attach(47);
    br.attach(21);
    
    // Allow the PWM signal to stabilize before moving
    delay(100); 
    setAllServosToZero();
}

void setAllServosToZero() {
    fl.write(180); 
    fr.write(0);
    bl.write(0);
    br.write(180);
}

void handleServoCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HOME") {
        setAllServosToZero();
        Serial.println("OK: Homing");
        return;
    }

    if (cmd.length() < 3) return;

    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, 0, 180);

    // Explicitly update the servo and provide feedback
    if (cmd.startsWith("FL")) { fl.write(180 - angle); Serial.println("OK: FL"); }
    else if (cmd.startsWith("FR")) { fr.write(angle); Serial.println("OK: FR"); }
    else if (cmd.startsWith("BL")) { bl.write(angle); Serial.println("OK: BL"); }
    else if (cmd.startsWith("BR")) { br.write(180 - angle); Serial.println("OK: BR"); }
}