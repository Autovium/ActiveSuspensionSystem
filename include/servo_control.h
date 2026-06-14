#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

void setupServos();
void handleServoCommand(String cmd);
void setAllServosToZero();

#endif