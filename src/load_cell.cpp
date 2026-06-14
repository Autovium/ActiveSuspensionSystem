#include "load_cell.h"
#include <Arduino.h>
#include "HX711.h"

// Pin definitions based on your requested layout
#define FL_DT 5
#define FL_SCK 4
#define FR_DT 38
#define FR_SCK 39
#define BL_DT 6
#define BL_SCK 7
#define BR_DT 36
#define BR_SCK 35

HX711 scaleFL, scaleFR, scaleBL, scaleBR;

unsigned long lastLoadCellRead = 0;
const unsigned long LOAD_CELL_INTERVAL = 3000; 

void setupLoadCell() {
    Serial.println("\n--- Initializing 4 Load Cells ---");

    scaleFL.begin(FL_DT, FL_SCK);
    scaleFR.begin(FR_DT, FR_SCK);
    scaleBL.begin(BL_DT, BL_SCK);
    scaleBR.begin(BR_DT, BR_SCK);

    // Optional: add .tare() here if you want to zero them out at startup
    Serial.println("Load Cells Initialized.");
}

void readLoadCell() {
    if (millis() - lastLoadCellRead >= LOAD_CELL_INTERVAL) {
        lastLoadCellRead = millis();

        Serial.println("\n--- Load Cell Data (Raw) ---");
        
        // Helper lambda to clean up printing
        auto printScale = [](const char* name, HX711& scale) {
            Serial.print(name);
            Serial.print(": ");
            if (scale.is_ready()) {
                Serial.println(scale.read());
            } else {
                Serial.println("OFFLINE");
            }
        };

        printScale("Front Left ", scaleFL);
        printScale("Front Right", scaleFR);
        printScale("Back Left  ", scaleBL);
        printScale("Back Right ", scaleBR);
    }
}