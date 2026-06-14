#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <HX711.h>
#include <math.h>

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
#define SDA_PIN 1
#define SCL_PIN 2
#define TCAADDR 0x70

#define PIN_FL_SERVO 12
#define PIN_FR_SERVO 42
#define PIN_BL_SERVO 47
#define PIN_BR_SERVO 21

#define FL_DT 5
#define FL_SCK 4
#define FR_DT 38
#define FR_SCK 39
#define BL_DT 6
#define BL_SCK 7
#define BR_DT 36
#define BR_SCK 35

// L298N Motor Pins
#define MOTOR_IN1 13
#define MOTOR_IN2 14
#define MOTOR_EN  11

// PWM Properties for Motor
const int pwmFreq = 5000;    // 5kHz
const int pwmResolution = 8; // 8-bit (0-255)

// ==========================================
// 2. KINEMATIC GEOMETRY CONSTANTS
// ==========================================
const float GEOMETRY_PITCH_MULT_POS = -5.1; 
const float GEOMETRY_PITCH_MULT_NEG = -4.0; 

const float GEOMETRY_ROLL_MULT_POS = 8.7;   
const float GEOMETRY_ROLL_MULT_NEG = 4.1;   

const float MAX_ANGLE_STEP = 90.0; // Increased to 90 to remove software speed limits (INSTANT SNAPPING)

// ==========================================
// 2.5 ACTIVE SUSPENSION TUNING
// ==========================================
const float Kp_ToF_Terrain = 1.5;   // Increased from 0.5 to 1.5 for a larger lift response to obstacles

// IMU Stabilization Gains
const float Kp_IMU_Pitch = 1.0;
const float Kp_IMU_Roll = 1.0;

// CRITICAL FIX: Explicitly forced to 0.0. If these are 0.1, HX711 drift will cause the robot to roll randomly!
const float Kp_Load_Extend = 0.0;   
const float Kp_Load_Retract = 0.0;  

const float TOF_DEADBAND = 1.0;   // Kept at 5mm to avoid hyper-sensitivity
const float LOAD_DEADBAND = 15.0; // Ignore load variations less than 15 units
const float MAX_LOCAL_OFFSET = 45.0; // Increased to 45.0 to allow the leg to lift higher over obstacles

bool local_sensors_enabled = true; // Toggle for ToF/Load cell overrides
bool plotter_mode = true;          // Toggle for Arduino Serial Plotter format

// ==========================================
// 3. SUCCESSIVE AVERAGE FILTERS (NOISE REDUCTION)
// ==========================================
template <int N>
struct SuccessiveAverage {
    float readings[N];
    int index;
    float sum;
    int count;
    float current_average;

    SuccessiveAverage() {
        for (int i = 0; i < N; i++) readings[i] = 0.0;
        index = 0;
        sum = 0.0;
        count = 0;
        current_average = 0.0;
    }

    void init(float val) {
        for (int i = 0; i < N; i++) readings[i] = val;
        sum = val * N;
        index = 0;
        count = N;
        current_average = val;
    }

    float add(float val) {
        // True Circular Buffer implementation (Rolling Average)
        sum -= readings[index];  // Remove oldest reading from sum
        readings[index] = val;   // Store new reading
        sum += val;              // Add new reading to sum
        index = (index + 1) % N; // Advance index cyclically
        
        if (count < N) count++;
        current_average = sum / count;
        return current_average;
    }
};

// Window sizes for averaging:
// 10 for IMU for smooth leveling (Reduced from 100 to completely eliminate Phase Lag oscillations!)
// 1 for ToF for INSTANT reaction to obstacles (zero software delay)
// 1 for Load cells for INSTANT impact detection
SuccessiveAverage<10> pitch_avg;
SuccessiveAverage<10> roll_avg;
SuccessiveAverage<1> tof_avg[4];  
SuccessiveAverage<1> load_avg[4]; 

// ==========================================
// 4. GLOBAL STATE VARIABLES
// ==========================================
Servo fl, fr, bl, br;
Adafruit_MPU6050 mpu;
HX711 scaleFL, scaleFR, scaleBL, scaleBR;
Adafruit_VL53L0X tofFL = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFR = Adafruit_VL53L0X();
Adafruit_VL53L0X tofBL = Adafruit_VL53L0X();
Adafruit_VL53L0X tofBR = Adafruit_VL53L0X();

bool mpu_ready = false;
bool tof_ready[4] = {false, false, false, false}; // FL, FR, BL, BR

// Baselines calculated at startup
float IMU_PITCH_OFFSET = 0.0, IMU_ROLL_OFFSET = 0.0;
float BASE_TOF[4] = {0, 0, 0, 0}; 
const int BASE_ANGLE_FRONT = 45;
const int BASE_ANGLE_BACK = 55;

// Current Sensor Data (Smoothed Output)
float curr_pitch = 0.0, curr_roll = 0.0;
float curr_tof[4] = {0, 0, 0, 0};
float curr_load[4] = {0, 0, 0, 0};

// Target Angles set by Serial Commands
float target_pitch = 0.0;
float target_roll = 0.0;

// Current Mechanical State
float currentFL = BASE_ANGLE_FRONT, currentFR = BASE_ANGLE_FRONT;
float currentBL = BASE_ANGLE_BACK, currentBR = BASE_ANGLE_BACK;

// Timing Variables
unsigned long lastControlTime = 0;
unsigned long lastDiagTime = 0;
uint8_t tof_poll_index = 0; // For round-robin ToF reading
const unsigned long PLOT_INTERVAL = 50;   // 20Hz refresh rate for plotting
const unsigned long DIAG_INTERVAL = 1000; // 1Hz refresh rate for text logging

// ==========================================
// 5. HARDWARE HELPERS
// ==========================================
void tcaSelect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission(true); 
    delay(5); 
}

float stepTowards(float current, float target, float max_step) {
    if (abs(target - current) <= max_step) return target;
    if (target > current) return current + max_step;
    return current - max_step;
}

// ==========================================
// 6. NON-BLOCKING SENSOR ACQUISITION
// ==========================================
void updateIMU() {
    if (!mpu_ready) return;
    tcaSelect(4);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float raw_roll  = (-atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI);
    float raw_pitch = (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI);
    
    curr_pitch = pitch_avg.add(raw_pitch - IMU_PITCH_OFFSET);
    curr_roll  = roll_avg.add(raw_roll - IMU_ROLL_OFFSET);
}

void updateToFRoundRobin() {
    VL53L0X_RangingMeasurementData_t measure;
    
    switch(tof_poll_index) {
        case 0:
            if (tof_ready[0]) {
                tcaSelect(0);
                tofFL.rangingTest(&measure, false);
                if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) {
                    curr_tof[0] = tof_avg[0].add(measure.RangeMilliMeter);
                }
            }
            break;
        case 1:
            if (tof_ready[1]) {
                tcaSelect(3);
                tofFR.rangingTest(&measure, false);
                if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) {
                    curr_tof[1] = tof_avg[1].add(measure.RangeMilliMeter);
                }
            }
            break;
        case 2:
            if (tof_ready[2]) {
                tcaSelect(1); 
                tofBL.rangingTest(&measure, false);
                if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) {
                    curr_tof[2] = tof_avg[2].add(measure.RangeMilliMeter);
                }
            }
            break;
        case 3:
            if (tof_ready[3]) {
                tcaSelect(2); 
                tofBR.rangingTest(&measure, false);
                if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) {
                    curr_tof[3] = tof_avg[3].add(measure.RangeMilliMeter);
                }
            }
            break;
    }
    
    tof_poll_index++;
    if (tof_poll_index > 3) tof_poll_index = 0;
}

void updateLoadCells() {
    if(scaleFL.is_ready()) curr_load[0] = load_avg[0].add(scaleFL.get_units(1));
    if(scaleFR.is_ready()) curr_load[1] = load_avg[1].add(scaleFR.get_units(1));
    if(scaleBL.is_ready()) curr_load[2] = load_avg[2].add(scaleBL.get_units(1));
    if(scaleBR.is_ready()) curr_load[3] = load_avg[3].add(scaleBR.get_units(1));
}

// ==========================================
// 7. OPEN-LOOP KINEMATICS ENGINE -> UPGRADED TO CLOSED-LOOP!
// ==========================================
void processKinematics() {
    // 1. Calculate Feed-Forward User Commands (Using the aggressive geometry multipliers)
    float cmd_pitch = (target_pitch >= 0) ? (target_pitch * GEOMETRY_PITCH_MULT_POS) : (target_pitch * GEOMETRY_PITCH_MULT_NEG);
    float cmd_roll  = (target_roll >= 0)  ? (target_roll * GEOMETRY_ROLL_MULT_POS)   : (target_roll * GEOMETRY_ROLL_MULT_NEG);

    // 2. Calculate Closed-Loop Auto-Leveling (Using gentle stabilization multipliers to prevent oscillation)
    float imu_pitch = (target_pitch - curr_pitch) * Kp_IMU_Pitch;
    float imu_roll  = (target_roll - curr_roll) * Kp_IMU_Roll;

    // Combine adjustments
    float total_pitch = cmd_pitch + imu_pitch;
    float total_roll  = cmd_roll + imu_roll;

    // 3. Calculate Targets for each leg
    float targetFL = BASE_ANGLE_FRONT - total_pitch - total_roll;
    float targetFR = BASE_ANGLE_FRONT - total_pitch + total_roll;
    float targetBL = BASE_ANGLE_BACK  + total_pitch - total_roll;
    float targetBR = BASE_ANGLE_BACK  + total_pitch + total_roll;

    // 4. Independent Local Suspension (ToF Proactive + Load Cell Reactive)
    if (local_sensors_enabled) {
        for (int i = 0; i < 4; i++) {
            float tof_offset = 0;
            float load_offset = 0;

            // Proactive Terrain Mapping (ToF)
            float tof_diff = BASE_TOF[i] - curr_tof[i];
            if (tof_diff > TOF_DEADBAND) { 
                // Distance decreased -> Bump approaching -> Retract leg (Positive offset)
                tof_offset = (tof_diff - TOF_DEADBAND) * Kp_ToF_Terrain;
            } else if (tof_diff < -TOF_DEADBAND) {
                // Distance increased -> Hole approaching -> Extend leg (Negative offset)
                tof_offset = (tof_diff + TOF_DEADBAND) * Kp_ToF_Terrain;
            }

            // Reactive Shock Absorption (Load Cells)
            if (curr_load[i] < -LOAD_DEADBAND) { 
                // Weight lost -> Wheel fell into a hole -> Extend leg downwards (Negative offset)
                load_offset = -abs(curr_load[i] + LOAD_DEADBAND) * Kp_Load_Extend;
            } else if (curr_load[i] > LOAD_DEADBAND) { 
                // Weight spiked -> Wheel hit a bump -> Retract leg to absorb impact (Positive offset)
                load_offset = abs(curr_load[i] - LOAD_DEADBAND) * Kp_Load_Retract;
            }

            float local_offset = constrain(tof_offset + load_offset, -MAX_LOCAL_OFFSET, MAX_LOCAL_OFFSET);

            if (i == 0) targetFL += local_offset;
            if (i == 1) targetFR += local_offset;
            if (i == 2) targetBL += local_offset;
            if (i == 3) targetBR += local_offset;
        }
    }

    // 4. Constrain absolute physical limits to prevent self-destruction
    targetFL = constrain(targetFL, 10, 80);
    targetFR = constrain(targetFR, 10, 80);
    targetBL = constrain(targetBL, 20, 90);
    targetBR = constrain(targetBR, 20, 90);

    // 5. Apply Slew Rate Limit (Smooth, non-blocking transition)
    currentFL = stepTowards(currentFL, targetFL, MAX_ANGLE_STEP);
    currentFR = stepTowards(currentFR, targetFR, MAX_ANGLE_STEP);
    currentBL = stepTowards(currentBL, targetBL, MAX_ANGLE_STEP);
    currentBR = stepTowards(currentBR, targetBR, MAX_ANGLE_STEP);

    // 6. Write to Hardware
    fl.write(180 - currentFL);
    fr.write(currentFR);
    bl.write(currentBL);
    br.write(180 - currentBR);
}

// ==========================================
// 8. SETUP & CALIBRATION
// ==========================================
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN); 
    // Reduced to 100kHz for stability over long wires to ToF sensors
    Wire.setClock(100000); 
    delay(1000);
    
    Serial.println("System Booting... Independent Active Suspension");

    // --- Servos Initialization ---
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    
    fl.setPeriodHertz(50); fr.setPeriodHertz(50); bl.setPeriodHertz(50); br.setPeriodHertz(50);
    
    Serial.println("Moving actuators to HOME position before IMU calibration...");
    
    // Staggered Startup to prevent ESP32 Brownout
    fl.attach(PIN_FL_SERVO);
    fl.write(180 - BASE_ANGLE_FRONT);
    delay(250); 
    
    fr.attach(PIN_FR_SERVO);
    fr.write(BASE_ANGLE_FRONT);
    delay(250);
    
    bl.attach(PIN_BL_SERVO);
    bl.write(BASE_ANGLE_BACK);
    delay(250);
    
    br.attach(PIN_BR_SERVO);
    br.write(180 - BASE_ANGLE_BACK);
    
    delay(1250); 

    // --- Drive Motor (L298N) Initialization ---
    Serial.println("Initializing Drive Motor (Kept STOPPED during setup)...");
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    
    ledcAttach(MOTOR_EN, pwmFreq, pwmResolution);
    ledcWrite(MOTOR_EN, 0); 

    // --- IMU ---
    tcaSelect(4);
    if (mpu.begin()) {
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu_ready = true;
        Serial.println("Calibrating IMU Baseline... Keep platform completely flat!");
        float p_sum = 0, r_sum = 0;
        for(int i=0; i<100; i++) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            r_sum += (-atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI);
            p_sum += (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI);
            delay(5);
        }
        IMU_PITCH_OFFSET = p_sum / 100.0;
        IMU_ROLL_OFFSET = r_sum / 100.0;
        
        pitch_avg.init(0.0);
        roll_avg.init(0.0);
    } else {
        Serial.println("IMU: FAILED TO INITIALIZE");
    }

    // --- ToF Sensors ---
    Serial.println("Initializing ToF Sensors (HIGH SPEED MODE)...");
    tcaSelect(0); 
    if (tofFL.begin()) { 
        tofFL.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED); 
        tof_ready[0] = true; 
        Serial.println("ToF FL: OK"); 
    } else { Serial.println("ToF FL: FAILED"); }
    
    tcaSelect(3); 
    if (tofFR.begin()) { 
        tofFR.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED); 
        tof_ready[1] = true; 
        Serial.println("ToF FR: OK"); 
    } else { Serial.println("ToF FR: FAILED"); }
    
    tcaSelect(1); 
    if (tofBL.begin()) { 
        tofBL.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED); 
        tof_ready[2] = true; 
        Serial.println("ToF BL: OK"); 
    } else { Serial.println("ToF BL: FAILED"); }
    
    tcaSelect(2); 
    if (tofBR.begin()) { 
        tofBR.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED); 
        tof_ready[3] = true; 
        Serial.println("ToF BR: OK"); 
    } else { Serial.println("ToF BR: FAILED"); }
    
    Serial.println("Calibrating ToF Baselines...");
    VL53L0X_RangingMeasurementData_t measure;
    for(int i=0; i<4; i++) {
        if(!tof_ready[i]) continue;
        uint8_t ch = (i==0)?0:(i==1)?3:(i==2)?1:2; 
        tcaSelect(ch);
        
        long sum = 0; int valid = 0;
        for(int j=0; j<20; j++) {
            if(i==0) tofFL.rangingTest(&measure, false);
            else if(i==1) tofFR.rangingTest(&measure, false);
            else if(i==2) tofBL.rangingTest(&measure, false);
            else if(i==3) tofBR.rangingTest(&measure, false);
            
            if(measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) { 
                sum += measure.RangeMilliMeter; valid++; 
            }
        }
        BASE_TOF[i] = (valid > 0) ? (sum / valid) : 100; 
        tof_avg[i].init(BASE_TOF[i]);
        curr_tof[i] = BASE_TOF[i];
    }

    // --- Load Cells ---
    Serial.println("Taring Load Cells...");
    scaleFL.begin(FL_DT, FL_SCK); scaleFR.begin(FR_DT, FR_SCK);
    scaleBL.begin(BL_DT, BL_SCK); scaleBR.begin(BR_DT, BR_SCK);
    
    scaleFL.set_scale(1000.0); scaleFR.set_scale(-1000.0);
    scaleBL.set_scale(-1000.0); scaleBR.set_scale(1000.0);
    
    delay(500); 

    bool lc_ready = true;
    if(scaleFL.is_ready()) { scaleFL.tare(20); } else { lc_ready = false; Serial.println("Load FL: FAILED"); }
    if(scaleFR.is_ready()) { scaleFR.tare(20); } else { lc_ready = false; Serial.println("Load FR: FAILED"); }
    if(scaleBL.is_ready()) { scaleBL.tare(20); } else { lc_ready = false; Serial.println("Load BL: FAILED"); }
    if(scaleBR.is_ready()) { scaleBR.tare(20); } else { lc_ready = false; Serial.println("Load BR: FAILED"); }
    
    for(int i=0; i<4; i++) {
        load_avg[i].init(0.0);
        curr_load[i] = 0.0;
    }

    // --- INTEGRITY CHECK & MOTOR START ---
    Serial.println("--- SYSTEM INTEGRITY CHECK ---");
    bool all_tofs_intact = tof_ready[0] && tof_ready[1] && tof_ready[2] && tof_ready[3];
    
    bool FORCE_MOTOR_RUN = true; 
    
    if ((mpu_ready && all_tofs_intact && lc_ready) || FORCE_MOTOR_RUN) {
        Serial.println("STATUS: INTACT (Or Forced) - Starting Drive Motor...");
        
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        
        ledcWrite(MOTOR_EN, 150); 
    } else {
        Serial.println("STATUS: DEGRADED - One or more sensors failed to initialize.");
        Serial.println("SAFETY LOCKOUT: Drive Motor will remain STOPPED. (Set FORCE_MOTOR_RUN=true to bypass)");
        digitalWrite(MOTOR_IN1, LOW); 
        digitalWrite(MOTOR_IN2, LOW); 
        ledcWrite(MOTOR_EN, 0); 
    }

    Serial.println("--- SYSTEM ONLINE ---");
    Serial.println("Commands: 'PITCH <deg>', 'ROLL <deg>', 'HOME', 'LOCAL OFF', 'LOCAL ON', 'PLOT ON', 'PLOT OFF'");
}

// ==========================================
// 9. MAIN FAST LOOP
// ==========================================
void loop() {
    unsigned long currentMillis = millis();

    // 1. Handle Serial Commands (Non-Blocking)
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        String upperInput = input;
        upperInput.toUpperCase();
        
        if (upperInput == "HOME") { 
            target_pitch = 0.0;
            target_roll = 0.0;
            if(!plotter_mode) Serial.println("OK: HOME (Target Pitch=0, Target Roll=0)"); 
        }
        else if (upperInput.startsWith("PITCH ")) {
            target_pitch = input.substring(6).toFloat();
            if(!plotter_mode) { Serial.print("OK: Commanded Geometric Pitch = "); Serial.println(target_pitch); }
        }
        else if (upperInput.startsWith("ROLL ")) {
            target_roll = input.substring(5).toFloat();
            if(!plotter_mode) { Serial.print("OK: Commanded Geometric Roll = "); Serial.println(target_roll); }
        }
        else if (upperInput == "LOCAL OFF") {
            local_sensors_enabled = false;
            if(!plotter_mode) Serial.println("OK: ToF & Load Cell Overrides DISABLED");
        }
        else if (upperInput == "LOCAL ON") {
            local_sensors_enabled = true;
            if(!plotter_mode) Serial.println("OK: ToF & Load Cell Overrides ENABLED");
        }
        else if (upperInput == "PLOT ON") {
            plotter_mode = true;
        }
        else if (upperInput == "PLOT OFF") {
            plotter_mode = false;
            Serial.println("OK: Plotter Mode DISABLED");
        }
    }

    // 2. Update Sensors (For Observation and Overrides)
    updateIMU();
    updateToFRoundRobin();
    updateLoadCells();

    // 3. Process Kinematics & Update Servos (~66Hz loop)
    if (currentMillis - lastControlTime >= 15) {
        lastControlTime = currentMillis;
        processKinematics();
    }

    // 4. Debug Diagnostics / Serial Plotter
    unsigned long diag_interval = plotter_mode ? PLOT_INTERVAL : DIAG_INTERVAL;
    if (currentMillis - lastDiagTime >= diag_interval) {
        lastDiagTime = currentMillis;

        if (plotter_mode) {
            float norm_tof_FL = BASE_TOF[0] - curr_tof[0];
            float norm_tof_FR = BASE_TOF[1] - curr_tof[1];
            float norm_tof_BL = BASE_TOF[2] - curr_tof[2];
            float norm_tof_BR = BASE_TOF[3] - curr_tof[3];

            Serial.print("IMU_Pitch:"); Serial.print(curr_pitch, 2); Serial.print(",");
            Serial.print("IMU_Roll:"); Serial.print(curr_roll, 2); Serial.print(",");
            Serial.print("ToF_FL:"); Serial.print(norm_tof_FL, 2); Serial.print(",");
            Serial.print("ToF_FR:"); Serial.print(norm_tof_FR, 2); Serial.print(",");
            Serial.print("ToF_BL:"); Serial.print(norm_tof_BL, 2); Serial.print(",");
            Serial.print("ToF_BR:"); Serial.print(norm_tof_BR, 2); Serial.print(",");
            Serial.print("Load_FL:"); Serial.print(curr_load[0], 2); Serial.print(",");
            Serial.print("Load_FR:"); Serial.print(curr_load[1], 2); Serial.print(",");
            Serial.print("Load_BL:"); Serial.print(curr_load[2], 2); Serial.print(",");
            Serial.print("Load_BR:"); Serial.println(curr_load[3], 2);
        } else {
            Serial.print("Target [P: "); Serial.print(target_pitch, 1);
            Serial.print(" | R: "); Serial.print(target_roll, 1);
            Serial.print("]   ||   Actual IMU [P: "); Serial.print(curr_pitch, 1);
            Serial.print(" | R: "); Serial.print(curr_roll, 1);
            Serial.print("]   ||   ToF(FL,FR,BL,BR): ["); 
            Serial.print(curr_tof[0], 0); Serial.print(","); Serial.print(curr_tof[1], 0); Serial.print(",");
            Serial.print(curr_tof[2], 0); Serial.print(","); Serial.print(curr_tof[3], 0);
            Serial.print("]   ||   Load(FL,FR,BL,BR): ["); 
            Serial.print(curr_load[0], 1); Serial.print(","); Serial.print(curr_load[1], 1); Serial.print(",");
            Serial.print(curr_load[2], 1); Serial.print(","); Serial.print(curr_load[3], 1); Serial.println("]");
        }
    }
}