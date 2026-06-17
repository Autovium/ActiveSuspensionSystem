#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

// ==========================================
// 0. SUCCESSIVE AVERAGE FILTER
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
        sum -= readings[index];
        readings[index] = val;
        sum += val;
        index = (index + 1) % N;
        if (count < N) count++;
        current_average = sum / count;
        return current_average;
    }
};

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
// Servo pins
#define PIN_FL_SERVO 12
#define PIN_FR_SERVO 42
#define PIN_BL_SERVO 47
#define PIN_BR_SERVO 21

// Drive Motor Pins
#define MOTOR_IN1 13
#define MOTOR_IN2 14
#define MOTOR_EN  11

// Mode Input Pins
#define PIN_MODE_40 40
#define PIN_MODE_41 41

// NeoPixel LED Pin 
#define PIN_NEOPIXEL 48 
#define NUMPIXELS    1

const int pwmFreq     = 5000;
const int pwmResolution = 8;

// I2C & Multiplexer Pins
#define SDA_PIN  1
#define SCL_PIN  2
#define TCAADDR  0x70

// ==========================================
// 2. GLOBAL OBJECTS & STATE
// ==========================================
Servo servoFL;
Servo servoFR;
Servo servoBL;
Servo servoBR;

Adafruit_VL53L0X tofFL = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFR = Adafruit_VL53L0X();
Adafruit_VL53L0X tofBL = Adafruit_VL53L0X();
Adafruit_VL53L0X tofBR = Adafruit_VL53L0X();

Adafruit_MPU6050 mpu;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool tofFL_ready = false;
bool tofFR_ready = false;
bool tofBL_ready = false;
bool tofBR_ready = false;
bool mpu_ready   = false;

int currentMotorSpeed = 0;
String currentMode = ""; // Start empty to force state initialization on first loop

// Variables for Non-Blocking State Machine
unsigned long lastToFPrintTime = 0;
unsigned long mux_switch_time  = 0;
uint8_t tof_read_state = 0;
int distFL = -1, distFR = -1, distBL = -1, distBR = -1;
float curr_pitch = 0.0, curr_roll = 0.0;

// Persistent Deviation Tracking (Fixes dropped frame cancellations)
int normFL = 0, normFR = 0, normBL = 0, normBR = 0;

// Baseline Measurements
int baseDistFL = 0, baseDistFR = 0, baseDistBL = 0, baseDistBR = 0;
SuccessiveAverage<5> avgToF[4];

float basePitch = 0.0, baseRoll = 0.0;
float gyroBiasPitch = 0.0, gyroBiasRoll = 0.0;
float comp_pitch = 0.0, comp_roll = 0.0;
unsigned long last_imu_time = 0;

float TOF_DEADBAND = 2.0;
float IMU_DEADBAND = 0.5;
float MAX_OFFSET   = 45.0;
int   baseHeight   = 45;
bool  auto_level   = false;

float PID_KP = 7.5, PID_KI = 0.1, PID_KD = 1;

struct LegPID {
    float output;
    float prev_err;
    float integral;
    bool  responsible;

    LegPID() : output(0), prev_err(0), integral(0), responsible(false) {}

    float update(float pitch_err, float roll_err, float dt, float sign_p, float sign_r, float kp, float ki, float kd, float max_out) {
        if (!responsible) {
            output   *= 0.92f;
            integral *= 0.92f;
            prev_err  = 0;
            return output;
        }
        float err = (sign_p * pitch_err) + (sign_r * roll_err);
        integral += err * dt;
        integral  = constrain(integral, -max_out / ki, max_out / ki);
        float derivative = (err - prev_err) / dt;
        float delta      = kp * err + ki * integral + kd * derivative;
        output   += delta * dt;
        output    = constrain(output, -max_out, max_out);
        prev_err  = err;
        return output;
    }
};

LegPID pidFL, pidFR, pidBL, pidBR;

#define offsetFL pidFL.output
#define offsetFR pidFR.output
#define offsetBL pidBL.output
#define offsetBR pidBR.output
#define respFL   pidFL.responsible
#define respFR   pidFR.responsible
#define respBL   pidBL.responsible
#define respBR   pidBR.responsible

// ==========================================
// 3. HARDWARE HELPERS
// ==========================================
void tcaSelect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission(true);
}

bool pingI2C(uint8_t addr) {
    for (int i = 0; i < 3; i++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) return true;
        delay(5);
    }
    return false;
}

void setMotorMode(int speed, String modeName, bool enableLeveling, uint32_t color) {
    if (currentMode != modeName) {
        currentMotorSpeed = constrain(speed, 0, 255);
        currentMode = modeName;
        auto_level = enableLeveling;
        
        ledcWrite(MOTOR_EN, currentMotorSpeed);
        
        pixels.setPixelColor(0, color);
        pixels.show();
    }
}

void setHeight(int h) {
    baseHeight = constrain(h, 0, 180);
    offsetFL = 0; offsetFR = 0; offsetBL = 0; offsetBR = 0;
    servoFL.write(baseHeight);
    servoFR.write(180 - baseHeight);
    servoBL.write(180 - baseHeight);
    servoBR.write(baseHeight);
}

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
    Serial.begin(115200);

    // Initialize NeoPixel
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
    pixels.show();

    // Hardware Pins
    pinMode(PIN_MODE_40, INPUT_PULLDOWN);
    pinMode(PIN_MODE_41, INPUT_PULLDOWN);

    // Start I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(150); 

    // ---------------------------------------------------------
    // TIMER FIX: Allocate only Timer 0 and 1 to the Servos.
    // ESP32Servo will automatically share Timer 0 for FL/FR and 
    // Timer 1 for BL/BR. This prevents timer exhaustion and 
    // guarantees free timers remain for the Drive Motor.
    // ---------------------------------------------------------
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    servoFL.setPeriodHertz(50); // Set standard 50Hz explicitly for sharing
    servoFR.setPeriodHertz(50);
    servoBL.setPeriodHertz(50);
    servoBR.setPeriodHertz(50);

    servoFL.attach(PIN_FL_SERVO, 500, 2400);
    servoFR.attach(PIN_FR_SERVO, 500, 2400);
    servoBL.attach(PIN_BL_SERVO, 500, 2400);
    servoBR.attach(PIN_BR_SERVO, 500, 2400);

    // Motor Initialization
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

    ledcAttach(MOTOR_EN, pwmFreq, pwmResolution);
    ledcWrite(MOTOR_EN, 0);

    // ToF Sensors - Detailed Output
    Serial.println("\n--- Initializing ToF sensors ---");
    
    Serial.print("Checking ToF FL (Port 0)... ");
    tcaSelect(0); delay(10); 
    if (pingI2C(0x29) && tofFL.begin()) { tofFL_ready = true; Serial.println("OK"); } else { Serial.println("FAILED"); }
    
    Serial.print("Checking ToF FR (Port 3)... ");
    tcaSelect(3); delay(10); 
    if (pingI2C(0x29) && tofFR.begin()) { tofFR_ready = true; Serial.println("OK"); } else { Serial.println("FAILED"); }
    
    Serial.print("Checking ToF BL (Port 1)... ");
    tcaSelect(1); delay(10); 
    if (pingI2C(0x29) && tofBL.begin()) { tofBL_ready = true; Serial.println("OK"); } else { Serial.println("FAILED"); }
    
    Serial.print("Checking ToF BR (Port 2)... ");
    tcaSelect(2); delay(10); 
    if (pingI2C(0x29) && tofBR.begin()) { tofBR_ready = true; Serial.println("OK"); } else { Serial.println("FAILED"); }

    // IMU
    Serial.println("\n--- Initializing IMU ---");
    Serial.print("Checking MPU6050 (Port 4)... ");
    tcaSelect(4); delay(10);
    if (pingI2C(0x68) && mpu.begin()) {
        mpu_ready = true;
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    setHeight(baseHeight);
    Serial.println("\nWaiting 1.5s for servos to home...");
    delay(1500);

    // Calibrate ToF
    Serial.print("Calibrating ToF baselines");
    long sumFL = 0, sumFR = 0, sumBL = 0, sumBR = 0;
    int  cntFL = 0, cntFR = 0, cntBL = 0, cntBR = 0;
    for (int i = 0; i < 10; i++) {
        Serial.print("."); 
        VL53L0X_RangingMeasurementData_t measure;
        if (tofFL_ready) { tcaSelect(0); delay(5); tofFL.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumFL += measure.RangeMilliMeter; cntFL++; } }
        if (tofFR_ready) { tcaSelect(3); delay(5); tofFR.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumFR += measure.RangeMilliMeter; cntFR++; } }
        if (tofBL_ready) { tcaSelect(1); delay(5); tofBL.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumBL += measure.RangeMilliMeter; cntBL++; } }
        if (tofBR_ready) { tcaSelect(2); delay(5); tofBR.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumBR += measure.RangeMilliMeter; cntBR++; } }
    }
    Serial.println(" Done!");
    
    if (cntFL > 0) { baseDistFL = sumFL / cntFL; avgToF[0].init(baseDistFL); }
    if (cntFR > 0) { baseDistFR = sumFR / cntFR; avgToF[1].init(baseDistFR); }
    if (cntBL > 0) { baseDistBL = sumBL / cntBL; avgToF[2].init(baseDistBL); }
    if (cntBR > 0) { baseDistBR = sumBR / cntBR; avgToF[3].init(baseDistBR); }

    // Calibrate IMU
    Serial.print("Calibrating IMU baselines");
    if (mpu_ready) {
        tcaSelect(4); delay(5);
        float p_sum = 0, r_sum = 0, gp_sum = 0, gr_sum = 0;
        for (int i = 0; i < 50; i++) {
            if (i % 5 == 0) Serial.print("."); 
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            r_sum  += (-atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI);
            p_sum  += (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI);
            gp_sum += (g.gyro.y * 180.0 / PI);
            gr_sum += (-g.gyro.x * 180.0 / PI);
            delay(10);
        }
        basePitch     = p_sum  / 50.0;
        baseRoll      = r_sum  / 50.0;
        gyroBiasPitch = gp_sum / 50.0;
        gyroBiasRoll  = gr_sum / 50.0;
        comp_pitch    = basePitch;
        comp_roll     = baseRoll;
        last_imu_time = millis();
        Serial.println(" Done!");
    } else {
        Serial.println(" Skipped (No IMU detected)");
    }
    
    Serial.println("\nSETUP COMPLETE - Entering Main Loop\n");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
void loop() {
    unsigned long current_time = millis();

    // --- 1. Calculate Motor Speeds & NeoPixel Colors based on Pins ---
    int pin40 = digitalRead(PIN_MODE_40);
    int pin41 = digitalRead(PIN_MODE_41);
    
    int hw_speed = 0;
    String hw_mode = "STOP";
    bool hw_auto = false;
    uint32_t hw_color = pixels.Color(255, 105, 180); // Pink (STOP default)

    if (pin40 == 1 && pin41 == 0) { 
        hw_speed = 255; hw_mode = "SMOOTH"; hw_auto = false; hw_color = pixels.Color(0, 255, 0); // Green
    }
    else if (pin40 == 0 && pin41 == 1) { 
        hw_speed = 200; hw_mode = "UNEVEN"; hw_auto = true; hw_color = pixels.Color(255, 255, 0); // Yellow
    }
    else if (pin40 == 1 && pin41 == 1) { 
        hw_speed = 150; hw_mode = "ROUGH"; hw_auto = true; hw_color = pixels.Color(255, 0, 0); // Red
    }
    else { 
        hw_speed = 0; hw_mode = "STOP"; hw_auto = false; hw_color = pixels.Color(255, 105, 180); // Pink
    }

    setMotorMode(hw_speed, hw_mode, hw_auto, hw_color);

    // --- 2. Fast IMU Polling (50 Hz) ---
    if (tof_read_state == 0 && (current_time - last_imu_time >= 20)) {
        float dt = (current_time - last_imu_time) / 1000.0;
        last_imu_time = current_time;

        if (mpu_ready) {
            tcaSelect(4);
            delayMicroseconds(500);

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            float acc_roll       = (-atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI);
            float acc_pitch      = (atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI);
            float gyro_pitch_rate = (g.gyro.y * 180.0 / PI) - gyroBiasPitch;
            float gyro_roll_rate  = (-g.gyro.x * 180.0 / PI) - gyroBiasRoll;

            const float alpha = 0.96;
            comp_pitch = alpha * (comp_pitch + gyro_pitch_rate * dt) + (1.0 - alpha) * acc_pitch;
            comp_roll  = alpha * (comp_roll  + gyro_roll_rate  * dt) + (1.0 - alpha) * acc_roll;

            curr_pitch = comp_pitch;
            curr_roll  = comp_roll;

            if (auto_level) {
                float pitch_err = curr_pitch - basePitch;
                float roll_err  = curr_roll  - baseRoll;

                pidFL.update(pitch_err, roll_err, dt, +1, -1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidFR.update(pitch_err, roll_err, dt, +1, +1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidBL.update(pitch_err, roll_err, dt, -1, -1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidBR.update(pitch_err, roll_err, dt, -1, +1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);

                servoFL.write(constrain((int)(baseHeight + pidFL.output), 0, 180));
                servoFR.write(constrain(180 - (int)(baseHeight + pidFR.output), 0, 180));
                servoBL.write(constrain(180 - (int)(baseHeight + pidBL.output), 0, 180));
                servoBR.write(constrain((int)(baseHeight + pidBR.output), 0, 180));
            }
        }
    }

    // --- 3. Non-Blocking ToF Polling (every 500 ms) ---
    if (tof_read_state == 0) {
        if (current_time - lastToFPrintTime > 500) {
            lastToFPrintTime = current_time;
            tcaSelect(0);
            mux_switch_time = current_time;
            tof_read_state  = 1;
        }
    }
    else if (tof_read_state == 1) {
        if (current_time - mux_switch_time >= 5) {
            if (tofFL_ready) {
                VL53L0X_RangingMeasurementData_t measure;
                tofFL.rangingTest(&measure, false);
                distFL = (measure.RangeStatus != 4) ? (int)avgToF[0].add(measure.RangeMilliMeter) : -1;
                if (distFL != -1) normFL = distFL - baseDistFL; // Update persistent cache
            }
            tcaSelect(3);
            mux_switch_time = millis();
            tof_read_state  = 2;
        }
    }
    else if (tof_read_state == 2) {
        if (current_time - mux_switch_time >= 5) {
            if (tofFR_ready) {
                VL53L0X_RangingMeasurementData_t measure;
                tofFR.rangingTest(&measure, false);
                distFR = (measure.RangeStatus != 4) ? (int)avgToF[1].add(measure.RangeMilliMeter) : -1;
                if (distFR != -1) normFR = distFR - baseDistFR; // Update persistent cache
            }
            tcaSelect(1);
            mux_switch_time = millis();
            tof_read_state  = 3;
        }
    }
    else if (tof_read_state == 3) {
        if (current_time - mux_switch_time >= 5) {
            if (tofBL_ready) {
                VL53L0X_RangingMeasurementData_t measure;
                tofBL.rangingTest(&measure, false);
                distBL = (measure.RangeStatus != 4) ? (int)avgToF[2].add(measure.RangeMilliMeter) : -1;
                if (distBL != -1) normBL = distBL - baseDistBL; // Update persistent cache
            }
            tcaSelect(2);
            mux_switch_time = millis();
            tof_read_state  = 4;
        }
    }
    else if (tof_read_state == 4) {
        if (current_time - mux_switch_time >= 5) {
            if (tofBR_ready) {
                VL53L0X_RangingMeasurementData_t measure;
                tofBR.rangingTest(&measure, false);
                distBR = (measure.RangeStatus != 4) ? (int)avgToF[3].add(measure.RangeMilliMeter) : -1;
                if (distBR != -1) normBR = distBR - baseDistBR; // Update persistent cache
            }

            float pitch_err = curr_pitch - basePitch;
            float roll_err  = curr_roll  - baseRoll;

            if (pitch_err > IMU_DEADBAND) {
                if (normFL >  TOF_DEADBAND) respFL = true;
                if (normFR >  TOF_DEADBAND) respFR = true;
                if (normBL < -TOF_DEADBAND) respBL = true;
                if (normBR < -TOF_DEADBAND) respBR = true;
            }
            if (pitch_err < -IMU_DEADBAND) {
                if (normFL < -TOF_DEADBAND) respFL = true;
                if (normFR < -TOF_DEADBAND) respFR = true;
                if (normBL >  TOF_DEADBAND) respBL = true;
                if (normBR >  TOF_DEADBAND) respBR = true;
            }
            if (roll_err > IMU_DEADBAND) {
                if (normFR >  TOF_DEADBAND) respFR = true;
                if (normBR >  TOF_DEADBAND) respBR = true;
                if (normFL < -TOF_DEADBAND) respFL = true;
                if (normBL < -TOF_DEADBAND) respBL = true;
            }
            if (roll_err < -IMU_DEADBAND) {
                if (normFR < -TOF_DEADBAND) respFR = true;
                if (normBR < -TOF_DEADBAND) respBR = true;
                if (normFL >  TOF_DEADBAND) respFL = true;
                if (normBL >  TOF_DEADBAND) respBL = true;
            }

            if (abs(normFL) <= TOF_DEADBAND) respFL = false;
            if (abs(normFR) <= TOF_DEADBAND) respFR = false;
            if (abs(normBL) <= TOF_DEADBAND) respBL = false;
            if (abs(normBR) <= TOF_DEADBAND) respBR = false;

            tof_read_state = 0;
        }
    }

    // Persist motor PWM every loop iteration
    ledcWrite(MOTOR_EN, currentMotorSpeed);

    // --- 4. Serial Plotter Output (20 Hz) ---
    static unsigned long lastPlotTime = 0;
    if (current_time - lastPlotTime >= 50) {
        lastPlotTime = current_time;

        float p_val = curr_pitch - basePitch;
        float r_val = curr_roll - baseRoll;

        // Prevent NaN errors from corrupting the graph
        if (isnan(p_val)) p_val = 0.0;
        if (isnan(r_val)) r_val = 0.0;

        Serial.printf("Pitch:%.2f Roll:%.2f ToF_FL:%d ToF_FR:%d ToF_BL:%d ToF_BR:%d Speed:%d RefHigh:100 RefLow:-100\n",
                      p_val, r_val, normFL, normFR, normBL, normBR, currentMotorSpeed);
    }
}