#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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

const int pwmFreq      = 5000;
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

bool tofFL_ready = false;
bool tofFR_ready = false;
bool tofBL_ready = false;
bool tofBR_ready = false;
bool mpu_ready   = false;

// Motor persistent speed — reapplied every loop to prevent PWM dropout
int currentMotorSpeed = 0;

// Variables for Non-Blocking State Machine
unsigned long lastToFPrintTime = 0;
unsigned long mux_switch_time  = 0;
uint8_t tof_read_state = 0;
int distFL = -1, distFR = -1, distBL = -1, distBR = -1;
float curr_pitch = 0.0, curr_roll = 0.0;

// Baseline ToF Measurements
int baseDistFL = 0;
int baseDistFR = 0;
int baseDistBL = 0;
int baseDistBR = 0;

// ToF Rolling Averagers (Window of 5 readings)
SuccessiveAverage<5> avgToF[4];

// Baseline IMU Measurements
float basePitch      = 0.0;
float baseRoll       = 0.0;
float gyroBiasPitch  = 0.0;
float gyroBiasRoll   = 0.0;

// Complementary Filter Variables
float comp_pitch     = 0.0;
float comp_roll      = 0.0;
unsigned long last_imu_time = 0;

// Targeted Active Suspension Configuration
float TOF_DEADBAND = 2.0;   // mm deadband to detect terrain anomaly
float IMU_DEADBAND = 0.5;   // Degrees of tilt to trigger leveling
float MAX_OFFSET   = 45.0;  // Maximum servo correction (degrees)
int   baseHeight   = 45;    // Central ride height (degrees)
bool  auto_level   = true;  // Toggle targeted leveling feature

// Incremental PID Gains (tune these)
// Kp: proportional — how hard to push per degree of tilt
// Ki: integral     — removes steady-state error over time
// Kd: derivative   — damps oscillation by resisting rapid change
float PID_KP = 5.5;
float PID_KI = 0.05;
float PID_KD = 0.8;

// Per-leg incremental PID state
// Each leg tracks its own accumulated output and previous errors
// so corrections are independent and don't fight each other
struct LegPID {
    float output;       // Accumulated servo offset (degrees)
    float prev_err;     // Error from last cycle (for D term)
    float integral;     // Accumulated error (for I term, with clamping)
    bool  responsible;  // Is this leg currently on uneven terrain?

    LegPID() : output(0), prev_err(0), integral(0), responsible(false) {}

    // Incremental PID: computes delta output each cycle.
    // sign_p / sign_r: +1 or -1 — which direction pitch/roll error
    //                  should drive this leg
    float update(float pitch_err, float roll_err, float dt,
                 float sign_p, float sign_r,
                 float kp, float ki, float kd, float max_out) {

        if (!responsible) {
            // Smoothly decay offset back to zero when on flat ground
            output   *= 0.92f;
            integral *= 0.92f;
            prev_err  = 0;
            return output;
        }

        // Combined error for this leg from both pitch and roll axes
        float err = (sign_p * pitch_err) + (sign_r * roll_err);

        // Incremental (velocity-form) PID:
        // delta = Kp*(e - e_prev) + Ki*e*dt + Kd*(e - 2*e_prev + e_pprev)/dt
        // Simplified here as standard positional PID on the combined error,
        // but applied as an increment to avoid step changes on gain tuning.
        integral += err * dt;
        integral  = constrain(integral, -max_out / ki, max_out / ki); // Anti-windup

        float derivative = (err - prev_err) / dt;
        float delta      = kp * err + ki * integral + kd * derivative;

        output   += delta * dt;  // Incremental: add delta scaled by dt
        output    = constrain(output, -max_out, max_out);
        prev_err  = err;

        return output;
    }

    void reset() {
        output = 0; prev_err = 0; integral = 0; responsible = false;
    }
};

LegPID pidFL, pidFR, pidBL, pidBR;

// Convenience accessors for plotter / serial output
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

// ---- Motor ----
void setMotorSpeed(int speed) {
    currentMotorSpeed = constrain(speed, 0, 255);
    ledcWrite(MOTOR_EN, currentMotorSpeed);

    // Auto-leveling follows motor state
    if (currentMotorSpeed > 0) {
        auto_level = true;
        Serial.printf("DRIVE MOTOR -> %d | Auto-Leveling ENABLED\n", currentMotorSpeed);
    } else {
        auto_level = false;
        Serial.printf("DRIVE MOTOR -> 0 | Auto-Leveling DISABLED\n");
    }
}

// ---- Servos ----
void setHeight(int h) {
    baseHeight = constrain(h, 0, 180);

    // Reset offsets when manually changing height
    offsetFL = 0; offsetFR = 0; offsetBL = 0; offsetBR = 0;

    servoFL.write(baseHeight);
    servoFR.write(180 - baseHeight);
    servoBL.write(180 - baseHeight);
    servoBR.write(baseHeight);

    Serial.printf("BASE HEIGHT SET TO %d -> FL:%d FR:%d BL:%d BR:%d\n",
                  baseHeight, baseHeight, 180 - baseHeight, 180 - baseHeight, baseHeight);
}

void setSingleServo(String cmd) {
    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, 0, 180);
    String id = cmd.substring(0, 2);

    if      (id == "FL") { servoFL.write(angle);         Serial.printf("FL -> %d\n", angle); }
    else if (id == "FR") { servoFR.write(angle);         Serial.printf("FR -> %d\n", angle); }
    else if (id == "BL") { servoBL.write(angle);         Serial.printf("BL -> %d\n", angle); }
    else if (id == "BR") { servoBR.write(angle);         Serial.printf("BR -> %d\n", angle); }
    else                 { Serial.println("Invalid servo command"); }
}

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
    Serial.begin(115200);

    // Start I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // ---- Servos first (they claim timers 0-3) ----
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servoFL.attach(PIN_FL_SERVO, 500, 2400);
    servoFR.attach(PIN_FR_SERVO, 500, 2400);
    servoBL.attach(PIN_BL_SERVO, 500, 2400);
    servoBR.attach(PIN_BR_SERVO, 500, 2400);

    // ---- Motor AFTER servos so LEDC channel allocation doesn't conflict ----
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);  // Reversed direction
    digitalWrite(MOTOR_IN2, HIGH);

    ledcAttach(MOTOR_EN, pwmFreq, pwmResolution);
    ledcWrite(MOTOR_EN, 0); // Start stopped

    // ---- ToF Sensors ----
    Serial.println("Initializing ToF sensors...");

    tcaSelect(0); delay(10);
    if (pingI2C(0x29) && tofFL.begin()) { tofFL_ready = true; Serial.println("ToF FL: OK"); }
    else { Serial.println("ToF FL: FAILED"); }

    tcaSelect(3); delay(10);
    if (pingI2C(0x29) && tofFR.begin()) { tofFR_ready = true; Serial.println("ToF FR: OK"); }
    else { Serial.println("ToF FR: FAILED"); }

    tcaSelect(1); delay(10);
    if (pingI2C(0x29) && tofBL.begin()) { tofBL_ready = true; Serial.println("ToF BL: OK"); }
    else { Serial.println("ToF BL: FAILED"); }

    tcaSelect(2); delay(10);
    if (pingI2C(0x29) && tofBR.begin()) { tofBR_ready = true; Serial.println("ToF BR: OK"); }
    else { Serial.println("ToF BR: FAILED"); }

    // ---- IMU ----
    Serial.println("Initializing IMU...");
    tcaSelect(4); delay(10);
    if (pingI2C(0x68) && mpu.begin()) {
        mpu_ready = true;
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("IMU: OK");
    } else {
        Serial.println("IMU: FAILED");
    }

    // Home position
    setHeight(baseHeight);
    Serial.println("Waiting 1.5s for servos to reach home position...");
    delay(1500);

    // ---- Calibrate ToF Baselines ----
    Serial.println("Calibrating ToF baselines (averaging 10 readings)...");
    long sumFL = 0, sumFR = 0, sumBL = 0, sumBR = 0;
    int  cntFL = 0, cntFR = 0, cntBL = 0, cntBR = 0;

    for (int i = 0; i < 10; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        if (tofFL_ready) { tcaSelect(0); delay(5); tofFL.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumFL += measure.RangeMilliMeter; cntFL++; } }
        if (tofFR_ready) { tcaSelect(3); delay(5); tofFR.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumFR += measure.RangeMilliMeter; cntFR++; } }
        if (tofBL_ready) { tcaSelect(1); delay(5); tofBL.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumBL += measure.RangeMilliMeter; cntBL++; } }
        if (tofBR_ready) { tcaSelect(2); delay(5); tofBR.rangingTest(&measure, false); if (measure.RangeStatus != 4) { sumBR += measure.RangeMilliMeter; cntBR++; } }
    }

    if (cntFL > 0) { baseDistFL = sumFL / cntFL; avgToF[0].init(baseDistFL); }
    if (cntFR > 0) { baseDistFR = sumFR / cntFR; avgToF[1].init(baseDistFR); }
    if (cntBL > 0) { baseDistBL = sumBL / cntBL; avgToF[2].init(baseDistBL); }
    if (cntBR > 0) { baseDistBR = sumBR / cntBR; avgToF[3].init(baseDistBR); }

    Serial.printf("Baselines (mm) -> FL:%d | FR:%d | BL:%d | BR:%d\n",
                  baseDistFL, baseDistFR, baseDistBL, baseDistBR);

    // ---- Calibrate IMU Baselines ----
    Serial.println("Calibrating IMU baselines & gyro bias (averaging 50 readings)...");
    if (mpu_ready) {
        tcaSelect(4); delay(5);
        float p_sum = 0, r_sum = 0, gp_sum = 0, gr_sum = 0;

        for (int i = 0; i < 50; i++) {
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

        Serial.printf("IMU Baselines -> P:%.1f | R:%.1f\n", basePitch, baseRoll);
        Serial.printf("Gyro Bias -> P_Rate:%.2f | R_Rate:%.2f\n", gyroBiasPitch, gyroBiasRoll);
    }

    Serial.println("\nREADY");
    Serial.println("Commands:");
    Serial.println("  H0-H180       : height control");
    Serial.println("  FL90 FR90 ... : individual servo");
    Serial.println("  M0-M255       : motor speed");
    Serial.println("  AUTO ON/OFF   : toggle active leveling");
    Serial.println("-------------------------------------------");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
void loop() {
    unsigned long current_time = millis();

    // --- 1. Serial Commands ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if      (cmd == "AUTO ON")        { auto_level = true;  Serial.println("Targeted Leveling ENABLED"); }
        else if (cmd == "AUTO OFF")       { auto_level = false; Serial.println("Targeted Leveling DISABLED"); }
        else if (cmd.startsWith("M"))     { setMotorSpeed(cmd.substring(1).toInt()); }
        else if (cmd.startsWith("H"))     { setHeight(cmd.substring(1).toInt()); }
        else if (cmd.length() >= 3)       { setSingleServo(cmd); }
    }

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

                // Incremental PID per leg.
                // Sign convention: (+pitch, -roll) means front-left corner is high.
                // Each leg's sign pair determines how pitch/roll error drives it:
                //   FL: +pitch raises front → push FL down (+p), roll left lowers FL → push FL up (-r)
                //   FR: +pitch raises front → push FR down (+p), roll right raises FR → push FR down (+r)
                //   BL: +pitch lowers rear  → push BL up  (-p), roll left  raises BL  → push BL up  (+r) -- wait, inverted: (-p,-r)
                //   BR: +pitch lowers rear  → push BR up  (-p), roll right lowers BR  → push BR down(-r) -- (+r) for BR
                // Simplified: FL(+p,-r)  FR(+p,+r)  BL(-p,-r)  BR(-p,+r)
                pidFL.update(pitch_err, roll_err, dt, +1, -1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidFR.update(pitch_err, roll_err, dt, +1, +1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidBL.update(pitch_err, roll_err, dt, -1, -1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);
                pidBR.update(pitch_err, roll_err, dt, -1, +1, PID_KP, PID_KI, PID_KD, MAX_OFFSET);

                // Apply to servos
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
            }

            float pitch_err = curr_pitch - basePitch;
            float roll_err  = curr_roll  - baseRoll;

            int normFL = (distFL != -1) ? (distFL - baseDistFL) : 0;
            int normFR = (distFR != -1) ? (distFR - baseDistFR) : 0;
            int normBL = (distBL != -1) ? (distBL - baseDistBL) : 0;
            int normBR = (distBR != -1) ? (distBR - baseDistBR) : 0;

            // Tag responsible legs based on tilt + ToF confirmation
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

            // Revoke responsibility once terrain returns to flat
            if (abs(normFL) <= TOF_DEADBAND) respFL = false;
            if (abs(normFR) <= TOF_DEADBAND) respFR = false;
            if (abs(normBL) <= TOF_DEADBAND) respBL = false;
            if (abs(normBR) <= TOF_DEADBAND) respBR = false;

            // Arduino Serial Plotter: Label:value pairs on one line
            // Pitch/Roll   = error from baseline (degrees)
            // ToF_xx       = distance delta from baseline (mm); positive = obstacle raised that corner
            // Offset_xx    = servo correction angle applied to that leg (degrees)
            Serial.printf("Pitch:%.2f Roll:%.2f ToF_FL:%d ToF_FR:%d ToF_BL:%d ToF_BR:%d Offset_FL:%.1f Offset_FR:%.1f Offset_BL:%.1f Offset_BR:%.1f\n",
                          pitch_err, roll_err,
                          normFL, normFR, normBL, normBR,
                          offsetFL, offsetFR, offsetBL, offsetBR);

            tof_read_state = 0;
        }
    }

    // --- 4. Persist motor PWM every loop iteration ---
    // Prevents PWM dropout caused by I2C bus activity or servo timer interference
    ledcWrite(MOTOR_EN, currentMotorSpeed);
}