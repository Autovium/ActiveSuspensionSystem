#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
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
// Change this to match your board's built-in WS2812 pin (e.g., 48 on standard ESP32-S3, or 8 on C3)
#define PIN_NEOPIXEL 5 
#define NUMPIXELS    1

const int pwmFreq     = 5000;
const int pwmResolution = 8;

// I2C & Multiplexer Pins
#define SDA_PIN  1
#define SCL_PIN  2
#define TCAADDR  0x70

// ==========================================
// 2. WIFI & WEBSERVER
// ==========================================
const char* ssid = "HomeWiFi";
const char* password = "H0meW1F1";

// Static IP Configuration (Set an IP outside your router's normal DHCP range)
IPAddress local_IP(192, 168, 1, 200); 
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

bool web_override = false;
int web_speed = 0;

// HTML & JS Dashboard (Raw String Literal)
const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Active Suspension Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial, sans-serif; background-color: #121212; color: #fff; text-align: center; margin: 0; padding: 20px; }
    .container { max-width: 900px; margin: auto; }
    .panel { background: #1e1e1e; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    h1 { margin-top: 0; }
    button { padding: 10px 20px; margin: 5px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; color: white; background: #3a3a3a; transition: 0.3s; }
    button:hover { background: #555; }
    .btn-on { background: #4CAF50; }
    .btn-off { background: #f44336; }
    canvas { background: #2a2a2a; border-radius: 5px; padding: 10px; }
    .status-text { font-size: 1.2em; font-weight: bold; color: #00bcd4; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Suspension Telemetry</h1>
    
    <div class="panel">
      <h3>Controls <span id="status" class="status-text">(Hardware Mode)</span></h3>
      <button onclick="setOverride(1)" class="btn-on">Web Override ON</button>
      <button onclick="setOverride(0)" class="btn-off">Web Override OFF</button>
      <br><br>
      <div id="web-controls" style="opacity: 0.5; pointer-events: none;">
        <button onclick="setSpeed(0)">STOP (0)</button>
        <button onclick="setSpeed(255)">SMOOTH (255)</button>
        <button onclick="setSpeed(200)">UNEVEN (200)</button>
        <button onclick="setSpeed(150)">ROUGH (150)</button>
      </div>
      <p>Current Speed: <span id="speed-display">0</span></p>
    </div>

    <div class="panel">
      <h3>IMU: Pitch & Roll (Degrees)</h3>
      <canvas id="imuChart" height="100"></canvas>
    </div>

    <div class="panel">
      <h3>ToF: Corner Distances (mm deviation)</h3>
      <canvas id="tofChart" height="100"></canvas>
    </div>
  </div>

  <script>
    let overrideActive = false;

    function setOverride(state) {
      overrideActive = state === 1;
      document.getElementById('web-controls').style.opacity = overrideActive ? "1" : "0.5";
      document.getElementById('web-controls').style.pointerEvents = overrideActive ? "auto" : "none";
      document.getElementById('status').innerText = overrideActive ? "(Web Override Mode)" : "(Hardware Mode)";
      // Added &t= parameter as a cache-buster so browsers don't ignore repeated button clicks
      fetch('/control?override=' + state + '&t=' + Date.now()).catch(e => console.log(e));
    }

    function setSpeed(speed) {
      if(overrideActive) {
          // Added &t= parameter as a cache-buster
          fetch('/control?speed=' + speed + '&t=' + Date.now()).catch(e => console.log(e));
      }
    }

    // Chart Setup
    const commonOptions = {
        responsive: true,
        animation: false,
        scales: { x: { display: false }, y: { grid: { color: '#444' } } },
        plugins: { legend: { labels: { color: '#fff' } } }
    };

    const imuCtx = document.getElementById('imuChart').getContext('2d');
    const imuChart = new Chart(imuCtx, {
      type: 'line',
      data: { labels: [], datasets: [
        { label: 'Pitch', borderColor: '#ff5722', data: [], fill: false, tension: 0.2 },
        { label: 'Roll', borderColor: '#03a9f4', data: [], fill: false, tension: 0.2 }
      ]},
      options: commonOptions
    });

    const tofCtx = document.getElementById('tofChart').getContext('2d');
    const tofChart = new Chart(tofCtx, {
      type: 'line',
      data: { labels: [], datasets: [
        { label: 'FL', borderColor: '#e91e63', data: [], fill: false, tension: 0.2 },
        { label: 'FR', borderColor: '#9c27b0', data: [], fill: false, tension: 0.2 },
        { label: 'BL', borderColor: '#8bc34a', data: [], fill: false, tension: 0.2 },
        { label: 'BR', borderColor: '#ffc107', data: [], fill: false, tension: 0.2 }
      ]},
      options: commonOptions
    });

    let time = 0;
    
    // Use chained setTimeout instead of setInterval to prevent network flooding/lockup
    function fetchData() {
      fetch('/data')
        .then(res => {
            if (!res.ok) throw new Error("Bad network response");
            return res.json();
        })
        .then(data => {
            document.getElementById('speed-display').innerText = data.speed + " (" + data.mode + ")";
            
            imuChart.data.labels.push(time);
            imuChart.data.datasets[0].data.push(data.pitch);
            imuChart.data.datasets[1].data.push(data.roll);
            
            tofChart.data.labels.push(time);
            tofChart.data.datasets[0].data.push(data.tof_fl);
            tofChart.data.datasets[1].data.push(data.tof_fr);
            tofChart.data.datasets[2].data.push(data.tof_bl);
            tofChart.data.datasets[3].data.push(data.tof_br);

            if (imuChart.data.labels.length > 40) {
              imuChart.data.labels.shift();
              imuChart.data.datasets.forEach(d => d.data.shift());
              tofChart.data.labels.shift();
              tofChart.data.datasets.forEach(d => d.data.shift());
            }

            imuChart.update('none'); // Update without heavy animations
            tofChart.update('none');
            time++;
            
            // Queue next fetch only AFTER this one succeeds
            setTimeout(fetchData, 250); 
        })
        .catch(err => {
            console.log("Fetch error: ", err);
            // If error, wait slightly longer before retrying to prevent spamming the ESP32
            setTimeout(fetchData, 1000); 
        });
    }
    
    // Start the recursive fetching
    fetchData();
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 3. GLOBAL OBJECTS & STATE
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

float PID_KP = 5.5, PID_KI = 0.05, PID_KD = 0.8;

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
// 4. HARDWARE HELPERS
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

        Serial.printf("MODE -> %s | Speed: %d | Auto-Leveling: %s\n", 
                      modeName.c_str(), currentMotorSpeed, auto_level ? "ENABLED" : "DISABLED");
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
// 5. SETUP
// ==========================================
void setup() {
    Serial.begin(115200);

    // Initialize NeoPixel - Set to BLUE during Setup phase
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
    pixels.show();

    // Hardware Pins
    pinMode(PIN_MODE_40, INPUT_PULLDOWN);
    pinMode(PIN_MODE_41, INPUT_PULLDOWN);

    // WiFi Setup
    Serial.print("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    
    // Apply Static IP Configuration
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("Static IP failed to configure");
    }
    
    WiFi.begin(ssid, password);
    
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 10) { 
        delay(500); 
        Serial.print("."); 
        wifi_attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected! Web Interface at:");
        Serial.println(WiFi.localIP());

        // WebServer Routes
        server.on("/", HTTP_GET, []() {
            server.send(200, "text/html", index_html);
        });

        // Protected JSON builder to prevent crash loops
        server.on("/data", HTTP_GET, []() {
            char jsonBuffer[256];
            float p_val = curr_pitch - basePitch;
            float r_val = curr_roll - baseRoll;
            
            snprintf(jsonBuffer, sizeof(jsonBuffer),
                "{\"pitch\":%.2f,\"roll\":%.2f,\"tof_fl\":%d,\"tof_fr\":%d,\"tof_bl\":%d,\"tof_br\":%d,\"speed\":%d,\"mode\":\"%s\"}",
                isnan(p_val) ? 0.0 : p_val, // Protect against NaN JSON breaks
                isnan(r_val) ? 0.0 : r_val,
                distFL != -1 ? distFL - baseDistFL : 0,
                distFR != -1 ? distFR - baseDistFR : 0,
                distBL != -1 ? distBL - baseDistBL : 0,
                distBR != -1 ? distBR - baseDistBR : 0,
                currentMotorSpeed,
                currentMode.c_str()
            );
            server.send(200, "application/json", jsonBuffer);
        });

        server.on("/control", HTTP_GET, []() {
            if (server.hasArg("override")) {
                web_override = (server.arg("override") == "1");
                Serial.printf("[WEB EVENT] Override toggled: %s\n", web_override ? "ON" : "OFF");
            }
            if (server.hasArg("speed")) {
                web_speed = server.arg("speed").toInt();
                Serial.printf("[WEB EVENT] Speed requested: %d\n", web_speed);
            }
            server.send(200, "text/plain", "OK");
        });
        server.begin();
    } else {
        Serial.println("\nWiFi Timeout! Proceeding with offline hardware control.");
    }

    // Start I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(150); // Important: Prevents I2C from hanging completely if a wire is loose

    // Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servoFL.attach(PIN_FL_SERVO, 500, 2400);
    servoFR.attach(PIN_FR_SERVO, 500, 2400);
    servoBL.attach(PIN_BL_SERVO, 500, 2400);
    servoBR.attach(PIN_BR_SERVO, 500, 2400);

    // Motor
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
        Serial.print("."); // Loading dots indicator
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
            if (i % 5 == 0) Serial.print("."); // Loading dots indicator
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
// 6. MAIN LOOP
// ==========================================
void loop() {
    unsigned long current_time = millis();

    // --- 1. Handle Web Requests ---
    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
    }

    // --- 2. Calculate Motor Speeds & NeoPixel Colors based on Pins vs Web Override ---
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

    if (web_override) {
        String webModeName = "STOP";
        bool web_auto = false;
        uint32_t web_color = pixels.Color(255, 105, 180);

        if (web_speed == 255) { 
            webModeName = "SMOOTH (WEB)"; web_auto = false; web_color = pixels.Color(0, 255, 0); 
        }
        else if (web_speed == 200) { 
            webModeName = "UNEVEN (WEB)"; web_auto = true; web_color = pixels.Color(255, 255, 0); 
        }
        else if (web_speed == 150) { 
            webModeName = "ROUGH (WEB)"; web_auto = true; web_color = pixels.Color(255, 0, 0); 
        }
        
        setMotorMode(web_speed, webModeName, web_auto, web_color);
    } else {
        setMotorMode(hw_speed, hw_mode, hw_auto, hw_color);
    }

    // --- 3. Fast IMU Polling (50 Hz) ---
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

    // --- 4. Non-Blocking ToF Polling (every 500 ms) ---
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
}