#include <Arduino.h>
#include "driver/mcpwm.h"
#include <PID_v1.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

// WiFi Credentials (Change these to your network)
const char* ssid = "ESP32-Buck-Controller";
const char* password = "buck12345678";

// Or connect to existing WiFi (uncomment and set your credentials)
// const char* ssid = "YourWiFiSSID";
// const char* password = "YourWiFiPassword";

WebServer server(80);

// Define pins
#define HI1 23
#define LO1 25
#define HI2 26
#define LO2 27
#define LED1 4
#define LED2 5

// Voltage measurement pins with voltage dividers
#define VOLTAGE_INPUT_1 33  // IO33 with 56k + 10k divider (6:1 ratio) - Output 1
#define VOLTAGE_INPUT_2 34  // IO34 with 56k + 10k divider (6:1 ratio) - Output 2
#define VOLTAGE_INPUT_IN 35 // IO35 with 220k + 10k divider (24.3:1 ratio) - Input voltage

// ADC Configuration
#define ADC_SAMPLES 32           // Number of samples for averaging
#define ADC_REF_VOLTAGE 3.3      // ESP32 ADC reference voltage
#define VOLTAGE_DIVIDER_RATIO 6.6 // 56k/(56k+10k) ≈ 1/6.6, so actual voltage is 6.6x ADC reading
#define ERROR_CORRECTION_FACTOR_OUTPUT 1.2 // Calibration factor for voltage readings (adjust as needed)
#define INPUT_DIVIDER_RATIO 24.33 // 220k/(220k+10k) ≈ 1/24.33, so actual voltage is 24.33x ADC reading
#define ERROR_CORRECTION_FACTOR_INPUT 1.333  // Calibration factor for input voltage readings (adjust as needed)

// Safe PWM Configuration
#define SWITCHING_FREQ 150000 // 150 kHz
#define DEAD_TIME_NS 100      // 100ns dead time
#define MIN_DUTY 10.0         // Minimum duty cycle 10% (safety margin)
#define MAX_DUTY 90.0         // Maximum duty cycle 90% (safety margin)

// Voltage Control Configuration
#define CONTROL_LOOP_FREQ 100     // Control loop frequency in Hz (100Hz = 10ms)
#define SETPOINT_CHANGE_RATE 0.5  // Maximum setpoint change per second in volts
#define MAX_OUTPUT_VOLTAGE 24.0   // Maximum output voltage (safety limit)
#define MIN_OUTPUT_VOLTAGE 1.0    // Minimum output voltage
#define MAX_INPUT_VOLTAGE 60.0    // Maximum expected input voltage

// PID Tuning Parameters (adjust these based on your system)
#define KP 0.1    // Proportional gain
#define KI 0.005  // Integral gain
#define KD 1.0    // Derivative gain

// Global variables
float current_duty = 50.0;    // Current duty cycle
float voltage1 = 0.0;         // Measured voltage on pin 33 (output 1)
float voltage2 = 0.0;         // Measured voltage on pin 34 (output 2)
float input_voltage = 0.0;    // Measured input voltage on pin 35
float setpoint_voltage = 12.0; // Target output voltage (default 12V)
bool voltage_control_enabled = false; // Whether voltage control is active
unsigned long last_control_time = 0;
float last_setpoint = 12.0;
unsigned long last_web_update = 0;
const long web_update_interval = 1000; // Update web clients every second

// PID Controller
double pid_input, pid_output, pid_setpoint;
PID myPID(&pid_input, &pid_output, &pid_setpoint, KP, KI, KD, DIRECT);

// WebSocket and system status
bool system_running = true;
String last_error = "";
unsigned long system_uptime = 0;

// Function declarations
void initPWM();
void setDutyCycle(float duty);
void processSerialCommands();
float readVoltage(int pin, float divider_ratio, float error_correction_factor);
void updateVoltageReadings();
void printVoltageReadings();
void initPIDController();
void runVoltageControl();
float smoothSetpointChange(float target, float current);
void printSystemInfo();
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleAPI();
void handleControl();
void handleStatus();
void handleSettings();
void handleNotFound();
String getSystemStatusJSON();
void applySafetyChecks();
void emergencyStop();

void setup()
{
  Serial.begin(115200);
  delay(100); // Give serial time to initialize

  // Initialize SPIFFS for web files
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }

  // Initialize LED pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Set safe initial states for LEDs
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // Initialize ADC pins
  pinMode(VOLTAGE_INPUT_1, INPUT);
  pinMode(VOLTAGE_INPUT_2, INPUT);
  pinMode(VOLTAGE_INPUT_IN, INPUT);

  // Configure ADC attenuation for 0-3.3V range
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Optional: Set ADC width to 12-bit (0-4095)
  analogReadResolution(12);

  // Initialize PWM for both half-bridges
  initPWM();

  // Initialize PID controller
  initPIDController();

  // Setup WiFi and Web Server
  setupWiFi();
  setupWebServer();

  Serial.println("\n===============================");
  Serial.println("   ESP32 Dual Buck Driver Controller");
  Serial.println("   with Web Interface");
  Serial.println("===============================");
  Serial.printf("Switching Frequency: %.1f kHz\n", SWITCHING_FREQ / 1000.0);
  Serial.printf("Dead Time: %d ns\n", DEAD_TIME_NS);
  Serial.printf("Duty Range: %.1f%% to %.1f%%\n", MIN_DUTY, MAX_DUTY);
  Serial.printf("Output Voltage Range: %.1fV to %.1fV\n", MIN_OUTPUT_VOLTAGE, MAX_OUTPUT_VOLTAGE);
  Serial.printf("Max Input Voltage: %.1fV\n", MAX_INPUT_VOLTAGE);
  Serial.printf("Control Loop: %d Hz\n", CONTROL_LOOP_FREQ);
  Serial.printf("PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", KP, KI, KD);
  Serial.printf("ADC Reference Voltage: %.2fV\n", ADC_REF_VOLTAGE);
  Serial.printf("Output Divider Ratio: %.2f:1\n", VOLTAGE_DIVIDER_RATIO);
  Serial.printf("Input Divider Ratio: %.2f:1\n", INPUT_DIVIDER_RATIO);
  Serial.printf("Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
  
  Serial.println("\nCommands:");
  Serial.println("  Enter a percentage (10-90) for manual duty control");
  Serial.println("  'v' - Print current voltage readings");
  Serial.println("  'c' - Toggle voltage control ON/OFF");
  Serial.println("  'sX.X' - Set target voltage (e.g., 's12.5' for 12.5V)");
  Serial.println("  'info' - Show system status");
  Serial.println("  'web' - Show web interface info");
  Serial.println("  'stop' - Emergency stop (set duty to 10%)");
  Serial.println("===============================\n");
  
  // Initial voltage readings
  updateVoltageReadings();
  
  Serial.println("Initial Readings:");
  Serial.printf("  Input Voltage: %.2fV\n", input_voltage);
  Serial.printf("  Output 1 (IO33): %.2fV\n", voltage1);
  Serial.printf("  Output 2 (IO34): %.2fV\n", voltage2);
  Serial.printf("  Current Duty: %.1f%%\n", current_duty);
  Serial.printf("  Target Voltage: %.2fV\n", setpoint_voltage);
  Serial.printf("  Voltage Control: %s\n", voltage_control_enabled ? "ON" : "OFF");
  
  // Safety warning if input voltage is too high
  if (input_voltage > MAX_INPUT_VOLTAGE)
  {
    Serial.println("\n⚠️  WARNING: Input voltage exceeds maximum!");
    Serial.printf("  Measured: %.2fV, Maximum: %.1fV\n", input_voltage, MAX_INPUT_VOLTAGE);
    last_error = "Input voltage exceeds maximum safe limit!";
  }
}

void setupWiFi() {
  // Create Access Point
  WiFi.softAP(ssid, password);
  
  // If you want to connect to existing WiFi instead, comment above and uncomment below:
  /*
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi, starting AP mode");
    WiFi.softAP(ssid, password);
  }
  */
  
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
}

void setupWebServer() {
  // Serve the main web page
  server.on("/", HTTP_GET, handleRoot);
  
  // API endpoints
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/control", HTTP_POST, handleControl);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/emergency", HTTP_POST, []() {
    emergencyStop();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Emergency stop activated\"}");
  });
  
  // Serve static files from SPIFFS
  server.on("/style.css", HTTP_GET, []() {
    if (SPIFFS.exists("/style.css")) {
      File file = SPIFFS.open("/style.css", "r");
      server.streamFile(file, "text/css");
      file.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
  });
  
  server.on("/script.js", HTTP_GET, []() {
    if (SPIFFS.exists("/script.js")) {
      File file = SPIFFS.open("/script.js", "r");
      server.streamFile(file, "application/javascript");
      file.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
  });
  
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}

void handleRoot() {
  // Serve the HTML page
  if (SPIFFS.exists("/index.html")) {
    File file = SPIFFS.open("/index.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  } else {
    // Fallback: send minimal HTML if file doesn't exist
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Buck Controller</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        h1 { color: #333; }
        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin: 20px 0; }
        .status-card { background: #f8f9fa; padding: 15px; border-radius: 5px; border-left: 4px solid #007bff; }
        .control-panel { background: #e9ecef; padding: 20px; border-radius: 5px; margin: 20px 0; }
        input[type="range"] { width: 100%; }
        button { background: #007bff; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; margin: 5px; }
        button:hover { background: #0056b3; }
        .error { color: #dc3545; }
        .success { color: #28a745; }
        .warning { color: #ffc107; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Dual Buck Controller</h1>
        <p>System is running. Please upload the complete web interface files to SPIFFS.</p>
        <p>Connect to ESP32 with: <strong>http://)=====";
    html += WiFi.softAPIP().toString();
    html += R"=====(</strong></p>
        <p>Use the serial monitor for complete control until web files are uploaded.</p>
    </div>
</body>
</html>
    )=====";
    server.send(200, "text/html", html);
  }
}

void handleStatus() {
  updateVoltageReadings();
  server.send(200, "application/json", getSystemStatusJSON());
}

void handleControl() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    
    if (doc.containsKey("duty")) {
      float new_duty = doc["duty"];
      if (new_duty >= MIN_DUTY && new_duty <= MAX_DUTY) {
        // Disable voltage control when manually setting duty
        if (voltage_control_enabled) {
          voltage_control_enabled = false;
          myPID.SetMode(MANUAL);
        }
        setDutyCycle(new_duty);
        server.send(200, "application/json", "{\"status\":\"ok\",\"duty\":" + String(current_duty, 1) + "}");
      } else {
        server.send(400, "application/json", "{\"error\":\"Duty out of range\"}");
      }
    }
    else if (doc.containsKey("voltage")) {
      float new_voltage = doc["voltage"];
      if (new_voltage >= MIN_OUTPUT_VOLTAGE && new_voltage <= MAX_OUTPUT_VOLTAGE) {
        setpoint_voltage = new_voltage;
        
        // Enable voltage control if not already enabled
        if (!voltage_control_enabled) {
          voltage_control_enabled = true;
          myPID.SetMode(AUTOMATIC);
        }
        
        server.send(200, "application/json", "{\"status\":\"ok\",\"voltage\":" + String(setpoint_voltage, 1) + "}");
      } else {
        server.send(400, "application/json", "{\"error\":\"Voltage out of range\"}");
      }
    }
    else if (doc.containsKey("mode")) {
      String mode = doc["mode"];
      if (mode == "manual") {
        voltage_control_enabled = false;
        myPID.SetMode(MANUAL);
        server.send(200, "application/json", "{\"status\":\"ok\",\"mode\":\"manual\"}");
      } else if (mode == "auto") {
        voltage_control_enabled = true;
        myPID.SetMode(AUTOMATIC);
        server.send(200, "application/json", "{\"status\":\"ok\",\"mode\":\"auto\"}");
      } else {
        server.send(400, "application/json", "{\"error\":\"Invalid mode\"}");
      }
    }
    else {
      server.send(400, "application/json", "{\"error\":\"Unknown command\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
  }
}

String getSystemStatusJSON() {
  DynamicJsonDocument doc(1024);
  
  doc["system"]["running"] = system_running;
  doc["system"]["uptime"] = millis() / 1000;
  doc["system"]["error"] = last_error;
  
  doc["voltages"]["input"] = input_voltage;
  doc["voltages"]["output1"] = voltage1;
  doc["voltages"]["output2"] = voltage2;
  doc["voltages"]["setpoint"] = setpoint_voltage;
  
  doc["control"]["duty"] = current_duty;
  doc["control"]["mode"] = voltage_control_enabled ? "auto" : "manual";
  doc["control"]["pid_enabled"] = voltage_control_enabled;
  
  doc["limits"]["min_duty"] = MIN_DUTY;
  doc["limits"]["max_duty"] = MAX_DUTY;
  doc["limits"]["min_voltage"] = MIN_OUTPUT_VOLTAGE;
  doc["limits"]["max_voltage"] = MAX_OUTPUT_VOLTAGE;
  doc["limits"]["max_input"] = MAX_INPUT_VOLTAGE;
  
  doc["settings"]["frequency"] = SWITCHING_FREQ;
  doc["settings"]["deadtime"] = DEAD_TIME_NS;
  doc["settings"]["pid_kp"] = KP;
  doc["settings"]["pid_ki"] = KI;
  doc["settings"]["pid_kd"] = KD;
  
  String json;
  serializeJson(doc, json);
  return json;
}

void emergencyStop() {
  setDutyCycle(MIN_DUTY);
  voltage_control_enabled = false;
  myPID.SetMode(MANUAL);
  last_error = "Emergency stop activated";
  Serial.println("EMERGENCY STOP: Duty set to minimum");
}

void initPWM()
{
  // Initialize MCPWM for both half-bridges using same unit but different timers
  
  // Half-Bridge 1: HI1/LO1 on MCPWM Unit 0, Timer 0 (MCPWM0A, MCPWM0B)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HI1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LO1);
  
  // Half-Bridge 2: HI2/LO2 on MCPWM Unit 0, Timer 1 (MCPWM1A, MCPWM1B)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, HI2);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, LO2);

  // Configure PWM for Timer 0 (HI1/LO1)
  mcpwm_config_t pwm_config0 = {
    .frequency = SWITCHING_FREQ,
    .cmpr_a = current_duty,
    .cmpr_b = current_duty,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };
  
  // Configure PWM for Timer 1 (HI2/LO2)
  mcpwm_config_t pwm_config1 = {
    .frequency = SWITCHING_FREQ,
    .cmpr_a = current_duty,
    .cmpr_b = current_duty,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };

  // Initialize both timers on the same unit
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config0);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config1);

  // Set dead time for both timers (80 MHz APB clock = 12.5ns per tick)
  uint32_t dead_time_ticks = DEAD_TIME_NS / 12.5;
  
  // Dead time for Timer 0 (HI1/LO1)
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dead_time_ticks, dead_time_ticks);
  
  // Dead time for Timer 1 (HI2/LO2)
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dead_time_ticks, dead_time_ticks);

  // Set complementary outputs for both timers
  
  // Timer 0: HI1/LO1
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // HI1 active high
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // LO1 active low
  
  // Timer 1: HI2/LO2
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1,
                      MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // HI2 active high
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1,
                      MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // LO2 active low
}

void initPIDController()
{
  // Initialize PID controller
  pid_setpoint = setpoint_voltage;
  pid_input = voltage1; // Use voltage1 as feedback
  pid_output = current_duty;
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000 / CONTROL_LOOP_FREQ); // Convert Hz to ms
  myPID.SetOutputLimits(MIN_DUTY, MAX_DUTY);
  
  // Adjust these based on your system response
  myPID.SetTunings(KP, KI, KD);
  
  // Limit the rate of change (optional, prevents abrupt changes)
  myPID.SetControllerDirection(DIRECT);
}

void setDutyCycle(float duty)
{
  // Safety constraints
  if (duty < MIN_DUTY)
    duty = MIN_DUTY;
  if (duty > MAX_DUTY)
    duty = MAX_DUTY;

  // Update PWM for both timers
  
  // Timer 0: HI1/LO1
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
  
  // Timer 1: HI2/LO2
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);

  current_duty = duty;
}

float readVoltage(int pin, float divider_ratio, float error_correction_factor)
{
  // Take multiple samples and average for better accuracy
  long sum = 0;
  
  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    sum += analogRead(pin);
    delayMicroseconds(10); // Small delay between samples
  }
  
  // Calculate average ADC value
  float avg_adc = (float)sum / ADC_SAMPLES;
  
  // Convert ADC value to voltage (0-3.3V)
  float adc_voltage = (avg_adc / 4095.0) * ADC_REF_VOLTAGE;
  
  // Apply voltage divider correction
  float actual_voltage = adc_voltage * divider_ratio * error_correction_factor;
  
  return actual_voltage;
}

void updateVoltageReadings()
{
  // Read all voltages
  voltage1 = readVoltage(VOLTAGE_INPUT_1, VOLTAGE_DIVIDER_RATIO, ERROR_CORRECTION_FACTOR_OUTPUT);
  voltage2 = readVoltage(VOLTAGE_INPUT_2, VOLTAGE_DIVIDER_RATIO, ERROR_CORRECTION_FACTOR_OUTPUT);
  input_voltage = readVoltage(VOLTAGE_INPUT_IN, INPUT_DIVIDER_RATIO, ERROR_CORRECTION_FACTOR_INPUT);
}

void printVoltageReadings()
{
  Serial.println("\n=== Voltage Readings ===");
  Serial.printf("Input Voltage (IO35): %.2fV\n", input_voltage);
  Serial.printf("  ADC reading: %.2fV (actual: %.2fV after divider)\n", 
                input_voltage / INPUT_DIVIDER_RATIO, input_voltage);
  
  Serial.printf("\nOutput 1 (IO33): %.2fV\n", voltage1);
  Serial.printf("  ADC reading: %.2fV (actual: %.2fV after divider)\n", 
                voltage1 / VOLTAGE_DIVIDER_RATIO, voltage1);
  
  Serial.printf("Output 2 (IO34): %.2fV\n", voltage2);
  Serial.printf("  ADC reading: %.2fV (actual: %.2fV after divider)\n", 
                voltage2 / VOLTAGE_DIVIDER_RATIO, voltage2);
  
  Serial.printf("\nCurrent Duty: %.1f%%\n", current_duty);
  Serial.printf("Target Voltage: %.2fV\n", setpoint_voltage);
  Serial.printf("Voltage Control: %s\n", voltage_control_enabled ? "ENABLED" : "DISABLED");
  
  // Calculate and display efficiency if input voltage is sufficient
  if (input_voltage > 0.5 && voltage1 > 0.5)
  {
    float efficiency = (voltage1 * 100.0) / (input_voltage * current_duty / 100.0);
    if (efficiency < 200.0) // Filter out unrealistic values
    {
      Serial.printf("Estimated Efficiency: %.1f%%\n", efficiency);
    }
  }
  
  Serial.println("=======================\n");
}

void printSystemInfo()
{
  Serial.println("\n=== System Information ===");
  Serial.printf("Input Voltage: %.2fV\n", input_voltage);
  
  if (input_voltage > MAX_INPUT_VOLTAGE)
  {
    Serial.printf("⚠️  WARNING: Input voltage exceeds maximum! (%.1fV max)\n", MAX_INPUT_VOLTAGE);
  }
  
  Serial.printf("\nTarget Output Voltage: %.2fV\n", setpoint_voltage);
  Serial.printf("Measured Output 1: %.2fV (Error: %.2fV)\n", 
                voltage1, voltage1 - setpoint_voltage);
  Serial.printf("Measured Output 2: %.2fV\n", voltage2);
  
  Serial.printf("\nCurrent Duty: %.1f%%\n", current_duty);
  Serial.printf("Voltage Control: %s\n", voltage_control_enabled ? "ENABLED" : "DISABLED");
  
  if (voltage_control_enabled)
  {
    Serial.printf("PID Output: %.1f%%\n", pid_output);
    Serial.printf("PID Parameters: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", KP, KI, KD);
  }
  
  // Display duty cycle limits based on input voltage (buck converter theory)
  if (input_voltage > 0.5 && setpoint_voltage > 0.5)
  {
    float theoretical_duty = (setpoint_voltage / input_voltage) * 100.0;
    Serial.printf("\nTheoretical Duty for %.1fV output: %.1f%%\n", 
                  setpoint_voltage, theoretical_duty);
    
    if (theoretical_duty > MAX_DUTY)
    {
      Serial.printf("⚠️  Note: Required duty (%.1f%%) exceeds maximum (%.1f%%)\n", 
                    theoretical_duty, MAX_DUTY);
      Serial.printf("   Maximum achievable voltage: %.1fV\n", 
                    input_voltage * MAX_DUTY / 100.0);
    }
    else if (theoretical_duty < MIN_DUTY)
    {
      Serial.printf("⚠️  Note: Required duty (%.1f%%) is below minimum (%.1f%%)\n", 
                    theoretical_duty, MIN_DUTY);
    }
  }
  
  Serial.println("==========================\n");
}

float smoothSetpointChange(float target, float current)
{
  // Limit the rate of setpoint change for smoother transitions
  unsigned long current_time = millis();
  float elapsed_seconds = (current_time - last_control_time) / 1000.0;
  float max_change = SETPOINT_CHANGE_RATE * elapsed_seconds;
  
  if (abs(target - current) > max_change)
  {
    if (target > current)
      return current + max_change;
    else
      return current - max_change;
  }
  
  return target;
}

void runVoltageControl()
{
  if (!voltage_control_enabled)
    return;
    
  unsigned long current_time = millis();
  
  // Run control loop at specified frequency
  if (current_time - last_control_time >= (1000 / CONTROL_LOOP_FREQ))
  {
    // Update voltage readings
    updateVoltageReadings();
    
    // Smooth setpoint transition
    setpoint_voltage = smoothSetpointChange(setpoint_voltage, last_setpoint);
    last_setpoint = setpoint_voltage;
    
    // Set PID setpoint
    pid_setpoint = setpoint_voltage;
    
    // Use voltage1 as feedback
    pid_input = voltage1;
    
    // Run PID computation
    bool pid_computed = myPID.Compute();
    
    if (pid_computed)
    {
      // Apply PID output to duty cycle
      setDutyCycle(pid_output);
      
      // Optional: Print debug info
      static unsigned long last_debug = 0;
      if (current_time - last_debug >= 1000)
      {
        Serial.printf("PID: Input=%.1fV, Target=%.1fV, Measured=%.1fV, Error=%.2fV, Duty=%.1f%%\n",
                     input_voltage, pid_setpoint, pid_input, 
                     pid_setpoint - pid_input, pid_output);
        last_debug = current_time;
      }
    }
    
    last_control_time = current_time;
  }
}

void processSerialCommands()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0)
    {
      // Check for 'v' command to print voltages
      if (input == "v" || input == "V")
      {
        updateVoltageReadings();
        printVoltageReadings();
        return;
      }
      
      // Check for 'web' command
      if (input == "web" || input == "WEB")
      {
        Serial.println("\n=== Web Interface ===");
        Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("SSID: %s\n", ssid);
        Serial.printf("Password: %s\n", password);
        Serial.println("Open a browser and navigate to the IP address above");
        Serial.println("=====================\n");
        return;
      }
      
      // Check for 'c' command to toggle voltage control
      if (input == "c" || input == "C")
      {
        voltage_control_enabled = !voltage_control_enabled;
        Serial.printf("Voltage control %s\n", 
                     voltage_control_enabled ? "ENABLED" : "DISABLED");
        
        if (voltage_control_enabled)
        {
          // When enabling control, set PID to current values
          pid_input = voltage1;
          pid_setpoint = setpoint_voltage;
          pid_output = current_duty;
          myPID.SetMode(AUTOMATIC);
          Serial.println("PID control activated");
        }
        else
        {
          myPID.SetMode(MANUAL);
          Serial.println("Manual control activated");
        }
        return;
      }
      
      // Check for 's' command to set target voltage
      if (input.startsWith("s") || input.startsWith("S"))
      {
        String valueStr = input.substring(1);
        float new_voltage = valueStr.toFloat();
        
        if (new_voltage >= MIN_OUTPUT_VOLTAGE && new_voltage <= MAX_OUTPUT_VOLTAGE)
        {
          // Check if input voltage is sufficient
          updateVoltageReadings();
          float required_duty = (new_voltage / input_voltage) * 100.0;
          
          if (input_voltage < new_voltage)
          {
            Serial.printf("Error: Input voltage (%.1fV) is less than target (%.1fV)\n",
                         input_voltage, new_voltage);
            Serial.println("   Buck converters cannot boost voltage!");
            return;
          }
          else if (required_duty > MAX_DUTY)
          {
            Serial.printf("Warning: Required duty (%.1f%%) exceeds maximum (%.1f%%)\n",
                         required_duty, MAX_DUTY);
            Serial.printf("   Maximum achievable: %.1fV\n", 
                         input_voltage * MAX_DUTY / 100.0);
            Serial.print("   Continue anyway? (y/n): ");
            
            // Wait for user confirmation
            unsigned long start = millis();
            while (millis() - start < 5000)
            {
              if (Serial.available() > 0)
              {
                char response = Serial.read();
                if (response == 'y' || response == 'Y')
                {
                  break;
                }
                else
                {
                  Serial.println("Cancelled");
                  return;
                }
              }
              delay(10);
            }
          }
          
          setpoint_voltage = new_voltage;
          Serial.printf("Target voltage set to: %.2fV\n", setpoint_voltage);
          
          // Enable voltage control if not already enabled
          if (!voltage_control_enabled)
          {
            voltage_control_enabled = true;
            myPID.SetMode(AUTOMATIC);
            Serial.println("Voltage control automatically enabled");
          }
        }
        else
        {
          Serial.printf("Error: Voltage must be between %.1fV and %.1fV\n",
                       MIN_OUTPUT_VOLTAGE, MAX_OUTPUT_VOLTAGE);
        }
        return;
      }
      
      // Check for 'info' command
      if (input == "info")
      {
        updateVoltageReadings();
        printSystemInfo();
        return;
      }
      
      // Check for 'stop' command (emergency stop)
      if (input == "stop" || input == "STOP")
      {
        emergencyStop();
        return;
      }

      // Try to parse as a float for manual duty control
      float new_duty = input.toFloat();

      // Check if valid number
      if (new_duty != 0.0 || input == "0" || input == "0.0")
      {
        if (new_duty >= MIN_DUTY && new_duty <= MAX_DUTY)
        {
          // Disable voltage control when manually setting duty
          if (voltage_control_enabled)
          {
            voltage_control_enabled = false;
            myPID.SetMode(MANUAL);
            Serial.println("Voltage control disabled (manual mode)");
          }
          
          setDutyCycle(new_duty);
          Serial.printf("Manual duty set to: %.1f%%\n", new_duty);
          
          // Show estimated output voltage based on input
          updateVoltageReadings();
          float estimated_output = input_voltage * new_duty / 100.0;
          Serial.printf("  Estimated output: %.1fV (assuming ideal buck converter)\n", estimated_output);
        }
        else
        {
          Serial.printf("Error: Duty must be between %.1f%% and %.1f%%\n",
                       MIN_DUTY, MAX_DUTY);
        }
      }
      else
      {
        Serial.println("Error: Unknown command");
        Serial.println("Commands: 'v' (voltages), 'c' (toggle control), 'web' (web info),");
        Serial.println("          'sX.X' (set voltage), 'info' (status), 'stop' (emergency),");
        Serial.println("          or enter a percentage (10-90) for manual duty");
      }
    }
  }
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void loop()
{
  // Handle web server clients
  server.handleClient();
  
  // Process serial commands
  processSerialCommands();
  
  // Run voltage control loop if enabled
  runVoltageControl();
  
  // Update voltage readings periodically
  static unsigned long last_display_update = 0;
  if (millis() - last_display_update >= 500)
  {
    updateVoltageReadings();
    last_display_update = millis();
    
    // Apply safety checks
    if (input_voltage > MAX_INPUT_VOLTAGE) {
      last_error = "Input voltage exceeds maximum!";
    } else if (voltage1 > MAX_OUTPUT_VOLTAGE * 1.1) { // 10% tolerance
      last_error = "Output voltage too high!";
      emergencyStop();
    } else if (last_error.length() > 0 && input_voltage < MAX_INPUT_VOLTAGE * 0.9) {
      last_error = ""; // Clear error if conditions improve
    }
    
    // Print automatic status updates if control is enabled
    static unsigned long last_status_update = 0;
    if (voltage_control_enabled && (millis() - last_status_update >= 2000))
    {
      Serial.printf("[Status] In=%.1fV, Out=%.1fV, Target=%.1fV, Duty=%.1f%%, Error=%.2fV\n",
                   input_voltage, voltage1, setpoint_voltage, current_duty, 
                   setpoint_voltage - voltage1);
      last_status_update = millis();
    }
  }

  // Blink LED based on control status
  static unsigned long last_blink = 0;
  static bool led_state = false;
  
  unsigned long blink_interval = voltage_control_enabled ? 500 : 1000;
  
  if (millis() - last_blink > blink_interval)
  {
    led_state = !led_state;
    digitalWrite(LED1, led_state);
    digitalWrite(LED2, !led_state);
    last_blink = millis();
  }

  // Small delay to prevent CPU hogging
  delay(10);
}