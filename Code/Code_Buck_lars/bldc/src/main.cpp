#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <MPU9250.h>
#include <LittleFS.h>

const char* ssid = "IB3";
const char* password = "ingenieursbeleving3";

Servo esc1, esc2;
const int esc1Pin = 18;
const int esc2Pin = 19;

MPU9250 mpu;
AsyncWebServer server(80);

// Store the "zero" positions
float pitchOff = 0, rollOff = 0, yawOff = 0;

// --- PID Constants ---
float Kp = 0.1, Ki = 0.0, Kd = 0.0;

// --- PID Variables ---
float targetAngle = 0.0;
float error, lastError, integral, derivative;
unsigned long lastPIDTime;
float pidOutput = 0; 

void computePID() {
    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    if (dt <= 0) return;

    error = targetAngle - (mpu.getYaw() - yawOff);

    // P Term
    float P = Kp * error;

    // I Term (Uncomment in code or use sliders to activate)
    integral += error * dt;
    float I = Ki * integral;

    // D Term (Uncomment in code or use sliders to activate)
    derivative = (error - lastError) / dt;
    float D = Kd * derivative;

    pidOutput = P + I + D;

    if (pidOutput > 100) pidOutput = 100;
    if (pidOutput < -100) pidOutput = -100;

    lastError = error;
    lastPIDTime = now;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    if (!LittleFS.begin(true)) Serial.println("LittleFS Failed");
    if (!mpu.setup(0x68)) Serial.println("MPU Failed");

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    esc1.attach(esc1Pin, 1000, 2000);
    esc2.attach(esc2Pin, 1000, 2000);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    ArduinoOTA.begin();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/setZero", HTTP_GET, [](AsyncWebServerRequest *request){
        pitchOff = mpu.getPitch();
        rollOff = mpu.getRoll();
        yawOff = mpu.getYaw();
        integral = 0; // Reset integral on zeroing
        request->send(200, "text/plain", "Zeroed");
    });

    server.on("/getAngles", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"pitch\":" + String(mpu.getPitch() - pitchOff) + ",";
        json += "\"roll\":" + String(mpu.getRoll() - rollOff) + ",";
        json += "\"yaw\":" + String(mpu.getYaw() - yawOff) + ",";
        json += "\"pid\":" + String(pidOutput) + ",";
        json += "\"ax\":" + String(mpu.getAccX()) + ",";
        json += "\"ay\":" + String(mpu.getAccY()) + ",";
        json += "\"az\":" + String(mpu.getAccZ());
        json += "}";
        request->send(200, "application/json", json);
    });

    // PID Tuning Routes
    server.on("/setKp", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) Kp = request->arg("val").toFloat();
        request->send(200);
    });
    server.on("/setKi", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) Ki = request->arg("val").toFloat();
        request->send(200);
    });
    server.on("/setKd", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) Kd = request->arg("val").toFloat();
        request->send(200);
    });

    server.on("/setThrottle", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasArg("motor") && request->hasArg("val")) {
            int m = request->arg("motor").toInt();
            int v = request->arg("val").toInt();
            if (m == 1) esc1.writeMicroseconds(v);
            else if (m == 2) esc2.writeMicroseconds(v);
            request->send(200);
        }
    });

    server.begin();
}

void loop() {
    ArduinoOTA.handle();
    mpu.update();

    static unsigned long pidTimer = 0;
    if (millis() - pidTimer >= 10) {
        computePID();
        pidTimer = millis();
    }
}