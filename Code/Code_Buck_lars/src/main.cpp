#include <Arduino.h>
#define LED1 4
#define LED2 5
#define PWM_PIN 23
#define INVERT_PIN 25

// PWM parameters for 50kHz
const int pwmChannel1 = 0;     // Channel for pin 23
const int pwmChannel2 = 1;     // Channel for pin 25 (inverted)
const int frequency = 50000;   // 50kHz frequency
const int resolution = 8;      // 8-bit resolution (0-255)

// Default constant PWM duty cycle value (0-255)
int constantDutyCycle = 128;  // Default: 50% duty cycle

void printStatus();

void setupPWM() {
  // Configure PWM channel for pin 23
  ledcSetup(pwmChannel1, frequency, resolution);
  ledcAttachPin(PWM_PIN, pwmChannel1);
  
  // Configure PWM channel for pin 25
  // ledcSetup(pwmChannel2, frequency, resolution);
  // ledcAttachPin(INVERT_PIN, pwmChannel2);
}

void setPWMWithInversion(int dutyCycle) {
  // Constrain duty cycle
  dutyCycle = constrain(dutyCycle, 0, 255);
  
  // Set PWM on pin 23
  ledcWrite(pwmChannel1, dutyCycle);
  
  // Set inverted PWM on pin 25 (true complement)
  // ledcWrite(pwmChannel2, 255 - dutyCycle);
  
  // Update the constant value
  constantDutyCycle = dutyCycle;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting 50kHz PWM with inverted outputs");
  Serial.println("Pin 23 and Pin 25 are always complementary");
  Serial.println("Send a value 0-255 via Serial to change PWM duty cycle");
  Serial.println("Default: 128 (50%)");
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  // Setup PWM on both pins
  setupPWM();
  
  // Set initial constant PWM value
  setPWMWithInversion(constantDutyCycle);
  
  printStatus();
}

void printStatus() {
  Serial.print("Current PWM: Pin23=");
  Serial.print(constantDutyCycle);
  Serial.print(" (");
  Serial.print((constantDutyCycle * 100) / 255);
  Serial.print("%), Pin25=");
  Serial.print(255 - constantDutyCycle);
  Serial.print(" (");
  Serial.print(((255 - constantDutyCycle) * 100) / 255);
  Serial.println("%)");
}

void loop() {
  // Handle Serial input to change constant PWM value
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      int newValue = input.toInt();
      if (newValue >= 0 && newValue <= 255) {
        setPWMWithInversion(newValue);
        Serial.print("Updated constant PWM to: ");
        Serial.println(newValue);
        printStatus();
      } else {
        Serial.println("Error: Please enter a value between 0 and 255");
      }
    }
  }
  
  // Blink LED1 every second to show the program is running
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) {
    lastBlink = millis();
    digitalWrite(LED1, !digitalRead(LED1));
  }
  
  // Optional: Blink LED2 every 0.5 seconds
  static unsigned long lastLED2Blink = 0;
  if (millis() - lastLED2Blink >= 500) {
    lastLED2Blink = millis();
    digitalWrite(LED2, !digitalRead(LED2));
  }
}