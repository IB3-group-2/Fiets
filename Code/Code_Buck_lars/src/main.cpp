#include <Arduino.h>

// Define LED pins (Digital Outputs)
#define LED1 4
#define LED2 5

// Define PWM Pins (Corrected definitions and unique names)
#define HI1 23
#define LO1 25
#define HI2 26
#define LO2 27 // Corrected from invalid pin 277

// PWM Configuration for Channel 0 (HI1_A)
#define PWM1_Ch 0
#define PWM1_Res 8     // 8-bit resolution (0 to 255)
#define PWM1_Freq 1000 // 1000 Hz

int PWM1_DutyCycle = 128; // 128 / 256 = 50% duty cycle

void setup()
{
  // Initialize the Timer/Channel
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  // Attach the pin to the channel
  ledcAttachPin(HI1, PWM1_Ch); // Attaching Channel 0 to Pin 23

  // Set the PWM Duty Cycle - THIS IS WHERE IT GOES
  ledcWrite(PWM1_Ch, PWM1_DutyCycle);

  // Initialize the digital pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LO1, OUTPUT);
  pinMode(LO2, OUTPUT);

  // Ensure driver pins are LOW initially
  digitalWrite(LO1, LOW);
  digitalWrite(HI2, LOW);
  digitalWrite(LO2, LOW);
}

void loop()
{
  // Toggle LED1
  digitalWrite(LED1, !digitalRead(LED1));
  delay(500);
}