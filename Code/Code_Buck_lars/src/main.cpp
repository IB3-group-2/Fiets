#include <Arduino.h>
#include "driver/mcpwm.h"

// Define pins
#define HI1 23
#define LO1 25
#define HI2 26
#define LO2 27
#define LED1 4
#define LED2 5

// Safe PWM Configuration
#define SWITCHING_FREQ 100000 // 100 kHz - SAFE AND RELIABLE
#define DEAD_TIME_NS 150      // 150ns dead time
#define MIN_DUTY 10.0         // Minimum duty cycle 10% (safety margin)
#define MAX_DUTY 90.0         // Maximum duty cycle 90% (safety margin)

// Global variables
float current_duty = 50.0; // Start at 50%

// Function declarations
void initPWM();
void setDutyCycle(float duty);
void processSerialCommands();

void setup()
{
  Serial.begin(115200);
  delay(100); // Give serial time to initialize

  // Initialize pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(HI2, OUTPUT);
  pinMode(LO2, OUTPUT);

  // Set safe initial states
  digitalWrite(HI2, LOW);
  digitalWrite(LO2, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // Initialize PWM
  initPWM();

  Serial.println("\n===============================");
  Serial.println("   ESP32 Buck Driver Controller");
  Serial.println("===============================");
  Serial.printf("Switching Frequency: %.1f kHz\n", SWITCHING_FREQ / 1000.0);
  Serial.printf("Dead Time: %d ns\n", DEAD_TIME_NS);
  Serial.printf("Duty Range: %.1f%% to %.1f%%\n", MIN_DUTY, MAX_DUTY);
  Serial.println("\nCommands:");
  Serial.println("  Enter a percentage (10-90)");
  Serial.println("  Example: '50' sets 50% duty");
  Serial.println("  Example: '75.5' sets 75.5% duty");
  Serial.println("===============================\n");
  Serial.printf("Current Duty: %.1f%%\n", current_duty);
}

void initPWM()
{
  // Initialize MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HI1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LO1);

  // Configure PWM - Initialize struct fields in correct order
  mcpwm_config_t pwm_config;
  pwm_config.frequency = SWITCHING_FREQ;
  pwm_config.cmpr_a = current_duty;
  pwm_config.cmpr_b = current_duty;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Set dead time (80 MHz APB clock = 12.5ns per tick)
  uint32_t dead_time_ticks = DEAD_TIME_NS / 12.5;
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dead_time_ticks, dead_time_ticks);

  // Set complementary outputs
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // HI1 active high
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // LO1 active low
}

void setDutyCycle(float duty)
{
  // Safety constraints
  if (duty < MIN_DUTY)
    duty = MIN_DUTY;
  if (duty > MAX_DUTY)
    duty = MAX_DUTY;

  // Update PWM
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);

  current_duty = duty;

  Serial.printf("Duty set to: %.1f%%\n", duty);
}

void processSerialCommands()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0)
    {
      // Try to parse as a float
      float new_duty = input.toFloat();

      // Check if valid number
      if (new_duty != 0.0 || input == "0" || input == "0.0")
      {
        if (new_duty >= MIN_DUTY && new_duty <= MAX_DUTY)
        {
          setDutyCycle(new_duty);
        }
        else
        {
          Serial.printf("Error: Duty must be between %.1f%% and %.1f%%\n",
                        MIN_DUTY, MAX_DUTY);
        }
      }
      else
      {
        Serial.println("Error: Please enter a number (10-90)");
      }
    }
  }
}

void loop()
{
  // Process serial commands
  processSerialCommands();

  // Blink LED slowly to show system is alive
  static unsigned long last_blink = 0;
  static bool led_state = false;

  if (millis() - last_blink > 1000)
  {
    led_state = !led_state;
    digitalWrite(LED1, led_state);
    digitalWrite(LED2, !led_state);
    last_blink = millis();
  }

  // Small delay to prevent CPU hogging
  delay(10);
}