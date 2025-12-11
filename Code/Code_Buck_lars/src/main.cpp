#include <Arduino.h>
#include "driver/mcpwm.h"



// Define pins
#define HI1 23
#define LO1 25
#define HI2 26
#define LO2 27

#define LED1 4
#define LED2 5

// Recommended Configuration
#define SWITCHING_FREQ 250000 // 250 kHz - GOOD BALANCE
// #define SWITCHING_FREQ 150000  // 150 kHz - MORE CONSERVATIVE

#define DEAD_TIME_NS 150 // 150ns dead time (slightly > 120ns rise)
#define DUTY_CYCLE 50.0  // 50% for testing

void setup()
{
  // 1. Initialize MCPWM for complementary PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HI1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LO1);

  // 2. Configure MCPWM
  mcpwm_config_t pwm_config;
  pwm_config.frequency = SWITCHING_FREQ; // Your chosen frequency
  pwm_config.cmpr_a = DUTY_CYCLE;        // Duty cycle %
  pwm_config.cmpr_b = DUTY_CYCLE;        // Same for complementary
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // 3. Set dead time - CRITICAL with 120ns rise time!
  // Calculate dead time in timer ticks
  // For 80 MHz APB clock: 1 tick = 12.5ns
  uint32_t dead_time_ticks = DEAD_TIME_NS / 12.5;

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dead_time_ticks, dead_time_ticks);

  // 4. Set complementary inverted outputs
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // HI1 active high
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0,
                      MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // LO1 active low

  Serial.begin(115200);
  Serial.printf("Buck Driver Configuration:\n");
  Serial.printf("Switching Frequency: %.1f kHz\n", SWITCHING_FREQ / 1000.0);
  Serial.printf("Rise Time: 120ns\n");
  Serial.printf("Dead Time: %dns\n", DEAD_TIME_NS);
  Serial.printf("Duty Cycle: %.1f%%\n", DUTY_CYCLE);
  Serial.printf("Period: %.2f µs\n", 1000000.0 / SWITCHING_FREQ);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop()
{
  // Your control logic here
  digitalWrite(LED1, digitalRead(LED1) ^ 1);
  digitalWrite(LED2, digitalRead(LED1) ^ 1);
  delay(500);

}