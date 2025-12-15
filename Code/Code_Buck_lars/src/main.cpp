#include <Arduino.h>
#include "driver/mcpwm.h"

// Define pins
#define HI1 23
#define LO1 25
#define HI2 26
#define LO2 27
#define LED1 4
#define LED2 5

// Voltage measurement pins with voltage dividers
#define VOLTAGE_INPUT_1 33  // IO33 with 56k + 10k divider (6:1 ratio)
#define VOLTAGE_INPUT_2 34  // IO34 with 56k + 10k divider (6:1 ratio)

// ADC Configuration
#define ADC_SAMPLES 32      // Number of samples for averaging
#define ADC_REF_VOLTAGE 3.3 // ESP32 ADC reference voltage
#define VOLTAGE_DIVIDER_RATIO 6.6 // 56k/(56k+10k) ≈ 1/6, so actual voltage is 6x ADC reading
#define ERROR_CORRECTION_FACTOR 1.2 // Calibration factor for voltage readings

// Safe PWM Configuration
#define SWITCHING_FREQ 150000 // 150 kHz
#define DEAD_TIME_NS 100      // 100ns dead time
#define MIN_DUTY 10.0         // Minimum duty cycle 10% (safety margin)
#define MAX_DUTY 90.0         // Maximum duty cycle 90% (safety margin)

// Global variables
float current_duty = 50.0; // Start at 50%
float voltage1 = 0.0;      // Measured voltage on pin 33 (actual voltage after divider)
float voltage2 = 0.0;      // Measured voltage on pin 34 (actual voltage after divider)

// Function declarations
void initPWM();
void setDutyCycle(float duty);
void processSerialCommands();
float readVoltage(int pin);
void updateVoltageReadings();
void printVoltageReadings();

void setup()
{
  Serial.begin(115200);
  delay(100); // Give serial time to initialize

  // Initialize LED pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Set safe initial states for LEDs
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // Initialize ADC pins
  pinMode(VOLTAGE_INPUT_1, INPUT);
  pinMode(VOLTAGE_INPUT_2, INPUT);

  // Configure ADC attenuation for 0-3.3V range
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Optional: Set ADC width to 12-bit (0-4095)
  analogReadResolution(12);

  // Initialize PWM for both half-bridges
  initPWM();

  Serial.println("\n===============================");
  Serial.println("   ESP32 Dual Buck Driver Controller");
  Serial.println("===============================");
  Serial.printf("Switching Frequency: %.1f kHz\n", SWITCHING_FREQ / 1000.0);
  Serial.printf("Dead Time: %d ns\n", DEAD_TIME_NS);
  Serial.printf("Duty Range: %.1f%% to %.1f%%\n", MIN_DUTY, MAX_DUTY);
  Serial.printf("ADC Reference Voltage: %.2fV\n", ADC_REF_VOLTAGE);
  Serial.printf("Voltage Divider Ratio: %.1f:1\n", VOLTAGE_DIVIDER_RATIO);
  Serial.println("\nCommands:");
  Serial.println("  Enter a percentage (10-90)");
  Serial.println("  Example: '50' sets 50% duty");
  Serial.println("  Example: '75.5' sets 75.5% duty");
  Serial.println("  'v' - Print current voltage readings");
  Serial.println("===============================\n");
  Serial.printf("Current Duty: %.1f%%\n", current_duty);
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

  Serial.printf("Duty set to: %.1f%% (both half-bridges)\n", duty);
}

float readVoltage(int pin)
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
  
  // Apply voltage divider correction (actual voltage is 6x higher)
  float actual_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO * ERROR_CORRECTION_FACTOR;
  
  return actual_voltage;
}

void updateVoltageReadings()
{
  voltage1 = readVoltage(VOLTAGE_INPUT_1);
  voltage2 = readVoltage(VOLTAGE_INPUT_2);
}

void printVoltageReadings()
{
  Serial.println("\n=== Voltage Readings ===");
  Serial.printf("Pin 33 (IO33): %.3fV (actual: %.3fV after divider)\n", 
                voltage1 / VOLTAGE_DIVIDER_RATIO, voltage1);
  Serial.printf("Pin 34 (IO34): %.3fV (actual: %.3fV after divider)\n", 
                voltage2 / VOLTAGE_DIVIDER_RATIO, voltage2);
  Serial.printf("Current Duty: %.1f%%\n", current_duty);
  Serial.println("=======================\n");
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
        Serial.println("Error: Please enter a number (10-90) or 'v' for voltage readings");
      }
    }
  }
}

void loop()
{
  // Process serial commands
  processSerialCommands();

  // Update voltage readings every second
  static unsigned long last_voltage_update = 0;
  if (millis() - last_voltage_update >= 1000)
  {
    updateVoltageReadings();
    last_voltage_update = millis();
    
    // Optional: Uncomment to auto-print voltages every second
    printVoltageReadings();
  }

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