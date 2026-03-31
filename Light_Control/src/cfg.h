
#pragma once

#define _luminaireId 0 
#define m -0.8 // Slope of log-log curve (assumed)
#define b 6.10 // Intercept of log-log curve (from calibration)

#define PWM_PIN 9 // PWM output to LED
#define LDR_PIN 26  // ADC input from LDR voltage divider


#define TS 0.01 // Control period in seconds (10 ms)
#define PWM_MAX 255 // Maximum PWM value for 8-bit resolution

#define R_FIXED 10000.0  // 10k resistor in divider

//#define interruptPin 0 // GPIO pin for MCP2515 interrupt