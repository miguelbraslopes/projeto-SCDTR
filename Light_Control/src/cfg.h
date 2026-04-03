
#pragma once


#define m -0.8 // Slope of log-log curve (assumed)

// Build-time role selection:
// -Drpi1 enables RPI 1 specific code paths.
// If no role is set, firmware behaves as RPI 2 by default.
#if defined(rpi1) || defined(RPI1) || defined(RPI_1)
#define IS_RPI_1 1
#define IS_RPI_2 0
#define _luminaireId 0
#elif defined(rpi2) || defined(RPI2) || defined(RPI_2)
#define IS_RPI_1 0
#define IS_RPI_2 1
#define _luminaireId 1

#else
#define IS_RPI_1 0
#define IS_RPI_2 1
#define _luminaireId 0
#endif

// Intercept of log-log curve (from calibration).
// Can be overridden from build flags, e.g. -DB_VALUE=6.25
#ifndef B_VALUE
	#if IS_RPI_1
		#define B_VALUE 6.10
	#else
		#define B_VALUE 5.839462
	#endif
#endif
#define b B_VALUE

#define PWM_PIN 9 // PWM output to LED
#define LDR_PIN 26  // ADC input from LDR voltage divider


#define TS 0.01 // Control period in seconds (10 ms)
#define PWM_MAX 255 // Maximum PWM value for 8-bit resolution

#define R_FIXED 10000.0  // 10k resistor in divider

#if IS_RPI_1
    #define debugger_can 0
    #define debugger_coms 1
    #define debug 1
#elif IS_RPI_2
    #define debugger_can 0
    #define debugger_coms 1
    #define debug 1
#else
    #define debugger_can 1
    #define debugger_coms 1
    #define debug 1
#endif
//#define interruptPin 0 // GPIO pin for MCP2515 interrupt