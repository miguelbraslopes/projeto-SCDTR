#pragma once
#include <Arduino.h>
#include "cfg.h"
#include "pid.h"
#include "canv2.h"
#include "shared.h"


enum CommandOrigin {
  ORIGIN_SERIAL = 0,  // Command from serial interface
  ORIGIN_CAN = 1      // Command received from CAN bus
};

struct Command {
  String mainCmd;      // First token: 'u', 'r', 'o', 'a', 'f', 's', 'S', 'g', 'status'/'p'
  String subCmd;       // Second token (for get commands): 'u', 'r', 'y', 'v', 'o', 'a', 'f', 'd', 'p', 't', 'b'
  int luminaireId;     // Luminaire index <i>
  float value;         // Value for set commands
  char charValue;      // Character value for occupancy states
  CommandOrigin origin; // Where command came from (SERIAL or CAN)
  uint8_t sourceLuminaireId; // For CAN responses: which luminaire sent this command
};

struct StreamData {
  unsigned long timestamp;
  int luminaireId;
  float lux_ref;
  float lux_meas;
  float ldrVoltage;
  float ldrResistance;
  float dutyCycle;
  bool antiWindupState;
  float beta; // setpoint weighting parameter
};

Command parseCommand(String input);
void executeCommand(Command cmd);
void handleSerial();
void print_to_serial();