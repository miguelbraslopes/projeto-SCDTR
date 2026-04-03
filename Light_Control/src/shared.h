#pragma once
#include <Arduino.h>
#include <pico/critical_section.h>

struct ControlInputs {
  float referenceLux;
  char occupancyState;
  bool antiWindupEnabled;
  bool feedbackEnabled;
  bool manualOverride;
  float pwm[3]; // for future use: store PWM values of other luminaires
};

struct ControlOutputs {
  uint32_t timestampMs;
  float duty;
  float luxMeasured;
  float ldrVoltage;
  float ldrResistance;
};

struct PendingCommands {
  bool hasDuty;
  float newDuty;

  bool hasReferenceLux;
  float newReferenceLux;

  bool hasOccupancyState;
  char newOccupancyState;

  bool hasAntiWindupEnabled;
  bool newAntiWindupEnabled;

  bool hasFeedbackEnabled;
  bool newFeedbackEnabled;

  bool hasManualOverride;
  bool newManualOverride;

  bool haspwm;
  float newpwm[3];
};

extern volatile ControlInputs gInputs;
extern volatile ControlOutputs gOutputs;
extern volatile PendingCommands gPending;
extern critical_section_t gStateLock;

extern bool waiting_can;

void initSharedState();