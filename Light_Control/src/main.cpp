#include <Arduino.h>
#include "cfg.h"
#include "pid.h"
#include "serial.h"
#include "canv2.h"
#include "shared.h"

/*
   Distributed Real-Time Control Systems Project
   Luminaire Local Controller - Complete Version
   Platform: Raspberry Pi Pico (Arduino IDE)
   Sampling: 100 Hz
*/
const uint8_t interruptPin {0};
PID controller(0.2, 0.1, 0.0,  // Kp, Ki, Kd
               1.0, 0.0,       // beta, gamma
               TS,           // Ts
               1.0,            // Kff
               false,           // use feedforward
               true);

static void applyPendingCommands() {
  PendingCommands pending = {};

  critical_section_enter_blocking(&gStateLock);
  // Safe to read volatile fields while in critical section
  pending.hasDuty = gPending.hasDuty;
  pending.newDuty = gPending.newDuty;
  pending.hasReferenceLux = gPending.hasReferenceLux;
  pending.newReferenceLux = gPending.newReferenceLux;
  pending.hasBeta = gPending.hasBeta;
  pending.newBeta = gPending.newBeta;
  pending.hasAntiWindupEnabled = gPending.hasAntiWindupEnabled;
  pending.newAntiWindupEnabled = gPending.newAntiWindupEnabled;
  pending.hasFeedbackEnabled = gPending.hasFeedbackEnabled;
  pending.newFeedbackEnabled = gPending.newFeedbackEnabled;
  pending.hasLuminanceControlEnabled = gPending.hasLuminanceControlEnabled;
  pending.newLuminanceControlEnabled = gPending.newLuminanceControlEnabled;
  pending.hasOccupancyState = gPending.hasOccupancyState;
  pending.newOccupancyState = gPending.newOccupancyState;
  
  gPending.hasDuty = false;
  gPending.newDuty = 0.0f;
  gPending.hasReferenceLux = false;
  gPending.newReferenceLux = 0.0f;
  gPending.hasBeta = false;
  gPending.newBeta = 0.0f;
  gPending.hasAntiWindupEnabled = false;
  gPending.newAntiWindupEnabled = false;
  gPending.hasFeedbackEnabled = false;
  gPending.newFeedbackEnabled = false;
  gPending.hasLuminanceControlEnabled = false;
  gPending.newLuminanceControlEnabled = false;
  gPending.hasOccupancyState = false;
  gPending.newOccupancyState = '\0';

  if (pending.hasDuty) {
    gOutputs.duty = pending.newDuty;
  }
  if (pending.hasReferenceLux) {
    gInputs.referenceLux = pending.newReferenceLux;
  }
  if (pending.hasBeta) {
    gInputs.beta = pending.newBeta;
  }
  if (pending.hasAntiWindupEnabled) {
    gInputs.antiWindupEnabled = pending.newAntiWindupEnabled;
  }
  if (pending.hasFeedbackEnabled) {
    gInputs.feedbackEnabled = pending.newFeedbackEnabled;
  }
  if (pending.hasLuminanceControlEnabled) {
    gInputs.luminanceControlEnabled = pending.newLuminanceControlEnabled;
  }
  if (pending.hasOccupancyState) {
    gInputs.occupancyState = pending.newOccupancyState;
  }
  critical_section_exit(&gStateLock);

  if (pending.hasBeta) {
    controller.setsetpointWeighting(pending.newBeta, 0.0);
  }
  if (pending.hasAntiWindupEnabled) {
    controller.setAntiWindup(pending.newAntiWindupEnabled);
  }
}


//Core 0: Control loop and sensor reading
unsigned long lastTimePID = 0;

void setup() {
  initSharedState();
  analogReadResolution(12);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastTimePID >= 10) { // 10ms control period
    lastTimePID = now;
    applyPendingCommands();

    float adcAvg = getMovingAverageADC();
    float lux = getavglux(adcAvg);
    float duty = gOutputs.duty;

    if (gInputs.feedbackEnabled && gInputs.luminanceControlEnabled) {
      duty = controller.compute(gInputs.referenceLux, lux);
    }
    setPWM(duty);

    float ldrVoltage = getavgvoltage(adcAvg);
    float ldrResistance = getLDRresistance(ldrVoltage);

    critical_section_enter_blocking(&gStateLock);
    gOutputs.timestampMs = now;
    gOutputs.duty = duty;
    gOutputs.luxMeasured = lux;
    gOutputs.ldrVoltage = ldrVoltage;
    gOutputs.ldrResistance = ldrResistance;
    critical_section_exit(&gStateLock);
  }
  else {
    // Collect samples between control periods
    addSampleToBufferADC();
  }
}


//Core 1: Communications

volatile bool got_irq {false};

//the interrupt service routine for core 1
void read_interrupt(uint gpio, uint32_t events) {
  got_irq = true;  
}

unsigned long lastTimePWM = 0;
unsigned long lastTimeSerial = 0;
unsigned long canTxLedOnUntil = 0;

void setup1() { 
  Serial.begin(115200); 
  init_can();
  gpio_set_irq_enabled_with_callback(interruptPin,GPIO_IRQ_EDGE_FALL,true,&read_interrupt ); 
}

void loop1() {
  unsigned long now = millis();
  if (now - lastTimeSerial >= 100) { // 100ms serial update to send messages
    lastTimeSerial = now;
    handleSerial();
  }
  if (got_irq)
  {
    got_irq = false; // Reset flag

    // Process CAN message
    processirq();
    can0.clearRXnOVRFlags();  
    can0.clearInterrupts();
  }

  if (now - lastTimePWM >= 2000) { // 50ms CAN update
    lastTimePWM = now;
    // Send PWM value over CAN
    critical_section_enter_blocking(&gStateLock);
    float dutyToSend = gOutputs.duty;
    critical_section_exit(&gStateLock);
    encode_and_send(INTERNAL,_luminaireId,0,0,dutyToSend);

    /*
    uint8_t eflags = can0.getErrorFlags();
    uint8_t TEC = can0.errorCountTX();
    uint8_t REC = can0.errorCountRX();

    uint8_t irq = can0.getInterrupts();
    uint8_t err = can0.getErrorFlags();
    
    
    Serial.print("EFLG: ");
    Serial.println(eflags, BIN);
    Serial.print("TEC: ");
    Serial.println(TEC);
    Serial.print("REC: ");
    Serial.println(REC);
    Serial.println("---");
    */
  }
}
