#include <Arduino.h>
#include "cfg.h"
#include "pid.h"
#include "serial.h"
#include "canv2.h"
#include "shared.h"
#include <string.h>

/*
   Distributed Real-Time Control Systems Project
   Luminaire Local Controller - Complete Version
   Platform: Raspberry Pi Pico (Arduino IDE)
   Sampling: 100 Hz
*/
const uint8_t interruptPin {0};
PID controller(0.02, 0.1, 0.0,  // Kp, Ki, Kd
               1.0, 0.0,       // beta, gamma
               TS,           // Ts
               1.0,            // Kff
               true,           // use feedforward
               true);

static void applyPendingCommands() {
  PendingCommands pending = {};

  critical_section_enter_blocking(&gStateLock);
  // Copy and clear the full pending struct so all fields are handled.
  memcpy(&pending, (const void*)&gPending, sizeof(PendingCommands));
  memset((void*)&gPending, 0, sizeof(PendingCommands));

  if (pending.hasDuty) {
    gOutputs.duty = pending.newDuty;
  }
  if (pending.hasReferenceLux) {
    gInputs.referenceLux = pending.newReferenceLux;
  }
  if (pending.hasOccupancyState) {
    gInputs.occupancyState = pending.newOccupancyState;
  }
  if (pending.hasAntiWindupEnabled) {
    gInputs.antiWindupEnabled = pending.newAntiWindupEnabled;
  }
  if (pending.hasFeedbackEnabled) {
    gInputs.feedbackEnabled = pending.newFeedbackEnabled;
  }
  if (pending.hasManualOverride) {
    gInputs.manualOverride = pending.newManualOverride;
  }
  if (pending.haspwm) {
    for (int i = 0; i < 3; i++) {
      gInputs.pwm[i] = pending.newpwm[i];
    }
  }
  critical_section_exit(&gStateLock); 
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
  
    // Skip PID computation if manual override is active (use preset duty cycle instead).
    if (!gInputs.manualOverride) {
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

void setup1() { 
  Serial.begin(115200); 
  Serial.setTimeout(10);
  init_can();
  gpio_set_irq_enabled_with_callback(interruptPin,GPIO_IRQ_EDGE_FALL,true,&read_interrupt ); 
}

void loop1() {
  unsigned long now = millis();

  if (got_irq){
    got_irq = false; // Reset flag
    // Process CAN message
    processirq();
    can0.clearRXnOVRFlags();  
    can0.clearInterrupts();
  }

  if (Serial.available() > 0) {
    handleSerial();
  }

  if (now - lastTimePWM >= 5) { // 20ms CAN update
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
