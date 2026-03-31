#include "pid.h"

// =====================
// GLOBAL VARIABLES DEFINITIONS
// =====================
float luxSum = 0.0;
int luxsumADC = 0;
int sampleCount = 0;
int sampleCountADC = 0;

// =====================
// LUX READING
// =====================

void addSampleToBufferADC() {

  luxsumADC += analogRead(LDR_PIN);
  sampleCountADC++;
}

float getMovingAverageADC() {
  if (sampleCountADC==0)
  {
    addSampleToBufferADC();
  }
  
  float average = (sampleCountADC > 0) ? luxsumADC / sampleCountADC : 0.0;
  
  // Reset for next period
  luxsumADC = 0.0;
  sampleCountADC = 0;
  
  return average;
}

float getavgvoltage(float ADCavg) {

  float v_out = (ADCavg * 3.3) / 4095.0;

  return v_out;
}

float getLDRresistance(float v_out) {

  float r_ldr = (R_FIXED * (3.3 - v_out)) / v_out;

  return r_ldr;
}

float getavglux(float ADCavg) {

  float v_out = getavgvoltage(ADCavg);
  float r_ldr = getLDRresistance(v_out);
  float log10_ldr = log10(r_ldr);
  float avglux = pow(10, (log10_ldr - b) / m);
  return avglux;
}


void setPWM(float dutyCycle) {

  if (dutyCycle > 1.0) dutyCycle = 1.0;
  if (dutyCycle < 0.0) dutyCycle = 0.0;

  analogWrite(PWM_PIN, dutyCycle * PWM_MAX);
}


// =====================
// PID CLASS IMPLEMENTATIONS
// =====================
PID::PID(float kp, float ki, float kd,
        float swbeta, float g,
        float ts,
        float kff,
        bool useFF,
        bool antiWindupEnabled)
{
    Kp = kp; Ki = ki; Kd = kd;
    beta = swbeta; gamma = g;
    Kff = kff;
    Ts = ts;

    integrator = 0.0;
    previousMeasurement = 0.0;
    previousReference = 0.0;

    outputMin = 0.0;
    outputMax = 1.0;

    useFeedforward = useFF;
    antiWindup = antiWindupEnabled;
}

void PID::reset() {
    integrator = 0.0;
    previousMeasurement = 0.0;
    previousReference = 0.0;
}

float PID::compute(float reference, float measurement) {

    if (useFeedforward) {
        // modo feedforward puro
        float output = Kff * reference;
        // saturação
        if (output > outputMax) output = outputMax;
        if (output < outputMin) output = outputMin;
        return output;
    }

    // modo feedback (PID completo)
    float error = reference - measurement;
    
    float P = Kp * (beta * reference - measurement);
    float D = Kd * ((gamma * reference - measurement) -
                    (gamma * previousReference - previousMeasurement)) / Ts;

    if (antiWindup) {
        if (!((P + integrator + D >= outputMax && error > 0) ||
              (P + integrator + D <= outputMin && error < 0))) 
        {
            integrator += Ki * Ts * error;
        }
    } else {
        integrator += Ki * Ts * error;
    }

    float output = P + integrator + D;

    // saturação
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    previousMeasurement = measurement;
    previousReference = reference;

    return output;
}

void PID::setOutputLimits(float minVal, float maxVal) {
    outputMin = minVal;
    outputMax = maxVal;
}

void PID::setsetpointWeighting(float swbeta, float g) {
    beta = swbeta;
    gamma = g;
}

void PID::setModeFeedforward(bool enable) {
    useFeedforward = enable;
}

void PID::setAntiWindup(bool enable) {
    antiWindup = enable;
}