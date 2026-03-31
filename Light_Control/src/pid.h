#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "cfg.h"

// =====================
// GLOBAL VARIABLES
// =====================
extern float luxSum;
extern int luxsumADC;
extern int sampleCount;
extern int sampleCountADC;

// =====================
// FUNCTION DECLARATIONS
// =====================
void addSampleToBufferADC();
float getMovingAverageADC();
float getavgvoltage(float ADCavg);
float getLDRresistance(float v_out);
float getavglux(float ADCavg);
void setPWM(float dutyCycle);

// =====================
// PID CLASS DECLARATION
// =====================
class PID {
private:
    float Kp, Ki, Kd;
    float beta, gamma;
    float Kff;
    float Ts;

    float integrator;
    float previousMeasurement;
    float previousReference;

    float outputMin, outputMax;

    bool useFeedforward;
    bool antiWindup;

public:
    PID(float kp, float ki, float kd,
        float swbeta, float g,
        float ts,
        float kff,
        bool useFF,
        bool antiWindupEnabled);

    void reset();
    float compute(float reference, float measurement);
    void setOutputLimits(float minVal, float maxVal);
    void setsetpointWeighting(float swbeta, float g);
    void setModeFeedforward(bool enable);
    void setAntiWindup(bool enable);
};

#endif  // PID_H