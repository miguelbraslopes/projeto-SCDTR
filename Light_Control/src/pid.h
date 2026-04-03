#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "cfg.h"
#include "shared.h"

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
    float luminaireGain[3];

    float integrator;
    float previousMeasurement;
    float previousReference;

    float outputMin, outputMax;

    bool FBon;
    bool AWon;

public:
    enum Weight : uint8_t {
        KP = 0,
        KI,
        KD,
        BETA,
        GAMMA,
        KFF,
        SAMPLE_TIME
    };

    PID(float kp, float ki, float kd,
        float swbeta, float g,
        float ts,
        float kff,
        bool FBon,
        bool AWon);

    void reset();
    float compute(float reference, float measurement);
    float getExternalLuminance();
    void setOutputLimits(float minVal, float maxVal);
    void setsetpointWeighting(float swbeta, float g);
    bool setWeight(Weight weight, float value);
    float getWeight(Weight weight) const;
    bool setLuminaireGain(uint8_t luminaireId, float gain);
    float getLuminaireGain(uint8_t luminaireId) const;
    void setModeFeedforward(bool enable);
    void setAntiWindup(bool enable);
};

extern PID controller;

#endif  // PID_H