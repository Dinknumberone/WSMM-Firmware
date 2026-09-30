#ifndef MFADER_H
#define MFADER_H

#include <Arduino.h>
#include <PID_v1.h>
#include <Filters.h>
#include <Preferences.h>

class mFader {
public:
    // pwm
    int PWM_FREQ = 20000;
    int PWM_RES = 8;
    const int PWM_CH_UP = 0;
    const int PWM_CH_DN = 1;
    // pins
    int upControlPin;
    int downControlPin;
    int potPin;
    int touchPin;
    // fixed points
    float limitMax;
    float limitMin;
    float rawMax;
    float rawMin;
    float curveOffset = 2.44f;
    float targetPos;
    float smoothedPos;
    float currentPos;
    float outputValue;
    // smoothing
    float alpha = 5;
    bool smoothMode = true;
    bool invert;
    FilterOnePole faderFilter;
    // tuning
    float deadZone;
    float searchTime = 80;
    float searchDelay = 100;
    int motorCutOff = 75;
    float dampZone = 20;
    float dampReduction = 0.95f;
    float margin;
    // moving bools
    bool isMoving = false;
    bool secondMove = false;
    unsigned long secondcheck = 0;
    // PID
    double pidInput;
    double pidOutput;
    double pidSetpoint;
    PID* pid;
    Preferences prefs;

    mFader(int uPin, int dPin, int pPin, int tPin, float max, float min, bool Invert = false);

    float linearizePosition(float raw);
    void startup(String nameSpace);
    float getCurrentPos(float Scale = 255);
    float getCurrentPosRaw();
    float updatePosition();
    void Update();
    void applyMotor(double cmd, float error);
    void GoTo(float target);
    void stopMotor();
    void setMax();
    void setMin();
    void calibrateCurveMidpoint();
    void testMotor();
    void testMotorUp();
};

#endif
