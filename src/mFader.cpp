#include "mFader.h"

mFader::mFader(int uPin, int dPin, int pPin, int tPin, float max, float min, bool Invert)
    : faderFilter(LOWPASS, alpha) {
    upControlPin = uPin;
    downControlPin = dPin;
    potPin = pPin;
    limitMax = max;
    limitMin = min;
    touchPin = tPin;

    pinMode(potPin, INPUT);
    if(uPin && dPin){
        pinMode(upControlPin, OUTPUT);
        pinMode(downControlPin, OUTPUT);
        

        //ledcAttach(upControlPin, PWM_FREQ, PWM_RES);
        //ledcAttach(downControlPin, PWM_FREQ, PWM_RES);

        ledcSetup(PWM_CH_UP, PWM_FREQ, PWM_RES);
        ledcSetup(PWM_CH_DN, PWM_FREQ, PWM_RES);

        ledcAttachPin(uPin, PWM_CH_UP);
        ledcAttachPin(dPin, PWM_CH_DN);
    }

    faderFilter.setToNewValue(1);

    invert = Invert;

    pidSetpoint = 0;
    pidInput = 0;
    pidOutput = 0;

    double Kp = 1.2;
    double Ki = 2;
    double Kd = 0.01;

    pid = new PID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

    pid->SetOutputLimits(-255, 255);
    pid->SetSampleTime(2);
    pid->SetMode(MANUAL);
}

float mFader::linearizePosition(float raw) {
    float y = raw / (float)(rawMax - rawMin);

    if (y < 0.0f) y = 0.0f;
    if (y > 1.0f) y = 1.0f;

    float linear = powf(y, 1.0f / curveOffset);
    return linear;
}

void mFader::startup(String nameSpace) {
    faderFilter.setFrequency(alpha);

    prefs.begin(nameSpace.c_str(), false);

    if (prefs.isKey("Max")) {
        rawMax = prefs.getFloat("Max");
    } else {
        rawMax = 2680;
        prefs.putFloat("Max", rawMax);
    }

    if (prefs.isKey("Min")) {
        rawMin = prefs.getFloat("Min");
    } else {
        rawMin = 50;
        prefs.putFloat("min", rawMin);
    }

    if (prefs.isKey("Margin")) {
        margin = prefs.getFloat("Margin");
    } else {
        margin = 4;
        prefs.putFloat("Margin", margin);
    }

    if (prefs.isKey("DeadZone")) {
        deadZone = prefs.getFloat("DeadZone");
    } else {
        deadZone = 5;
        prefs.putFloat("DeadZone", deadZone);
    }
}

float mFader::getCurrentPos(float Scale) {
    float rawRead;

    if (invert)
        rawRead = (rawMax - (analogRead(potPin) - rawMin));
    else
        rawRead = analogRead(potPin) - rawMin;

    float linear = rawRead / (rawMax - rawMin);

    if (smoothMode) {
        smoothedPos = faderFilter.input(linear) * Scale;
    }

    if (smoothedPos - margin < 0) {
        smoothedPos = 0;
    } else if (smoothedPos + margin > 255) {
        smoothedPos = 255;
    }

    float final = linear * Scale;

    return final;
}

float mFader::getCurrentPosRaw() {
    return analogRead(potPin);
}

float mFader::updatePosition() {
    currentPos = getCurrentPos();
    return currentPos;
}

void mFader::Update() {
    if (!isMoving && !secondMove) return;

    float proccessPos = currentPos;

    if (smoothMode) proccessPos = smoothedPos;

    pidInput = proccessPos;

    float error = targetPos - proccessPos;

    if ((proccessPos > limitMax - deadZone && error > 0) ||
        (proccessPos < limitMin + deadZone && error < 0)) {
        stopMotor();
        return;
    }

    pid->Compute();

    if (secondMove && secondcheck > millis()) return;

    applyMotor(pidOutput, error);

    if (abs(error) <= deadZone && isMoving) {
        secondMove = true;
        secondcheck = millis() + searchDelay;
        isMoving = false;

        ledcWrite(PWM_CH_UP, 255);
        ledcWrite(PWM_CH_DN, 255);
        return;
    }

    if (secondMove && abs(error) <= deadZone) {
        stopMotor();
        secondMove = false;
        return;
    }
}

void mFader::applyMotor(double cmd, float error) {
    if (abs(error) < dampZone) cmd *= dampReduction;

    if (cmd > deadZone) {
        cmd = ((abs(cmd) / 255) * motorCutOff) + 255 - motorCutOff;
        ledcWrite(PWM_CH_UP, cmd);
        ledcWrite(PWM_CH_DN, 0);
    } else if (cmd < (deadZone * -1)) {
        cmd = ((abs(cmd) / 255) * motorCutOff) + 255 - motorCutOff;
        ledcWrite(PWM_CH_DN, cmd);
        ledcWrite(PWM_CH_UP, 0);
    } else {
        ledcWrite(PWM_CH_UP, 255);
        ledcWrite(PWM_CH_DN, 255);
    }
}

void mFader::GoTo(float target) {
    if (target > limitMax || target < limitMin) return;

    float diff = target - smoothedPos;

    if (abs(diff) <= deadZone) {
        isMoving = false;
        secondMove = false;
        pid->SetMode(MANUAL);
        Serial.println("D:already at target");
        return;
    }

    targetPos = target;
    pidSetpoint = target;
    pid->SetMode(AUTOMATIC);
    isMoving = true;
    secondMove = false;

    Update();
}

void mFader::stopMotor() {
    isMoving = false;
    Serial.println("D:stopped");
    pid->SetMode(MANUAL);
    ledcWrite(PWM_CH_UP, 0);
    ledcWrite(PWM_CH_DN, 0);
}

void mFader::setMax() {
    rawMax = getCurrentPosRaw();
    prefs.putFloat("Max", rawMax);
}

void mFader::setMin() {
    rawMin = getCurrentPosRaw();
    prefs.putFloat("Min", rawMin);
}

void mFader::calibrateCurveMidpoint() {
    float rawMidPoint = getCurrentPosRaw();

    float y = (rawMax - (rawMidPoint - rawMin)) / (float)(rawMax - rawMin);

    if (y < 0.01f) y = 0.01f;
    if (y > 0.99f) y = 0.99f;

    float x = 0.5f;

    curveOffset = logf(y) / logf(x);

    Serial.print("Calibrated exponent: ");
    Serial.print(curveOffset);

    Serial.print(" calibrated y value: ");
    Serial.println(y);
}

void mFader::testMotor() {
    ledcWrite(PWM_CH_UP, 200);
    ledcWrite(PWM_CH_DN, 0);
    delay(100);
    ledcWrite(PWM_CH_UP, 0);
    ledcWrite(PWM_CH_DN, 200);
    delay(100);
    ledcWrite(PWM_CH_UP, 0);
    ledcWrite(PWM_CH_DN, 0);
}
void mFader::testMotorUp() {
    ledcWrite(PWM_CH_UP, 200);
    ledcWrite(PWM_CH_DN, 0);
    delay(1000);
    ledcWrite(PWM_CH_UP, 0);
    ledcWrite(PWM_CH_DN, 0);
}
