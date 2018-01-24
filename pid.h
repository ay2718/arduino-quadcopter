#include "Arduino.h"
#ifndef pid_h
#define pid_h

class Pid{
public:
    Pid();
    Pid(float, float, float);
    float getErr(){return err;};
    float setErr(float var, float setp, float dt, float deriv);
    void resetIErr() {iErr = 0.f;};
private:
    const float KP, KI, KD;
    float iErr, err;
};

#endif // pid_h
