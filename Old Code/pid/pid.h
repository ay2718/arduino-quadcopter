#ifndef pid_h
#define pid_h

class pid{
public:
    pid();
    pid(float, float, float);
    float getErr(){return err;};
    float setErr(float var, float setp, float dt, float deriv);
private:
    const float KP, KI, KD;
    float iErr, err;
};

#endif // pid_h
