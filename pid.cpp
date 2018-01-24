#include "pid.h"

#define UPPER 0.1

Pid::Pid() : KP(0), KI(0), KD(0){
	iErr = err = 0;
}

Pid::Pid(float p, float i, float d) : KP(p), KI(i), KD(d){
	iErr = err = 0;
};

float Pid::setErr(float var, float setp, float dt, float deriv){
	err = var - setp;
	iErr += dt*err*KI;
	
	if(iErr > UPPER)
		iErr = UPPER;
	
	if(iErr < -UPPER)
		iErr = -UPPER;
	
	err = KP*err + iErr + deriv*KD;
	return err;
}
