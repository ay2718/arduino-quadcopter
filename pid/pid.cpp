#include "pid.h"

#define UPPER 0.1

pid::pid() : KP(0), KI(0), KD(0){
	iErr = err = 0;
}

pid::pid(float p, float i, float d) : KP(p), KI(i), KD(d){
	iErr = err = 0;
};

float pid::setErr(float var, float setp, float dt, float deriv){
	err = var - setp;
	iErr += dt*err*KI;
	
	if(iErr > UPPER)
		iErr = UPPER;
	
	if(iErr < -UPPER)
		iErr = -UPPER;
	
	err = KP*err + iErr + deriv*KD;
	return err;
}
