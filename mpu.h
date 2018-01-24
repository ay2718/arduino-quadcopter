#include "I2C.h"
#include "Arduino.h"
#ifndef mpu_h
#define mpu_h

extern volatile bool mpuInterrupt;

class Vector {
    public:
        Vector();
        Vector(float*);
        Vector(float, float, float);
        Vector(Vector*);
        float x, y, z;
        Vector cross(Vector*);
        float magnitude() {return x*x + y*y + z*z;}; // actually returns amplitude squared
        float dot(Vector* v) {return x*v -> x + y*v -> y + z*v -> z;};
        void operator*=(float val) {x *= val; y *= val; z *= val;};
        void operator+=(Vector& v) {x += v.x; y += v.y; z += v.z;};
        void operator-=(Vector& v) {x -= v.x; y -= v.y; z -= v.z;};
};

class Quaternion {
    public:
        Quaternion();
        float w, x, y, z;
        void normalize();
        void increment(Vector*, float);
};

class Mpu {
    public:
        Mpu(); // constructor
        void interruptSetup(); // set up interrupts
        void setup(); // set up chip 
        void readBuffer(); // read buffer
        void integrateGyro();
        void waitForInterrupt() { while (!mpuInterrupt); mpuInterrupt = false; };
        void getPosition(Vector*); // read position (vector)
        void getGyro(Vector*); // read gyro (vector, rad/s) 
        void getAccel(Vector*); // read accel (vector)
        void getGyroDeriv(Vector*); //read gyro derivative

        const uint8_t SMPRT_DIV = 0x09;
        const float h = (SMPRT_DIV + 1)/1000.f;
    private:
        uint8_t buf[14];

        const float KP = -0.2;
        const float pcorrMax = 0.001;

        const float rawGyro = 0.0010652644360316951;
        const float rawAccel = 0.00048828125;

        Vector accel;
        Vector position;
        Vector gyro;
        Vector gyroLast;
        Vector gyroDeriv;
        Vector gyrocorr;

        Vector corr;
        Vector pcorr;
        Vector icorr;

        Quaternion angle;
};

#endif // mpu_h
