#include "mpu.h"
#include "I2C.h"

const uint8_t MPU_ADDR = 0x68;

Mpu::Mpu() : position(0.f, 0.f, 1.f), icorr(0.0375f, -0.0091f, 0.0058f){}

void mpuDataReady() {
    mpuInterrupt = true;
}


void Mpu::interruptSetup() {
    attachInterrupt(0, mpuDataReady, RISING);
}

void Mpu::setup() {
    I2c.begin();
    I2c.setSpeed(true);
    I2c.timeOut(2);

    // Wakes up the gyro
    I2c.write(MPU_ADDR, (uint8_t)0x6B, (uint8_t)0x00);

    // sets refresh rate to 1 kHz / (1 + SMPRT_DIV)
    I2c.write(MPU_ADDR, (uint8_t)0x19, SMPRT_DIV);

    // Digital low pass filter set to 100 Hz (200 Hz is 0x01)
    I2c.write(MPU_ADDR, (uint8_t)0x1A, (uint8_t)0x03);

    // Gyro config to 2000 deg/s, gyro to 16g
    I2c.write(MPU_ADDR, (uint8_t)0x1B, (uint8_t)0x18);
    I2c.write(MPU_ADDR, (uint8_t)0x1C, (uint8_t)0x18);

    // Data ready interrupt
    I2c.write(MPU_ADDR, (uint8_t)0x38, (uint8_t)0x01);
}

void Mpu::readBuffer() {
    I2c.read(MPU_ADDR, (uint8_t)0x3B, (uint8_t)14, buf);

    gyroLast.x = gyro.x;
    gyroLast.y = gyro.y;
    gyroLast.z = gyro.z;

    accel.x = float(int16_t(buf[0] << 8 | buf[1]))*rawAccel;
    accel.y = float(int16_t(buf[2] << 8 | buf[3]))*rawAccel;
    accel.z = float(int16_t(buf[4] << 8 | buf[5]))*rawAccel;

    gyro.x = float(int16_t(buf[8] << 8 | buf[9]))*rawGyro;
    gyro.y = float(int16_t(buf[10] << 8 | buf[11]))*rawGyro;
    gyro.z = float(int16_t(buf[12] << 8 | buf[13]))*rawGyro;

    gyro += icorr;

    gyroDeriv.x = gyro.x - gyroLast.x;
    gyroDeriv.y = gyro.y - gyroLast.y;
    gyroDeriv.z = gyro.z - gyroLast.z;

    gyroDeriv *= 1/h;
}

void Mpu::integrateGyro() {
    gyrocorr = gyro;
    gyrocorr += corr;
    angle.increment(&gyrocorr, h);
    position.x = 2*(angle.w*angle.y + angle.x*angle.z);
    position.y = 2*(angle.y*angle.z - angle.w*angle.x);
    position.z = angle.w*angle.w - angle.x*angle.x - angle.y*angle.y + angle.z*angle.z;
    pcorr = position.cross(&accel);
    float pcorrMagnitude = pcorr.magnitude();
    if (pcorrMagnitude > pcorrMax) {
        pcorr *= 1/sqrt(pcorrMagnitude);
    }
    pcorr *= KP;
    corr.x = pcorr.x;
    corr.y = pcorr.y;
    corr.z = pcorr.z;
}

void Mpu::getPosition(Vector* v) {
    v -> x = position.x;
    v -> y = position.y;
    v -> z = position.z;
}

void Mpu::getGyro(Vector* v) {
    v -> x = gyro.x;
    v -> y = gyro.y;
    v -> z = gyro.z;
}

void Mpu::getAccel(Vector* v) {
    v -> x = accel.x;
    v -> y = accel.y;
    v -> z = accel.z;
}

void Mpu::getGyroDeriv(Vector* v) {
    v -> x = gyroDeriv.x;
    v -> y = gyroDeriv.y;
    v -> z = gyroDeriv.z;
}

Quaternion::Quaternion() {
    x = y = z = 0;  w = 1;
}

void Quaternion::normalize() {
    float amplitude = w*w + x*x + y*y + z*z;
    amplitude = (1.5 - 0.5*amplitude);
    w *= amplitude;
    x *= amplitude;
    y *= amplitude;
    z *= amplitude;
}

void Quaternion::increment(Vector* _v, float h) {
    h *= -0.5;
    Vector v(h*_v->x, h*_v->y, h*_v->z);

    float ww, xx, yy, zz;
    ww = w - (x*(v.x) + y*(v.y) + z*(v.z));
    xx = (v.y)*z - (v.z)*y + x + (v.x)*w;
    yy = (v.z)*x - (v.x)*z + y + (v.y)*w;
    zz = (v.x)*y - (v.y)*x + z + (v.z)*w;
    
    w = ww; x = xx; y = yy; z = zz;

    normalize();
}

Vector::Vector() {
    x = y = z = 0;
}

Vector::Vector(float* v) {
    x = v[0]; y = v[1]; z = v[2];
}

Vector::Vector(float xx, float yy, float zz) {
    x = xx; y = yy; z = zz;
}

Vector::Vector(Vector* v) {
    x = v -> x; y = v -> y; z = v -> z;
}

Vector Vector::cross(Vector* v) {
    Vector w;
    w.x = y*(v -> z) - z*(v -> y);
    w.y = z*(v -> x) - x*(v -> z);
    w.z = x*(v -> y) - y*(v -> x);
    return w;
}
