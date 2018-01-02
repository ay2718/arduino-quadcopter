#include "mpu.h"
#include "Arduino.h"
//#include "I2C.h"
#include "Wire.h"

// 1000 deg/s, 200 hz
#define G2DR 5.3263221801584764920766930190693e-6f
#define MPA 0x68

// accel
#define A2DCM 0.000244140625f

// proportional constant for gyro
#define KP 0.002f

void write(uint8_t device, uint8_t reg, uint8_t dat){
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.write(dat);
    Wire.endTransmission();
}

void write(uint8_t device, uint8_t reg, uint8_t *dat, uint8_t num){
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.write(dat, num);
    Wire.endTransmission();
}

void read(uint8_t device, uint8_t reg, uint8_t num, uint8_t *dat){
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(device, num);
    for(uint8_t i = 0; i < num; i++){
        dat[i] = Wire.read();
    }
}

mpu::mpu(){gravity.x = gravity.y = 0.f; gravity.z = -1.f;};

// 0x6B set to 0 to turn on
// 0x1B set to ______ for sensitivity of 1000 deg/s
void mpu::mpuBegin(){
/*
    I2c.begin();
    I2c.setSpeed(0);
    I2c.pullup(1);
    I2c.write(MPA, 0x6B, 0x00);
    I2c.write(MPA, 0x1B, 0x10);
    I2c.write(MPA, 0x1C, 0x10);
    I2c.write(MPA, 0x38, 0x01);
    I2c.write(MPA, 0x19, 0x09);
    I2c.write(MPA, 0x1A, 0x01);
*/

    Wire.begin();
    TWBR = 24;
    write(MPA, 0x6B, 0x00);
    write(MPA, 0x1B, 0x10);
    write(MPA, 0x1C, 0x10);
    write(MPA, 0x38, 0x01);
    write(MPA, 0x19, 0x4F);
    write(MPA, 0x1A, 0x00);

    int16_t xg, yg, zg, xa, ya, za;
    xg = 68;
    yg = -23;
    zg = 12;

    xa = 1793;
    ya = -490;
    za = 803;

    uint8_t g[6];
    uint8_t a[6];
    uint8_t *gp = g;
    uint8_t *ap = a;

    g[0] = xg >> 8; g[1] = xg;
    g[2] = yg >> 8; g[3] = yg;
    g[4] = zg >> 8; g[5] = zg;

    a[0] = xa >> 8; a[1] = xa;
    a[2] = ya >> 8; a[3] = ya;
    a[4] = za >> 8; a[5] = za;

    write(MPA, 0x13, gp, 6);

    write(MPA, 0x06, ap, 6);
};

// 0x3B to 0x40
void mpu::readAccels(){

	//read mpu
	uint8_t a[6];  //temporary storage
	uint8_t *ap = a;

	//read mpu
	read(MPA, 0x3B, 6, ap);

    int ax = a[0] << 8|a[1];
	int ay = a[2] << 8|a[3];
	int az = a[4] << 8|a[5];


	accels.x = A2DCM * float(ax);
	accels.y = A2DCM * float(ay);
	accels.z = A2DCM * float(az);

};

// 0x43 to 0x48
void mpu::readGyros(){
	uint8_t g[6];  //temporary storage
	uint8_t *gp = g;

	//read mpu
	read(MPA, 0x43, 6, gp);

    int gx = g[0] << 8|g[1];
	int gy = g[2] << 8|g[3];
	int gz = g[4] << 8|g[5];

	gyros.x = -G2DR * float(gx);
	gyros.y = -G2DR * float(gy);
	gyros.z = -G2DR * float(gz);

	rates = gyros;
	rates.multiply(100.f);

};

void mpu::compute(){
	//gyro corrections
	gyros.add(gyro_corr);

	//rotation
	gravity.rotate(gyros);
    gyro_corr = accels.cross(gravity);
	gyro_corr.multiply(KP);
};

Vector::Vector(){x = y = z = 0.f;}

float Vector::dot(Vector& v){
	return x*x + y*y + z*z;
}

void Vector::rotate(Vector& w){
	float tx = x;
	float ty = y;
	float tz = z;

	tx -= y * w.z;
	tx += z * w.y;

	ty -= z * w.x;
	ty += x * w.z;

	tz -= x * w.y;
	tz += y * w.x;

	x = tx;
	y = ty;
	z = tz;

	normalize();
}

Vector Vector::cross(Vector& v){
	Vector u;

	u.x = y*v.z - z*v.y;
	u.y = z*v.x - x*v.z;
	u.z = x*v.y - y*v.x;

	return u;
}

void Vector::add(Vector& v){
	x += v.x;
	y += v.y;
	z += v.z;
}

void Vector::multiply(float c){
	x *= c;
	y *= c;
	z *= c;
}

void Vector::normalize(){
	multiply((3.f - x*x - y*y - z*z)*0.5f);
}


