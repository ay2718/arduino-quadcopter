#include "mpu.h"
#include "receiver.h"
#include "pid.h"
#include "Servo.h"

// Define if you want to debug
// #define DEBUG

// pid controls
#define KP 0.05f
#define KI 0.1f
#define KD 0.004f
#define KPY 0.1f
#define KIY 0.1f
#define KDY 0.003f
#define STAB 4.0f

Mpu mpu;
Receiver receiver;
Pid pitch (KP, KI, KD);
Pid roll (KP, KI, KD);
Pid yaw (KPY, KIY, KDY);
Vector position(0, 0, 1);
Vector gyro;
Vector deriv;

// this is true if a data ready interrupt has happened
volatile bool mpuInterrupt = false;

// The following variables are not static class members to decrease latency
// Temporary variables for timing radio signals
volatile uint8_t PCINT0_timer2Time;
volatile uint16_t PCINT0_timer2Ovf;

// Timer 2 is an 8-bit counter, and we need another 8 bits
volatile uint16_t timer2Ovf;

// Circular buffer for determining when radio signals are sent/received
volatile uint16_t rcTimes[16];
volatile uint8_t rcPins[16];
volatile uint8_t rcIndex = 0;



// motor control and pid begins here
float zcorr(float z){return 1.3333333333333f - 0.333333333333f*z;}
const int start_servo_pins = 4;
Servo motors[4];

int16_t thrustToServo(float thrust){
    if (thrust < 0.0) {return 1060;}
    if (thrust > 1.0) {thrust = 1.0;}
    return 1060 + int16_t(800*sqrt(thrust));
}

float rcValues[4];
bool isError = false;

float thr = 0.0;

float setP = 0.0;
float setR = 0.0;
float setY = 0.0;

float pErr = 0.0;
float rErr = 0.0;
float yErr = 0.0;

float xTarget = 0.0;
float yTarget = 0.0;

float zc;
float radius;

float thrValues[4];

const int thrChannel = 2;
const int pitchChannel = 1;
const int rollChannel = 0;
const int yawChannel = 3;

const float thrOffset = 1200.f;
const float pitchOffset = 1500.f;
const float rollOffset = 1500.f;
const float yawOffset = 1500.f;

const float thrScale = 0.0012f;
const float pitchScale = 0.0014f;
const float rollScale = 0.0014f;
const float yawScale = 0.01f;

void trim(float &val) {
    if (val > 1.0) val = 1.0;
    if (val < 0.0) val = 0.0;
}

#ifdef DEBUG
uint32_t nextMillis = 0;
#endif

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    Serial.println(mpu.h);
    pinMode(13, OUTPUT);
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);
    pinMode(11, INPUT_PULLUP);
    digitalWrite(13, LOW);
    
    cli();
    receiver.pciSetup();
    receiver.pciClear();
    receiver.pciEnable();
    receiver.timer2Setup();
    sei();

    mpu.interruptSetup();
    mpu.setup();

    for(int i=0; i<4; i++){
        motors[i].attach(i + start_servo_pins);
        motors[i].writeMicroseconds(1000);
    }

    delay(3000);

    receiver.waitMinMax(1200., 1750., 2);

    digitalWrite(13, HIGH);
}

void loop() {
   
    mpu.waitForInterrupt();

    mpu.readBuffer();
    mpu.integrateGyro();

    mpu.getPosition(&position);
    mpu.getGyro(&gyro);
    mpu.getGyroDeriv(&deriv);

    if (receiver.getRCValues(rcValues)) {
        digitalWrite(13, LOW);
        for (uint8_t i = 0; i < 4; i++) thrValues[i] = 0;
        thr = 0;
    } else {
        digitalWrite(13, HIGH);
        thr = rcValues[thrChannel] > thrOffset ? float(rcValues[thrChannel] - thrOffset)*thrScale:0.0;
        setP = float(rcValues[pitchChannel] - pitchOffset)*pitchScale;
        setR = float(rcValues[rollChannel] - rollOffset)*rollScale;
        setY = float(rcValues[yawChannel] - yawOffset)*yawScale;

        zc = zcorr(position.z);
        radius = setP*setP + setR*setR;
        if (radius > 0.49f) {
            radius = 0.7/sqrt(radius);
            setP = setP*radius;
            setR = setR*radius;
        }

        xTarget = zc*position.x - setP;
        yTarget = -zc*position.y - setR;

        pErr = pitch.setErr(gyro.y, STAB*xTarget - position.y*setY, mpu.h, deriv.y); 
        rErr = roll.setErr(gyro.x, STAB*yTarget - position.x*setY, mpu.h, deriv.x);
        yErr = yaw.setErr(gyro.z, -position.z*setY, mpu.h, deriv.z);
        
        thrValues[0] = thr - pErr + rErr + yErr;
        thrValues[1] = thr - pErr - rErr - yErr;
        thrValues[2] = thr + pErr - rErr + yErr;
        thrValues[3] = thr + pErr + rErr - yErr;
    }

    if (thr > 0.01) {
        for (uint8_t i = 0; i < 4; i++) {
            motors[i].writeMicroseconds(thrustToServo(thrValues[i]));
        }
    } else {
        pitch.resetIErr();
        roll.resetIErr();
        yaw.resetIErr();

        for (uint8_t i = 0; i < 4; i++) {
            motors[i].writeMicroseconds(1000);
        }
    }

#ifdef DEBUG
    if (nextMillis < millis()) {
        nextMillis = millis() + 200;
//         Serial.print(position.x, 4);
//         Serial.print('\t');
//         Serial.print(position.y, 4);
//         Serial.print('\t');
//         Serial.print(position.z, 4);
//         Serial.print("\t\t");
//         
//         Serial.print(gyro.x, 4);
//         Serial.print('\t');
//         Serial.print(gyro.y, 4);
//         Serial.print('\t');
//         Serial.print(gyro.z, 4);
//         Serial.print("\t\t");

        Serial.print(rcValues[0]);
        Serial.print('\t');
        Serial.print(rcValues[1]);
        Serial.print('\t');
        Serial.print(rcValues[2]);
        Serial.print('\t');
        Serial.print(rcValues[3]);
        Serial.print('\t');

        Serial.println();
    }
#endif
}
