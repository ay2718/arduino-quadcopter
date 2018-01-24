#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "pid.h"
#include "Servo.h"

MPU6050 mpu;

//uncomment for serial outputs
//#define DEBUG

// pid controls
#define KP 0.05f
#define KI 0.1f
#define KD 0.004f
#define KPY 0.1f
#define KIY 0.1f
#define KDY 0.003f
#define STAB 4.0f
#define G2R 0.001065f
#define STEP 0.01f
#define FREQ 100.0f;

float zcorr(float z){return 1.3333333333333f + 0.333333333333f*z;}

const int start_servo_pins = 4;

bool noMpuErr = false;
volatile bool noRcErr = false;

void timer2Setup(){
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TIMSK2 = (1 << TOIE2);
  TCCR2B |= (1 << CS21);
}

void pciSetup(){
  PCMSK0 |= 0b00001111;
}

void pciClear(){
  PCIFR |= 0b00000001;
}

void pciEnable(){
  PCICR |= 0b00000001;
}

void pciDisable(){
  PCICR &= 0b11111110;
}

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile uint16_t ovf = 0;
volatile uint16_t rcTimes[16];
volatile byte rcPins[16];
volatile byte rcIndex = 0;
uint16_t startValues[4] = {0, 0, 0, 0};
float rcValues[4] = {1000, 1000, 1000, 1000};
bool isRecording[4] = {false, false, false, false};

//Converts servo value to thrust value
int thrToServ(float th){
  if(th < 0.015625)
    return 1100;
  if(th > 1)
    th = 1;
  return 1060 + int(800.*sqrt(th));
}

volatile bool mpuInterrupt  = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

ISR(PCINT0_vect){
  cli();
  byte t = TCNT2;
  uint16_t m = ovf;
  if((TIFR2 & 1) && (t < 255)){
    m += 256;
  }
  sei();
  rcTimes[rcIndex] = m|t;
  rcPins[rcIndex] = PINB;
  rcIndex ++;
  rcIndex &= 0b00001111;
}

ISR(TIMER2_OVF_vect){
  ovf += 256;
}

uint32_t lastData = 0;

void rcAnalyze(){
  byte mask = 1;
  if(rcIndex > 0){
    lastData = millis();
  }
  for(byte i = 0; i < 4; i++){
    for(byte j = 0; j < rcIndex; j++){
      if(isRecording[i] && ((rcPins[j] & mask) == 0)){
        isRecording[i] = false;
        uint16_t interval = rcTimes[j] - startValues[i];
        if((2000 <= interval) && (interval <= 4000))  rcValues[i] = float(interval)/2.f; 
      }
      if(!isRecording[i] && ((rcPins[j] & mask) == mask)){
        isRecording[i] = true;
        startValues[i] = rcTimes[j];
      }
    }
    mask <<= 1;
  }
  rcIndex = 0;
}

Quaternion q;
VectorFloat gravity;
int gyro[3];
float rad_gyro[3];

float thr, setP, setR, setY;

float gyro_last[3];
float deriv[3] = {0, 0, 0};

bool failure = false;

pid pitch(KP, KI, KD);
pid roll(KP, KI, KD);
pid yaw(KPY, KIY, KDY);

float thValues[4];

Servo motors[4];

float last_loop; //safety:  if main loop stops running, quadcopter cuts all motors.

void setup() {
  #ifdef DEBUG
    Serial.begin(38400);
  #endif
  Wire.begin();
  TWBR = 24;
  
  //sets pin inputs and outputs
  DDRD = DDRD | 0b11110000;
  DDRB = DDRB & 0b11110000;
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  cli();
  pciSetup();
  pciClear();
  pciEnable();
  timer2Setup();
  sei();
  
  thr = setP = setR = setY = 0;
  
  mpu.initialize();
  failure = !mpu.testConnection();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-23);
  mpu.setZGyroOffset(12);
  mpu.setXAccelOffset(1793);
  mpu.setYAccelOffset(-490);
  mpu.setZAccelOffset(803);

    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  
  for(int i=0; i<4; i++){
    motors[i].attach(i + start_servo_pins);
    motors[i].writeMicroseconds(1000);
  }
  delay(3000);
  
  while(rcValues[2] < 1750){rcAnalyze();}
  while(rcValues[2] > 1200){rcAnalyze();}
  
  digitalWrite(13, HIGH);
  
  last_loop = millis();
}

void loop() {
  noMpuErr = ((millis() - last_loop) < 100);
  noRcErr = ((millis() - lastData) < 200);
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize);
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      last_loop = millis();
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);
      
      mpu.dmpGetGravity(&gravity, &q);
      for(int i=0; i<3; i++){
        rad_gyro[i] = gyro[i]*G2R;
      }
      
      rcAnalyze();

      thr = rcValues[2] > 1200?float(rcValues[2] - 1200)*0.0012:0.0;
      
      
      for(int i=0; i<3; i++){
        deriv[i] = (rad_gyro[i] - gyro_last[i])*FREQ;
        gyro_last[i] = rad_gyro[i];
      }
      
      if(thr > 0.01){
      
        setP = float(rcValues[1] - 1500)*0.0014f;
        setR = float(rcValues[0] - 1500)*0.0014f;
        setY = float(rcValues[3] - 1500)*.01f;

        float zc = zcorr(gravity.z);

        float radius = setP*setP + setR*setR;
        if(radius > 0.49f){
          float cfact = 0.7/sqrt(radius);
          setP = setP*cfact;
          setR = setR*cfact;
        }

        float xtarget = zc*gravity.x - setP;
        float ytarget = -zc*gravity.y - setR;
        
        float pErr = pitch.setErr(rad_gyro[1], STAB*xtarget + gravity.y*setY, STEP, deriv[1]);
        float rErr = roll.setErr(rad_gyro[0], STAB*ytarget - gravity.x*setY, STEP, deriv[0]);
        float yErr = yaw.setErr(rad_gyro[2], -setY, STEP, deriv[2]);
        
        /* 0 ------- 1
                |
                |
           3 ------- 2 */
        
        thValues[0] = thr - pErr + rErr + yErr;
        thValues[1] = thr - pErr - rErr - yErr;
        thValues[2] = thr + pErr - rErr + yErr;
        thValues[3] = thr + pErr + rErr - yErr;
      }
      noMpuErr = true;
      for(int i=0; i<4; i++){
        int serv_val;
        if((thr > 0.015625) && noMpuErr && noRcErr)
          serv_val = thrToServ(thValues[i]);
        else
          serv_val = 1000;
        motors[i].writeMicroseconds(serv_val);
        #ifdef DEBUG
          Serial.print(rcValues[i]);
          Serial.print("\t");
        #endif
      }
      #ifdef DEBUG
      if(!noRcErr)
        Serial.print("AAAAAAAAAAAAAAAAAA");
        Serial.println(noRcErr);
      #endif
  }
  if(!noMpuErr){
    thr = 0;
    for(int i=0; i<4; i++){
      motors[i].writeMicroseconds(1000);
    }
  }

}
