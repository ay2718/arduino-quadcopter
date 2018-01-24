#include "receiver.h"

// Interrupt function for pins 8-13, detects incoming radio signals and adds them to the circular buffer
ISR(PCINT0_vect){
    cli();
    PCINT0_timer2Time = TCNT2;
    PCINT0_timer2Ovf = timer2Ovf;
    if((TIFR2 & 1) && (PCINT0_timer2Time < 255)){
        PCINT0_timer2Ovf += 256;
    }
    sei();
    rcTimes[rcIndex] = PCINT0_timer2Ovf|PCINT0_timer2Time;
    rcPins[rcIndex] = PINB;
    rcIndex ++;
    rcIndex &= 0b00001111;
}

// Increases timer2Ovf when timer2 overflows
ISR(TIMER2_OVF_vect){
    timer2Ovf += 256;
}

// static receiver data
uint16_t Receiver::startValues[4] = {0, 0, 0, 0};
uint32_t Receiver::lastDataTimeStamp = 0;
bool Receiver::isRecording[4] = {false, false, false, false};
float Receiver::rcValues[4] = {1000, 1000, 1000, 1000};

Receiver::Receiver(){}

void Receiver::timer2Setup(){
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    TIMSK2 = (1 << TOIE2);
    TCCR2B |= (1 << CS21);
}

void Receiver::waitMinMax(float min, float max, int channel) {
    while (rcValues[channel] > min) {rcAnalyze();}
    while (rcValues[channel] < max) {rcAnalyze();}
    while (rcValues[channel] > min) {rcAnalyze();}
}

void Receiver::pciSetup(){
    PCMSK0 |= 0b00001111;
}

void Receiver::pciClear(){
    PCIFR |= 0b00000001;
}

void Receiver::pciEnable(){
    PCICR |= 0b00000001;
}

void Receiver::pciDisable(){
    PCICR &= 0b11111110;
}

bool Receiver::rcAnalyze(){
    uint8_t mask = 1;
    if (rcIndex > 0){
        lastDataTimeStamp = millis();
    }
    if (millis() - lastDataTimeStamp > 200) {
        return true;
    }
    for (uint8_t i = 0; i < 4; i++){
        for (uint8_t j = 0; j < rcIndex; j++){
            if (isRecording[i] && ((rcPins[j] & mask) == 0)){
                isRecording[i] = false;
                uint16_t interval = rcTimes[j] - startValues[i];
                if ((2000 <= interval) && (interval <= 4000))  rcValues[i] = float(interval)/2.f; 
            }
            if (!isRecording[i] && ((rcPins[j] & mask) == mask)){
                isRecording[i] = true;
                startValues[i] = rcTimes[j];
            }
        }
        mask <<= 1;
    }
    rcIndex = 0;
    return false;
}

bool Receiver::getRCValues(float* buf){
    if (rcAnalyze()) {return true;}
    
    for(uint8_t i = 0; i < 4; i++){
        buf[i] = rcValues[i];
    }

    return false;
}

