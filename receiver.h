#include "Arduino.h"
#ifndef receiver_h
#define receiver_h

class Receiver{
    public:
        Receiver();
        // We use timer 2 to read radio time signals
        void timer2Setup();

        void waitMinMax(float, float, int);

        // These functions control the pin change interrupt buffer
        void pciSetup();
        void pciClear();
        void pciEnable();
        void pciDisable();

        // Analyzes and clears the circular buffer to get updated radio signals
        bool rcAnalyze();
        
        // Writes current thrust values to some buffer
        bool getRCValues(float*);

    private:
        // record of when servo pins were turned on
        static uint16_t startValues[4];

        // last time data was received
        static uint32_t lastDataTimeStamp;

        // records whether current servo channels are recording
        static bool isRecording[4];

        // Most updated servo values
        static float rcValues[4];
};

// The following variables are not static class members to decrease latency
// Temporary variables for timing radio signals
extern volatile uint8_t PCINT0_timer2Time;
extern volatile uint16_t PCINT0_timer2Ovf;

// Timer 2 is an 8-bit counter, and we need another 8 bits
extern volatile uint16_t timer2Ovf;

// Circular buffer for determining when radio signals are sent/received
extern volatile uint16_t rcTimes[16];
extern volatile uint8_t rcPins[16];
extern volatile uint8_t rcIndex;



#endif // receiver_h
