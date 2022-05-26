#ifndef PSD_h
#define PSD_h
#include "Arduino.h"

class PSD
{
public:
    PSD();
    float PSDRead();
private:
    int adc_key_in;
    float e;
    float E = 0.0;
    float H_0 = 0.0;
};

#endif
