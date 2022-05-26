#include "PSD.h"
#include "Arduino.h"
#include "Macros.h"

PSD::PSD()
{
}

float PSD::PSDRead()
{
    for (unsigned short i = 0; i < 20; i++)
    {
        adc_key_in = analogRead(PSDPin);
        e = adc_key_in * 3.3 / 4096.0;
        E = E + e;
    }
    E = E / 20.0;
    float H = 8.1892 * pow(E, 4) - 64.14 * pow(E, 3) + 188.5 * pow(E, 2) - 247.73 * E + 124.94;
    if (!(H > 1.0 && H < 5.5))
    {
        H = H_0;
    }
    H_0 = H;
    return H;
}