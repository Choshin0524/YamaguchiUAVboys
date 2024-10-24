#ifndef YUBMath_h
#define YUBMath_h
#include "Arduino.h"

int ServoMap(const int &input, const int &inputMIN, const int &inputMAX, const int &offsetCut)
{
    int mid = (inputMAX - inputMIN) / 2 + inputMIN;

    if (inputMIN != inputMAX)
    {
        return 90 + (input - mid) * (90 - offsetCut) / (mid - inputMIN);
    }
    else
    {
        return -1;
    }
}

int ServoReverse(const int &input)
{
    if (input >= 0 && input <= 180)
    {
        return 180 - input;
    }
    else
    {
        return -1;
    }
}
#endif