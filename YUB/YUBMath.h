#ifndef YUBMath_h
#define YUBMath_h
#include "Arduino.h"

int ServoMap(int input, int inputMIN, int inputMAX, int offsetCut)
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

int ServoReverse(int input)
{
    if (input >= 0 && input <= 180)
    {
        return 180 - input;
    }
}
#endif