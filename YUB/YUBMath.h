#ifndef YUBMath_h
#define YUBMath_h
#include "Arduino.h"

int ServoMap(int input, int inputMIN, int inputMAX, int offsetCut)
    {
        int mid = (inputMAX - inputMIN) / 2 +  inputMIN;

        if(inputMIN < inputMAX)
        {
            return 90 + (input - mid) * (90 - offsetCut) / (mid - inputMIN);
        }
        else if(inputMIN > inputMAX)
        {
            return 90 - (input - mid) * (90 - offsetCut) / (mid - inputMIN);
        }
        else
        {
            return -1;
        }
    }

#endif