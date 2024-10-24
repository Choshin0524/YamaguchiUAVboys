#ifndef SDCardModule_h
#define SDCardModule_h

#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class SDCardModule
{
public:
    SDCardModule();
    void SDCardInitialize(const char *message);
    void Write(File &file, const char *message);
    void WriteData(File &file, const float &data);
};

#endif
