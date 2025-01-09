#include "Sbus.h"
#include "Arduino.h"

Sbus::Sbus()
{
}

bool Sbus::SbusRead(HardwareSerial &uart)
{
    //  buffer[0] check
    while (true)
    {
        if (uart.available() > 0)
        {
            dataBuffer[0] = uart.read();
        }
        if (dataBuffer[0] == 0x0f)
        {
            break;
        }
    }

    //  buffer data read
    unsigned int bufferPos = 1;
    while (uart.available())
    {
        dataBuffer[bufferPos++] = uart.read();
    }

    chBuffer[0] = ((dataBuffer[1] | dataBuffer[2] << 8) & 0x07FF);
    chBuffer[1] = ((dataBuffer[2] >> 3 | dataBuffer[3] << 5) & 0x07FF);
    chBuffer[2] = ((dataBuffer[3] >> 6 | dataBuffer[4] << 2 | dataBuffer[5] << 10) & 0x07FF);
    chBuffer[3] = ((dataBuffer[5] >> 1 | dataBuffer[6] << 7) & 0x07FF);
    chBuffer[4] = ((dataBuffer[6] >> 4 | dataBuffer[7] << 4) & 0x07FF);
    chBuffer[5] = ((dataBuffer[7] >> 7 | dataBuffer[8] << 1 | dataBuffer[9] << 9) & 0x07FF);
    chBuffer[6] = ((dataBuffer[9] >> 2 | dataBuffer[10] << 6) & 0x07FF);
    chBuffer[7] = ((dataBuffer[10] >> 5 | dataBuffer[11] << 3) & 0x07FF);
    chBuffer[8] = ((dataBuffer[12] | dataBuffer[13] << 8) & 0x07FF);
    chBuffer[9] = ((dataBuffer[13] >> 3 | dataBuffer[14] << 5) & 0x07FF);
    chBuffer[10] = ((dataBuffer[14] >> 6 | dataBuffer[15] << 2 | dataBuffer[16] << 10) & 0x07FF);
    chBuffer[11] = ((dataBuffer[16] >> 1 | dataBuffer[17] << 7) & 0x07FF);
    return true;
}

int16_t Sbus::GetCh(unsigned int chNum) const
{
    return this->chBuffer[chNum];
}

int16_t Sbus::GetOffset(unsigned int offsetNum) const
{
    return this->offset[offsetNum];
}

void Sbus::DataMonitor(bool ifCheck) const
{
    if (ifCheck == true)
    {
        for (int i = 0; i < 12; i++)
        {
            Serial.print(GetCh(i));
            Serial.print("--");
        }
        Serial.println();
    }
}
