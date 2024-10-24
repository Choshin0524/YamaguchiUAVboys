#include "SDCardModule.h"
#include "Arduino.h"

SDCardModule::SDCardModule()
{
}

void SDCardModule::SDCardInitialize(const char *message)
{
    if (!SD.begin())
    {
        Serial.println("Card Mount Failed");
    }
    else
    {
        Serial.println("Card OK!");
    }
    File file1 = SD.open("/flightDataRPY.txt", FILE_APPEND);
    File file2 = SD.open("/flightDataCTL.txt", FILE_APPEND);
    if (!file1 || !file2)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (message == nullptr)
    {
        Serial.println("No message.");
        file1.close();
        file2.close();
        return;
    }

    file1.print(message);
    file2.print(message);
    file1.close();
    file2.close();
}

void SDCardModule::Write(File &file, const char *message)
{
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (message == nullptr)
    {
        Serial.println("No message.");
        return;
    }
    if (!file.print(message))
    {
        Serial.println("Append failed");
    }
}

void SDCardModule::WriteData(File &file, const float &data)
{
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (!file.print(data))
    {
        Serial.println("Append failed");
    }
}
