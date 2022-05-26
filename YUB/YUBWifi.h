#ifndef YUBWifi_h
#define YUBWifi_h
#include "Arduino.h"
#include <WiFi.h>
#include <WiFiUdp.h>

class YUBWifi
{
public:
    YUBWifi();
    void WifiRead();
    byte GetWifiData() const;
private:
    byte x1H = 255;
    const char* ssid = "";                   // SSID
    const char* pass = "";                   // password
    const IPAddress ip(192, 168, 4, 2);       // IPアドレス(ゲートウェイも兼ねる)
    const IPAddress subnet(255, 255, 255, 0); // サブネットマスク
    const char* udpAddress = "192.168.4.2";
    WiFiUDP UDP1;
}

#endif
