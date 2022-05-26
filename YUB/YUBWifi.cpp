#include "YUBWifi.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Arduino.h"

YUBWifi::YUBWifi()
{
    WiFi.softAP(ssid, pass);           // SSIDとパスの設定
    delay(100);                        // 追記：このdelayを入れないと失敗する場合がある
    WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクの設定
    Serial.print("AP IP address: ");
    IPAddress myIP = WiFi.softAPIP();
    Serial.println(myIP);
    Serial.println("Starting UDP");
    UDP1.begin(111); // UDP通信の開始(引数はポート番号)
    delay(100);
}

void YUBWifi::WifiRead()
{
    if (!UDP1.parsePacket())
    {
        x1H = UDP1.read(); // wifiデータ受信(1バイト)
    }
}

byte YUBWifi::GetWifiData() const
{
    return x1H;
}