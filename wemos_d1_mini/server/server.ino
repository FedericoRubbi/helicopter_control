#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include "server.h"

AsyncWebServer server(80);

const char *ssid = "";
const char *password = "";

void recvMsg(uint8_t *data, size_t len)
{
    digitalWrite(LED_BUILTIN, LOW); // turn led on
    WebSerial.println("Received Data...");
    String d = "";
    for (int i = 0; i < len; i++)
        d += char(data[i]);
    digitalWrite(LED_BUILTIN, HIGH); // turn led off
}

void setup()
{
    Serial.begin(BAUDRATE);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn led off
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.printf("[DEBUG] WiFi setup failed.");
        return;
    }
    Serial.println("[DEBUG] IP Address: ");
    Serial.println(WiFi.localIP()); // WebSerial is accessible at "<IP Address>/webserial"
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
    server.begin();
}

void loop()
{
    WebSerial.println("[DEBUG] Running.");
    delay(UPDATE_MS);
}