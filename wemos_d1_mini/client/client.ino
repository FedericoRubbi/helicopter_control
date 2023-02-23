#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include "client.h"

bool new_command_flag = false; // set when server sends status code 201

bool read_premble()
{
    uint8_t preamble_index = 0;
    while (true) // hang execution until some data is read
    {
        if (Serial.available())
        {
            uint8_t character = Serial.read();
            if (character != preamble[preamble_index]) // character not matching pattern
                return false;
            else if (preamble_index == PREAMBLE_SIZE - 1) // matched end of preamble
                return true;
            preamble_index++;
        }
    }
}

bool connect()
{
    bool connected = false;
    while (!connected)
    { // hang execution until connected
        if (read_premble())
        {
            Serial.write(CONNECTED_FLAG); // reply to preamble
            return true;
        }
    }
    return false; // should never occur
}

bool readPacket(struct Packet_s *packet)
{
    for (uint8_t index = 0; index < PACKET_SIZE;) // read the entire packet one character at a time
        ((uint8_t *)packet)[index] = Serial.read();

    uint8_t eop_flag = Serial.read(); // read next character
    if (eop_flag != EOP_FLAG)
        return false; // error occured, some bytes are gone lost
    else
        return true;
}

bool writeCommand(Cmd_t *cmd)
{
    uint8_t index = 0;
    return Serial.write(cmd->cmd_code);
    // for (uint i = 0; i < CMD_DATA_SIZE) // send data
    //     Serial.write(cmd->data[i]);
}

void setup()
{
    Serial.begin(BAUDRATE);
    WiFi.begin(STASSID, STAPSK);
    while (WiFi.status() != WL_CONNECTED) // hang execution
        delay(500);
    // Serial.print("Connected! IP address: ");
    // Serial.println(WiFi.localIP());
}

void getCommand(HTTPClient &http)
{
    if (!new_command_flag)
        return;
    int httpCode = http.GET();

    if (httpCode > 0)
    {
        if (httpCode == HTTP_CODE_OK)
        {
            String serverCommand = http.getString();
            writeCommand((Cmd_t *)serverCommand.c_str());
            new_command_flag = false;
        }
    }
    else
        ; // error
}

void postSensorData(HTTPClient &http)
{
    static struct RawData_s data; // packet must be unserialized
    readPacket((struct Packet_s *)&data);
    String sensor_data = "time: ";
    sensor_data += data.time.hour;
    sensor_data += ":";
    sensor_data += data.time.minute;
    sensor_data += "\taltitude:";
    sensor_data += data.altitude;
    int httpCode = http.POST(sensor_data); // start connection and send HTTP header and body

    if (httpCode > 0) // httpCode will be negative on error
    {
        if (httpCode == HTTP_CODE_OK) // data posted, there's no new command
            return;
        else if (httpCode == 201) // data posted, new command available
            getCommand(http);
    }
    else
        ; // access http.errorToString(httpCode).c_str() to debug;
}

void loop()
{
    if ((WiFi.status() == WL_CONNECTED)) // wait for WiFi connection
    {
        WiFiClient client;
        HTTPClient http;
        http.begin(client, "http://" SERVER_IP "/data-monitor/"); // HTTP
        http.addHeader("Content-Type", "application/json");
        postSensorData(http);
        getCommand(http);
        http.end();
    }

    delay(1000);
}