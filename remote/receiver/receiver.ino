#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include "receiver.h"

// Read packet from TX and print to serial.
RF24 radio(CE_PIN, CSN_PIN);

void readPacket()
{
  static struct TxPayload_s receivedData;

  uint8_t flag = UNSET_EOP;
  while (flag != SET_EOP)
  {
    radio.read(&receivedData, sizeof(receivedData));
    flag = receivedData.eop;

    Serial.println("[DEBUG]\tData received:");

    Serial.println(receivedData.eop);
    Serial.println('\n');
  }
  Serial.println("[DEBUG]\tPacket received");
  Serial.println('\n');
}

// Send ack with command.

void sendAck(uint8_t *data)
{
  radio.writeAckPayload(1, data, sizeof(data));
}


// Called after every loop() cycle if serial available.
// TODO: set macro for pipe address.
void serialEvent()
{
  static uint8_t serialBuf[PAYLOAD_SIZE];
  static uint8_t serialBufIndex;

  for (; Serial.available(); ++serialBufIndex)
  {
    serialBuf[serialBufIndex] = (uint8_t)Serial.read(); // read byte

    if (serialBuf[0] == '0' || serialBufIndex == PAYLOAD_SIZE) // no payload data required
    {
      serialBufIndex = 0;
      Serial.println("[DEBUG] Sending command buffer:");
      for (char *c = serialBuf; c < serialBuf + PAYLOAD_SIZE; c++)
        Serial.print(*c);
      Serial.println();
      sendAck(serialBuf);
      return;
    }
  }
}



void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("[DEBUG]\tNrf24L01 Receiver Starting");
  radio.begin();
  radio.openReadingPipe(PIPE, tx_pipe_addr);
  radio.openWritingPipe(rx_pipe_addr);
  radio.startListening();
  radio.enableAckPayload();
}

void loop()
{
  if ( radio.available() )
  {
    readPacket();
  }
  else
  {
    Serial.println("[DEBUG]\tNot working");
  }
  delay(200);
}
