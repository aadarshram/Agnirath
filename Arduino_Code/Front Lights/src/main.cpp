#include <Arduino.h>
// Include CAN library
#include <Adafruit_MCP2515.h>
#include <Wire.h>

// CAN Module Pins

#define CS_PIN 10
#define INT_PIN 2

void onReceive(int packetSize);

// DRL Pins
#define DRL1_PIN 3
#define DRL2_PIN 4

// Indicator Pins
#define RIGHT_IND_PIN 5
#define LEFT_IND_PIN 6

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

    // Instantiate CAN object
    Adafruit_MCP2515 mcp(CS_PIN);
long prev_time;

void blink(int pin, int interval, uint32_t id);

void setup()
{
  // put your setup code here, to run once:
  pinMode(DRL1_PIN, OUTPUT);
  pinMode(DRL2_PIN, OUTPUT);
  pinMode(RIGHT_IND_PIN, OUTPUT);
  pinMode(LEFT_IND_PIN, OUTPUT);

  Serial.println("MCP2515 Receiver Callback test!");

  if (!mcp.begin(CAN_BAUDRATE))
  {
    Serial.println("Error initializing MCP2515.");
    while (1)
      delay(10);
  }
  Serial.println("MCP2515 chip found");

  // register the receive callback
  mcp.onReceive(INT_PIN, onReceive);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

void onReceive(int packetSize)
{
  // received a packet
  Serial.print("Received ");

  Serial.print("packet with id 0x");
  Serial.print(mcp.packetId(), HEX);

  if (mcp.packetRtr())
  {
    Serial.print(" and requested length ");
    Serial.println(mcp.packetDlc());
  }
  else
  {
    Serial.print(" and length ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets
    while (mcp.available())
    {

      // Always turn DRL lamps on

      digitalWrite(DRL1_PIN, HIGH);
      digitalWrite(DRL2_PIN, HIGH);

      // 0x320 Turns on Right indicator
      // Hazard
      if (mcp.packetId() == 323)
      {
        blink(RIGHT_IND_PIN, 100, mcp.packetId());
        blink(LEFT_IND_PIN, 100, mcp.packetId());
      }

      else if (mcp.packetId() == 321)
      {
        blink(RIGHT_IND_PIN, 500, mcp.packetId());
      }

      else if (mcp.packetId() == 320)
      {
        blink(LEFT_IND_PIN, 500, mcp.packetId());
      }
      // Sequence-1
      else if (mcp.packetId() == 324)
      {
        blink(DRL1_PIN, 300, mcp.packetId());
        blink(DRL2_PIN, 300, mcp.packetId());
        blink(RIGHT_IND_PIN, 300, mcp.packetId());
        blink(LEFT_IND_PIN, 300, mcp.packetId());
      }
    }

    Serial.println();
  }

  Serial.println();
}

void blink(int pin, int interval, uint32_t id)
{
  digitalWrite(pin, HIGH);

  while (mcp.packetId() == id)
  {
    prev_time = millis();
    if ((millis() - prev_time) >= interval)
    {
      if (digitalRead(pin == HIGH))
      {
        digitalWrite(pin, LOW);
      }
      else if (digitalRead(pin == LOW))
      {
        digitalWrite(pin, HIGH);
      }
    }
  }
}