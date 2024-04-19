// Include CAN library
#include <Adafruit_MCP2515.h>

// CAN Module Pins

#define CS_PIN 10
#define INT_PIN 2

// DRL Pins
#define HORN_PIN 2
#define CSL_PIN 3

// Indicator Pins
#define RIGHT_IND_PIN 4
#define LEFT_IND_PIN 5

// Set CAN bus baud rate
#define CAN_BAUDRATE (250000)

// Instantiate CAN object
Adafruit_MCP2515 mcp(CS_PIN);
long prev_time;

void blink(int pin, int interval, uint32_t id);

void setup()
{
  // put your setup code here, to run once:
  pinMode(CSL_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
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

  // if (mcp.packetExtended()) {
  //   Serial.print("extended ");
  // }

  // if (mcp.packetRtr()) {
  //   // Remote transmission request, packet contains no data
  //   Serial.print("RTR ");
  // }

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

      // 0x320 Turns on Right indicator
      if (mcp.packetId() == 320)
      {
        blink(RIGHT_IND_PIN, 500, mcp.packetId());
      }

      if (mcp.packetId() == 321)
      {
        blink(LEFT_IND_PIN, 500, mcp.packetId());
      }

      // Braking

      if (mcp.packetId() == 322)
      {
        blink(CSL_PIN, 200, mcp.packetId());
      }

      // Hazard
      if (mcp.packetId() == 323)
      {
        blink(CSL_PIN, 100, mcp.packetId());
        blink(RIGHT_IND_PIN, 100, mcp.packetId());
        blink(LEFT_IND_PIN, 100, mcp.packetId());
        blink(HORN_PIN, 100, mcp.packetId());
      }

      // REVERSE
      if (mcp.packetId() == 324)
      {
        blink(CSL_PIN, 200, mcp.packetId());
      }

      // HORN
      if (mcp.packetId() == 327)
      {
        while (mcp.packetId() == 327)
        {
          digitalWrite(HORN_PIN, HIGH);
        }
        digitalWrite(HORN_PIN, LOW);
      }

      // Sequence-1
      if (mcp.packetId() == 324)
      {
        blink(CSL_PIN, 300, mcp.packetId());
        blink(RIGHT_IND_PIN, 300, mcp.packetId());
        blink(LEFT_IND_PIN, 300, mcp.packetId());
      }

      Serial.println();
    }

    Serial.println();
  }
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