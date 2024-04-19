#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MCP2515.h>

#define CS_PIN_CAN 5
#define CAN_BAUDRATE (1000000)

// IDs of temp messages

uint16_t temp_id[] = {0x301, 0x302, 0x303, 0x303, 0x304, 0x305, 0x306, 0x307, 0x308, 0x309, 0x3A0, 0x3B0, 0x3C0};

Adafruit_MCP2515 mcp(CS_PIN_CAN);

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS A0

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

int deviceCount = 0;
float tempC;

void floatToBytes(float value, byte *byteArray)
{
  memcpy(byteArray, &value, sizeof(float));
}

void setup(void)
{
  sensors.begin(); // Start up the library
  Serial.begin(9600);

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");

  if (!mcp.begin(CAN_BAUDRATE))
  {
    Serial.println("Error initializing MCP2515.");
    while (1)
      delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop(void)
{
  // byte array to store float
  byte byteArray[sizeof(float)];
  // To store reading from the sensor
  float tempC;

  // Display temperature from each sensor
  for (int i = 0; i < deviceCount; i++)
  {
    tempC = sensors.getTempCByIndex(i);

    floatToBytes(tempC, byteArray);

    Serial.print("Float in Bytes: ");
    Serial.print(tempC);
    Serial.print(" ADDRESS ");
    Serial.print(temp_id[i], HEX);
    Serial.print(" ");
    mcp.beginPacket(temp_id[i]);
    for (int j = 0; j < sizeof(float); j++)
    {
      mcp.write(byteArray[j]);
      Serial.println(byteArray[j]);
    }
    mcp.endPacket();
    Serial.println(" END OF PACKET");

    delay(1000);
    ;
  }

  Serial.println("");
  delay(1000);
}