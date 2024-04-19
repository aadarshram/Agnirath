#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setTxPower(20, 1);
}

void loop()
{
  if (Serial.available() > 0)
  {
    String receivedData = Serial.readStringUntil('\n');

    // Process the received comma-separated string of float values
    float receivedFloats[10]; // Adjust the array size as needed
    int floatCount = 0;

    char *token = strtok(const_cast<char *>(receivedData.c_str()), ",");
    while (token != NULL)
    {
      receivedFloats[floatCount] = atof(token);
      floatCount++;
      token = strtok(NULL, ",");
    }

    // Concatenate the received float values into a single string
    String concatenatedValues = "";
    for (int i = 0; i < floatCount; i++)
    {
      concatenatedValues += String(receivedFloats[i]);
      if (i < floatCount - 1)
      {
        concatenatedValues += ",";
      }
    }

    // Send the concatenated string back to the computer
    // Serial.println(concatenatedValues);
    Serial.print("Sending packet: ");
    Serial.println(concatenatedValues);
    LoRa.beginPacket();
    LoRa.print(concatenatedValues);
    LoRa.endPacket();

    counter++;
  }
}