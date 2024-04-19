#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>

// Create an MCP2515 object
Adafruit_MCP2515 mcp(10);

// Define the pins for digital inputs
const int safestatePin = 2;
const int accessoriesRunPin = 3;
const int onStatePin = 4;
const int runStatePin = 5;
const int brakePin = 6; // Digital input for the brake

// Define the pins for analog inputs
const int throttle1Pin = A0; // First throttle input
const int throttle2Pin = A1; // Second throttle input

// New 32-bit float variables
float motorCurrent = 0.0;
float busCurrent = 0.0;
float motorVelocity = 0.0;
float regenPercentage = 0.0;
float clutch = 0.0;
float cruise = 0.0;
float vehicleVelocity = 0.0;
float rpm = 0.0;

void setup()
{
  // Initialize the MCP2515 module
  if (!mcp.begin(1000000))
  {
    Serial.println("MCP2515 initialization failed.");
    while (1)
      ;
  }
  Serial.begin(9600);

  // Set the digital pins as inputs
  pinMode(safestatePin, INPUT);
  pinMode(accessoriesRunPin, INPUT);
  pinMode(onStatePin, INPUT);
  pinMode(runStatePin, INPUT);
  pinMode(brakePin, INPUT); // Brake as a digital input

  // Set the analog pins as inputs
  pinMode(throttle1Pin, INPUT);
  pinMode(throttle2Pin, INPUT);
}

void sendCanMessage(uint32_t messageID, int data1, int data2, int data3, int data4)
{
  // Create a CAN message
  can_frame frame;
  frame.can_id = messageID;
  frame.can_dlc = 8; // 8 bytes (4 ints)

  // Pack the integers into the data bytes (little-endian)
  frame.data[0] = data1 & 0xFF;
  frame.data[1] = (data1 >> 8) & 0xFF;
  frame.data[2] = data2 & 0xFF;
  frame.data[3] = (data2 >> 8) & 0xFF;
  frame.data[4] = data3 & 0xFF;
  frame.data[5] = (data3 >> 8) & 0xFF;
  frame.data[6] = data4 & 0xFF;
  frame.data[7] = (data4 >> 8) & 0xFF;

  // Send the CAN message
  if (mcp.tryToSend(frame))
  {
    Serial.println("Sent CAN message.");
  }
  else
  {
    Serial.println("Message sending failed.");
  }
}

void sendTwoFloats(uint32_t messageID, float float1, float float2)
{
  // Create a CAN message
  can_frame frame;
  frame.can_id = messageID;
  frame.can_dlc = 8; // 8 bytes (2 floats)

  // Pack the floats into the data bytes (little-endian)
  memcpy(frame.data, &float1, sizeof(float));
  memcpy(frame.data + sizeof(float), &float2, sizeof(float));

  // Send the CAN message
  if (mcp.tryToSend(frame))
  {
    Serial.println("Message sent successfully.");
  }
  else
  {
    Serial.println("Message sending failed.");
  }
}

void parsePacketToFloats(float &value1, float &value2)
{
  int packetSize = mcp.parsePacket();

  if (packetSize == 8)
  {
    uint8_t buffer[8];
    mcp.read(buffer, 8);

    // Interpret the bytes as little-endian 32-bit floats
    value1 = *((float *)&buffer[0]);
    value2 = *((float *)&buffer[4]);
  }
}

void loop()
{
  int safestateValue = digitalRead(safestatePin);
  int accessoriesRunValue = digitalRead(accessoriesRunPin);
  int onStateValue = digitalRead(onStatePin);
  int runStateValue = digitalRead(runStatePin);

  // Read the brake value
  int brakeValue = digitalRead(brakePin);

  // Read the two identical throttle values (0-1023) and convert to percentage
  int throttle1Value = map(analogRead(throttle1Pin), 0, 1023, 0, 1);
  int throttle2Value = map(analogRead(throttle2Pin), 0, 1023, 0, 1);

  // Calculate the average throttle percentage
  float throttlePercentage = (throttle1Value + throttle2Value) / 2;

  int packetSize = mcp.parsePacket();

  if (packetSize)
  {
    if (mcp.packetId()== 0x113){
      cruise = 1;
    }
  }

  parsePacketToFloats(vehicleVelocity, rpm);
  parsePacketToFloats(regenPercentage, clutch);

  if (accessoriesRunValue == HIGH && onStateValue == HIGH && runStateValue == HIGH)
  {
    // All three buttons are pressed
    if (cruise == 1){
      motorVelocity = rpm;
      motorCurrent = 1;
      mcp.beginPacket(0x131);
      mcp.write(1);
      mcp.endPacket();
    }

    if (brakeValue == HIGH or regenPercentage > 0.05)
    {
      // Brake is pressed, override throttle input
      cruise = 0;
      mcp.beginPacket(0x132);
      mcp.write(1);
      mcp.endPacket();
      int throttlePercentage = 0;
      if (regenPercentage > 0.05){
        motorCurrent = regenPercentage;
        motorVelocity = 0;
      }
    }
    else
    {
      // Brake is not pressed, set motorCurrent to throttlePercentage
      cruise = 0;
      mcp.beginPacket(0x132);
      mcp.write(1);
      mcp.endPacket();
      motorCurrent = static_cast<float>(throttlePercentage);

      if (clutch == 2)
      {
        // Set motor velocity to 10000 when clutch is 2
        motorVelocity = 10000.0;
      }
      else if (clutch == 3)
      {
        // Set motor velocity to 20000 when clutch is 3
        motorVelocity = 20000.0;
      }
      else if (clutch == 0)
      {
        // Set motor velocity to -10000 when clutch is 0
        motorVelocity = -10000.0;
      }
      else if (clutch == 1)
      {
        // Set motor velocity to 0 when clutch is 1
        motorVelocity = 0.0;
        motorCurrent = 0.0;
      }
    }

    // Send 0x501 message with motor velocity and motor current
    sendTwoFloats(0x501, motorVelocity, motorCurrent);

    // Send 0x502 message with 0 and busCurrent as the two floats
    sendTwoFloats(0x502, 0.0, busCurrent);

    // Send 0x505 message with integers as specified
    sendCanMessage(0x505, 0x0030, 0x0030, 0x0030, 0x0030);
  }
  else if (accessoriesRunValue == HIGH && onStateValue == HIGH)
  {
    // Accessories Run and On buttons are pressed
    sendCanMessage(0x505, 0x0070, 0x0070, 0x0070, 0x0070);
  }
  else if (accessoriesRunValue == HIGH)
  {
    // Accessories Run button is pressed
    sendCanMessage(0x505, 0x0010, 0x0010, 0x0010, 0x0010);
  }
  else if (safestateValue == HIGH)
  {
    // Safestate button is pressed
    sendCanMessage(0x505, 0x0000, 0x0000, 0x0000, 0x0000); // Send all integers as 0x0000
  }

  // Add appropriate delays to control the loop rate
  delay(100); // Adjust as needed
}
