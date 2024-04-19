#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>

const int csPin = 10;   // LoRa radio chip select
const int resetPin = 9; // LoRa radio reset
const int irqPin = 2;   // change for your board; must be a hardware interrupt pin
const int buzzerPin = 2;
const int safeStatePin = 2;
const int fanPin = 2;

String outgoing; // outgoing message
String concatenatedValues = "";

byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xBB; // address of this device
byte destination = 0xBB;  // destination to send to
long lastSendTime = 0;    // last send time
int interval = 2000;      // interval between sends

int txPower = 20;        // Transmit power in dBm
int spreadingFactor = 7; // Spreading factor (SF7)
int codingRate = 5;      // Coding rate (4/5)

void sendMessage(String outgoing);
void onReceive(int packetSize);

void setup()
{
  Serial.begin(9600); // initialize serial
  while (!Serial)
    ;

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin

  if (!LoRa.begin(433E6))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ; // if failed, do nothing
  }

  LoRa.setTxPower(txPower, 1);
  LoRa.setSpreadingFactor(spreadingFactor);
  LoRa.setCodingRate4(codingRate);
  Serial.println("LoRa init succeeded.");

  pinMode(buzzerPin, OUTPUT);
  pinMode(safeStatePin, OUTPUT);
  pinMode(fanPin, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0){
    String receivedData = Serial.readStringUntil('\n');
    // Process the received comma-separated string of float values
    float receivedFloats[300]; // Adjust the array size as needed
    int floatCount = 0;

    char *token = strtok(const_cast<char *>(receivedData.c_str()), ",");
    while (token != NULL)
    {
      receivedFloats[floatCount] = atof(token);
      floatCount++;
      token = strtok(NULL, ",");
    }
    
    String concatenatedValues = "";
    for (int i = 0; i < floatCount; i++)
    {
      concatenatedValues += String(receivedFloats[i]);
      if (i < floatCount - 1)
      {
        concatenatedValues += ",";
      }
    }
    if (millis() - lastSendTime > interval)
    {
      String message = concatenatedValues; // send a message
      sendMessage(message);
      Serial.println("Sending " + message);
      lastSendTime = millis(); // timestamp the message
      interval = 1000;         // 2-3 seconds
    }
    // parse for a packet, and call onReceive with the result:
    onReceive(LoRa.parsePacket());
    if (receivedFloats[266] == 1){
      digitalWrite(safeStatePin, HIGH);
    }else{
      digitalWrite(safeStatePin, LOW);
    }
    
  }
  digitalWrite(fanPin, HIGH);
}

void sendMessage(String outgoing)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(destination);       // add destination address
  LoRa.write(localAddress);      // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != destination)
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}