#include <Arduino.h>
#include <MCP3208.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

// put function declarations here:
void onReceive(int);
void Send_CAN_Data();
void discharge_sequence();
void Contactor_Status();

float read_voltage();
float read_current();

const float Vmult = 68.1106; // Voltage Multiplier determined by 5*(R1+R2)/R2
const float Imult = 0.87825; // Current Multiplier determined by 5*/(Rshunt*Gain)digitalwrite
int count = 0, track = 0;
int precharge_relay_pin = A0;
int B_nve_pin = A2;
int B_pve_pin = A3;
int discharge_relay_pin = A1;
// Contactor AUX pins
int aux_pin1 = 7;
int aux_pin2 = 8;
// int aux_pin3 = A1;
// int aux_pin4 = A1;

float threshhold = 144.0;

int dig_code;
bool charge;
// float volts, amps;

#define CAN_BAUDRATE (1000000)

// initialises can module
Adafruit_MCP2515 mcp(10);
// initialises adc
MCP3208 adc;

void precharge_sequence();

    void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Serial.println("MCP3208 simple test.");

  // Hardware SPI (specify CS, use any available digital pin)
  adc.begin(9);

  if (!mcp.begin(CAN_BAUDRATE))
  {
    Serial.println("Error initializing MCP2515.");
    while (1)
      delay(10);
  }

  // GPIO pins to control the Gate of the MOSFET driving the relays and contactors (Only 2 pins have been defined for testing purpose, define more pins as necessary).
  pinMode(precharge_relay_pin, OUTPUT);
  pinMode(B_nve_pin, OUTPUT);
  pinMode(B_pve_pin, OUTPUT);
  pinMode(discharge_relay_pin, OUTPUT);

  mcp.onReceive(2, onReceive);

  // Discharge is a usually closed relay, now open the relay before starting precharge sequence
}

float read_voltage()
{
  float volts;
  dig_code = adc.readADC(2); // 2 is the channel number i.e. pin 3 of MCP3208
  volts = Vmult * dig_code / 4096;
  Serial.print(volts,4);
  Serial.print("V  ");
  return volts;
}

float read_current()
{
  float amps;
  dig_code = adc.readADC(1);
  amps = Imult * dig_code / 4096;
  Serial.print(amps,4);
     Serial.println("A  ");
  return amps;
}

void loop()
{
  Send_CAN_Data();
  delay(500);
  Contactor_Status();
  delay(500);
}

void onReceive(int packet_size)
{
  float v1, v2;
  int precharge_addr = 259;
  int discharge_addr = 260;

  // Receiver code
  Serial.print("Received ");

  if (mcp.packetExtended())
  {
    Serial.print("extended ");
  }

  if (mcp.packetRtr())
  {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

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
    // Serial.println(packetSize);

    // only print packet data for non-RTR packets
    while (mcp.available())
    {
      Serial.print(mcp.read(), HEX);
    }
    Serial.println();
  }

  Serial.println();
  if (true)
    // if ((int)mcp.packetId() == precharge_addr)
    // {
    //   charge = true;
    //   digitalWrite(discharge_relay_pin, LOW);
    //   digitalWrite(B_nve_pin, HIGH);
    //   digitalWrite(precharge_relay_pin, HIGH);
    // }
    precharge_sequence();

    // while (charge == true)
    // {

    //   v1 = read_voltage();
    //   Serial.print(v1, 6);
    //   Serial.println("V");
    //   delay(1);
    //   Serial.print(v1, 6);
    //   Serial.println("V");
    //   v2 = read_voltage();

    //   // Checking if charge is building up across the capacitor
    //   if (v2 > v1)
    //   {
    //     delay(1);
    //     // continue;
    //   }
    //   else // if voltage reading has not increased
    //   {
    //     // delayMicroseconds(500);
    //     v2 = read_voltage(); // sample reading 0.5 ms later

    //     if ((v2 > v1) == false)
    //     {
    //       // delayMicroseconds(250); // sample reading 0.25 ms later
    //       v2 = read_voltage();    // double checking

    //       if ((v2 > v1) == false)
    //       {
    //         Serial.print("Charging Fault. Open Precharge"); // still not increased call fault
    //         charge = false;
    //         discharge_sequence();
    //       }
    //     }
    //   }

    //   if (read_voltage() > threshhold)
    //   {
    //     digitalWrite(precharge_relay_pin, LOW);
    //     digitalWrite(B_pve_pin, HIGH);
    //     charge = true;
    //   }
    // }

    // // Discharge Sequence
    // if (mcp.packetId() == discharge_addr)
    // {
    //   discharge_sequence();
    // }
}

void Send_CAN_Data()
{
  typedef union _data
  {
    float f;
    uint8_t s[4];
  } myData;

  myData volt_data;
  volt_data.f = read_voltage();
  myData amps_data;
  amps_data.f = read_current();

  mcp.beginPacket(0x101);
  mcp.write(volt_data.s[7]);
  mcp.write(volt_data.s[6]);
  mcp.write(volt_data.s[5]);
  mcp.write(volt_data.s[4]);

  mcp.write(amps_data.s[3]);
  mcp.write(amps_data.s[2]);
  mcp.write(amps_data.s[1]);
  mcp.write(amps_data.s[0]);

  mcp.endPacket();
}

void Contactor_Status()
{

  bool con1_stat = digitalRead(aux_pin1);
  bool con2_stat = digitalRead(aux_pin2);
  // bool con3_stat = digitalRead(aux_pin3);
  // bool con4_stat = digitalRead(aux_pin4);

  mcp.beginPacket(0x102);
  mcp.write(con1_stat);
  mcp.write(con2_stat);
  // mcp.write(con3_stat);
  // mcp.write(con4_stat);
  mcp.endPacket();
}

void discharge_sequence()
{
  digitalWrite(B_pve_pin, LOW);
  digitalWrite(B_nve_pin, LOW);
  digitalWrite(precharge_relay_pin, LOW);
  delay(1000);
  digitalWrite(discharge_relay_pin, HIGH);
}

void precharge_sequence()
{
  digitalWrite(precharge_relay_pin, HIGH);
  digitalWrite(B_pve_pin, HIGH);
  digitalWrite(precharge_relay_pin, HIGH);
  digitalWrite(B_nve_pin, HIGH);
  
  delay(5000);
  digitalWrite(precharge_relay_pin, LOW);
}
