#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_MCP2515.h>

#define I2C_COMMUNICATION // I2C communication. Comment out this line of code if you want to use SPI communication.

#ifdef I2C_COMMUNICATION
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
#else
uint8_t csPin = D3;
DFRobot_ENS160_SPI ENS160(&SPI, csPin);
#endif

#define SENSOR_ADDRESS 0x0002 // Replace with the actual I2C address of your sensor

// Register addresses for the sensor
#define REG_TEMP 0x000A
#define REG_HUMIDITY 0x000B
#define REG_ATMOSPHERE_PRESSURE 0x000C
#define REG_ULTRAVIOLET_INTENSITY 0x0008
#define REG_LUMINOUS_INTENSITY 0x0009

#define HPA 0x01 // Define HPA as 0x01

void readRegister(uint16_t reg, uint8_t *data, uint8_t length);
float readTemperature();
float readHumidity();
uint16_t readPressure(uint8_t units);
float readUVIntensity();
float readLuminousIntensity();

#define CAN_BAUDRATE (1000000)
Adafruit_MCP2515 mcp(10); 

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  // Init the sensor
  while (NO_ERR != ENS160.begin())
  {
    Serial.println("Air Quality Sensor Fail");
    delay(3000);
  }
  if (!mcp.begin(CAN_BAUDRATE))
  {
    Serial.println("Error initializing MCP2515.");
    while (1)
      delay(10);
  }
  Serial.println("Air Quality Ready");
  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);
  delay(500); // Wait for the sensor to initialize
}

void loop()
{
  float temperatureCelsius = readTemperature();
  float humidity = readHumidity();
  uint16_t pressureHPa = readPressure(HPA); // Hectopascal
  float uvIntensity = readUVIntensity();
  float luminousIntensity = readLuminousIntensity();

  Serial.print("Temperature (°C): ");
  Serial.println(temperatureCelsius);

  Serial.print("Humidity (%): ");
  Serial.println(humidity);

  Serial.print("Pressure (hPa): ");
  Serial.println(pressureHPa);

  Serial.print("UV Intensity: ");
  Serial.println(uvIntensity);

  Serial.print("Luminous Intensity: ");
  Serial.println(luminousIntensity);

  uint8_t AQI = ENS160.getAQI();
  Serial.print("Air Quality: ");
  Serial.println(AQI);

  uint16_t TVOC = ENS160.getTVOC();
  Serial.print("TVOC: ");
  Serial.print(TVOC);
  Serial.println(" ppb");

  uint16_t ECO2 = ENS160.getECO2();
  Serial.print("CO2: ");
  Serial.print(ECO2);
  Serial.println(" ppm");

  uint16_t O2_Value = analogRead(A0);

  Serial.print("02 Level: ");
  Serial.println(O2_Value);

  delay(1000); // Delay for one second before taking the next reading
}

float readTemperature()
{
  uint8_t buffer[2];
  readRegister(REG_TEMP, buffer, 2);
  uint16_t rawData = (buffer[0] << 8) | buffer[1];
  float temperature = (-45) + ((rawData * 175.00) / 1024.00 / 64.00);
  return temperature;
}

float readHumidity()
{
  uint8_t buffer[2];
  readRegister(REG_HUMIDITY, buffer, 2);
  uint16_t rawData = (buffer[0] << 8) | buffer[1];
  float humidity = (float)rawData * 100 / 65536;
  return humidity;
}

uint16_t readPressure(uint8_t units)
{
  uint8_t buffer[2];
  readRegister(REG_ATMOSPHERE_PRESSURE, buffer, 2);
  uint16_t pressure = (buffer[0] << 8) | buffer[1];
  if (units == HPA)
  {
    pressure /= 10; // Convert to hPa (hectopascal)
  }
  return pressure;
}

float readUVIntensity()
{
  uint8_t buffer[2];
  readRegister(REG_ULTRAVIOLET_INTENSITY, buffer, 2);
  uint16_t uvLevel = (buffer[0] << 8) | buffer[1];
  float ultraviolet;
  // You can add your UV intensity conversion logic here based on the sensor's version.
  // For simplicity, this code assumes a linear conversion.
  ultraviolet = (float)uvLevel / 1800.0;
  return ultraviolet;
}

float readLuminousIntensity()
{
  uint8_t buffer[2];
  readRegister(REG_LUMINOUS_INTENSITY, buffer, 2);
  uint16_t rawData = (buffer[0] << 8) | buffer[1];
  float luminous = rawData;
  luminous = luminous * (1.0023f + luminous * (8.1488e-5f + luminous * (-9.3924e-9f + luminous * 6.0135e-13f)));
  return luminous;
}

void readRegister(uint16_t reg, uint8_t *data, uint8_t length)
{
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(reg >> 8);   // MSB
  Wire.write(reg & 0xFF); // LSB
  Wire.endTransmission();

  Wire.requestFrom(SENSOR_ADDRESS, length);
  for (int i = 0; i < length; i++)
  {
    data[i] = Wire.read();
  }
}