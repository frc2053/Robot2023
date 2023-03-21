#include <Wire.h>

enum class GainFactor
{
    k1x = 0,
    k3x = 1,
    k6x = 2,
    k9x = 3,
    k18x = 4
};

enum class LEDPulseFrequency
{
    k60kHz = 0x18,
    k70kHz = 0x40,
    k80kHz = 0x28,
    k90kHz = 0x30,
    k100kHz = 0x38,
};

enum class LEDCurrent
{
    kPulse2mA = 0,
    kPulse5mA = 1,
    kPulse10mA = 2,
    kPulse25mA = 3,
    kPulse50mA = 4,
    kPulse75mA = 5,
    kPulse100mA = 6,
    kPulse125mA = 7,
};

enum class ProximityResolution
{
    k8bit = 0x00,
    k9bit = 0x08,
    k10bit = 0x10,
    k11bit = 0x18,
};

enum class ProximityMeasurementRate
{
    k6ms = 1,
    k12ms = 2,
    k25ms = 3,
    k50ms = 4,
    k100ms = 5,
    k200ms = 6,
    k400ms = 7,
};

enum class ColorResolution
{
    k20bit = 0x00,
    k19bit = 0x10,
    k18bit = 0x20,
    k17bit = 0x30,
    k16bit = 0x40,
    k13bit = 0x50,
};

enum class ColorMeasurementRate
{
    k25ms = 0,
    k50ms = 1,
    k100ms = 2,
    k200ms = 3,
    k500ms = 4,
    k1000ms = 5,
    k2000ms = 7,
};

enum class Register
{
    kMainCtrl = 0x00,
    kProximitySensorLED = 0x01,
    kProximitySensorPulses = 0x02,
    kProximitySensorRate = 0x03,
    kLightSensorMeasurementRate = 0x04,
    kLightSensorGain = 0x05,
    kPartID = 0x06,
    kMainStatus = 0x07,
    kProximityData = 0x08,
    kDataInfrared = 0x0A,
    kDataGreen = 0x0D,
    kDataBlue = 0x10,
    kDataRed = 0x13
};

enum class MainCtrlFields
{
    kProximitySensorEnable = 0x01,
    kLightSensorEnable = 0x02,
    kRGBMode = 0x04
};

static constexpr int kAddress = 0x52;
static constexpr int kExpectedPartID = 0xC2;

bool read_i2c_data(Register reg, uint8_t *buffer, uint8_t readLen)
{
    buffer[0] = static_cast<uint8_t>(reg);
    Wire.beginTransmission(kAddress);
    int result = Wire.write(buffer, 1);
    if (result != 1)
    {
        return false;
    }
    result = Wire.requestFrom(kAddress, readLen, true);
    for(int i = 0; i < result; i++) {
      buffer[i] = Wire.read();
    }
    return result == readLen;
}

void init_device(uint8_t *i2cBuffer)
{
    Wire.beginTransmission(kAddress);
    i2cBuffer[0] = static_cast<uint8_t>(Register::kMainCtrl);
    i2cBuffer[1] = static_cast<uint8_t>(MainCtrlFields::kRGBMode) |
                   static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) |
                   static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable);
    Wire.write(i2cBuffer, 2);
    Wire.endTransmission();

    Wire.beginTransmission(kAddress);
    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorRate);
    i2cBuffer[1] = static_cast<uint8_t>(ProximityResolution::k11bit) | static_cast<uint8_t>(ProximityMeasurementRate::k100ms);
    Wire.write(i2cBuffer, 2);
    Wire.endTransmission();

    Wire.beginTransmission(kAddress);
    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorPulses);
    i2cBuffer[1] = 32;
    Wire.write(i2cBuffer, 2);
    Wire.endTransmission();
}


char outputBuffer[512];
unsigned int values[10];
uint8_t i2cBuffer[20];
bool currentValid0 = false;
bool currentValid1 = false;
bool reset;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  init_device(i2cBuffer);
}


void loop() {
  currentValid0 = read_i2c_data(Register::kMainStatus, i2cBuffer, 15);
  if (currentValid0)
  {
      if ((i2cBuffer[0] & 0x20) != 0)
      {
          init_device(i2cBuffer);
      }
      else
      {
          values[4] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
          values[3] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
          values[2] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
          values[1] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
          values[0] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
      }
  }

  currentValid1 = read_i2c_data(Register::kMainStatus, i2cBuffer, 15);
  if (currentValid1)
  {
      if ((i2cBuffer[0] & 0x20) != 0)
      {
          init_device(i2cBuffer);
      }
      else
      {
          values[9] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
          values[8] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
          values[7] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
          values[6] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
          values[5] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
      }
  }

  snprintf(outputBuffer, sizeof(outputBuffer), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
            currentValid0, currentValid1, values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9]);

  Serial.print(outputBuffer);

  delay(100);
}
