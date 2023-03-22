#include <Wire.h>
#include <SoftwareSerial.h>
 
//These values are the readings of the whitepoint of the led. Take the back of vinyl sticker and put it againts face of sensor
#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f
 
//Calculate the balancing factors
#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN)
#define BAL_BLUE (LED_GREEN/LED_BLUE)
 
#define I2C_ADDR 0x52
 
uint8_t readBuff[9];
uint16_t ir=0;
uint16_t red=0;
uint16_t green=0;
uint16_t blue=0;
 
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;

SoftwareSerial sendPin(16, 15);
 
void setup() {
  Wire.begin();
  Serial.begin(115200);
  i2cWrite(0x00,0b0110);  //enable light sensor and activate rgb mode
  i2cWrite(0x04,0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms
  sendPin.begin(111520);
}
 
void loop() {
  i2cRead(0x0A,readBuff,12);
 
  ir=(readBuff[1]<<8)|readBuff[0];
  green=(readBuff[4]<<8)|readBuff[3];
  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
 
  red*=BAL_RED;
  green*=BAL_GREEN;
  blue*=BAL_BLUE;
 
 
  //Normalize the readings to brightest channel then apply log scale to better discern the colors.
  float maxV=max(blue,max(red,green));
  red=255*pow(red/maxV,5);
  green=255*pow(green/maxV,5);
  blue=255*pow(blue/maxV,5);
   
  Serial.print("R:");
  Serial.print(red);
  Serial.print(" G:");
  Serial.print(green);
  Serial.print(" B:");
  Serial.println(blue);  
  
  sendPin.print("R:");
  sendPin.print(red);
  sendPin.print(" G:");
  sendPin.print(green);
  sendPin.print(" B:");
  sendPin.println(blue);
  
  delay(25);
}
 
 
void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}
void i2cRead(uint8_t reg,uint8_t *val,uint16_t len){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, len);
    for(uint8_t i=0;i<len;i++){
      val[i]=Wire.read();
    }
}