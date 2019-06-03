#include <Arduino.h>
#include <Adafruit_ADS1015.h>
#include <SPI.h>
#include "Adafruit_FRAM_SPI.h"
#include <Wire.h>
#include "Adafruit_MCP9808.h"



Adafruit_ADS1115 ads;

#define LED_A      17
#define LED_B      18
#define PORT_PWM   19
#define PORT_DAC   25


int Freq = 10000; //Def PWM parameter
int LedChannel = 0;
int Resolution = 8;

uint8_t FRAM_CS = 33;

//Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);  // use hardware SPI

uint8_t FRAM_SCK= 14;
uint8_t FRAM_MISO = 12;
uint8_t FRAM_MOSI = 13;

//Or use software SPI, any pins!
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

////////////////////////////////////////////////////////////////////////////////

void I2C ()
{
  tempsensor.wake();
  float c = tempsensor.readTempC();
  Serial.print(c, 4); Serial.print("*C"); 
}

////////////////////////////////////////////////////////////////////////////////

void Spi ()
{
  fram.writeEnable(true);
  fram.write8(0x01, 0x50);
  fram.writeEnable(false);

  int value = fram.read8(0x01);
  Serial.print(value);
}

////////////////////////////////////////////////////////////////////////////////

void Port_A ()
{
  digitalWrite(LED_A, HIGH);
  delay(500);
  digitalWrite(LED_A, LOW);
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////

void Port_B ()
{
  digitalWrite(LED_B, HIGH);
  delay(500);
  digitalWrite(LED_B, LOW);
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////

void PWM ()
{
  int DutyCycle = 128;
  ledcWrite(LedChannel, DutyCycle);
}

////////////////////////////////////////////////////////////////////////////////

void Fibonacci (void)
{
  uint32_t Fib1 = 0, Fib2 = 1, Fib3;
  uint32_t FibEnd;

  for(int count = 0 ; count < 50 ; count++)
  {
    Fib3 = Fib1 + Fib2;
    Fib1 = Fib2;
    Fib2 = Fib3;
  }
  FibEnd = Fib3;

  Serial.print(FibEnd);
}

////////////////////////////////////////////////////////////////////////////////

void ADC ()
{
  int Read = analogRead(4);
  float Volt = Read * (3.6 / 4096.0);

  Serial.println(Volt);
}

////////////////////////////////////////////////////////////////////////////////

void DAC ()
{
  dacWrite(PORT_DAC, 255);
  delay(200);
  dacWrite(PORT_DAC, 128);
  delay(200);
}

////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//

void setup()
{
  Serial.begin(115200);

  tempsensor.setResolution(1);
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms

  // SPI FRAM
  fram.begin();

  ledcSetup(LedChannel, Freq, Resolution);
  ledcAttachPin(PORT_PWM, LedChannel);

  // ADC Config
  analogReadResolution(12);             // Sets the sample bits and read resolution
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs
  analogSetPinAttenuation(4, ADC_11db);
  adcAttachPin(4);

}

//----------------------------------------------------------------------------//

void loop()
{
  Port_A();   //// First Stage
  Port_B();   //// Seconde Stage


  PWM();

  Spi();
  Serial.print(" ; ");
  Fibonacci();
  Serial.print(" ; ");
  ADC();


  delay(500);

}
