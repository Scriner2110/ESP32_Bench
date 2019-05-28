#include <Arduino.h>
#include <Adafruit_ADS1015.h>



Adafruit_ADS1115 ads;

int LED_A = 17; // Def Pin LED_A
int LED_B = 18; // Def Pin LED_B

////////////////////////////////////////////////////////////////////////////////

void I2C ()
{
  int16_t Results;
  float multiplier = 0.0625F; // 2V Range

  Results = ads.readADC_Differential_0_1();
  Serial1.print("Differential: ");Serial1.print("("); Serial1.print(Results * multiplier); Serial1.println("mV)");Serial1.println(";");
  delay(500);
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

////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------------//

void setup()
{
  ads.setGain(GAIN_TWO);
  ads.begin();

  Serial1.begin(115200);
}

//----------------------------------------------------------------------------//

void loop()
{
  Port_A();
  Port_B();
  I2C();
}
