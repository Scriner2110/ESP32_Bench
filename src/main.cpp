#include <Arduino.h>
#include <Adafruit_ADS1015.h>



Adafruit_ADS1115 ads;

#define LED_A      17
#define LED_B      18
#define PORT_PWM   19
#define PORT_DAC   25

///// DEF PWM /////
int Freq = 10000; //Def PWM parameter
int LedChannel = 0;
int Resolution = 8;

////////////////////////////////////////////////////////////////////////////////

void I2C ()
{
  int16_t Results;
  float multiplier = 0.0625F; // 2V Range

  Results = ads.readADC_Differential_0_1();
  Serial1.print("Differential: ");Serial1.print("("); Serial1.print(Results * multiplier); Serial1.print("mV)");Serial1.print(";");
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

int Fibonacci (void)
{
  uint32_t Fib1 = 0, Fib2 = 1, Fib3;
  uint32_t FibEnd;

  for(int count = 0 ; count < 100 ; count++)
  {
    Fib3 = Fib1 + Fib2;
    Fib1 = Fib2;
    Fib2 = Fib3;
  }
  FibEnd = Fib3;

  Serial.print(FibEnd);
}

////////////////////////////////////////////////////////////////////////////////

int ADC ()
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
  ads.setGain(GAIN_TWO);
  ads.begin();

  Serial.begin(115200);

  ledcSetup(LedChannel, Freq, Resolution);
  ledcAttachPin(PORT_PWM, LedChannel);

  //ADC Config
  analogReadResolution(12);             // Sets the sample bits and read resolution
  //analogSetWidth(12);                   // Sets the sample bits and read resolution
  //analogSetCycles(8);                   // Set number of cycles per sample, default is 8.
  //analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  //analogSetClockDiv(1);               // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs
  analogSetPinAttenuation(4, ADC_11db);
  adcAttachPin(4);

}

//----------------------------------------------------------------------------//

void loop()
{
  Port_A();   //// First Stage
  Port_B();   //// Seconde Stage

  //I2C();
  PWM();
  Fibonacci();
  Serial.print(" ; ");
  ADC();

 delay(500);

}
