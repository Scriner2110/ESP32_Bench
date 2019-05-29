#include <Arduino.h>
#include <Adafruit_ADS1015.h>



Adafruit_ADS1115 ads;

int LED_A = 17; // Def Pin LED_A
int LED_B = 18; // Def Pin LED_B
int PORT_PWM = 19; // Def Pin LED_B

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

  for(int count = 0 ; count < 19 ; count++)
  {
    Fib3 = Fib1 + Fib2;
    Fib1 = Fib2;
    Fib2 = Fib3;
  }
  FibEnd = Fib3;

  return FibEnd;
}

////////////////////////////////////////////////////////////////////////////////

int ADC (void)
{
  int16_t Val = analogRead(4);

  return Val;
}

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
  analogSetWidth(12);                   // Sets the sample bits and read resolution
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8.
  analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(255);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_0db);       // Sets the input attenuation for ALL ADC inputs

   adcAttachPin(4);

}

//----------------------------------------------------------------------------//

void loop()
{
  //Port_A();
  //Port_B();
  //I2C();
  //PWM();

  Serial.print(Fibonacci());
  Serial.print(" ; ");
  Serial.println(ADC());

  //Serial.print();
  //Serial.print();
  //Serial.print();

  delay(1000);
}
