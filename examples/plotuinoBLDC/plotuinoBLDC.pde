#include <WProgram.h>
#include "../../plotuino.h"

#define VOLTAGE_SENSE_PIN A0
#define CURRENT_SENSE_PIN A1
#define FORCE_SENSE_PIN A2

/** This function measures the voltage of the power source with the ADC. The
 * power source is connected through a voltage divider consisting of a 10Kohm
 * and 4.7Kohm resistor (2% tolerance). This produces a dividing factor:
 * 
 * V_out/V_in = R1 / (R1 + R2) = 4.7 / (4.7 + 10) = 0.3197
 * 
 * The Atmega328p has a 10-bit ADC and the analog reference voltage is 5.0V. Therefore,
 * there are 5.0/1024 V/step = 0.004883 V/step. The volts (V) in this expression is the 
 * same as V_out. Therefore,
 * 
 * 0.004883 V_out/step x 1/0.3197 V_in/V_out = 0.01527 V_in/step
 * 
 * This gives a max reading of 1023*0.01527 = 15.62 V_in.
 * 
 * A low pass filter is used to smooth out the analog readings. It has a time constant
 * of 0.1 s and is implemented with a pretty standard discrete formula:
 * 
 * x(k+1) = x(k)*(1-alpha) + u(k+1)*alpha
 * 
 * where x is the output and u is the measured input. Alpha is calculated as
 * 
 * alpha = t/(t+T)
 * 
 * where T is the desired time constant (Tau) and t is period length of the 
 * sampling frequency. For the low pass with time constant of 0.1, and sampling
 * frequency of 100 Hz,
 * 
 * alpha = 0.01/(0.01+0.1) = 0.0909
 */
float measureVoltage() {
  const static float alpha = 0.0909;
  const static float k = 0.01527;
  
  static float voltage = analogRead(VOLTAGE_SENSE_PIN)*k; /// Initialization only happens on first call to function.

  voltage = voltage*(1-alpha) + analogRead(VOLTAGE_SENSE_PIN)*k*alpha;
  
  return voltage;
}

float measureCurrent() {
  
  return 0;
}

void setup() {
  Serial.begin(115200);
  Plotuino::init(&Serial);
}

void loop() {
  Plotuino::beginTransfer(0x01);
  Plotuino::send(analogRead(0));
  Plotuino::send(analogRead(1));
  Plotuino::send(analogRead(2));
  Plotuino::send(analogRead(3));
  Plotuino::endTransfer();
}
