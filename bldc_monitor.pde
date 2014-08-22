#include <WProgram.h>
#include <util/atomic.h>
#include "Transfer.h"

#define OUTPUT_TRANSFER 1
#define OUTPUT_READABLE 2

#define OUTPUT_TYPE OUTPUT_TRANSFER

#define VOLTAGE_SENSE_PIN A0
#define CURRENT_SENSE_PIN_A A2
#define CURRENT_SENSE_PIN_B A3
#define FORCE_SENSE_PIN_A A6
#define FORCE_SENSE_PIN_B A7
#define TACHOMETER_INT_PIN_A 2
#define TACHOMETER_INT_PIN_B 3 // ??
#define SERVO1 9
#define SERVO2 10

#define NUMBER_OF_MOTOR_POLES 12

#define ESC_MAX_PULSE_WIDTH 1060
#define ESC_MIN_PULSE_WIDTH 1860

#define DISTANCE_FROM_PROPELLER_AXIS_TO_PIVOT (11.0f)
#define DISTANCE_FROM_LOAD_CELL_AXIS_TO_PIVOT (12.0f)

#define FORCE_RATIO (DISTANCE_FROM_LOAD_CELL_AXIS_TO_PIVOT/DISTANCE_FROM_PROPELLER_AXIS_TO_PIVOT)

volatile uint32_t pulseCountA = 0;
volatile uint32_t pulseTimerA = 0;
volatile uint32_t lastPulseTimerA = 0;
volatile int32_t rpsA = 0;
volatile uint32_t pulseCountB = 0;
volatile uint32_t pulseTimerB = 0;
volatile uint32_t lastPulseTimerB = 0;
volatile int32_t rpsB = 0;

uint32_t outputTimer = 0;

float thrust = 0;
float tareThrust = 0;

struct BLDCMonitorStruct {
  int16_t rpmA;
  int16_t rpmB;
  float currentA;
  float currentB;
  float voltage;
  float thrust;
  int16_t pwmA;
  int16_t pwmB;
} data;

struct BLDCCommandStruct {
  int16_t pwmA;
  int16_t pwmB;
} command;

Transfer transfer;

/** The following interrupt routine captures pulses from the optocouple/low-pass
 * circuit. A timeout is implemented to prevent counting a single pulse as multiple
 * pulses (basically debouncing the pulse).
 */
ISR(INT0_vect) {
  // Pulses are between 480 us and 200 us (at 1.67 kHz)
  if ( micros()-lastPulseTimerA > 200 ) {
    pulseCountA++;
    lastPulseTimerA = micros();
  }
  
  if ( pulseCountA > 10 ) {
    rpsA = pulseCountA/(float(micros()-pulseTimerA)/1000000.0f);
    pulseTimerA = micros();
    pulseCountA = 0;
  }
}

ISR(INT1_vect) {
  // Pulses are between 480 us and 200 us (at 1.67 kHz)
  if ( micros()-lastPulseTimerB > 200 ) {
    pulseCountB++;
    lastPulseTimerB = micros();
  }
  
  if ( pulseCountB > 10 ) {
    rpsB = pulseCountB/(float(micros()-pulseTimerB)/1000000.0f);
    pulseTimerB = micros();
    pulseCountB = 0;
  }
}

/** The RPM measurement is done in the interrupt handling code. If the motor
 * stops then there are no interrupts and therefore the sensor hangs on the final 
 * value instead of going to zero RPM. This function checks for more than 0.1 seconds
 * with no pulses and resets the measurement to zero if necessary. The AVR "atomic"
 * macros are used to prevent this function from being triggered falsely. */
static __inline__ void checkForZeroPulses() {
  uint32_t safePulseTimer = 0;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    safePulseTimer = pulseTimerA;
  }
  
  if ( micros()-safePulseTimer > 300000l ) {
    rpsA = 0;
  }

  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    safePulseTimer = pulseTimerB;
  }
  
  if ( micros()-safePulseTimer > 300000l ) {
    rpsB = 0;
  }
}

/** This function initializes the interrupt (INT0) for the tachometer
 * pulses. It also has code to initialize the servo output, which will be
 * moved to a separate function eventually.
 */
void initTachometer() {
  // Initialize input/output pins
  pinMode(TACHOMETER_INT_PIN_A,INPUT);
  pinMode(TACHOMETER_INT_PIN_B,INPUT);
  pinMode(SERVO1,OUTPUT);
  pinMode(SERVO2,OUTPUT);
  
  // Initialize PWM output for 50 Hz (Digital pin 9)
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = 40000; // CPU/prescaler/frequency = 16000000/8/50 = 10000 // 50 Hz PWM rate
  
  // Attach the interrupt pins (INT0 and INT1, Arduino Pin 2, 3)
  EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11);
  EIMSK |= (1 << INT0) | (1 << INT1);
}

/** This function filters the RPM with a low-pass filter. */
void filterRPM(int16_t &rpm,int16_t newRPM,float dt) {
  const static float tau = 0.25;

  float alpha = dt/(dt+tau);
  
  rpm = rpm*(1-alpha) + newRPM/NUMBER_OF_MOTOR_POLES*2*alpha;
}

/** This function is from http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino.
 * It measures the actual value of VCC by comparing it with the Atmega328's internal 1.1V reference,
 * which is more accurate than the voltage regulator. This value can then be used to more
 * accurate measure the analog measurements. 
 *
 * The result is returned as a float.
 */
float readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Provides voltage in mV
  return result/1000.0f;
}

/** This function measures the voltage of the power source with the ADC. The
 * power source is connected through a voltage divider consisting of a 10Kohm
 * and 4.7Kohm resistor (2% tolerance). This produces a dividing factor:
 * 
 * V_out/V_in = R1 / (R1 + R2) = 4.7 / (4.7 + 10) = 0.3197
 * 
 * The Atmega328p has a 10-bit ADC and the analog reference voltage is Vcc. Therefore,
 * there are Vcc/1023 V/step. The volts (V) in this expression is the 
 * same as V_out. Therefore,
 * 
 * Vcc/1023 V_out/step x 1/0.3197 V_in/V_out = Vcc*0.00305761 V_in/step
 * 
 * This gives a max reading of 1023*Vcc*0.00306 = 15.6 V_in.
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
 * sampling frequency. For the low pass with time constant of 0.25, and sampling
 * frequency of 100 Hz,
 * 
 * alpha = 0.01/(0.01+0.1) = 0.0909
 */
void measureVoltage(float dt) {
  const static float k = 0.00305761;
  const static float tau = 0.25;
  
  static bool initialized = false;
  if ( !initialized ) {
    initialized = true;
    data.voltage = analogRead(VOLTAGE_SENSE_PIN)*readVcc()*k;
  }
  
  float alpha = dt/(dt+tau);

  data.voltage = data.voltage*(1-alpha) + analogRead(VOLTAGE_SENSE_PIN)*readVcc()*k*alpha;
}

/** This function measure the current supplied from the power source to the speed 
 * controller using a ACS715 current sensor chip. The output of this current 
 * sensor is 0.133 V/A. Like specified above, the Atmega328p has a 10 bit ADC and
 * 5.0 V reference voltage so that it has 0.004883 V/step. Combining this with the
 * current sensor relation provides:
 * 
 * Vcc/1023 V/step x 1/0.133 A/V = Vcc*0.0073498 A/step
 * 
 * The sensor measures 0-30A and is biased at 0.5 V.
 * 
 * A low pass filter is also used on the current measurement to smooth it out
 * slightly. The low pass filter has a time constant of 0.25 s.
 */
void measureCurrent(float dt) {
  const static float k = 0.0073498;
  const static float tau = 0.25;
  const static int16_t center = 102; // equivalent to 0.5 V
  
  static bool initialized = false;
  if ( !initialized ) {
    initialized = true;
    data.currentA = (analogRead(CURRENT_SENSE_PIN_A)-center)*readVcc()*k;
    data.currentB = (analogRead(CURRENT_SENSE_PIN_B)-center)*readVcc()*k;
  }
  
  float alpha = dt/(dt+tau);
  
  data.currentA = data.currentA*(1-alpha) + (analogRead(CURRENT_SENSE_PIN_A)-center)*readVcc()*k*alpha;
  data.currentB = data.currentB*(1-alpha) + (analogRead(CURRENT_SENSE_PIN_B)-center)*readVcc()*k*alpha;
}

/** This function measures the force on the FC22 compression load cell. The load
 * cell is already amplified with an output range of 0.5 to 4.5 V and a measurement
 * range of 0-25 lb. 
 * 
 * The range of the sensor is 4.0 V and the offset is 0.5 V, so that the relationship 
 * between voltage and force is:
 * 
 * F = 25.0/4.0*(V-0.5) = 6.25*(V-0.5)
 * 
 * The voltage can be calculated from:
 * 
 * V = ADC/1023*Vcc = ADC/1023*Vcc
 * 
 * So the resulting relationship is:
 * 
 * F = 6.25*(ADC*Vcc/1023-0.5)
 * 
 * In this measurement scenario, the sensor is preloaded with a tare weight that allows
 * both positive and negative loads to be measured. This function only provides the 
 * measured force without accounting for the tare weight. That must be recorded separately
 * and subtracted from this measurement.
 */
void measureForce(float dt) {
  const static float Kf = 6.25;
  const static float Kadc = 1.0/1023.0;
  const static float offset = 0.5;
  
  const static float tau = 0.1;
  
  float newThrust = Kf*(readVcc()*Kadc*analogRead(FORCE_SENSE_PIN_A)-offset);
  
  float alpha = dt/(dt+tau);
  
  thrust = thrust*(1-alpha) + newThrust*alpha;
}

/** This function sets the tare force to be subtracted from the measured force.
 * It first reads the sensor for 1 second to ensure that the measurement is stable
 * and the first order filter has time to settle. */
void setTareForce() {
  measureForce(100000.0f); // Large dt so that the filter is ignored
  
  for ( uint8_t i = 0 ; i < 100 ; i++ ) {
    measureForce(0.01);
    delay(10);
  }
  
  tareThrust = thrust;
}

/** Calculates the motor thrust from the measured force, tare force, and force ratio
 * based on the moment arm lengths that are defined at the top. */
float getThrust() {
  return (thrust-tareThrust)*FORCE_RATIO;
}

/** Outputs a PWM signal to control the motor controller. Outputs to digital pins 9 and 10. */
void outputPWM(uint16_t _pwm) {
  OCR1A = _pwm*2;
  OCR1B = _pwm*2;
}

void setup() {
  Serial.begin(115200);
  initTachometer();
  setTareForce();

  transfer.setStream(&Serial);
  
  outputPWM((ESC_MAX_PULSE_WIDTH+ESC_MIN_PULSE_WIDTH)/2);
  delay(1000);
}

void loop() {
  
  static long measurementTimer;
  static long outputTimer;
  static long inputTimer;
  
  float dt = float(micros()-measurementTimer)/1000000l;
  
  /** Measurement loop. I would put this in a timer interrupt but since these
   * are relatively long calculations the interrupt could block the RPM pulse
   * interrupt from occuring. */
  if ( dt > 0.005 ) {
    measureVoltage(dt);
    measureCurrent(dt);
    measureForce(dt);
    checkForZeroPulses();
    filterRPM(data.rpmA,rpsA*60,dt);
    filterRPM(data.rpmB,rpsB*60,dt);
    data.thrust = getThrust();
    measurementTimer = micros();
  }
  
  /** Output loop. Output frequency can be adjusted. Currently, the output message
   * is 2+1+1+6*4+2 = 30 bytes = 240 bits/message. Therefore, at 115200 bps, 
   * 
   * 115200 bits/s x 1/240 messages/bit = 480 messages/s
   * 
   * can be sent under ideal conditions. In practice, this number will be lower. */
  if ( float(micros()-outputTimer)/1000000l > 0.1 ) {
    outputTimer = micros();
    
    switch ( OUTPUT_TYPE ) {
    case OUTPUT_TRANSFER:
			
	{
        transfer.send(&data);
			}
			break;
    case OUTPUT_READABLE:
			{
        Serial.write(27);       // ESC command
        Serial.print("[2J");    // clear screen command
        Serial.write(27);
        Serial.print("[H");     // cursor to home command
				Serial.println("== Motor A ==");
        Serial.print(getThrust()); Serial.println(" lb");
				Serial.print(data.rpmA); Serial.println(" RPM");
				Serial.print(data.currentA*data.voltage); Serial.println(" W");
        Serial.print(data.currentA); Serial.println(" A");
        Serial.print(data.voltage); Serial.print(" V");
				Serial.println("");
        Serial.println("== Motor B ==");
        Serial.print(getThrust()); Serial.println(" lb");
        Serial.print(data.rpmB); Serial.println(" RPM");
        Serial.print(data.currentB*data.voltage); Serial.println(" W ");
        Serial.print(data.currentB); Serial.println(" A");
        Serial.print(data.voltage); Serial.print(" V");
        Serial.println("");
			}
			break;
		default:
			Serial.println("Must define OUTPUT_TYPE.");    
    } 
  }

  // Sweep through throttle
  if (false) {
    static bool initialized = false;
    static long startTime;
    if ( !initialized ) {
      startTime = millis();
      initialized = true;
    }
    //static long startTime = millis();
    command.pwmA = 410*sin(2*PI/60*(millis()-startTime)/1000.0f)+(ESC_MIN_PULSE_WIDTH+ESC_MAX_PULSE_WIDTH)/2;
    outputPWM(command.pwmA);
    data.pwmA = command.pwmA;
  }
  
  /** Serial input to control motor speed */
  if ( float(micros()-inputTimer)/1000000l > 0.1 ) {
    inputTimer = micros();
    switch (OUTPUT_TYPE) {
    case OUTPUT_TRANSFER:
      if ( transfer.receive(&command) ) {
        outputPWM(command.pwmA-40);
        data.pwmA = command.pwmA;
        data.pwmB = command.pwmB;
      }  
      break;
    case OUTPUT_READABLE:
      if (Serial.available() > 0) {
        command.pwmA = map(Serial.read(),0,256,ESC_MIN_PULSE_WIDTH,ESC_MAX_PULSE_WIDTH);
        outputPWM(command.pwmA);
      }
      break;
    }
  }

}
