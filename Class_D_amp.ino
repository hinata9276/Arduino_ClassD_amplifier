/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int ledPin = 13;      // select the pin for the LED
int pwmu = 5;  // variable to store the value coming from the sensor
int pwmd = 6;
signed int x;
bool state;
unsigned short top = 255;
unsigned short y;
void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  pinMode(pwmu, OUTPUT);
  pinMode(pwmd, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A4, LOW);
  digitalWrite(7, LOW);
  digitalWrite(4, LOW);
  Serial.begin(19200);
  
  //settung up ADC
// clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  cbi(ADMUX,ADLAR);
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B00000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  ADMUX |= 2; //A2
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  sbi(ADCSRA,ADEN);
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  sbi(ADCSRA,ADATE);
  
  // Clear ADTS2..0 in ADCSRA (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRA &= B11111000;
  
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000011; //8
  
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  sbi(ADCSRA,ADIE);

  // Kick off the first ADC
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  sbi(ADCSRA,ADSC);
  
  //setting up PWM output from Timer 0
  TCCR0A = 0;//reset the register
  TCCR0B = 0;//reset the register
  sbi(TCCR0A,COM0A1);
  cbi(TCCR0A,COM0A0); // enable pin 6 PWM non-inverting mode
  sbi(TCCR0A,COM0B1);
  cbi(TCCR0A,COM0B0); // enable pin 5 PWM non-inverting mode
  sbi(TCCR0A,WGM01);
  sbi(TCCR0A,WGM00); // fast PWM mode
  cbi(TCCR0B,WGM02);
  
  //setting up PWM output from Timer 2
  TCCR2A = 0;//reset the register
  TCCR2B = 0;//reset the register
  sbi(TCCR2A,COM2A1);
  cbi(TCCR2A,COM2A0); // enable pin 11 PWM non-inverting mode
  sbi(TCCR2A,COM2B1);
  cbi(TCCR2A,COM2B0); // enable pin 3 PWM non-inverting mode
  sbi(TCCR2A,WGM21);
  sbi(TCCR2A,WGM20); // fast PWM mode
  cbi(TCCR2B,WGM22);
  
  
  TCCR0B = 0x01;// prescaler 1
  TCCR2B = 0x01;// prescaler 1
  OCR0A=0;//duty cycle for pin 6
  OCR0B=0;//duty cycle for pin 5
  OCR2A=58; //create 1.1V on pin 11
  OCR2B=58; //create 1.1V on pin 3
}

ISR(ADC_vect) 
{
  x = (ADCL | (ADCH<<8));  // read 10 bit value from ADC
  x -= 512;
  if (x > 0){
    y = map(x,0,512,0,255);
    //y &= 255;
    //PORTD &= B10111111; //overwrite pin 5and 6 as low when PWM is disabled
    //TCCR0A &= B00111111; //disable pin 6 PWM mode, enable pin 5 PWM
    OCR0A=0;//duty cycle for pin 6    
    OCR0B=y;//duty cycle for pin 5
  }
  else{
    y = abs(x);
    y = map(y,0,512,0,255);
    //y &= 255;
    //PORTD &= B11011111; //overwrite pin 5and 6 as low when PWM is disabled
    //TCCR0A &= B11001111; //enable pin 6 PWM mode, disable pin 5 PWM mode
    OCR0A=y;//duty cycle for pin 6
    OCR0B=0;//duty cycle for pin 5
  }
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(100);
  Serial.println(x);
  digitalWrite(ledPin, LOW);
  delay(100);
  Serial.println(x);
  /*
  digitalWrite(ledPin, HIGH);
  delay(10);
  digitalWrite(ledPin, LOW);
  delay(1000);
  */
}
