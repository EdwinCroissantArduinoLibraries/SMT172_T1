/*
 Demo sketch for the SMT172_T1 library. 
 This sketch is intended for the ATmega328P (Uno, Nano, Pro Mini etc.)
 There is no timer2 on the ATmega32U4 (Pro Micro)
 This sketch will output to serial.
 Connect the output of the SMT172 to pin 8 (Input Capture Pin of timer 1)
 Timer 2 is set up in phase correct PWM and output a duty cycle of
 10.98% ~-45.06 C on pin 3 and a duty cycle of 92.94% ~139.58 C on pin 11
 Connect pin 8 to pin 3 or pin 11 to check the working if no SMT172 is available

 On Arduino Uno WiFi Rev2 (ATmega4809) test functionality is provided only on Pin 11
 In this example 50% duty cycle and frequencies of 3000 and 800 Hz are shown
 Any value can be tested by modifying the TCA0.SINGLE.PERBUF and TCA0.SINGLE.CMP0BUF
 accordin to the explanation in the comments. Sensor output (or pin D11 for testing) is
 connected to pin D9, preserving the compatability with Arduino Uno R3 (ATmega328P)
*/

#include<arduino.h>
#include <SMT172_T1.h>

uint32_t LastSensorUpdate;

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(115200);
	
	Serial.println(F("Demo sketch SMT172"));

#if defined(__AVR_ATmega4809__)
  pinMode(9, INPUT); //sensor input
  SMT172_T1::TCB1_init(); //for sensor disconnect detection

  PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc; //The TCA corresponding register in Port Multiplexer set to route the module outputs to port E
  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_DSBOTTOM_gc; //Waveform Generation mode set to channel 0 with a Dual-Slope PWM mode
  TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm); //set the timer to count clock ticks instead of events
  
  //This should read 38.48 degree C, about 3000Hz, with duty cycle of 50.0x
  TCA0.SINGLE.PERBUF = 667; //set the frequency (3000Hz) of the PWM signal PERBUF = 16000000/2x4x3000
  TCA0.SINGLE.CMP0BUF = 334; // half of the PERBUF for 50% duty cycle
  
  //This should read sensor disconnected
  //TCA0.SINGLE.PERBUF = 2500; //set the frequency (800Hz) of the PWM signal PERBUF = 16000000/2x4x800
  //TCA0.SINGLE.CMP0BUF = 1250; // half of the PERBUF for 50% duty cycle

  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_ENABLE_bm; //Set prescaler to 4 and start the counter with enable bit in the same register
  pinMode(11, OUTPUT); //PE0 is pin D11 in Arduino Uno WiFi Rev2


#else
//	The following code fragment sets up phase-correct PWM on pins 3 and 11 (Timer 2).
//	The waveform generation mode bits WGM are set to to 001 for phase-correct PWM.
//	The other bits are the same as for fast PWM.
  pinMode(8, INPUT);
  pinMode(12, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(11, OUTPUT);
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
	// TCCR2B = _BV(CS22); // Output frequency: 16 MHz / 64 / 255 / 2 =  490.196 Hz
	TCCR2B = _BV(CS21); // Output frequency: 16 MHz /  8 / 255 / 2 = 3921.569 Hz
	OCR2A = 237; // Output A duty cycle: 237 / 255 = 92.94%	= 129.58 C	on pin 11
	OCR2B = 28;	 // Output B duty cycle:  28 / 255 = 10.98%	= -45.06 C	on pin 3
#endif
}

// The loop function is called in an endless loop
void loop() {
	// read the sensor every 250 millisecond
	if ((unsigned long) (millis() - LastSensorUpdate) >= 250) {
		LastSensorUpdate = millis();

		SMT172_T1::startTemperature(0.002);

		repeat:
		switch (SMT172_T1::getStatus()) {
		case 0: goto repeat; // O Dijkstra, be merciful onto me, for I have sinned against you :)
		case 1: Serial.print(F("Measuring time   [ms]: "));
				Serial.println(SMT172_T1::getTime() * 1e3, 2); // convert to milliseconds
				Serial.print(F("Sensor frequency [Hz]: "));
				Serial.println(SMT172_T1::getFrequency(), 2);
				Serial.print(F("Duty cycle        [%]: "));
				Serial.println(SMT172_T1::getDutyCycle() * 100, 2);
				Serial.print(F("Temperature       [C]: "));
				Serial.println(SMT172_T1::getTemperature(), 2);
				Serial.print(F("Error            [mK]: "));
				Serial.println(SMT172_T1::getError() * 1000, 2);
				Serial.println();
				break;
		case 2: Serial.println(F("**** Sensor disconnected ****"));
				Serial.println();
		}
	}
}
