/*
 ---------------------------------------------------------------------------
 Created by Edwin Croissant
 Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html

 See "SMT172_T1_fp.h" for purpose, syntax, version history, links, and more.
 ---------------------------------------------------------------------------
*/

#include "SMT172_T1_fp.h"

namespace {
volatile uint16_t cycleCount;
volatile uint32_t minTicks;
volatile uint8_t status;	// 0 = busy, 1 = success, 2 = not connected
volatile uint16_t prevFallingEdgeStamp;
volatile uint16_t prevRaisingEdgeStamp;
volatile uint32_t totalTicks;
volatile float totalDutyCyle;
volatile uint32_t interimSensorError;
}

ISR (TIMER1_COMPA_vect) {
	// sensor frequency is lower then 488 Hz @ 32 Mhz and 244 Hz @ 16 Mhz. Disconnected sensor?

	TIMSK1 = 0;    // no more interrupts for now from timer 1
	TIFR1 = _BV(ICF1) | _BV(TOV1) | _BV(OCF1A); // clear pending interrupts as we are done
	status = 2;
}	// end of TIMER1_COMPA_vect

ISR (TIMER1_CAPT_vect) {
	uint16_t tempStamp;
	uint16_t cycleTicks;
	uint16_t highLevelTicks;
	tempStamp = ICR1;//	The ICR1 Register should be read as early in the interrupt handler routine as possible.
	TCCR1B ^= _BV(ICES1);//	Toggle Input Capture Edge Select directly after ICR1 is read
	TIFR1 |= _BV(ICF1);		//	Clear ICF1 directly after edge direction change
	if (TCCR1B & _BV(ICES1)) // if true we have a falling edge as ICES1 was already toggled
	{
		if (cycleCount > 0) {
			highLevelTicks = tempStamp - prevRaisingEdgeStamp;
			cycleTicks = tempStamp - prevFallingEdgeStamp;
			totalTicks += cycleTicks;
			totalDutyCyle += float(highLevelTicks) / cycleTicks;
			if ((totalTicks >= minTicks) // always measure a multiple of 8 sensor cycles
						&& (cycleCount % 8 == 0)) {
				TIMSK1 = 0;    // no more interrupts for now from timer 1
				TIFR1 = _BV(ICF1) | _BV(OCF1A); // clear pending interrupts as we are done
				status = 1;
				return;
			}
		}
		prevFallingEdgeStamp = tempStamp;
		cycleCount++;
	} else {
		OCR1A = tempStamp; // advance the output compare register
		prevRaisingEdgeStamp = tempStamp;
	}
} // end of TIMER1_CAPT_vect

void SMT172_T1_fp::startTemperatureByTime(uint16_t ms) {
	/* 	initialize Timer1 to measure the duty cycle at the Input Capture Pin 1
		during a minimum of ms time	*/

	cycleCount = 0;
	status = 0;
	totalTicks = 0;
	totalDutyCyle = 0;
	minTicks = F_CPU / 1000 * ms;

	// reset timer 1
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;          // Timer/Counter to zero

	// Set up Timer 1
	OCR1A = 0; // Generates a interrupt when sensor frequency is lower then 488 Hz @ 32 Mhz and 244 Hz @ 16 Mhz
	TCCR1B = _BV(CS10) | _BV(ICES1); // start Timer 1, no prescaler and Input Capture Edge Select rising on D8
	TIFR1 = _BV(ICF1) | _BV(OCF1A); // clear pending interrupts so we don't get a bogus one
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A); // and enable interrupt on Timer 1 input capture and compare A
}

uint16_t SMT172_T1_fp::startTemperature(float sensorError) {
	/* Calculate the minimum time required in ms for the required sensor error based on the
	 * previous captured sensor frequency. Returns the minimum time in ms. */

	uint16_t ms;

	if (totalTicks == 0)
		ms = 1;
	else
		ms = 1000 * (20000.00 * getFrequency())
				/ (3 * pow(sensorError, 2) * pow(F_CPU, 2));
	startTemperatureByTime(ms);
	return ms;
}

uint8_t SMT172_T1_fp::getStatus(void) {
	// return 0 when busy, 1 when success, 2 when not connected
	return status;
}


float SMT172_T1_fp::getTemperature(void) {
	// return the temperature from the previous startTemperature command
	// in degrees Celsius
	float dutyCycle = totalDutyCyle / cycleCount;
	return -1.43 * pow(dutyCycle, 2) + 214.56 * dutyCycle - 68.6;
}

float SMT172_T1_fp::getError(void) {
	// return the standard deviation of the sampling noise
	float sensorPeriod = getTime() / (cycleCount);
	float clockCycleTime = 1 / float(F_CPU);
	return (200 * clockCycleTime) / sqrt(6 * getTime() * sensorPeriod);
}

float SMT172_T1_fp::getFrequency(void) {
	// return the frequency of the sensor
	return float(F_CPU) / (totalTicks / cycleCount);
}

float SMT172_T1_fp::getDutyCycle(void) {
	// return the duty cycle of the sensor
	return totalDutyCyle / cycleCount;
}

float SMT172_T1_fp::getTime(void) {
	// return the measuring time in seconds
	return float(totalTicks) / F_CPU;
}

uint16_t SMT172_T1_fp::getCycleCount(void) {
	// returns the amount of measured cycles
	return cycleCount;
}
