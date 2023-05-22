/*
 ---------------------------------------------------------------------------
 Created by Edwin Croissant, ATmega4809 support added by Milan Milenovic
 Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html

 See "SMT172_T1.h" for purpose, syntax, version history, links, and more.
 ---------------------------------------------------------------------------
*/

#include "SMT172_T1.h"

namespace {
volatile uint16_t cycleCount;
volatile uint32_t minTicks;
volatile uint8_t status;	// 0 = busy, 1 = success, 2 = not connected
volatile uint16_t prevFallingEdgeStamp;
volatile uint16_t prevRaisingEdgeStamp;
volatile uint32_t totalTicks;
volatile uint32_t totalHighLevelTicks;
volatile uint32_t interimSensorError;
}

#if defined(__AVR_ATmega4809__)

void SMT172_T1::TCB1_init (void)
{
   //TCB1 is used for sensor disconnected detection. TODO: It is probably possible to do this only using TCB0
    EVSYS.CHANNEL1 = 0b01001000; // event channel 1 connects to pin PB0 (Port B, Pin 0 = Arduino ~D9) WARNING! Not all generators can be connected to all channels!
    EVSYS.USERTCB1 = 2;          // connect the counter to event channel 1 (2-1)

    /* Load the Compare or Capture register with the timeout value*/
    TCB1.CCMP = 3686; //900 Hz    16000:65535=900:CCMP

    /* Enable TCB and set CLK_PER divider to 1 (No Prescaling) */
    TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;

    /* Configure TCB in Periodic Timeout mode */
    TCB1.CTRLB = TCB_CNTMODE_TIMEOUT_gc;

    /* Enable Capture or Timeout interrupt */
    TCB1.INTCTRL = TCB_CAPT_bm;

    /* Enable Event Input and Event Edge*/
 
    TCB1.EVCTRL = 1 << TCB_CAPTEI_bp    // TCBn.EVCTRL: Event Input Enable (enabled)
                  | 0 << TCB_EDGE_bp    // TCBn.EVCTRL: Event Edge: (positive edge)
                  | 1 << TCB_FILTER_bp; // TCBn.EVCTRL: Input Capture Noise Cancellation Filter (1 enabled, 0 disabled)
}

ISR(TCB1_INT_vect)
{

    TCB1.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
    status=2; // sensor disconnected
}

ISR(TCB0_INT_vect)
{
   
   uint16_t tempStamp;
   uint16_t cycleTicks;
   uint16_t highLevelTicks;
   tempStamp = TCB0.CCMP;
             
   TCB0.EVCTRL ^= _BV(TCB_EDGE_bp); // Toggle Input Capture Edge Select directly after interrupt is read
   
   if (bitRead(TCB0.EVCTRL, TCB_EDGE_bp) == 0) // if true we have a falling edge
   {
      if (cycleCount > 0)
      {
         highLevelTicks = tempStamp - prevRaisingEdgeStamp;
         totalHighLevelTicks += highLevelTicks;
         cycleTicks = tempStamp - prevFallingEdgeStamp;
         totalTicks += cycleTicks;
         if ((totalTicks >= minTicks) // always measure a multiple of 8 sensor cycles
             && (cycleCount % 8 == 0))
         {
            
            TCB0.INTCTRL = 0 << TCB_CAPT_bp; // disable interrupts from this timer for now
            TCB0.INTFLAGS = 1;               // clear pending interrupts as we are done. Anything else to clear?
            
            status = 1;
            return;
         }
      }
      prevFallingEdgeStamp = tempStamp;
      cycleCount++;
   }
   else
   {
      prevRaisingEdgeStamp = tempStamp;
   }
}

void SMT172_T1::startTemperatureByTime(uint16_t ms)
{
   /* 	initialize Timer1 to measure the duty cycle
      during a minimum of ms time	*/

   cycleCount = 0;
   status = 0;
   totalTicks = 0;
   totalHighLevelTicks = 0;
   minTicks = F_CPU / 1000 * ms;

   EVSYS.CHANNEL1 = 0b01001000; // event channel 1 connects to pin PB0 (Port B, Pin 0 = Arduino ~D9) WARNING! Not all generators can be connected to all channels!
   EVSYS.USERTCB0 = 2;          // connect the counter to event channel 1 (2-1)

   TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc   // TCBn.CTRLA: CLK_PER (CLKDIV1 aka No-Prescaling)
                | 1 << TCB_ENABLE_bp    // TCBn.CTRLA: Enable bit position (enabled)
                | 0 << TCB_RUNSTDBY_bp; // TCBn.CTRLA: Run Standby bit position (disabled)

   TCB0.CTRLB = 0 << TCB_ASYNC_bp      // TCBn.CTRLB: Asynchronous Enable: disabled
                | TCB_CNTMODE_CAPT_gc; // TCBn.CTRLB: Input Capture Event

   TCB0.CNT = 0x0000; // Timer/Counter to zero

   TCB0.INTFLAGS = 1;               // clear pending interrupts so we don't get a bogus one
  
   TCB0.EVCTRL = 1 << TCB_CAPTEI_bp    // TCBn.EVCTRL: Event Input Enable (enabled)
                 | 0 << TCB_EDGE_bp    // TCBn.EVCTRL: Event Edge: (positive edge)
                 | 1 << TCB_FILTER_bp; // TCBn.EVCTRL: Input Capture Noise Cancellation Filter (1 enabled, 0 disabled)

   TCB0.INTCTRL = 1 << TCB_CAPT_bp; // TCBn.INTCTRL: Capture Interrupt Enable (enabled)
   TCB0.CTRLA |= TCB_ENABLE_bm; // TCBn.CTRLA: bit0=Enable bit mask (Not needed?)
}

uint16_t SMT172_T1::startTemperature(float sensorError)
{
   /* Calculate the minimum time required in ms for the required sensor error based on the
    * previous captured sensor frequency. Returns the minimum time in ms. */

   uint16_t ms;

   if (totalTicks == 0)
      ms = 1;
   else
      ms = 1000 * (20000.00 * getFrequency()) / (3 * pow(sensorError, 2) * pow(F_CPU, 2));
   startTemperatureByTime(ms);
   return ms;
}

uint8_t SMT172_T1::getStatus(void)
{
   // return 0 when busy, 1 when success, 2 when not connected
   return status;
}
#else
//other boards code goes here
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
			totalHighLevelTicks += highLevelTicks;
			cycleTicks = tempStamp - prevFallingEdgeStamp;
			totalTicks += cycleTicks;
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

void SMT172_T1::startTemperatureByTime(uint16_t ms) {
	/* 	initialize Timer1 to measure the duty cycle at the Input Capture Pin 1
		during a minimum of ms time	*/

	cycleCount = 0;
	status = 0;
	totalTicks = 0;
	totalHighLevelTicks = 0;
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

uint16_t SMT172_T1::startTemperature(float sensorError) {
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

uint8_t SMT172_T1::getStatus(void) {
	// return 0 when busy, 1 when success, 2 when not connected
	return status;
}
#endif

float SMT172_T1::getTemperature(void) {
	// return the temperature from the previous startTemperature command
	// in degrees Celsius
	float dutyCycle = float(totalHighLevelTicks) / totalTicks;
	return -1.43 * pow(dutyCycle, 2) + 214.56 * dutyCycle - 68.6;
}

float SMT172_T1::getError(void) {
	// return the standard deviation of the sampling noise
	float sensorPeriod = getTime() / (cycleCount);
	float clockCycleTime = 1 / float(F_CPU);
	return (200 * clockCycleTime) / sqrt(6 * getTime() * sensorPeriod);
}

float SMT172_T1::getFrequency(void) {
	// return the frequency of the sensor
	return float(F_CPU) / (totalTicks / cycleCount);
}

float SMT172_T1::getDutyCycle(void) {
	// return the duty cycle of the sensor
	return float(totalHighLevelTicks) / totalTicks;
}

float SMT172_T1::getTime(void) {
	// return the measuring time in seconds
	return float(totalTicks) / F_CPU;
}

uint16_t SMT172_T1::getCycleCount(void) {
	// returns the amount of measured cycles
	return cycleCount;
}
