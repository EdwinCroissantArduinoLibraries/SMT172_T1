# SMT172_T1 Library for the ATmega microcontroller family and the LogicGreen lgt8f328p.
The previous SMT172 library did not work with the LogicGreen lgt8f328p as the overflow interrupt from timer 1 occasionally failed. As at 32 Mhz clock speed the amount of clock cycles at the lowest sensor frequency of 500 Hz is 64.000 and still within the counting range of 65.535 of the timer, the overflow interrupt could be omitted resulting in a slightly smaller library with less overhead in the interrupt routine.

Please note that the sensor frequency used to calculate the minimum time required to obtain the desired sampling error is now based on the previous measurement so that the very first measuring time is only 8 sensor cycles long.

Earlier documentation advised to calculated the duty cycle as the ratio between the on time and the total sensor cycle time over a multiple of 8 cycles. The present documentation advice to calculate the duty cycle individually for each sensor cycle and subsequently calculate the average duty cycle for the multiple of 8 sensor cycles. This library is based on the earlier documentation. For completeness a library is placed in the extras folder that uses floating point calculations inside the interrupt routine to calculate the duty cycle according to the present documentation. Maybe this library yield a more accurate result but uses many more clock cycles within the interrupt routine. 
    

The SMT172 is a ultra-low power, high accuracy temperature sensor. The output signal is pulse width modulated (PWM) and the duty cycle is a function of the measured temperature. For more details see the home page of the manufacturer: http://smartec-sensors.com/ and the documentation. Accuracy can be up to 0.1 degree Celsius for a limited temperature range and
the resolution up to 0.001 degree Celsius.

This library uses Timer1 and ICP1. 

The SMT172 needs a clean powersupply (a 100uF capacitor with a 100 nF ceramic capacitor in parallel fed through a small signal diode), a 220 ohm resistor in series and a 100nF ceramic capacitor as close to the sensor as possible. See also the diagram in the extras folder.

