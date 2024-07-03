

#include <stdint.h>
#include "BSP.h"
#include "ServoTD8120MG.h"
#include "PWM.h"
#include "GPIO.h"

// ------------Control the Period of PWM for DC Motor------------
// Joystick is not stable, so there is a small offset. 
// Position Zero is between 0 and 200
// Input: speed is an integer , that indikates how many takts has duty cycle (max 2500 = 1 msec = 1 KHz, 1 mikrosec = 2,5 Takts )
// Output: Void


void DCforward(uint32_t speed , uint32_t direction){
	
	if (speed == 0) {
		PWM0_3_A_disable();   // Deaktivate PWM for DC Motor if velocity is 0
		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_disable();  //  DC IN2 LOW

	} else {

		PortB_2_enable();  //  DC IN1 HIGH
		PortB_3_disable();  //  DC IN2 LOW
		
		PWM0_3_A_disable();
		// New Duty Cycle
		PWM0_3_A_Duty(speed-1);
		// Activate PWM Signal
		PWM0_3_A_enable();

	}
	
}	
// ------------Control the Period of PWM for DC Motor------------
// Joystick is not stable, so there is a small offset. 
// Position Zero is between 0 and 200
// Input: speed is an integer , that indikates how many takts has duty cycle (max 2500 = 1 msec = 1 KHz, 1 mikrosec = 2,5 Takts )
// Output: Void	
void DCbackward(uint32_t speed , uint32_t direction){

if (speed == 0) {
		PWM0_3_A_disable();   // Deaktivate PWM for DC Motor if velocity is 0
		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_disable();  //  DC IN2 LOW

	} else {

		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_enable();  //  DC IN2 HIGH
		
		PWM0_3_A_disable();
		// New Duty Cycle
		PWM0_3_A_Duty(speed-1);
		// Activate PWM Signal
		PWM0_3_A_enable();
	}
}
