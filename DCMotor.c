

#include <stdint.h>
#include "BSP.h"
#include "ServoTD8120MG.h"
#include "PWM.h"
#include "GPIO.h"

// ------------Control the Period of PWM for DC Motor------------

// Output: Void
// Input: speed 5 Takt in 1 mikros * 250 mikros = 20kHZ, Servo steering 

void DCforward(uint32_t speed , uint32_t direction){
	
	if (speed == 0) {
		PWM0_1_A_disable();   // Deaktivate PWM for DC Motor if velocity is 0
		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_disable();  //  DC IN2 LOW

	} else {

		PortB_2_enable();  //  DC IN1 HIGH
		PortB_3_disable();  //  DC IN2 LOW
		
		PWM0_1_A_disable();
		// New Duty Cycle
		PWM0_1_A_Duty(speed-1);
		// Activate PWM Signal
		PWM0_1_A_enable();

	}
	
}	
// ------------Control the Period of PWM for DC Motor------------
// 
// Output: Void	
// Input: speed 5 Takt in 1 mikros * 250 mikros = 20kHZ, Servo steering 
void DCbackward(uint32_t speed , uint32_t direction){

if (speed == 0) {
		PWM0_1_A_disable();   // Deaktivate PWM for DC Motor if velocity is 0
		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_disable();  //  DC IN2 LOW

	} else {

		PortB_2_disable();  //  DC IN1 LOW
		PortB_3_enable();  //  DC IN2 HIGH
		
		PWM0_1_A_disable();
		// New Duty Cycle
		PWM0_1_A_Duty(speed-1);
		// Activate PWM Signal
		PWM0_1_A_enable();
	}
}
