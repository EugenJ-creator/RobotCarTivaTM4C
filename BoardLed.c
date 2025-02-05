#include "GPIO.h"



void BoardLed_init(void) {
	
	PortF_2_init();
	
}



void ledBlueToggle(void){
	
	ledToggle();
	
}

