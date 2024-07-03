

#include "tm4c123gh6pm.h"
//#include "timer.h"
#include "ServoTD8120MG.h"
#include <stdint.h>
#include "PWM.h"
#include "os.h"
#include "Profile.h"
#include "Texas.h"
#include "CortexM.h"
#include "Math.h"


int32_t absolute(int32_t value);

//int32_t lastPosition;

//int32_t static referenz;
int32_t Duty;




// ------------Control the Period of PWM for Stearing Servo Motor------------
// Trigger pulse in mikrosec, offset = 890 Mikrosec, 90 Degree = 1340 Mikrosec , Pulse width = 1565
// Input: degree is an integer 0..180 , 0 Right, 180 Left, that indicates how many takts has duty cycle (min = 890, max = 1760 )
// Output: Void

void Angle(uint32_t degree){
		//DisableInterrupts();



			if (degree>90)
			{
				Duty = (((degree-90) * 420)/90)+1340;
			} 
			else if (degree<90)
			{
				Duty = ((degree * 460)/90)+890;
			}
			else if (degree==90)
			{
				Duty = 1340;
			}
			
		
			PWM1_1_A_disable();
			// New Duty Cycle
			PWM1_1_A_Duty( (5*Duty));
			// Activate PWM Signal
			PWM1_1_A_enable();
				//EnableInterrupts();	
	}
		

// delay in microsec, target in degrees
	
	
	
void servoupdownarm(uint32_t target , uint32_t delay){
	
	
	Angle(target);
	

	
//	///  Verifikationen Einfügen für Winkel und Delay
//	
//	if (referenz == 0){   // Referenz Fahrt wenn erstes Mal
//		Angle(NULLPOSITION);
////		Delay_MicroSecond(2000000);
//		referenz = 1;
//		lastPosition = NULLPOSITION;
//	}
//	
//	int32_t dif = target - lastPosition;
//	
//	int32_t difABS = absolute(target - lastPosition);

//	if (delay == 0)
//	{
//		Angle(target); // tell servo to go to position in variable 'pos'
//		lastPosition = target;		
//	} else
//	{
//		if (dif>0)
//		{
//			for(int32_t pos = lastPosition; pos <=target; pos += 1)  // goes from 0 degrees to 180 degrees 
//			{                                  								// in steps of 3 degree 
//										
//				if(pos>target) pos=target; //limit upper value

//				Angle(pos); // tell servo to go to position in variable 'pos' 
//				lastPosition = pos;	
////				Delay_MicroSecond(delay); // waits 150ms to slow servo movement
//			}
//		} else
//		{
//			for(int32_t pos = lastPosition; pos >=target; pos -= 1)  // goes from 0 degrees to 180 degrees 
//			{                                  								// in steps of 3 degree 
//				if(pos<target) pos=target; //limit lower value

//				Angle(pos); // tell servo to go to position in variable 'pos' 
//				lastPosition = pos;
////				Delay_MicroSecond(delay); // waits 150ms to slow servo movement
//			}
//		}	
//	}

}
								



