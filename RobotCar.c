// RobotCar.c
// Runs on either MSP432 or TM4C123
// Starter project to Lab 4.  Take sensor readings, process the data,
// and output the results.  Specifically, this program will
// measure steps using the accelerometer, audio sound amplitude using
// the microphone, temperature using the TMP006, and light using the
// OPT3001.
// Daniel and Jonathan Valvano
// August 22, 2016

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2016

 Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
	
 // TO DO!!!!!	
 // 1. Profiles to do if need
 // 2. Read finctions implement for bluetooth 

#include <stdint.h>
#include "BSP.h"
#include "PWM.h"
#include "Profile.h"
#include "Texas.h"
#include "../inc/CortexM.h"
#include "os.h"
#include "ServoTD8120MG.h"
#include "DCMotor.h"
#include "Math.h"
#include "../inc/BSP.h"
#include "../inc/UART0.h"
#include "../inc/Profile.h"
#include "../inc/AP.h"
#include "AP_Services.h"
#include "PLL.h"
#include "GPIO.h"
#include "Buzzer.h"
#include "I2C3.h"
#include "I2C1.h"
#include "TempHumidity.h"
#include "GyroAccelMag.h"
#include "LightSensor.h"
//#include "RobotCar.h"
#include "ADCT0.h"

#define THREADFREQ 1000   // frequency in Hz of round robin scheduler

//---------------- Global variables shared between tasks ----------------
// Fake Global Variables
uint32_t Time;              // elasped time in 100 ms units
uint32_t Steps;             // number of steps counted
uint32_t Magnitude;         // will not overflow (3*1,023^2 = 3,139,587)
                            // Exponentially Weighted Moving Average
uint32_t EWMA;              // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
uint16_t SoundData;         // raw data sampled from the microphone
int32_t  SoundAvg;

uint32_t SoundRMS;            // Root Mean Square average of most recent sound samples
uint32_t LightData;           // 100 lux
int32_t  TemperatureData;     // 0.1C
uint8_t  TemperatureByteData; // 1C




//  Global parameters
uint32_t SpeedWheels;    // 0...255
uint32_t DirectionWheels = 0;    // 0-Forward/1-back
uint32_t AngleWheels;    // 0...180
//uint32_t SpeedAngleWheels;    // 0..9 Sensitivity
uint32_t BuzzerVolume;  // 0...1600
uint32_t selectedOptions; // 8 bit   0 bit - tempomat, ...
uint8_t tempHumidityData[6];  // Data Array for bluetooth notification
//int16_t magnetometerData[3];  // x, y, z   little indian
uint8_t magnetometerData[6];  // x, y, z   little indian
int16_t magnetometerData2[3];  // x, y, z   little indian
uint8_t speedPhotoData[3];   // Sensor Speed data

uint32_t ControlledSpeed = 0;  // Flag,  if speed is in tempomat modus


extern uint32_t lightIntensity;   //   Light intensity , Measured all 10 HZ
extern uint32_t Period ;              // (1/clock) units
uint32_t lastLightIntensity = 70; // Last measured intenity

// semaphores
int32_t NewBluetoothDirectionData;  // true when new Speed Value comes from Bluetooth Client
int32_t NewBluetoothSpeedData;  // true when new Speed Value comes from Bluetooth Client
//int32_t NewBluetoothAngleData;  // true when new Angle Value comes from Bluetooth Client
int32_t AnglePositionChanged; 	// true after triggering new angle position
//int32_t NewBluetoothBuzzerData; // true when new Buzzer volume comes from Bluetooth Client
int32_t NewTempHumidityData; 		// true when new Temp&Humidity Sensor data are ready


uint8_t SendTempHumidityFlag=0;
uint8_t SendMagnetometerFlag=0;

uint8_t TakeJoystickData; // binary semaphore



//------------ end of Global variables shared between tasks -------------



////---------------- Task0 samples data from Joystick ----------------
//// High priority thread run by OS in real time at 1000 Hz

//// *********Task0*********
//// Task0 measures Joistick Input
//// Periodic main thread runs in real time at 1000 Hz
//// collects data from joystock, high priority
//// Inputs:  none
//// Outputs: none

//void Task0(void){ 
//	uint16_t x,y; uint8_t button;
//	while(1){
//	
//	// Toggle PE1
//	Profile_Toggle0();
//		
//	/*	
//	//   In case Joysic is attached	
//		
//	OS_Wait(&TakeJoystickData);  // signaled by OS every 1ms
//	BSP_Joystick_Input(&x,&y,&button);
//     // DesiredPlace = ((113*(1023-y))>>10)+5; // 0 to 1023 mapped to 5 to 117
//	SpeedWheels=absolute((2500*(512-y))>>9);
////		SpeedWheels=absolute(250*((511-y)>>10)); // 0 to 1023 mapped to 0 to 250    // 0...250
//		DirectionWheels=-(512-y);    // 0-Forward/1-back  !!!!!!!!!!!!!!!!!!!!!To implement
//		AngleWheels=-(((180*x)>>10)-180); // 0 to 1023 mapped to 0 to 180
//		//SpeedAngleWheels;    // 0..9 Sensitivity

//	OS_Signal(&NewData); // makes task1 run every 1 sec
//	OS_Sleep(10);     // waits about 10 milisec
//	*/
//		
//	}
//}


//---------------- Task0 dummy function ----------------
// *********Task0*********
// Main thread scheduled by OS round robin preemptive scheduler
// Task0 checks for Bluetooth incoming frames but never blocks or sleeps
// Inputs:  none
// Outputs: none
uint32_t Count0;
void Task0(void){ 
	Count0 = 0;
  while(1){
    Count0++;
		AP_BackgroundProcess();
		
		
//		if (SendTempHumidityFlag){
//			if (AP_SendNotification(0)){   // Send data for 0 Characteristic , Temp&Humidity
//				SendTempHumidityFlag = 0;
//			}
//		}
		
//		if (SendMagnetometerFlag){
//			if (AP_SendNotification(1)){   // Send data for 1 Characteristic , Magnetometer
//				SendMagnetometerFlag = 0;
//			}
//		}


		WaitForInterrupt();
	}
}
/* ****************************************** */
/*          End of Task0 Section              */
/* ****************************************** */




// Main thread scheduled by OS round robin preemptive scheduler
// Task1 Control Wheels 
// Inputs:  none
// Outputs: none
void Task1(void){

  while(1){
		
		uint32_t periodVelocity = Period;
		// Toggle PE2
		//Profile_Toggle1();
		
//		OS_Wait(&NewBluetoothAngleData); // New wheels position is ready
//		servoupdownarm(AngleWheels, 0); // 0..160
//		if (SpeedWheels==0){
//			OS_Signal(&AnglePositionChanged);  // Signal that Wheel position was changed
//		}
//		servoupdownarm(140, 0);


  }
}


// Main thread scheduled by OS round robin preemptive scheduler
// Task2 Controls speed of motor
// Inputs:  none
// Outputs: none
void Task2(void){

  while(1){
		
//		OS_Wait(&NewBluetoothSpeedData); // New speed parameters are ready  
//		if (DirectionWheels==0)
//		{
//			DCforward(SpeedWheels , 0);  //between(0..250)  Forward Direction
//		} 
//		else
//		{
//			DCbackward(SpeedWheels , 0);  //between(0..250)  Backward Direction
//		}
		//WaitForInterrupt();
	}
}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 Disconnect PWM for wheels if no action for 3 seconds
// Inputs:  none
// Outputs: none
void Task3(void){

		while(1){
			OS_Wait(&AnglePositionChanged);
			OS_Sleep(3000);     // waits about 3 sec after what wheel position was changed
			if (SpeedWheels==0){  // If still speed of wheels is Null, disconnect PWM for stearing
				PWM0_3_A_disable();
			}
//		WaitForInterrupt();
  }

}


// Main thread scheduled by OS round robin preemptive scheduler
// Task2 Controls Buzzer
// Inputs:  none
// Outputs: none
void Task4(void){

  while(1){
		//OS_Wait(&NewBluetoothBuzzerData); // New Buzzer Volume are ready
		
		
//		WaitForInterrupt();
  }
}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 Send Temp and Humidity sensor data
// Inputs:  none
// Outputs: none
void Task5(void){


		while(1){
			//OS_Wait(&NewBluetoothBuzzerData); // Waint intil sensor data are ready
			TempHumiditySensor_Start();
			OS_Sleep(250);     // waits about 250 msec	
			TempHumiditySensor_End(tempHumidityData);
			SendTempHumidityFlag = 1;
			//OS_Signal(&NewTempHumidityData);  // Signal that Temp&Humidity sensor data are ready 
			OS_Sleep(1000);     // waits about 1 sec for new measurement
		}
	}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 does nothing but never blocks or sleeps
// Inputs:  none
// Outputs: none
void Task6(void){

  while(1){

		// Send notification with index null
		if (ReadMagnetometerData(magnetometerData, magnetometerData2)==0){
			SendMagnetometerFlag = 1;
		}
		OS_Sleep(400);     // waits about 1 sec	
		
	}
	
		
//		WaitForInterrupt();

}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 does nothing but never blocks or sleeps
// Inputs:  none
// Outputs: none
void Task7(void){

  while(1){
//  Modify will be trigered all the time 
//		//WaitForInterrupt();
//		if (((lightIntensity<20)&&(lastLightIntensity > 50))|| ((lightIntensity<20)&&(lastLightIntensity < 20))){
//			BuzzerOn(200);
//			lastLightIntensity = lightIntensity;
//		} else if (((lightIntensity>50) && (lastLightIntensity<20))||((lightIntensity>50) && (lastLightIntensity>50))) {
//			BuzzerOff();
//			lastLightIntensity = lightIntensity;
//		} else {
//			lastLightIntensity = lightIntensity;
//		}
		
  }
}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 does nothing but never blocks or sleeps
// Inputs:  none
// Outputs: none
void Task8(void){

  while(1){
		//int i= 1;
//		WaitForInterrupt();
  }
}

// Main thread scheduled by OS round robin preemptive scheduler
// Task2 does nothing but never blocks or sleeps
// Inputs:  none
// Outputs: none
void Task9(void){

  while(1){
		//int i= 1;
//		WaitForInterrupt();
  }
}

// *********Task0*********
// Periodic event thread runs in real time at 1000 Hz
// Disable Stearing Motor after 3 sec
// Inputs:  none
// Outputs: none
void Task10(void){
  //static int32_t soundSum = 0;
  

}
/* ****************************************** */
/*          End of Task0 Section              */
/* ****************************************** */






//------------LaunchPad__Output------------
// Set new value of Stearing angle from bluetooth client to launchpad
// Input: 
// Output: none
void Bluetooth_Write_Stear_Angle_Value(){  // write angle

		servoupdownarm(AngleWheels, 0); // 0..160
		if (SpeedWheels==0){
			OS_Signal(&AnglePositionChanged);  // Signal that Wheel position was changed
		}
	
	//AngleWheels = CharacteristicList[1].pt[0];  
  //OS_Signal(&NewBluetoothAngleData); // Set Semaphore NewAngleData
  
}


void Bluetooth_Read_Stear_Angle_Value(){
}

////------------LaunchPad__Output------------
//// Set new value of Motor Speed from bluetooth client to launchpad
//// Data from UART are Little Endian
//// Input: 
//// Output: none
//void Bluetooth_Write_Speed_Motor_Value(){  // write angle

////	uint32_t Speed=0;
////	
////	for (int i=0;i<4;i++){
////		Speed= Speed<<8;
////		Speed|= CharacteristicList[5].pt[i];
////	}
////	SpeedWheels = Speed;
//	

//		if (DirectionWheels==0)
//		{
//			DCforward(SpeedWheels , 0);  //between(0..250)  Forward Direction
//			//int a = 1;
//		} 
//		else
//		{
//			 //int a = 0;
//			DCbackward(SpeedWheels , 0);  //between(0..250)  Backward Direction
//		}
//	
//	
////	
////	SpeedWheels = CharacteristicList[2].pt[0];
////  OS_Signal(&NewBluetoothSpeedData);  // Set Semaphore NewSpeedData

//}



//------------LaunchPad__Output------------
// Set new value of Motor Speed from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_Speed_Forward_Motor_Value(){  // write angle
		if (!ControlledSpeed) {
			DCforward(SpeedWheels *6 , 0);  //between(0..250)* 16  for range (0..4000)  Forward Direction
		} else {
			setDesiredSpeed(SpeedWheels, 0);
		}
}




//------------LaunchPad__Output------------
// Set new value of Motor Speed from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_Speed_Backward_Motor_Value(){  // write angle
    if (!ControlledSpeed) {
			DCbackward(SpeedWheels *6 , 0);  //between(0..250)* 16  for range (0..4000)  Backward Direction
		} else {
			setDesiredSpeed(SpeedWheels, 1);
		}

}

//void Bluetooth_Read_Speed_Motor_Value(){
//}


void Bluetooth_Read_Speed_Forward_Motor_Value(){
}

void Bluetooth_Read_Speed_Backward_Motor_Value(){
}


void Bluetooth_Read_Options_Value(){
}

void Bluetooth_Write_Options_Value(){
	ControlledSpeed = selectedOptions&0x1;
}



//------------LaunchPad__Output------------
// Set new value of Motor Direction from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_Wheel_Direction_Value(){
	
	// Change direction only if motor is stopped
//	if (SpeedWheels==0){
//		DirectionWheels = CharacteristicList[0].pt[0];
//	}
}

void Bluetooth_Reed_Wheel_Direction_Value(){
	
}




//------------LaunchPad__Output------------
// Set new value of Buzzer Valume from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_Buzzer_Value(){  // write angle

	//BuzzerVolume = CharacteristicList[3].pt[0];   ///   Is not necessary
	if (BuzzerVolume!=0){
			BuzzerOn(BuzzerVolume);
		} else {
			BuzzerOff();
		}
	
	
  //OS_Signal(&NewBluetoothBuzzerData);  // Set Semaphore NewSpeedData

}


void Bluetooth_Read_Buzzer_Value(){
}

//------------LaunchPad__Output------------
// Set new value of Temp&Humidity from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_TempHumidity_Value(){  // write temp , humidity

	
/// Thread will send periodically the notification if CCCD is activated	. No need

}


//------------LaunchPad__Output------------
// Read new value of Temp&Humidity from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Read_TempHumidity_Value(){  // write temp , humidity

	
/// Thread will send periodically the notification if CCCD is activated	. No need

}
//------------LaunchPad__Output------------
// Set new value of Magnetometer from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_Magnet_Value(){  
	
}
//------------LaunchPad__Output------------

// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Read_SpeedSensor_Value(){
	
	speedPhotoData[2] =  Period >> 16 & 0xFF;
	speedPhotoData[1] =  Period >> 8 & 0xFF;
	speedPhotoData[0] =  Period  & 0xFF;

}

//------------LaunchPad__Output------------
// Set new value of Sensor Speed from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Write_SpeedSensor_Value(){
	
}


//------------LaunchPad__Output------------
// Read new value of Magnetometer from bluetooth client to launchpad
// Data from UART are Little Endian
// Input: 
// Output: none
void Bluetooth_Read_Magnet_Value(){  
	
}

extern uint16_t edXNum; // actual variable within TExaS
void Bluetooth_Init(void){volatile int r;
	
  //EnableInterrupts();
  UART0_OutString("\n\rLab 6 Application Processor\n\r");
  r = AP_Init(); 
  GetStatus();  // optional
  GetVersion(); // optional
  AddService(0xFFE0); 
	
	//AddCharacteristic(0xFFE1,1,&DirectionWheels,0x03,0x0A,"Direction",&Bluetooth_Reed_Wheel_Direction_Value,&Bluetooth_Write_Wheel_Direction_Value);   //  write current direction value, 1 byte
	AddCharacteristic(0xFFE1,1,&AngleWheels,0x03,0x0A,"Angle",&Bluetooth_Read_Stear_Angle_Value,&Bluetooth_Write_Stear_Angle_Value);   //  write current Angle value, 1 byte
	AddCharacteristic(0xFFE2,1,&SpeedWheels,0x03,0x0A,"SpeedFoward",&Bluetooth_Read_Speed_Forward_Motor_Value,&Bluetooth_Write_Speed_Forward_Motor_Value);   //  write current Speed value, 1 bytes
	AddCharacteristic(0xFFE3,1,&SpeedWheels,0x03,0x0A,"SpeedBackward",&Bluetooth_Read_Speed_Backward_Motor_Value,&Bluetooth_Write_Speed_Backward_Motor_Value);   //  write current Speed value, 1 bytes
	AddCharacteristic(0xFFE4,1,&BuzzerVolume,0x03,0x0A,"Buzzer",&Bluetooth_Read_Buzzer_Value,&Bluetooth_Write_Buzzer_Value);   //  write current Speed value, 1 bytes
	
	AddCharacteristic(0xFFE5,6,&tempHumidityData[0],0x03,0x0A,"TempHumidity",&Bluetooth_Read_TempHumidity_Value,&Bluetooth_Write_TempHumidity_Value);   //  write current Speed value, 1 bytes
	AddCharacteristic(0xFFE6,6,&magnetometerData[0],0x03,0x0A,"Magnetometer",&Bluetooth_Read_Magnet_Value,&Bluetooth_Write_Magnet_Value);   //  write current Speed value, 1 bytes
	
	AddCharacteristic(0xFFE7,3,&speedPhotoData[0],0x03,0x0A,"SpeedPhotoSensor",&Bluetooth_Read_SpeedSensor_Value,&Bluetooth_Write_SpeedSensor_Value);   //  write current Speed value, 1 bytes
	AddCharacteristic(0xFFE8,1,&selectedOptions,0x03,0x0A,"SelectedFunctions",&Bluetooth_Read_Options_Value,&Bluetooth_Write_Options_Value);   //  write options, 1 bytes
//	AddNotifyCharacteristic(0xFFE5,6,&tempHumidityData[0],"TempHumidity",&Bluetooth_Write_TempHumidity_Value);
//	AddNotifyCharacteristic(0xFFE6,6,&magnetometerData[0],"Magnetometer",&Bluetooth_Write_Magnet_Value);
	
  RegisterService();
  StartAdvertisement();
  GetStatus();
  //DisableInterrupts(); // optional
}




int main(void){
	PLL_Init();                      // bus clock at 80 MHz
	
	AD_Init();  // initilize AD converter

	GPIO_DC_Init();	//  initialize pins for forward and backward dierction

  OS_Init();
  //Profile_Init();  // initialize the 7 hardware profiling ,   pins can conflict, check
 
  //OS_InitSemaphore(&TakeJoystickData,0);   // 0 means no data
  OS_InitSemaphore(&NewBluetoothSpeedData, 0);  // 0 means no data
	//OS_InitSemaphore(&NewBluetoothAngleData, 0);  // 0 means no data
	OS_InitSemaphore(&AnglePositionChanged, 0);  // 0 means no data
	//OS_InitSemaphore(&NewBluetoothBuzzerData, 0);  // 0 means no data
	OS_InitSemaphore(&NewTempHumidityData, 0);  // 0 means no data
	
	// Task 6 should run every 1000ms
  //OS_AddPeriodicEventThread(&Task6, 1000);  //  Measures acceleration ,   Only for prio 1 Events that should be quickly executed. Not thread!!. It is a Task event.
	
  OS_AddThreads(&Task0,0, &Task1,0, &Task2,0, &Task3,0, &Task4,0, &Task5,0, &Task6,0, &Task7,0, &Task8,0, &Task9,0);

	//BSP_Joystick_Init();
	
	// Input:  Period(Takts), DutyCicle(Takts)
	// initialize PWM0_3_A, 5 Takt in 1 mikros * 2500 mikros , Servo steering  
	//   Port change
	PWM0_3_A_Init((5*PERIOD_SERVO), (5*POSITION_NULL));   // PC4       
	
	
	// Input:  Period(Takts), DutyCicle(Takts)
//	// initialize PWM0_1_A, 20 KHz Period, DC Motor Control, 5 Takt in 1 mikros * 250 mikros = 20kHZ
//	PWM0_1_A_Init(250, 1);       // PB4   
	
	
	// Initialize TIMER 1A for PWM, 20 KHz Period, DC Motor Control, 80 Takt in 1 mikros *  20 microsec (period 50 khz)  = 1600
	TIMER_1_A_PWM_Init(PERIOD_DC, SPEED_NULL);
	
	// Input:  Period(Takts), DutyCicle(Takts)
	// initialize PWM0_0A, 3 KHz Period, Buzzer , 50% volume
	PWM0_0_B_Init(PERIOD_BUZZER, VOLUME_50);
	
	
	UART0_Init();     //  will affect PWM0_0_A
	Bluetooth_Init();
	
	Timer3_Init(4000000); // initialize timer3 (20 Hz, 50ms)
	PeriodMeasure_Init();  // Initialize Edge triggered capture mode interrupt
	EnableInterrupts();
	
	I2C3_Init(); // Init I2C Module3 for Temp Humidity Sensor
	I2C1_Init(); // Init I2C Module1 for Gyroskop Accel magnetometer Sensor
	
	if (Init_MPU9250() == 1){   //Initialize Magnetometer 
		// Notify user:  for example with Led:    Initialization of magnetometer couldn't be done
	}
	
	PortF_2_init();  // Initialize port F2   , Led on board 
	PortF_1_init();  // Initialize port F2   , Led on board 
	
	//OS_PeriodTrigger0_Init(&TakeJoystickData,5000);  // every 5000 ms ,  Set flag with some period for one thread. This is explicitely for one thread, period time can be less then 1HZ.  For example 5 times pro second
	//OS_PeriodTrigger0_Init(&TakeJoystickData,1);  // every 1 ms ,  Set flag with some period. This is explicitely for one thread


 //TExaS_Init(LOGICANALYZER, 1000); // initialize the Lab 4 logic analyzer
  OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


