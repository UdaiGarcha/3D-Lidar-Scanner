/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <stdbool.h>




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define TOF_PIN 								0b00000010  // Sensor pin
#define MOTOR_PIN 							0b00000001  // Motor pin

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortM0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;  //0b00000000    								      // Make PM0 input
  GPIO_PORTM_DEN_R = 0b00001111;  //0b00000001
	return;
}

void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;  //0b00000000    								      
  GPIO_PORTH_DEN_R = 0b00001111;  //0b00000011
	return;
}

void send_bool(bool value) {
    if (value == true) {
        UART_printf("T\n");
    }else {
        UART_printf("F\n");
		}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM0_Init();
	PortH0H1H2H3_Init();
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

status = VL53L1X_StartRanging(dev);	// This function has to be called to enable the ranging
//----------------------//
int ToF_Start = false;
int Motor_Start = false;
//----------------------//
int ToF_capture;
int Number_of_Step = 1;
int Number_of_Count;
int Motor_Direction;
//----------------------//
uint8_t Motor_Button = false;
uint8_t Data_Button = false;
//----------------------//

while(1){
	    if ((GPIO_PORTM_DATA_R & TOF_PIN) == 0) {
	        if (!Data_Button) {
	            ToF_Start = !ToF_Start; // turns false into true so line 218 can be true and print "T" into python
	            FlashLED2(1);
	            Data_Button = true;
						
	            if (ToF_Start) {
	                send_bool(true); // sends a print message to UART "T" so python can later read the message
	            } else {
	                send_bool(false);
	            }
	        }
	    } else {
	        Data_Button = false;
	    }
    
    if ((GPIO_PORTM_DATA_R & MOTOR_PIN) == 0) {
        if (!Motor_Button) {
            Motor_Start = !Motor_Start;
            while ((GPIO_PORTM_DATA_R & MOTOR_PIN) == 0) { SysTick_Wait1us(10); }
            Motor_Button = true;
        }
    } else {
        Motor_Button = false;
    }

    if(Motor_Start){	//rotate
        if (Motor_Direction == 1){
					for(int i=0; i<Number_of_Step; i++){
						GPIO_PORTH_DATA_R = 0b00001100;
						SysTick_Wait1us(6000);
						GPIO_PORTH_DATA_R = 0b00000110;
						SysTick_Wait1us(6000);
						GPIO_PORTH_DATA_R = 0b00000011;
						SysTick_Wait1us(6000);
						GPIO_PORTH_DATA_R = 0b00001001;
						SysTick_Wait1us(6000);
						Number_of_Count = Number_of_Count + 1; // could be count += 1; (helps me visualize whats going on in the code)
			}
      } else {
        for(int i=0; i<Number_of_Step; i++){
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait1us(6000);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait1us(6000);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait1us(6000);
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait1us(6000);
					Number_of_Count = Number_of_Count + 1;
		}
        }
    }

    if (ToF_capture) {
        send_bool(true);

        if (!ToF_Start) {
            send_bool(false);
        }

		
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED1(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = false;
	  
		//read the data values from ToF sensor
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

		FlashLED2(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
		
		ToF_capture = false;
		
  }
	
    if(ToF_Start && Number_of_Count % 16 == 0 && Number_of_Count != 0){	// 360/45 = 8, 512/8 = 64
        ToF_capture = true;
    }

    if(Number_of_Count >= 512){ //512 steps is a full rotation, deemed to be 512 in studio 4B
				//UART_printf("Walk Forward\n");
				SysTick_Wait10ms(325);
        Number_of_Count = 0;		//reset count so we can operate again
        Motor_Direction = !Motor_Direction;
    }
 }
	
  VL53L1X_StopRanging(dev);
  while(1) {}
}