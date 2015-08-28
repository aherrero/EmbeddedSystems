// SpaceInvaders.c
// Runs on LM4F120/TM4C123
// Alejandro Herrero
// This is a basic project of a game based in the starter project for the edX Lab 15

// July 16, 2015

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

   "Embedded Systems: Introduction to Arm Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
 
// ******* Required Hardware I/O connections*******************
// Slide pot pin 1 connected to ground
// Slide pot pin 2 connected to PE2/AIN1
// Slide pot pin 3 connected to +3.3V 
// fire button connected to PE0
// special weapon fire button connected to PE1
// 8*R resistor DAC bit 0 on PB0 (least significant bit)
// 4*R resistor DAC bit 1 on PB1
// 2*R resistor DAC bit 2 on PB2
// 1*R resistor DAC bit 3 on PB3 (most significant bit)
// LED on PB4
// LED on PB5

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// SSI0Fss       (SCE, pin 3) connected to PA3
// Reset         (RST, pin 4) connected to PA7
// Data/Command  (D/C, pin 5) connected to PA6
// SSI0Tx        (DN,  pin 6) connected to PA5
// SSI0Clk       (SCLK, pin 7) connected to PA2
// back light    (LED, pin 8) not connected, consists of 4 white LEDs which draw ~80mA total

#include "..//tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "Random.h"
#include "TExaS.h"

#include "images.h"
#include "ADC.h"
#include "Switch.h"
#include "LED.h"

//States
#define START 0
#define GAME 1
#define END 2
int STATE;
#define yERRORPIXEL 3
#define xERRORPIXEL 4

//Functions
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Timer2_Init(unsigned long period);
void Delay100ms(unsigned long count); // time delay in 0.1 seconds
void currentGame(void);
void SysTick_Handler(void);
void SysTick_Init(unsigned long period);
void Draw(void);
void MovePlayer(void);
void MoveEnemy(void);
void FirePlayer(void);
void InitWorld(void);
unsigned long Convert(unsigned long sample);
unsigned long Random3(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

//VARIABLES software
unsigned long TimerCount;
unsigned long Semaphore;
unsigned long FrameCount = 0;
//Players types
struct Object
{
	unsigned long x;
	unsigned long y;
	const unsigned char *img[2]; // two pointers to images
	unsigned long life;
	unsigned long width;
	unsigned long height;
};
typedef struct Object Player;
typedef struct Object Monster;
typedef struct Object Test;
typedef struct Object Shoot;

//Globals players
Player player;
Monster enemy[3];
Test test;
Shoot shoot;

//VARIABLES hardware
unsigned long Flag;       // 0 - non interruption, 1 - interruption
//Slider
unsigned long sliderDataPixels;   // 0 to size height (47px)
unsigned long ADCdata;    // 12-bit 0 to 4095 sample
//Switches
unsigned long buttonL;
unsigned long buttonR;


int main(void)
{
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
  Random_Init(1);
  Nokia5110_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer
	
	ADC0_Init(); // initialize ADC0, sequencer 3, software start 
	PortE_Init();
	PortB_Init();
	SysTick_Init(2000000);        // initialize SysTick timer, every 1ms
	
	InitWorld();
  Draw();
	
	EnableInterrupts(); 
	
  while(1){
		//Wait Interrupt
		Flag = 0;
		
		//Game
		currentGame();
		
		//Draw
		Nokia5110_ClearBuffer();
		Nokia5110_DisplayBuffer();      // draw buffer
    Draw();
		
		//Nokia5110_SetCursor(1, 1);
		//Nokia5110_OutUDec(shoot.x);
    Delay100ms(2);
			
  }


}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long Random3(void){
  return ((Random()>>24)%5)+1;  // returns 1, 2, 3
}

unsigned long Convert(unsigned long sample){	
	//Lineal
  return map(sample, 0, 4095, 12, 47);		//height player = 13px, max height screen 47px
}

void InitWorld(void)
{
	int i;
	
	STATE = 0;
	buttonL = 0;
	buttonR = 0;
	
	player.x = 0;
	player.y = 47;
	player.img[0] = SpaceshipSmall;
	player.img[1] = SpaceshipSmall;
	player.life = 1;
	player.width = 12;
	player.height = 10;
	
	shoot.x = 4;
	shoot.y = player.y - 4;	//player height
	shoot.img[0] = shootimage;
	shoot.img[1] = shootimage;
	shoot.life = 0;
	
	
	for(i=0;i<3;i++)
	{
    enemy[i].x = 50 + 8*i;
    enemy[i].y = 10 + 16*i;
    enemy[i].life = 1;
		
		enemy[i].width = 10;
	  enemy[i].height = 8;
   }
	
	 enemy[0].img[0] = SmallEnemy10PointA;
	 enemy[0].img[1] = SmallEnemy10PointB;

	 
	 enemy[1].img[0] = SmallEnemy20PointA;
	 enemy[1].img[1] = SmallEnemy20PointB;
	 
	 enemy[2].img[0] = SmallEnemy30PointA;
	 enemy[2].img[1] = SmallEnemy30PointB;
	 
	 //PB4 = 0x10;
	 //PB5 = 0x20;
	 PB4 = 0x00;
	 PB5 = 0x00;
}

							

void MovePlayer(void)
{
	player.y = sliderDataPixels;
}

void MoveEnemy(void)
{
	int i;
	for(i = 0;i<3;i++)
		enemy[i].x = 	enemy[i].x - 1;
}

void FirePlayer(void)
{
	shoot.life = 1;
	shoot.x = 4;
	shoot.y = player.y - 4;	// player height;
	buttonR = 0;
	PB4 = 0x10;
}


void Draw(void){
	int i;
	
	
	Nokia5110_ClearBuffer();
	
	if (STATE == GAME)
	{
		if(player.life == 1)
			Nokia5110_PrintBMP(player.x, player.y, player.img[0], 0);
		
		if(shoot.life == 1)
			Nokia5110_PrintBMP(shoot.x, shoot.y, shoot.img[0], 1);
		
		for(i=0;i<3;i++){
			if(enemy[i].life > 0){
			 Nokia5110_PrintBMP(enemy[i].x, enemy[i].y, enemy[i].img[FrameCount], 0);
			}
		}
		
		FrameCount = (FrameCount+1)&0x01; // 0,1,0,1,...
		
		Nokia5110_DisplayBuffer();      // draw buffer

	}
	
	if (STATE == END)
	{
		Nokia5110_Clear();
		Nokia5110_SetCursor(1, 1);
		Nokia5110_OutString("GAME OVER");
		Nokia5110_SetCursor(1, 2);
		Nokia5110_OutString("Nice try,");
		Nokia5110_SetCursor(1, 3);
		Nokia5110_OutString("Earthling!");
		Nokia5110_SetCursor(2, 4);
		//Nokia5110_OutUDec(1234);
	}
	
}

// Initialize SysTick interrupts to trigger at 40 Hz, 25 ms
void SysTick_Init(unsigned long period){
	// priority 2
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000;           
  NVIC_ST_CTRL_R = 0x07; // enable SysTick with core clock and interrupts
  // finish all initializations and then enable interrupts

}

	
// executes every 25 ms, collects a sample, converts and stores in mailbox
void SysTick_Handler(void){ 
	//Switches	
	if(PE0 == 0x01)
	{
		Flag = 1; 
		buttonR = 1;
	}
	
	if(PE1 == 0x02)
	{
		Flag = 1; 
		buttonL = 1;
	}
		
	//Slider
  ADCdata = ADC0_In();
  sliderDataPixels = Convert(ADCdata);
  if (sliderDataPixels != 0) 
	{
    Flag = 1;                //  Set Flag =1 when the data on  "Distance" variable is ready.
	}		
}



void currentGame(void)
{
	int i;

	
	switch (STATE)
	{
		//START
		case START:
			STATE = GAME;
		break;
		
		//main game
		case GAME:
			MoveEnemy();
			MovePlayer();
		
			//led green off
		  PB4 = 0x00;
			
			if(buttonR)
			{
				FirePlayer();
			}
			if(shoot.life == 1)
			{
				shoot.x = shoot.x + 5;
				if(shoot.x > 80)
					shoot.life = 0;
			}
			
			//colisions ENEMY - SHOOT
			for(i = 0; i<3; i++)
			{
				if (enemy[i].x - (shoot.x + shoot.width)< xERRORPIXEL)
				{
					if(enemy[i].y - enemy[i].height/2 - shoot.y < yERRORPIXEL || shoot.y - (enemy[i].y - enemy[i].height/2)< yERRORPIXEL)
					{
						//touch!
						enemy[i].life = 0;
						shoot.life = 0;
					}
				}
			}
			
			//colisions ENEMY - PLAYER
			for(i = 0; i<3; i++)
			{
				if(player.life && enemy[i].life)
				{
					//x
					if(enemy[i].x - (player.x + player.width) < xERRORPIXEL)
					{
						//y
						if(player.y - enemy[i].y < yERRORPIXEL || enemy[i].y - player.y < yERRORPIXEL)
						{
							//touch!
							player.life = 0;
							STATE = END;
						}						


					}//end monster array
				}//enf player life
			}//endfor
			
			
		break;
		
		//GAME OVER
		case END:
			i = 0;
			//red led on
			PB5 = 0x20;
		break;
		
	}
	
	
	
	//end
	
	
	//n = Random()%3;
}


// You can use this timer only if you learn how it works
void Timer2_Init(unsigned long period){ 
  unsigned long volatile delay;
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  delay = SYSCTL_RCGCTIMER_R;
  TimerCount = 0;
  Semaphore = 0;
  TIMER2_CTL_R = 0x00000000;    // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER2_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAILR_R = period-1;    // 4) reload value
  TIMER2_TAPR_R = 0;            // 5) bus clock resolution
  TIMER2_ICR_R = 0x00000001;    // 6) clear timer2A timeout flag
  TIMER2_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 39, interrupt number 23
  NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
  TIMER2_CTL_R = 0x00000001;    // 10) enable timer2A
}
void Timer2A_Handler(void){ 
  TIMER2_ICR_R = 0x00000001;   // acknowledge timer2A timeout
  TimerCount++;
  Semaphore = 1; // trigger
}
void Delay100ms(unsigned long count){unsigned long volatile time;
  while(count>0){
    time = 727240;  // 0.1sec at 80 MHz
    while(time){
	  	time--;
    }
    count--;
  }
}
