// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0

// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)

// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
//#include "TExaSscope.h"

// ***** 2. Global Declarations Section *****

//ADDRESS
#define SENSOR			(*((volatile unsigned long *)0x4002401C))		//PORTE, PE2-PE0 (10+8+4 = 1C)
#define CARLIGHT		(*((volatile unsigned long *)0x400050FC))		//PORTB, PB5-PB0 (80+40+20+10+8+4=FC)
#define PEDLIGHT		(*((volatile unsigned long *)0x40025028))		//PORTF, PF3 AND PF1
	
//SYSTICK REGISTER (Timer)
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
	
// Linked data structure
struct State {
  unsigned long OutCarLights;  						// 6-bit pattern to output
	unsigned long OutPederastianLights;  		// 2-bit pattern to output
  unsigned long Time; 		// delay
  unsigned long Next[8];	// next state for inputs 0-7
};

typedef const struct State STyp;
#define goWest					0
#define waitWest				1
#define goSouth					2
#define waitSouth				3
#define waitWSwalk			4
#define waitWSdtwalk1		5
#define waitWSoffwalk1	6
#define waitWSdtwalk2		7
#define waitWSoffwalk2	8

#define longdelay				2000
#define shortdelay			500

STyp FSM[9]={
 {0x0C, 0x02, longdelay,	{goWest,goWest,waitWest,waitWest,waitWest,waitWest,waitWest,waitWest}}, //0
 {0x14, 0x02, shortdelay,	{goSouth,goSouth,goSouth,goSouth,waitWSwalk,waitWSwalk,goSouth,goSouth}},	//1
 {0x21, 0x02, longdelay,	{goSouth,waitSouth,goSouth,waitSouth,waitSouth,waitSouth,waitSouth,waitSouth}},	//2
 {0x22, 0x02, shortdelay,	{goWest,goWest,goWest,goWest,waitWSwalk,waitWSwalk,waitWSwalk,waitWSwalk}}, //3
 
 {0x24, 0x08, longdelay,	{waitWSwalk,waitWSdtwalk1,waitWSdtwalk1,waitWSdtwalk1,waitWSwalk,waitWSdtwalk1,waitWSdtwalk1,waitWSdtwalk1}}, //4
 {0x24, 0x02, shortdelay,	{waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1,waitWSoffwalk1}}, //5
 {0x24, 0x00, shortdelay,	{waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2,waitWSdtwalk2}}, //6
 {0x24, 0x02, shortdelay,	{waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2,waitWSoffwalk2}}, //7
 {0x24, 0x00, shortdelay,	{goWest,goWest,goSouth,goWest,	waitWSwalk,goWest,goSouth,goWest}}	//8
};
unsigned long S;  // index to the current state 
unsigned long Input; 

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

//void PLL_Init(void);

void SysTick_Init(void);	// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay);
void SysTick_Wait1ms(unsigned long delay);

void PortInit(void);

// ***** 3. Main Section *****
int main(void)
{ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
  //TExaS_Scope();
	
	//PLL_Init();
  SysTick_Init();   
	PortInit();
  
  EnableInterrupts();
	
	S = goWest;  
  while(1)
	{
    CARLIGHT = FSM[S].OutCarLights;  					// set car lights
		PEDLIGHT = FSM[S].OutPederastianLights;  	// set pederastian lights
    SysTick_Wait1ms(FSM[S].Time);							// delay time
		
    Input = SENSOR;     											// read sensors
    S = FSM[S].Next[Input];  									// change to the next state
  }
}

void PortInit(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000032;     // 1) activate clock for Port B,E, & F
	delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  
  // Port E
  if (GPIO_PORTE_LOCK_R != 0)       // 3) unlock GPIO Port E if locked
    GPIO_PORTE_LOCK_R = 0x4C4F434B; 
  GPIO_PORTE_CR_R = 0x07;           // 4) allow changes to PE0-PE2
  GPIO_PORTE_AMSEL_R = 0x00;        // 5) disable analog on PE
  GPIO_PORTE_AFSEL_R = 0x00;        // 6) disable alt funct on PE
  GPIO_PORTE_PCTL_R = 0x00000000;   // 7) PCTL GPIO on PE
  GPIO_PORTE_PUR_R = 0x00;          // 8) disable pullups on PE
  GPIO_PORTE_DIR_R = 0x00;          // 9) PE0-PE7 inputs
  GPIO_PORTE_DEN_R = 0x07;          // 10) enable digital I/O on PE0-PE2
  
  // Port B
  if (GPIO_PORTB_LOCK_R != 0)       // 3) unlock GPIO Port B if locked
    GPIO_PORTB_LOCK_R = 0x4C4F434B; 
  GPIO_PORTB_CR_R = 0x3F;           // 4) allow changes to PB0-PB5
  GPIO_PORTB_AMSEL_R = 0x00;        // 5) disable analog on PB
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) disable alt funct on PB
  GPIO_PORTB_PCTL_R = 0x00000000;   // 7) PCTL GPIO on PB
  GPIO_PORTB_PUR_R = 0x00;          // 8) disable pullups on PB
  GPIO_PORTB_DIR_R = 0x3F;          // 9) PB0-PB5 outputs
  GPIO_PORTB_DEN_R = 0x3F;          // 10) enable digital I/O on PB0-PB5
  
  // Port F
  if (GPIO_PORTF_LOCK_R != 0)       // 3) unlock GPIO Port F if locked
    GPIO_PORTF_LOCK_R = 0x4C4F434B; 
  GPIO_PORTF_CR_R = 0x0A;           // 4) allow changes to PF1 & PF3
  GPIO_PORTF_AMSEL_R = 0x00;        // 5) disable analog on PF
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 7) PCTL GPIO on PF
  GPIO_PORTF_PUR_R = 0x00;          // 8) disable pullups on PF
  GPIO_PORTF_DIR_R = 0x0A;          // 9) PF1 & PF3 outputs
  GPIO_PORTF_DEN_R = 0x0A;          // 10) enable digital I/O on PF1 & PF3
}

/*
void PLL_Init(void)
{
  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6
                 + 0x00000540;   // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
}
*/

void SysTick_Init(void)
{
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay)
{
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait1ms(unsigned long delay)
{
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(80000);  // wait 1ms
  }
}
