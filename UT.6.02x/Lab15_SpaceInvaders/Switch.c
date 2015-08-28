#include "Switch.h"
#include "tm4c123gh6pm.h"

void PortE_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;     // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	/*
  GPIO_PORTE_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port E
  GPIO_PORTE_CR_R = 0x1F;           // allow changes to PE4-0
  // only PF0 needs to be unlocked, other bits can't be locked
	*/
  GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTE_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
	
  GPIO_PORTE_DIR_R = 0x00;          // 5) PE0 INPUT (=0), PE1 INPUT (=0) // OUTPUT (=1)
	
  GPIO_PORTE_AFSEL_R = 0x00;        // 6) disable alt funct on PE7-0
	
	GPIO_PORTE_DEN_R = 0x1F;          // 7) enable digital I/O on PE4-0
	
  GPIO_PORTE_PUR_R = 0x00;          // enable pull-up on PE1 and PE0
 
}

