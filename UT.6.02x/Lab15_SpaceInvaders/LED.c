#include "LED.h"
#include "tm4c123gh6pm.h"

void PortB_Init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;     // 1) activate clock for Port B
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start

  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
	
  GPIO_PORTB_DIR_R = 0x30;          // 5) PB4 AND PB5 OUTPUT (=1)	// INPUT (=0)
	
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) disable alt funct on PE7-0
	
	GPIO_PORTB_DEN_R = 0x30;          // 7) enable digital I/O on PB4,PB5
	
  GPIO_PORTB_PUR_R = 0x00;          // enable pull-up on PE1 and PE0
}
