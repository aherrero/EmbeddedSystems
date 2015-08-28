#define PE0 (*((volatile unsigned long *)0x40024004))	
#define PE1 (*((volatile unsigned long *)0x40024008))	

void PortE_Init(void);

