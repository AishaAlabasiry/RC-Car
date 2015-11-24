#include "tm4c123gh6pm.h"

#define FLIGHT_ON 0x04
#define BLIGHT_ON 0x08
#define HORN_ON 0x10
#define FORWARD 0x08
#define BACKWARD 0x04
#define RIGHT 0x10
#define LEFT 0x20
#define BLUE 0x02

//void PWM0Dual_Init(unsigned long period, unsigned short dcpwm0,  unsigned short dcpwm1);

void UART_init(void);

unsigned char UART_receiveChar(void);

void PortA_init(void);

void PortB_init(void);

void PortF_init(void);

int main(void)
{
	char motion=0;
	UART_init();
	PortA_init();
	PortB_init();
	PortF_init();
	
	/* Blue underbody kit */
	GPIO_PORTF_DATA_R |= BLUE;
	
	while(1)
	{
		motion=UART_receiveChar();
		switch(motion)
		{
			/* Forward */
			case 'F':
				GPIO_PORTB_DATA_R |= FORWARD;
				break;
			
			/* Backward */
			case 'B':
				GPIO_PORTB_DATA_R |= BACKWARD;
				break;
			
			/* Left */
			case 'L':
				GPIO_PORTB_DATA_R |= LEFT;
				break;
			
			/* Right */
			case 'R':
				GPIO_PORTB_DATA_R |= RIGHT;
				break;
			
			/* Forward Left*/
			case 'G':
				GPIO_PORTB_DATA_R |= (FORWARD | LEFT);
				break;
			
			/* Forward Right */
			case 'I':
				GPIO_PORTB_DATA_R |= (FORWARD | RIGHT);
				break;
			
			/* Backward Left */
			case 'H':
				GPIO_PORTB_DATA_R |= (BACKWARD | LEFT);
				break;
			
			/* Backward Right */
			case 'J':
				GPIO_PORTB_DATA_R |= (BACKWARD | RIGHT);
				break;
			
			/* Stop */
			case 'S':
				GPIO_PORTB_DATA_R &= ~(FORWARD | BACKWARD | LEFT | RIGHT);
				break;
			
			/* Front Lights ON */
			case 'W':
				GPIO_PORTA_DATA_R |= FLIGHT_ON;
				break;
			
			/* Front Lights OFF */
			case 'w':
				GPIO_PORTA_DATA_R &= ~(FLIGHT_ON);
				break;
			
			/* Back Lights ON */
			case 'U':
				GPIO_PORTA_DATA_R |= BLIGHT_ON;
				break;
			
			/* Back Lights OFF */
			case 'u':
				GPIO_PORTA_DATA_R &= ~(BLIGHT_ON);
				break;
			
			/* Horn ON */
			case 'V':
				GPIO_PORTA_DATA_R |= HORN_ON;
				break;
			
			/* Horn OFF */
			case 'v':
				GPIO_PORTA_DATA_R &= ~(HORN_ON);
				break;
		}
	}
}

/* UART initialization */
void UART_init(void)
{
	SYSCTL_RCGCUART_R |= SYSCTL_RCGC1_UART1;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;
	UART1_CTL_R &= ~UART_CTL_UARTEN;
	/* 50,000,000/(16*9600) */
	UART1_IBRD_R = 325;
	UART1_FBRD_R = 34;
	UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
	UART1_CC_R = 0x0;
	UART1_CTL_R |= UART_CTL_UARTEN;
	GPIO_PORTC_AFSEL_R |= 0x30;
	GPIO_PORTC_DEN_R |= 0x30;
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0xFF00FFFF) + 0x00220000;
	GPIO_PORTC_AMSEL_R &= ~(0x30);
}

/* UART Receive */
unsigned char UART_receiveChar(void)
{
	while((UART1_FR_R & UART_FR_RXFE) != 0);
	return ((unsigned char)(UART1_DR_R & 0xFF));
}

/* PortA initialization */
void PortA_init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;  // 1) activate clock for Port A
	__asm 								           // allow time for clock to start waiting 3 bus cycles 
	{
		NOP
		NOP
		NOP
	}
	GPIO_PORTA_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port A
	GPIO_PORTA_CR_R |= 0x1C;           // allow changes to PA4-2
	GPIO_PORTA_AMSEL_R &= 0xE3;        // 3) disable analogue on PA
	GPIO_PORTA_PCTL_R &= 0xFFF000FF;  // 4) PCTL GPIO on PA4-2
	GPIO_PORTA_DIR_R |= 0x1C;          // 5) PA4-2 out
	GPIO_PORTA_AFSEL_R &= 0xE3;        // 6) disable alternative function on PA4-2
	GPIO_PORTA_PUR_R &= 0xE3;          // disable pull-up on PA4-2
	GPIO_PORTA_DEN_R |= 0x1C;          // 7) enable digital I/O on PA4-2
	GPIO_PORTA_DATA_R &= 0xE3;
}

/* PortB initialization */
void PortB_init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;  // 1) activate clock for Port B
	__asm 								           // allow time for clock to start waiting 3 bus cycles 
	{
		NOP
		NOP
		NOP
	}
	GPIO_PORTB_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port B
	GPIO_PORTB_CR_R |= 0x3C;           // allow changes to PB5-2
	GPIO_PORTB_AMSEL_R &= 0xC3;        // 3) disable analogue on PB
	GPIO_PORTB_PCTL_R &= 0xFF0000FF;  // 4) PCTL GPIO on PB5-2
	GPIO_PORTB_DIR_R |= 0x3C;          // 5) PB5-2 out
	GPIO_PORTB_AFSEL_R &= 0xC3;        // 6) disable alternative function on PB5-2
	GPIO_PORTB_PUR_R &= 0xC3;          // disable pull-up on PB5-2
	GPIO_PORTB_DEN_R |= 0x3C;          // 7) enable digital I/O on PB5-2
	GPIO_PORTB_DATA_R &= 0xC3;
}

/* PortF initialization */
void PortF_init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;  // 1) activate clock for Port F
	__asm 								           // allow time for clock to start waiting 3 bus cycles 
	{
		NOP
		NOP
		NOP
	}
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
	GPIO_PORTF_CR_R |= 0x0E;           // allow changes to PF3-1
	GPIO_PORTF_AMSEL_R &= 0xF1;        // 3) disable analogue on PF
	GPIO_PORTF_PCTL_R &= 0xFFFF000F;  // 4) PCTL GPIO on PF3-1
	GPIO_PORTF_DIR_R |= 0x0E;          // 5) PF3-1 out
	GPIO_PORTF_AFSEL_R &= 0xF1;        // 6) disable alternative function on PF3-1
	GPIO_PORTF_PUR_R &= 0xF1;          // disable pull-up on PF3-1
	GPIO_PORTF_DEN_R |= 0x0E;          // 7) enable digital I/O on PF3-1
	GPIO_PORTF_DATA_R &= 0xF1;
}
