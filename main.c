// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <stdlib.h>
#include<ctype.h>
#include<math.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
char str[20];
uint8_t startfield[2];
uint32_t lookuptable[4096],DelPhi,Accumulator=0;
float voltage, frequency;
// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
	while(PUSH_BUTTON);
}

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
      	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
       GPIO_PORTB_DIR_R |= 0xb0;                        // make bits 4 and 7 outputs
       GPIO_PORTB_DR2R_R |= 0xb0;                       // set drive strength to 2mA
       GPIO_PORTB_AFSEL_R |= 0xb0;                      // select alternative functions for MOSI, SCLK pins
       GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK|GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
       GPIO_PORTB_DEN_R |= 0xb0;                        // enable digital operation on TX, CLK pins
       GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

       // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
       SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
       SSI2_CR1_R = 0;                                  // select master mode
       SSI2_CC_R = 0;                                   // select system clock as the clock source
       SSI2_CPSR_R = 20;                                // set bit rate to 1 MHz (if SR=0 in CR0)
       SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
       SSI2_CR1_R |= SSI_CR1_SSE;

}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
void step2()
{
	uint8_t i;
	for(i=0;i<20;i++)
	{
	str[i] = getcUart0();
	if (str[i]==13)
		break;
	if (str[i]==8 && i!=0)
		i=i-2;
	}
	//putsUart0("\n\r");
	//putsUart0(str);
}
void step3()
{
	uint8_t i,j;
	for(i=0;i<20;i++)
	{
		if(str[i]>=65 && str[i]<=90)
			str[i] = str[i]+32;
		if(!((str[i]>=97 && str[i]<=122) || (str[i]>='0' && str[i]<='9') || str[i]=='.'|| str[i]=='-' || str[i] == ' '))
		{
			for(j=i;j<20;j++)
			{

				str[j]=str[j+1];
				i=i-2;
			}
		}
		//putsUart0("\n\r");
		//putsUart0(str);
	}
}
void step4()
{
	uint8_t i,j=0;
	for(i=0;i<20;i++)
	{
		if((str[i]>='0' && str[i]<='9') || str[i]=='-' || str[i]=='.')
		{
			startfield[j]=i;
			while((str[i]>='0' && str[i]<='9') || str[i]=='-' || str[i]=='.')
				i++;
			j++;
		}

	}

}
void step6()
{
	voltage=atof(&str[startfield[0]]);
	frequency=0;
	if(voltage>0)
		SSI2_DR_R=0X3000+round(2063-(1909*voltage/5));
	else
		SSI2_DR_R=0X3000+round(2063-(1924*voltage/5));
}
void step7()
{
	uint32_t i;
	voltage=atof(&str[startfield[1]]);
	frequency=atof(&str[startfield[0]]);
	for(i=0;i<4096;i++)
	{
		if(sin(i)>0)
			lookuptable[i]=0X3000+2063-(1909*voltage/5)*sin(2*3.14*i/4096);
		else
			lookuptable[i]=0X3000+2063-(1924*voltage/5)*sin(2*3.14*i/4096);
	}
	i=0;
	DelPhi=frequency*pow(2,32)/100000;

}
void timer1Isr(void)
{
		Accumulator=Accumulator+DelPhi;
		SSI2_DR_R=lookuptable[Accumulator>>20];
		TIMER1_ICR_R =TIMER_ICR_TATOCINT;
}
void step8()
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           		// configure as 16-bit timer (A)
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
	TIMER1_TAILR_R =0x190;
	TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
	TIMER1_CTL_R |= TIMER_CTL_TAEN;

}
int main(void)
{
	// Initialize hardware
	initHw();

while(1)
{
	putsUart0("\n\r");
	step2();
	step3();
	step4();

	if(strstr(str,"dc"))
	{
		step6();
		if(voltage<-5 || voltage > 5)
		{
			putsUart0("Error, Enter again!");
		}
		if(strstr(str,"sine"))
	{
		step7();
		if(voltage <-5 || voltage > 5 || frequency > 100000 || frequency < 0 )
				{
					putsUart0("Error, Enter again!");
				}
		step8();
	}
	putsUart0("\n\r");
	putsUart0(str);
}
}
