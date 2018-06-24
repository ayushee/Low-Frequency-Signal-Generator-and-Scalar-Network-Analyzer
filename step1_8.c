#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include<stdlib.h>
#include<stdio.h>
#include<math.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#define CS_NOT  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define pi 3.14
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
bool freqUpdate = false;
bool timeUpdate = false;

uint32_t deltaPhi,acc=0;
float vtg,freq;
char str[30];
uint32_t s_table[4096];
uint8_t fieldStart[3],fieldLen[3],temp;

void initHw()
{
	//Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
	//Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A B and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOE |SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOD;

    GPIO_PORTF_DIR_R = 0x0E;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
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

    GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
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
void waitMicrosecond(uint32_t us)
{
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
void LED()
{
	GREEN_LED=1;
	waitMicrosecond(500000);
	GREEN_LED=0;
	waitMicrosecond(500000);
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

uint8_t isbks(char s)
{
	if(s==8)
		return 1;
	else
		return 0;
}
uint8_t isenter(char s)
{
	if(s==13)
		return 1;
	else
		return 0;
}
char iscapital(char s)
{
	if(s>=65 && s<=90)
		s=s+32;
	return s;
}
uint8_t isdelim(char s)
{
	if((s>=48 && s<=57)||s==46||(s>=97 && s<=122)||s==32||s==45)
		return 0;
	else return 1;
}
uint8_t isvalid(char s)
{
	if(s>=32)
		return 1;
	else
		return 0;
}

void getsUart0()
{
	uint8_t i,j,count=0;
	for(i=0;i<30;i++)
	{
		str[i]=NULL;
	}
	str[count]=getcUart0();
		while(!isenter(str[count]))
		{
			if(count<30)
			{
			    if((isbks(str[count]) && count!=0))
			    {
			    				count--;
			    				str[count]=NULL;
			    	  			putsUart0("\r");
			    	  			putsUart0(str);

			    }
			    	else if(isvalid(str[count]))
			    	{
			    		putcUart0(str[count]);
			    		count++;
			    	}
			       	str[count]=getcUart0();
			  }
			else
				break;
		}
		for(j=count;j<=30;j++)
		str[j]=NULL;
		for(i=0;(i<30 && str[i]!=NULL);i++)
		{
		   	str[i]=iscapital(str[i]);
		   	if(isdelim(str[i]))
		   	{
		   		for(j=i;j<30;j++)
		      		{
		      			str[j]=str[j+1];
		      		}
		      		str[j]=NULL;
		      		i--;
		     	}
		    }

}
void get_wave_freq_vtg()
{
	uint8_t i,temp=0;
	for(i=0;i<strlen(str);i++)
	{
		if(str[i]>='a' && str[i]<='z')
		{
			fieldStart[temp]=i;
			while((str[i]>='a' && str[i]<='z')>0)
					i++;
			temp++;
			}
		if( (str[i] >= '0' && str[i] <= '9')||str[i]=='.'||str[i]=='-')
		{
			fieldStart[temp]=i;
			while((str[i] >='0'  && str[i] <= '9')||str[i]=='.'||str[i]=='-')
						i++;
			temp++;

		}
	}
		freq=atof(&str[fieldStart[1]]);
		vtg=atof(&str[fieldStart[2]]);


}
void dc()
{
	int32_t temp;
	vtg=freq;
	if(vtg<0)
    	{
			temp=2069-(1907*(vtg)/5);
			SSI2_DR_R=0x3000+temp;
     		while (SSI2_SR_R & SSI_SR_BSY);

    	}
    	else
    	{
    		temp=2070-(1911*(vtg)/5);
    		SSI2_DR_R=0x3000+temp;
    		while (SSI2_SR_R & SSI_SR_BSY);
    	}
}
void table_sine(void)
{
	uint32_t i;
		for(i=0;i<4096;i++)
		{
				if(sin(i)<=0)
					s_table[i]=0x3000+2069+((1907/10)*sin(2*pi*i/4096)*(vtg));
				else
					s_table[i]=0x3000+2070+((1911/10)*sin(2*pi*i/4096)*(vtg));
		}

}
int delPhi()
{
	uint32_t fre1;
	fre1=(freq*4294967296)/100000;
	return(fre1);
}


void timer1Isr(void)
{
	acc=acc+deltaPhi;
	SSI2_DR_R=s_table[acc>>20];
	TIMER1_ICR_R =TIMER_ICR_TATOCINT;

}

void init_timer()
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
int error()
{
	if(freq==0)
		return 1;
	if(strstr(str,"dc"))
	{
		if(freq>5)
			return 1;
		else if (freq<-5)
			return 1;
		else
			return 0;
	}
	if(strstr(str,"dc"))
		{
		if(vtg>5)
			return 1;
		else if(vtg<-5)
			return 1;
		else if(temp>4)
			return 1;
		else if(freq<0)
			return 1;
		else
			return 0;
		}

}
int main(void)
{

	initHw();
	LED();
	while(1)
	{
	putsUart0("\n\r");
	getsUart0();
	get_wave_freq_vtg();
	if((!error()))
	{
		if(strstr(str,"dc"))
		{
			dc();
		}
		if(strstr(str,"sine"))
		{
			table_sine();
			deltaPhi = delPhi();
			init_timer();
		}
	}
	else
	{
		putsUart0("\n\r");
		putsUart0("Incorrect entry!Enter again");
	}
}


}
