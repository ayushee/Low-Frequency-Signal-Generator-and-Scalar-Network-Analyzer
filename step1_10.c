#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define pi 3.14
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
bool freqUpdate = false;
bool timeUpdate = false;

uint32_t deltaPhi,acc=0;
float vtg=0,freq=0,raw,instant_adc=0,freq1=0,freq_temp=0,vtg1=0;
char str[30],vtg_temp[5],fre_temp[5];
uint32_t s_table[4096];
uint8_t fieldStart[4],field_size,count=2;

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
	for(i=0;i<strlen(str);i++)
	{
		str[i]=NULL;
	}
	str[count]=getcUart0();
	while(!isenter(str[count]))
	{
		if(count<strlen(str))
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

	for(j=count;j<=strlen(str);j++)
		str[j]=NULL;
	for(i=0;(i<strlen(str) && str[i]!=NULL);i++)
	{
	   	str[i]=iscapital(str[i]);
	   	if(isdelim(str[i]))
	   	{
	   		for(j=i;str[j]!=NULL;j++)
	   			str[j]=str[j+1];
	  		str[j]=NULL;
	  		i--;
	 	}
	}
}
void getVoltFreq()
{
	uint8_t i;
	field_size=0;
	for(i=0;i<3;i++)
		fieldStart[i]=0;
	for(i=0;i<strlen(str);i++)
	{
		if( (str[i] >= '0' && str[i] <= '9')||str[i]=='.'||str[i]=='-')
		{
			fieldStart[field_size]=i;
			while((str[i] >='0'  && str[i] <= '9')||str[i]=='.'||str[i]=='-')
						i++;
			field_size++;

		}
	}
		freq=atof(&str[fieldStart[0]]);
		vtg=atof(&str[fieldStart[1]]);
		//duty=atof(&str[fieldStart[2]]);
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
					s_table[i]=0x3000+2069+((1907/5)*sin(2*pi*i/4096)*(vtg));
				else
					s_table[i]=0x3000+2070+((1911/5)*sin(2*pi*i/4096)*(vtg));
		}

}



void table_square(void)
{
	uint32_t i,j;
	float temp=0;
	for(i=0;i<4096;i++)
		s_table[i]=0;
		for(i=0;i<(4096);i++)
		{
			temp=0;
			for(j=0;j<25;j++)
			{
				temp=temp-sin(pi*(2*j+1)*i/2069)/(2*j+1);
			}
			s_table[i]=temp*(vtg+0.5)*1907/(5)+2069+0x3000;
		}
}

void table_sawtooth(void)
{
	uint32_t i;
	for(i=0;i<(4096);i++)
		s_table[i]=	0x3000+2070-(1911*(vtg)*i/(4096*5));
	for(;i<(4096);i++)
		s_table[i]=	0x3000+2070-(1907*(vtg)*i/(5*4096));

}
int delPhi()
{
	return(freq*4294967296/100000);
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


void init_slowadc()
 {
	    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
		GPIO_PORTE_AFSEL_R |= 0x06;                      // select alternative functions for AN2 (PE3)
	    GPIO_PORTE_DEN_R &= ~0x06;                       // turn off digital operation on pin PE3
	    GPIO_PORTE_AMSEL_R |= 0x06;                      // turn on analog operation on pin PE3
	    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
	    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
	    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
	    ADC0_SSMUX3_R = 1;                               // set first sample to AN0
	    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
	    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void sweep(void)
{
	freq_temp=freq;
	vtg=vtg1;
	table_sine();
	count=1;
	putsUart0("\n\rFreq ----> Amplitude");

	while(freq_temp<=freq1)
	{
			char vtg_temp[]={0};
			char fre_temp[]={0};

		deltaPhi=(freq_temp*4294967296)/100000;
		if(freq_temp<1000)
		{

			init_timer();
			waitMicrosecond(3000000);
		}
		else
		{
			init_timer();
			ADC0_SSMUX3_R = 2;
			waitMicrosecond(500000);
		}

		raw = readAdc0Ss3();
		instant_adc= (10.33/2)*(raw+0.5)/4096;
		if(count>1)
		{
			putsUart0("\n\r");
			sprintf(fre_temp,"%0.1f",freq_temp);
			putsUart0(fre_temp);
			putsUart0(" ----> ");
			sprintf(vtg_temp,"%0.2f",instant_adc);
			putsUart0(vtg_temp);
		}
			freq_temp=(freq_temp+pow(2,count));
			count++;

	}
}

int error()
{
	if(strstr(str,"dc"))
	{
		vtg=freq;
		if(vtg>5)
			return 1;
		else if (vtg<-5)
			return 1;
		else if(fieldStart[0] == 0)
			return 1;
		return 0;
	}
	if(strstr(str,"sine")||strstr(str,"square")|| strstr(str,"sawtooth"))
		{
		if(vtg > 5)
			return 1;
		else if(vtg < -5)
			return 1;
		else if(field_size > 4)
			return 1;
		else if(freq < 0 || freq > 10000)
			return 1;
		else if(fieldStart[0] == 0 || fieldStart[1] == 0)
			return 1;
		return 0;
		}
	if(strstr(str,"sweep"))
	{
		freq1=vtg;
		if(freq < 0 || freq > 10000 || fieldStart[0]==0)
			return 1;
		else if(freq1 < 0 || freq1 > 10000 || fieldStart[1]==0)
			return 1;
		else if(freq == freq1)
			return 1;
		else
			return 0;
	}
	if(strstr(str,"voltage"))
		return 0;
	if(strstr(str,"menu"))
		return 0;
	if(strstr(str,"reset"))
		return 0;
	return 1;
}
void menu()
{
	putsUart0("\n\rEnter anyone of the following");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r1)DC vtg");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r2)sine freq vtg");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r3)square freq vtg");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r4)sawtooth freq vtg");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r5)voltage");
	putsUart0("\n\r-------------------");
	putsUart0("\n\r6)sweep freq1 freq2");
	putsUart0("\n\r-------------------");
}

int main(void)
{

	initHw();
	init_slowadc();
	LED();
	putsUart0("\nPress Menu to view options");
	while(1)
	{
	putsUart0("\n\r");
	getsUart0();
	getVoltFreq();
	if((!error()))
	{
		if(strstr(str,"menu"))
				menu();
		if(strstr(str,"dc"))
		{
			dc();
		}
		if(strstr(str,"sine"))
		{
			vtg1=vtg;
			table_sine();
			deltaPhi = delPhi();
			init_timer();
		}
		if(strstr(str,"square"))
		{
			table_square();
			deltaPhi = delPhi();
			init_timer();
		}
		if(strstr(str,"sawtooth"))
		{
			table_sawtooth();
			deltaPhi = delPhi();
			init_timer();
		}
		if(strstr(str,"voltage"))
		{
			init_slowadc();
			waitMicrosecond(3500000);
			raw = readAdc0Ss3();
			instant_adc= (5.33*(raw+0.5)/4096);
			sprintf(vtg_temp,"%0.3f",instant_adc);
			putsUart0(vtg_temp);

		}
		if(strstr(str,"sweep"))
		{
				sweep();
		}
		if(strstr(str,"reset"))
		{
			__asm("    .global _c_int00\n"
				       "    b.w     _c_int00");
		}

	}
	else
	{
		putsUart0("\n\r");
		putsUart0("Incorrect entry!Enter again");
		menu();
	}
}


}
