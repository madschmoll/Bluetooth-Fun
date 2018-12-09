#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "c:/ti/TivaWare_C_Series-2.1.4.178/inc/hw_types.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/inc/hw_memmap.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/inc/hw_gpio.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/driverlib/sysctl.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/driverlib/pin_map.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/driverlib/rom_map.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/driverlib/gpio.h"
#include "c:/ti/TivaWare_C_Series-2.1.4.178/inc/tm4c123gh6pm.h"
#include "C:/ti/TivaWare_C_Series-2.1.4.178/driverlib/interrupt.h" 
#include "C:/ti/TivaWare_C_Series-2.1.4.178/driverlib/timer.h"
#include "C:/ti/TivaWare_C_Series-2.1.4.178/driverlib/uart.h"
#include <string.h>

char rxChar[10];
char txChar;
uint8_t  timer=60;
char dir='f'; 
char dirstring[10];
uint8_t indx=0;

void writeCharToUart3(char c)
{
	
	while (UART3_FR_R & UART_FR_TXFF); //wait for tx to not be full
	UART3_DR_R = c; // write a char to UART 3
	
}


void writeStringToUart3(char* string) // write a string to UART 3
{
    for (int i = 0; i < strlen(string); i++){
			writeCharToUart3(string[i]); 
		}
}


void UART2_Handler() // UART 2 ISR to recieve data from RN42  
{
    rxChar[indx] = UART2_DR_R;
		indx++;
		// test data recieving with LED commands 
		if(rxChar[indx-1]==13)
		{
			rxChar[indx-1]='\0'; 
			if(strcmp(rxChar,"red")==0)
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0X02);
			if(strcmp(rxChar,"blue")==0)
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0X04);
			if(strcmp(rxChar,"green")==0)
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0X08);
			indx=0;
		}

    UART2_ICR_R=UART_ICR_RXIC;	//clear interrupt
}


void UART3_Handler() // UART 3 ISR to transmit data over RN42 
{
		txChar = UART3_DR_R;
    UART3_ICR_R=UART_ICR_TXIC; //clear interrupt
	
}


void Timer1A_Handler(void)
{	// how to space notifications evenly? 

	if(timer>0){
		timer--;
	}

	if(timer==0)
	{
		timer=30; //this will execute every 3 sec
		if(dir == 'r'){	
			char direction[] = "Right Turn";
			sprintf(dirstring,"%s",direction);
			writeStringToUart3(dirstring);
			writeStringToUart3("\n\r");
			dir = 0;
		}
		if(dir == 'r'){	
			char direction[] = "Left Turn";
			sprintf(dirstring,"%s",direction);
			writeStringToUart3(dirstring);
			writeStringToUart3("\n\r");
			dir = 0;
		}
		if(dir == 'f'){	
			char direction[] = "Moving Forward";
			sprintf(dirstring,"%s",direction);
			writeStringToUart3(dirstring);
			writeStringToUart3("\n\r");
		}
	}

	TIMER1_ICR_R=TIMER_ICR_TATOCINT; 

}

void Timer1A_Init(){
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // configure timer
	TimerLoadSet(TIMER1_BASE, TIMER_A, 1600000 - 1 ); // reload value into timer (remeber clock is set at 40Mz)
	IntPrioritySet(INT_TIMER1A, 0x00); // timer has first (0th) priority
	IntEnable(INT_TIMER1A); // enable interuupt
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // ARM timeout interrupt
	TimerEnable(TIMER1_BASE, TIMER_A); // enable subtimer 0A of timer 0

}

void configureUART2(){
	// configure UART2 - 9600 baud 8 N 1
	//PD6 for TXD on bluetooth module	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6);
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	// enable interrupt
	UART2_IM_R = UART_IM_RXIM;                      
	NVIC_EN1_R = 1<<1;
}

void configureUART3(){
	// configure UART3 -- 9600 baud 8 N 1
	// PC7 for rx on RN42
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7);
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	// enable interrupt
	UART3_IM_R = UART_IM_TXIM;                       
	NVIC_EN1_R = 1<<27;
}


void PortInit()
{
	// set clock to 40MHz
//SysCtlClockSet((4 << SYSCTL_RCC_SYSDIV_S) | SYSCTL_RCC_USESYSDIV |  SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	// Enable Peripheral clocks for ports we want to use
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD;
	
	GPIO_PORTF_DIR_R |= 0x0E; // enable LEDs for testing
	GPIO_PORTF_DEN_R |= 0x0E;

	configureUART2();
	configureUART3();
}


int main(void)
{
    PortInit();
		Timer1A_Init();
    while(1);
}
