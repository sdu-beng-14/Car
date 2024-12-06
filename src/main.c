#define F_CPU 16000000UL //needs to be defined for the delay functions to work.
#define BAUD 9600
#define NUMBER_STRING 1001
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h> //here the delay functions are found
#include "usart.h"
#include <avr/interrupt.h>
const int Number_Variables=6; // Number of variables sent by the nextion
const int Tire_Length=0.1954; //distance around a tire, hence we can calculate the distance travelled by 1 rps 
int main(void)
{
	//Distance,Time and Reversal status 
	int Distance[69];
	int Time[69];
	int Reversal[69];
	int counter=0;
	int *HugeQueuer[3];

	HugeQueuer[0]=Distance;
	HugeQueuer[1]=Time;
	HugeQueuer[2]=Reversal;

	char readBuffer[100];
	uart_init();//initialize communication with PC - debugging
	io_redirect();//redirect printf function to uart, so text will be shown on PC
	//opto code
	TCCR1B = ( 1<< ICNC1) | (1 << ICES1) | (1 << CS12) | ( 1 << CS10);
	TCCR1A= 0x00;
	TCCR1B= 0xC5;
	DDRB &= ~0x01;
	PORTB |= 0x01;
	//

	DDRB |= (1 <<PB1);
	PORTB |= (1 << PB1);
	DDRB |= (1 <<PB2);
	PORTB |= (1 << PB2);
	DDRB &= ~(1 <<PB4);
	PORTB &= ~(1 << PB4);
	
	DDRD |= (1 << PD6); // sets PD6 as an output pin
	DDRD |= (1 << PD5); //sets PD5 as an output pin
	TCCR0A |= 0xA3; // sets fast pwn non inverting mode on Oc0A (which is for PD6) and Oc0B (which is for PD5)
	TCCR0B |= 0x05; // sets prescaler to 1024 for Timer0
	OCR0A = 0;  
    OCR0B = 0; 
	printf("page 0%c%c%c",255,255,255);//init at 9600 baud.
	_delay_ms(20);
	uint32_t readValue = 1;
    while (1) 
    {
		//opto code
		if (PINB & (1 << PB0)){
			printf("page1.rotationGuard.val=%d%c%c%c", 1, 255,255,255);
		}
		//printf("get %s.val%c%c%c","page0.n0",255,255,255);	//sends "get page0.n0.val"	
		int typeExpected = 0;
	
			for(int i = 0; i<8;i++)
			{
				scanf("%c", &readBuffer[i]);
				if(readBuffer[i] == 0x71)//Expect number string
				{
					typeExpected = NUMBER_STRING;
					readBuffer[0] = 0x71;//Move indicator to front, just to keep the nice format
					break;
				}
			}
			if(typeExpected == NUMBER_STRING)
			{
				for(int i = 1; i<8; i++)
				{
					scanf("%c", &readBuffer[i]);
				}

				if(readBuffer[0] == 0x71 && readBuffer[5] == 0xFF && readBuffer[6] == 0xFF && readBuffer[7] == 0xFF)//This is a complete number return
				{
					readValue = readBuffer[1] | (readBuffer[2] << 8) | (readBuffer[3] << 16)| (readBuffer[4] << 24);
				}
			}
		
		for(int i = 0; i<7; i++)
		{
				scanf("%c", &readBuffer[i]);
				if(readBuffer[i] == 0x1A)//some error occurred - retrieve the 0xFF commands and start over
				{
					scanf("%c", &readBuffer[i]);
					scanf("%c", &readBuffer[i]);
					scanf("%c", &readBuffer[i]);
					continue;
				}
		}
		// Algorithm to figure out which variable it came from
		int temp_varLocation=(int)readValue%Number_Variables;
		int temp_varMagnitude=((int)readValue-temp_varLocation)/Number_Variables;
		if (temp_varLocation>2){
			HugeQueuer[temp_varLocation-3][counter]=temp_varMagnitude;
		}else{
			if(temp_varLocation==1){
				counter--;
				for(int i=0; i<counter;i++){
					printf("page0.n0.val=%d%c%c%c", HugeQueuer[0][i], 255,255,255);
					printf("page0.n1.val=%d%c%c%c", HugeQueuer[1][i], 255,255,255);
					printf("page0.c0.val=%d%c%c%c", HugeQueuer[2][i], 255,255,255);
					printf("covx page0.n0.val,page0.MetersTxt.txt,0,0%c%c%c",255,255,255);
  					printf("covx page0.n1.val,page0.TimeTxt.txt,0,0%c%c%c",255,255,255);
					printf("covx page0.c0.val,page0.ReverseTxt.txt,0,0%c%c%c",255,255,255);
					printf("page0.tm0.en=%d%c%c%c",1,255,255,255);
				}
			}else if(temp_varLocation==2){
				counter=0;
			}else if(temp_varLocation==0){
				//Algorithm for executing the queues
				for(int i=0; i<counter; i++){
					printf("page 1%c%c%c",255,255,255);
					printf("page1.n3.val=%d%c%c%c", i, 255,255,255);
					printf("page1.TimeRunning.val=%d%c%c%c",HugeQueuer[1][i],255,255,255);
					printf("page1.DistanceMax.val=%d%c%c%c",HugeQueuer[0][i],255,255,255);

					// add some more globblobity bop here for the other value changes
					
				}
				for(int i=0; i<counter; i++){
					printf("page 2%c%c%c",255,255,255);
					// add some globblobity bop here for page 2
					_delay_ms(5000); // reading delay time
				}
			}
		}
		// opto code
		// add some alogrithim here cause the speed can't be 255m/s with octo
		OCR0A = (int)readValue;
		OCR0B = (int)readValue;
    }
}