/*
 * Lab6_NguyenPhan.c
 *
 * Created: 18/10/2019 8:16:44 AM
 * Author : nphan
 */ 


#define F_CPU 16000000UL
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)
#define ROWS_D 4
#define COLUMNS_B 4
#include <util\delay.h>
#include <avr/io.h>
#include <math.h>

void USART_Init(void);
void myTransmit(unsigned char myChar);
void GPIO_Init();
void scan_keypad();
void myDelay(float ms);
float notePeriod(uint8_t index);
void timer_Init();

void timer_Init()
{
	TCCR0A |= (1 <<WGM01) ; //Set timer to CTC
	TCCR0A &= ~((1 << WGM00)|(1 << WGM02)) ;
	
	OCR0A = 255 ; // Timer max
	OCR0B = 0; // 0 for Duty Cycle
	
	//Set prescaler 256
	TCCR0B |= (1 << CS02) ;

}

float notePeriod(uint8_t index)
{
		float p,T;
		p = pow(2.0,1/12.0);
		T = pow(p,index);
		T = T*440;
		T= 1000/(2*T);
		return T;
}

void myDelay(float n)
{
	double i = 0.00;
	while(i <= n){
		_delay_ms(0.01);
		i += 0.01;
	}
}



void myTransmit(unsigned char myChar)
{
	//Wait for buffer to be empty
	while(!(UCSR0A & (1<< UDRE0)));
	
	//Send data
	UDR0 = myChar;
}
void USART_Init(void)
{
	//uint16_t BAUDRATE = FOSC/16/BAUD-1;
	//Set baud rate
	UBRR0H= (BAUDRATE>>8);
	UBRR0L= BAUDRATE;

	// Set frame format: 8data, 1stop bit
	UCSR0C |= (1<<UCSZ00)| (1<<UCSZ01) 	;
	UCSR0C &= ~(1<<USBS0);
	//Enable receiver and transmitter
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0); 

	
}

void GPIO_Init()
{
	//Declare output, input
	DDRD |= (1<<DDRD4)|(1<<DDRD5)|(1<<DDRD6)|(1<<DDRD7) ; // select output mode 
	DDRC |= (1 << DDRC5); //Set output on port PC5
	DDRB &= ~((1<<DDRB0)|(1<<DDRB1)|(1<<DDRB2)|(1<<DDRB3)); // select input mode 
	//Enable pull up 
	PORTB |= (1<<PORTB0)|(1<<PORTB1)|(1<<PORTB2)|(1<<PORTB3);
	PORTD |= (1<<4|1<<5|1<<6|1<<7); // Set high for output
	
}
void scan_keypad()
{
	 uint8_t keychar[ROWS_D][COLUMNS_B] =	{{0,1,2,3},
											{4,5,6,7},
											{8,9,10,11},
											{12,13,14,15}};
	float Twinke[2][4] = { {1.911,1.703,1.517,1.432},{1.276,1.136,0,0}};
	
	for (int i=4; i<=7; i++)
	{
		PORTD &= ~(1<<(i)); // Set low at row i
		for(int j= 0; j< COLUMNS_B; j++)
		{	
			if(!(PINB & (1 << j)))
			{	
				OCR0A = (uint8_t)(255*Twinke[i-4][j]/4.096) ;
				OCR0B = OCR0A/2;
				
				//Logic 1
				PORTC |= (1 << PORTC5); // Logic 1
				while ((TIFR0 & (1 << OCF0B)) == 0) {}	//Wait for overflow event
				TIFR0 |= ( 1 << OCF0B); // Reset 0CR0B
				
				//Logic 0
				PORTC &= ~(1 << PORTC5); // Logic 0
				while ( (TIFR0 & (1 << OCF0A)) == 0){}// wait for OCR0A overflow
				TIFR0 |= (1 << OCF0A); // Reset OCR0A
			}
		} // End j for loop
		PORTD |= 1<<i;
	}// End i for loop
}

int main(void)
{
	
	USART_Init();
	GPIO_Init();
	timer_Init();
	/* Replace with your application code */
	while (1)
	{
		scan_keypad();
	}
}

/*void scan_keypad()
{
uint8_t keychar[ROWS_D][COLUMNS_B] = {{0,1,2,3},
{4,5,6,7},
{8,9,10,11},
{12,13,14,15}};
float Twinke[2][4] = { {1.911,1.703,1.517,1.432},{1.276,1.136,0,0}};


for (int i=4; i<=7; i++)
{
PORTD &= ~(1<<(i)); // Set low at row i
for(int j= 0; j< COLUMNS_B; j++)
{
if(!(PINB & (1 << j)))
{
//float periodT = notePeriod(keychar[i-4][j]);
float periodT = Twinke[i-4][j];
PORTC |= (1 << PORTC5); // Logic 1
myDelay(periodT*0.5);
//myTransmit((char)keychar[i-4][j]);
//myDelay((uint8_t)notePeriod(keychar[i-4][j]));
PORTC &= ~(1 << PORTC5); // Logic 1
//myDelay((uint8_t)(keychar[i-4][j]));
myDelay(periodT*1.5);
}
} // End j for loop
PORTD |= 1<<i;
}// End i for loop
} */