/*
 * All.c
 *
 * Created: 4/17/2022 10:53:35 AM
 * Author : hasitha sawbhagya
 */ 

/*
 * DHT11.c
 *
 * Created: 12/5/2021 6:46:33 PM
 * Author : hasitha sawbhagya
 */ 

//#define F_CPU 1000000
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#define DHT11_PIN 6
#define data_lcd PORTB
#define LCD_DPRT PORTB
#define LCD_DDDR DDRB
#define data_dir DDRB
#define control_lcd PORTD
#define control_dir DDRD
#define enable 4 // responsible to accept information
#define LCD_EN 4
#define rw 3     // responsible for read and write
#define rs 2     // decide sent info. is character/control
#define LCD_RS 2
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

void check_lcd(void);
void on_and_off(void);
void send_command(unsigned char command);
void send_character(unsigned char character);
void send_string(char *StringofCharacters);

void lcd_gotoxy(unsigned char x, unsigned char y);
void lcddata(unsigned char data);

void Request()				/* Microcontroller send start pulse/request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTD |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	for (uint8_t q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

int main(void)
{
	char data[5];
	 control_dir |= (1<<rs) | (1<<rw) | (1<<enable); /* set control signal in output mode */
	 _delay_ms(15);
	 send_command(0x01); /* Clear display */
	 _delay_ms(1.52);
	 send_command(0x38); /* Function Set (8-bit mode) */
	 _delay_us(37);
	 send_command(0x0C); /* Display On-OFF Control */
	 send_string ("    Project x");// displaying name
	 send_command(0xC0);
	 send_string("hello");
	 _delay_ms(1000);
	  send_command(0x01); /* Clear display */
	  lcd_gotoxy(0,0);		/* Enter column and row position */
	while(1)
	{
		Request();		/* send start pulse */
		Response();		/* receive response */
		I_RH=Receive_data();	/* store first eight bit in I_RH */
		D_RH=Receive_data();	/* store next eight bit in D_RH */
		I_Temp=Receive_data();	/* store next eight bit in I_Temp */
		D_Temp=Receive_data();	/* store next eight bit in D_Temp */
		CheckSum=Receive_data();/* store next eight bit in CheckSum */
		
		if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
		{
			lcd_gotoxy(0,0);
			send_string("Error");
		}
		
		else
		{
			itoa(I_RH,data,10);
			//lcd_gotoxy(11,0);
			lcd_gotoxy(0,0);
			send_string(data);
			send_string(".");
			
			itoa(D_RH,data,10);
			send_string(data);
			send_string("%");

			itoa(I_Temp,data,10);
			//lcd_gotoxy(6,1);
			lcd_gotoxy(8,0);
			send_string(data);
			send_string(".");
			
			itoa(D_Temp,data,10);
			send_string(data);
			lcddata(0xDF);
			send_string("C ");
			
			itoa(CheckSum,data,10);
			send_string(data);
			send_string(" ");
		}
		
		_delay_ms(10);
	}
}
/* check if LCD is busy------------This is 
 only time when we are reading from LCD */
void check_lcd()
{
 data_dir = 0; // data direction as input
 control_lcd |= 1 << rw; // put LCD in read mode, RW ON
 control_lcd &= ~(1 << rs); // put LCD in command mode RS OFF
 while(data_lcd >= 0b10000000) // D7 pin will be 1 if LCD is busy
 {
 on_and_off();
 }
 data_dir = 0xFF;
}
/* Keep enable signal on and off to see information on LCD */
void on_and_off()
{
 control_lcd |= 1 << enable; 
 asm volatile("nop");
 asm volatile("nop");
 control_lcd &= ~(1 << enable);
}
/* send command to instruct lcd */
void send_command(unsigned char command)
{
 check_lcd();
 data_lcd = command;
 control_lcd &= ~((1<<rw) | (1<<rs)); // RW-OFF & RS-OFF
 	_delay_us(1);
 	control_lcd &= ~(1<<enable);
 	_delay_ms(3);
 on_and_off();
 data_lcd = 0;
}
/* send character to lcd */
void send_character(unsigned char character)
{
 check_lcd();
 data_lcd = character;
 control_lcd &= ~(1 << rw); // RW-OFF
 control_lcd |= 1 << rs; // RS-ON
 on_and_off();
 data_lcd = 0;
}
/* send string on lcd */
void send_string(char *StringofCharacters)
{
 while(*StringofCharacters>0)
 {
 send_character(*StringofCharacters++);
 }
}
void lcddata(unsigned char data)
{
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data & 0xf0);		/* SEND DATA TO DATA PORT */
	LCD_DPRT |= (1<<LCD_RS);						/* MAKE RS = 1 FOR DATA */
	LCD_DPRT |= (1<<LCD_EN);						/* EN=0 FOR H TO L PULSE */
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE */
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE */
	_delay_us(100);									/* WAIT FOR MAKE ENABLE WIDE */
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data << 4);		/*  */
	LCD_DPRT |= (1<<LCD_EN);						/* EN=0 FOR H TO L PULSE*/
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE*/
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE*/
	_delay_us(2000);								/* WAIT FOR MAKE ENABLE WIDE*/
}
void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstcharadd[]={0x80, 0xC0};
	send_command(firstcharadd[y] + x);
}

