/*
 * uart_atmega16.c
 *
 * Created: 2/13/2023 7:08:49 PM
 * Author : minh
 */ 

#define F_CPU 8000000UL			/* Define frequency here its 8MHz */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// LCD setting
#define LCD_Data_Dir DDRA		/* Define LCD data port direction */
#define LCD_Command_Dir DDRC		/* Define LCD command port direction register */
#define LCD_Data_Port PORTA		/* Define LCD data port */
#define LCD_Command_Port PORTC		/* Define LCD data port */
#define RS PC0				/* Define Register Select (data/command reg.)pin */
#define RW PC1				/* Define Read/Write signal pin */
#define EN PC2				/* Define Enable signal pin */

// Global variables
char count = 0;

//functions definition
void UART_init(long USART_BAUDRATE)
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
	UBRRL = BAUD_PRESCALE;		/* Load lower 8-bits of the baud rate value */
	UBRRH = (BAUD_PRESCALE >> 8);	/* Load upper 8-bits*/
}

unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);			/* Return the byte*/
}

void UART_TxChar(char ch)
{
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = ch ;
}

void UART_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)		/* Send string till null */
	{
		UART_TxChar(str[j]);
		j++;
	}
}

void T1delay2s()
{
	
	char c;
	TIMSK = (1<<TOIE1);  //enable timer 1 interrupt
	TCCR1B |= (1<<CS11);  //setting prescaler 8
	TCNT1 = 0;
	sei();
	
	
	while (1)
	{
		c=UART_RxChar();
		UART_TxChar(c);
		if (count >= 60)
		{
			TCCR1B = 0;
			count = 0;
			break;
		}
	}
}

void LCD_Command(unsigned char cmnd)
{
	LCD_Data_Port= cmnd;
	LCD_Command_Port &= ~(1<<RS);	/* RS=0 command reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 Write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(3);
}

void LCD_Char (unsigned char char_data)	/* LCD data write function */
{
	LCD_Data_Port= char_data;
	LCD_Command_Port |= (1<<RS);	/* RS=1 Data reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable Pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(1);
}

void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Command_Dir = 0xFF;		/* Make LCD command port direction as o/p */
	LCD_Data_Dir = 0xFF;		/* Make LCD data port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command (0x38);		/* Initialization of 16X2 LCD in 8bit mode */
	LCD_Command (0x0C);		/* Display ON Cursor OFF */
	LCD_Command (0x06);		/* Auto Increment cursor */
	LCD_Command (0x01);		/* Clear display */
	LCD_Command (0x80);		/* Cursor at home position */
}

void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* clear display */
	LCD_Command (0x80);		/* cursor at home position */
}

//Interrupt subroutines
ISR(TIMER1_OVF_vect){
	count++;
}


int main(void)
{
    char c = 0, prev_c = 0;
	DDRB = 0x03;
	PORTB = 0x03;
    UART_init(9600);
	LCD_Init();
    while(1)
    {
		prev_c = c;
		
		if (prev_c == '1')
		{
			T1delay2s();
		}
		
	    c=UART_RxChar();
		
		if (prev_c != c){
			LCD_Clear();
		}
		else{
			UART_TxChar(c);
			continue;
		}
		
		if (c == '0'){
			PORTB = 0x01;
			LCD_String("CLOSE");
		}
		else if (c == '1'){
			PORTB = 0x02;
			LCD_String("OPEN");
		}
		
	    UART_TxChar(c);
		
    }
}

