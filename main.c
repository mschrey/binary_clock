// ***********************************
// *                                 *
// *          BINÄRUHR               *
// *          1.09.2016              *
// *                                 *
// ***********************************

//lfuse 0xe2 ( avrdude -pm88 -cusbasp -v -F -U lfuse:w:0xe2:m)
//hfuse 0xDF

#define F_CPU 	8000000UL
#define BAUD 	9600UL
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)      // Fehler in Promille, 1000 = kein Fehler.
 
#if ((BAUD_ERROR<980) || (BAUD_ERROR>1020))
  #error Systematischer Fehler der Baudrate grösser 1% und damit zu hoch! 
#endif
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>			    
#include <util/delay.h>


volatile uint8_t second = 0;
volatile uint16_t time_temp = 0;
volatile uint16_t time_final = 0;
volatile uint8_t bit = 0;
volatile uint8_t invalid = 0;  //will be set to 0 at the beginning of the datagram, will be set to !0 if wrong datagram received
//invalid = 1 -> rising edge to early
//invalid = 2 -> wrong parity
//invalid = 4 -> all bits are zero. no dcf77 signal?
//invalid = 8 -> missing pulse (usually at second 0) came too early -> bad reception
volatile uint8_t minute_inc = 0;  //will be set to 1 with bad dcf77 signal at the minute border
volatile uint8_t hour_tens, hour_ones, minute_tens, minute_ones, second_tens, second_ones;  

#define SR_STROBE  (1<<5)   //PD5  shift register strobe
#define SR_DATA     (1<<6)   //PD6  shift register data
#define SR_CLOCK   (1<<7)   //PD7  shift register clock
#define SR_OE      (2)      //PB2  shift register output enable
#define DCF_DATA   (2)      //PD2
#define DCF_PON	   (5)      //PC5  DCF77 receiver power on
//PCI2 interrupt for PCINT23..16
//PCI1 interrupt for PCINT14..8
//PCI0 interrupt for PCINT7..0 (page 65)
//PCINT18 (PD2) chosen
#define RISING_EDGE_TOO_EARLY   0
#define WRONG_PARITY            1
#define ALL_BITS_ZERO           2
#define MISSING_PULSE_TOO_EARLY 3


void uart_init(void)
{
  UBRR0H = UBRR_VAL >> 8;
  UBRR0L = UBRR_VAL & 0xFF;
 
  UCSR0B |= (1<<TXEN0 | 1<<RXEN0);      // UART TX und RX einschalten
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);     // Asynchron 8N1 
  do {
  	UDR0;
  } while (UCSR0A & (1<<RXC0));
} 
 
 
/* ATmega88 */
int uart_putc(unsigned char c)
{
    while (!(UCSR0A & (1<<UDRE0))) { /* warten bis Senden moeglich */
    }                             

    UDR0 = c;    
    return 0;
}
 
 
void uart_puts (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        uart_putc(*s);
        s++;
    }
}



ISR(PCINT2_vect)  //will be executed every second, at rising edge
{
    uint16_t timerval;
    _delay_us(10);
    if (PIND & (1<<DCF_DATA)) {  //if rising edge...
        timerval = TCNT1;
        if (timerval < 6500) {   //if rising edge too early...
            invalid |= 1<<RISING_EDGE_TOO_EARLY;
            time_temp = 0;
            //uart_puts("rising edge too early\r\n"); 
        } else {
            second++;
            TCNT1 = 0;     //reset the timer
        }
    } 
}


ISR(TIMER1_COMPA_vect)  //will be executed every second, 150ms after rising edge
{
    bit = (PIND & (1<<DCF_DATA))>>DCF_DATA;  //bit is 1 or 0
    if ((second >= 22) & (second <= 36)) {
        time_temp += bit * 0b1000000000000000;
        time_temp >>= 1;
    }
} 

ISR(TIMER1_COMPB_vect)  //will be executed at missing pulse (second 0 only)
{
    if ((__builtin_popcount(time_temp) % 2) == 1) {   //even parity for mins and hours implies an always even number of ones
        invalid |= 1<<WRONG_PARITY;
    }
    if(time_temp == 0) {
        invalid |= 1<<ALL_BITS_ZERO;
    }
    if(second < 59) {
        invalid |= 1<<MISSING_PULSE_TOO_EARLY;
    } else {
        second = 0;
    }
    if(invalid == 0) {
        time_final = time_temp;
    } 
    //uart_puts("new minute\r\n"); 

    time_temp = 0;
}


void output_shiftreg(uint32_t data)
{
    data = data & 0xFFFFFF;  //mask lower three bytes
    uint8_t i = 0;
    PORTD &= ~(SR_CLOCK);
    PORTD &= ~(SR_DATA);
    _delay_us(10);
    for(i=0;i<24;i++) {
        if (data & 0x800000) 
            PORTD |= SR_DATA;
        else 
            PORTD &= ~(SR_DATA);
        _delay_us(10);
        //clock pulse
        PORTD |= SR_CLOCK;
        _delay_us(10);
        PORTD &= ~(SR_CLOCK);
        data <<= 1;
    }
    //strobe pulse
    _delay_us(10);
    PORTD |= SR_STROBE;
    _delay_us(10);
    PORTD &= ~(SR_STROBE);
}

void output_uart(uint8_t hour_tens, uint8_t hour_ones, uint8_t minute_tens, uint8_t minute_ones, uint8_t second, uint32_t data2shiftreg)
{
    char buf[40];
    uart_puts("h10:");
    utoa(hour_tens, buf, 10);
    uart_puts(buf);
    uart_puts(" h1:");
    utoa(hour_ones, buf, 10);
    uart_puts(buf);

    uart_puts(" m10:");
    utoa(minute_tens, buf, 10);
    uart_puts(buf);
    uart_puts(" m1:");
    utoa(minute_ones, buf, 10);
    uart_puts(buf);

    utoa(second, buf, 10);
    uart_puts(" s:");
    uart_puts(buf);

    //ultoa(data2shiftreg, buf, 2);
    ultoa(time_temp, buf, 2);
    uart_puts(" 0b");
    uart_puts(buf);

    uart_puts("\r\n");
}

void output_uart_short(uint8_t hour_tens, uint8_t hour_ones, uint8_t minute_tens, uint8_t minute_ones, uint8_t second, uint32_t data2shiftreg)
{
    char buf[40];
    utoa(hour_tens, buf, 10);
    uart_puts(buf);
    utoa(hour_ones, buf, 10);
    uart_puts(buf);

    uart_puts(":");
    utoa(minute_tens, buf, 10);
    uart_puts(buf);
    utoa(minute_ones, buf, 10);
    uart_puts(buf);

    uart_puts(":");
    utoa(second, buf, 10);
    uart_puts(buf);

    //ultoa(data2shiftreg, buf, 2);
    ultoa(time_temp, buf, 2);
    uart_puts(" 0b");
    uart_puts(buf);

    uart_puts("\r\n");
}

void update_time()
{
    //this will be executed only if the dcf signal was invalid, but a minute start sequence could be found
    minute_ones++;
    if(minute_ones == 10) {
        minute_ones = 0;
        minute_tens++;
    }
    if((minute_tens == 6) & (minute_ones == 0)) {
        minute_tens = 0;
        hour_ones++;
    }
    if(hour_ones == 10) {
        hour_ones = 0;
        hour_tens++;
    }
    if((hour_tens == 2) & (hour_ones == 4)) {
        hour_ones = 0;
        hour_tens = 0;
    }    
}


int main (void)
{
    DDRB = (1<<SR_OE);
    DDRC = (1<<DCF_PON);
    DDRD = 0b11100000;
    PORTC |= (1<<DCF_PON);
    PORTD &= ~(SR_CLOCK);
    PORTD |=  (SR_STROBE);
    PORTD &= ~(SR_DATA);

    uart_init();

    //pin change interrupt
    PCMSK2 = (1<<PCINT18);  //PCINT18 enabled (PD2)
    PCICR = (1<<PCIE2);     //PCINT2 enabled
    //timer compareA interrupt (150ms after timer start with 8MHz and 1024 prescaler -> 1172 = 4*256+148)
    OCR1AH = 4;
    OCR1AL = 148;
    //timer compareB interrupt (1050ms after timer start with 8MHz and 1024 prescaler -> 8203 = 32*256+11)
    OCR1BH = 32;
    OCR1BL = 11;
    TIMSK1 = ((1<<OCIE1A)|(1<<OCIE1B));
    TCCR1B |= ((1<<CS12)|(0<<CS11)|(1<<CS10));  //enable timer 
    PORTB |= (1<<SR_OE);                        //enable outputs of shift register

    uint32_t data2shiftreg = 0;     
    uint8_t old_second;
    char buf[20];

    uart_puts("LED test\r\n");
    output_shiftreg(0b11111111111111111111);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);
    _delay_ms(255);

    PORTC &= ~(1<<DCF_PON);   //create falling edge for DCF77 receiver power on

    sei();

    uart_puts("\r\n\r\n\r\n\r\n");
//time_final is in the following binary form
//0bhhhhhhhmmmmmmmm where h=hour, m=minute
//0bpttooooptttoooo with p=parity, t=tens, o=ones (always MSB left)
//  010000010000010
    while(1) {
        if (second != old_second) {
            second_ones = second % 10;
            second_tens = (second - second_ones)/10;
            if (second == 0) {
                if(invalid == 0) {
                    uart_puts(" accepted\r\n"); 
                    minute_ones = (time_final & 0b0000000000001111);
                    minute_tens = (time_final & 0b0000000001110000) >> 4;
                    hour_ones   = (time_final & 0b0000111100000000) >> 8;
                    hour_tens   = (time_final & 0b0011000000000000) >> 12;
                } else {
                    uart_puts("invalid! ");
                    if(invalid & 1<<RISING_EDGE_TOO_EARLY) {
                        uart_puts("wrong timing, "); 
                        invalid &= ~(1<<RISING_EDGE_TOO_EARLY);
                    }
                    if(invalid & 1<<WRONG_PARITY) {
                        uart_puts("wrong parity, "); 
                        invalid &= ~(1<<WRONG_PARITY);
                    }
                    if(invalid & 1<<ALL_BITS_ZERO) {
                        uart_puts("all bits zero, "); 
                        invalid &= ~(1<<ALL_BITS_ZERO);
                    }
                    if(invalid & 1<<MISSING_PULSE_TOO_EARLY) {
                        uart_puts("missing pulse too early, "); 
                        invalid &= ~(1<<MISSING_PULSE_TOO_EARLY);
                    } 
                    uart_puts("\r\nlast dcf77 time was: ");
                    output_uart_short((time_final & 0b0011000000000000) >> 12, (time_final & 0b0000111100000000) >> 8, (time_final & 0b0000000001110000) >> 4, (time_final & 0b0000000000001111), 0, 0);
                    update_time();
                    uart_puts("now: ");
                    output_uart_short(hour_tens, hour_ones, minute_tens, minute_ones, second, data2shiftreg);
                }
                invalid = 0;
            }

            data2shiftreg = (uint32_t)1 << 23;           
            data2shiftreg += (second_ones & 0b1111);
            data2shiftreg += (second_tens & 0b0111) << 4;   
            data2shiftreg += (minute_ones & 0b1111) << 7;   
            data2shiftreg += (minute_tens & 0b0111) << 11;  
            data2shiftreg += ((uint32_t)hour_ones   & 0b1111) << 14;  
            data2shiftreg += ((uint32_t)hour_tens   & 0b0011) << 18;  

            output_uart(hour_tens, hour_ones, minute_tens, minute_ones, second, data2shiftreg);
            output_shiftreg(data2shiftreg);
        }
        old_second = second;

        _delay_ms(19);
    }
}


