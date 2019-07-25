/*
 * RgbGarland2.c
 *
 * Created: 20.07.2019 19:39:20
 * Author : Alexey
 */ 

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define RESET_PIN PORTD0
#define CLOCK_PIN PORTD1

#define LED_BUSES 5

#define BLACK       (0b000)
#define RED         (0b100)
#define GREEN       (0b010)
#define BLUE        (0b001)
#define MAGENTA     (RED | BLUE)
#define CYAN        (GREEN | BLUE)
#define YELLOW      (RED | GREEN)
#define WHITE       (RED | GREEN | BLUE)
#define CLEAR_COLOR (~WHITE)

#define ADDITIONAL_PRESCALER(PRESCALER)     (256 - (PRESCALER))
#define INIT_TCNT0  ADDITIONAL_PRESCALER(4)

const uint8_t colors[LED_BUSES] = {RED, GREEN, BLUE, MAGENTA, YELLOW};

void tick_led_buses(uint8_t reset);

int main(void)
{
    DDRD = 0xff;  // Port D for output
    DDRB = 0b111;  // Bits 0..2 of port B for output

    PORTB &= 0b11111000;  // reset bits 0..2 in port B
    PORTD =  0b11111101;  // switch on all indicators plus set high level to RST pin of counter chip
    _delay_ms(500);       // show start of program
    PORTD =  0b00000000;  // switch off all indicators plus set low level to RST pin of counter chip (reset the counter)

    TIMSK |= (1 << TOIE0);  // enabled interrupts on overflow timer0
    TCNT0 = INIT_TCNT0;  // initialize timer0
    TCCR0 |= (1 << CS02) /*| (1 << CS00)*/;  // set prescaler to 256 and start the timer0

    sei();  // Enable global interrupts by setting global interrupt enable bit in SREG

    while (1) 
    {
    }
}

// timer0 overflow interrupt
ISR (TIMER0_OVF_vect)
{
    //PORTD ^=  0b11111100;  // switch on/off all indicators
    static uint8_t flag = 1;
    static uint8_t c = 0;

    if (flag)
    {
        tick_led_buses(c == 0);
        PORTB |= colors[c];
    } 
    else
    {
        PORTB &= CLEAR_COLOR;
        tick_led_buses(c == 0);
        ++c;
        if (c >= LED_BUSES)
        {
            c = 0;
        }
    }

    flag ^= 1;
    TCNT0 = INIT_TCNT0;  // reset timer0 counter
}

void tick_led_buses(uint8_t reset)
{
    if (reset)
    {
        PORTD ^= (1 << RESET_PIN);  // reset
    }
    else
    {
        PORTD ^= (1 << CLOCK_PIN);  // clock
    }
}
