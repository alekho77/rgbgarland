/*
* RgbGarland1.c
*
* Created: 19.07.2019 20:33:08
* Author : Alexey
*/

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>

#define LED_BUSES 5

#define BLACK       (0b000)
#define RED         (0b100)
#define GREEN       (0b010)
#define BLUE        (0b001)
#define MAGENTA     (RED | BLUE)
#define CYAN        (GREEN | BLUE)
#define YELLOW      (RED | GREEN)
#define WHITE       (RED | GREEN | BLUE)

const uint8_t colors[LED_BUSES] = {RED, GREEN, BLUE, MAGENTA, YELLOW};

int main(void)
{
    DDRD = 0xff;
    DDRB = 0b111;

    PORTB = 0b00000000;
    PORTD = 0b11111101;
    _delay_ms(500);
    PORTD = 0b00000000;

    uint8_t c = 0;
    while (1)
    {
        if (c > 0)
        {
            PORTD |= 0b00000010;  // clock up
        } 
        else
        {
            PORTD |= 0b00000001;  // reset up
        }
        PORTB |= colors[c];
        _delay_ms(1);
        PORTB &= 0b11111000;
        if (c > 0)
        {
            PORTD &= 0b11111101;  // clock down
        } 
        else
        {
            PORTD &= 0b11111110;  // reset down
        }
        ++c;
        if (c >= LED_BUSES)
        {
            c = 0;
        }
    }
}
