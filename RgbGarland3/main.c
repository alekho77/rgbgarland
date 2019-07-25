/*
 * RgbGarland3.c
 *
 * Created: 22.07.2019 9:59:01
 * Author : Alexey
 */ 

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define RESET_PIN PORTD0
#define CLOCK_PIN PORTD1

#define LED_BUSES 5

#define MUTABLE_COLOR_FLAG  (0b1000)
enum COLORS {
    BLACK           = (0b0000),
    RED             = (0b0100),
    GREEN           = (0b0010),
    BLUE            = (0b001),
    MAGENTA         = (RED | BLUE),
    CYAN            = (GREEN | BLUE),
    YELLOW          = (RED | GREEN),
    WHITE           = (RED | GREEN | BLUE),
    MUTABLE_CYCLE   = (MUTABLE_COLOR_FLAG + 0),
    MUTABLE_SRANDOM = (MUTABLE_COLOR_FLAG + 1),
    MUTABLE_ROLL    = (MUTABLE_COLOR_FLAG + 2),
    MUTABLE_MRANDOM = (MUTABLE_COLOR_FLAG + 3),
};

enum BLINK_MODES {
    BLINK_STATIC    = 0,
    BLINK_BLINK     = 1,
    BLINK_RUN       = 2,
    BLINK_STRIP     = 3,
};
#define BLINK_OFF_FLAG  (1 << 7)

#define ADDITIONAL_PRESCALER(PRESCALER)     (256 - (PRESCALER))
#define INIT_TCNT0  ADDITIONAL_PRESCALER(4)
#define INIT_TCNT2  ADDITIONAL_PRESCALER(125)

uint8_t colors[LED_BUSES] = {0};
int mode = 0;
uint32_t clocks_ms = 0;

void tick_led_buses(bool reset);
void init_mode(int blink_mode, int color);

inline int rand_color()
{
    return rand() % 7 + 1;
}

inline int get_blink_mode()
{
    return (mode >> 4) & 0b111;
}

inline int get_color_mode()
{
    return mode & 0xf;
}

int main(void)
{
    DDRD = 0xff & (~(1 << PORTD2));       // Port D for output excluding D2 (INT0)
    DDRB = 0b111;                         // Bits 0..2 of port B for output

    PORTB = 0b11000000;                  // reset in port B, pull-up bits B6 and B7

    PORTD =  0b11111001 | (1 << PORTD2);  // switch on all indicators, set high level to RST pin of counter chip, pull-up D2
    _delay_ms(500);                       // show start of program
    PORTD &= 0b00000000 | (1 << PORTD2);  // switch off all indicators plus set low level to RST pin of counter chip (reset the counter)

    // Timer 0
    TIMSK |= (1 << TOIE0);                // enabled interrupts on overflow timer0
    TCCR0 |= (1 << CS02);                 // set prescaler to 256 and start the timer0

    // Timer 1
    OCR1A = 325;                          // about 3Hz
    TCCR1B |= (1 << WGM12);               // Mode 4, CTC on OCR1A
    TIMSK |= (1 << OCIE1A);               // Set interrupt on compare match
    TCCR1B |= (1 << CS12) | (1 << CS10);  // set prescaler to 1024 and start the timer1

    // Timer 2
    TCCR2 |= (1 << CS21);  // set prescale to 8 for timer2
    TCCR2 &= ~((1 << WGM21) | (1 << WGM20));  // set mode to Normal
    TIMSK |= (1 << TOIE2);  // enabled interrupts on overflow timer2
    TCNT2 = INIT_TCNT2;

    // INT0
    MCUCR |= (1 << ISC01) | (1 << ISC00);  // set ISC01 and ISC00, the rising edge INT0
    GICR |= (1 << INT0);                      // Turns on INT0

    init_mode(BLINK_STATIC, RED);

    uint8_t prev_rotar = (PINB >> 6) & 0b11;
    int x = 2;
    while (1)
    {
        uint8_t curr_rotar = (PINB >> 6) & 0b11;
        if (curr_rotar != prev_rotar)
        {
            if (curr_rotar == 0b10)  // CW
            {
                x = (x + 1) % 4;
            } 
            else if (curr_rotar == 0b01)  // CCW
            {
                x = x > 0 ? x - 1 : 3;
            }
            prev_rotar = curr_rotar;
            cli();
            PORTD = (PORTD & 0b10000111) | (1 << (3 + x));
            sei();
            _delay_ms(50);
        }
    }
}

void init_mode(int blink_mode, int color)
{
    cli();  // Disable interrupts
    mode = (blink_mode << 4) | color;
    switch(color)
    {
        case MUTABLE_ROLL:
            for (int i = 0; i < LED_BUSES; ++i)
            {
                colors[i] = i % 7 + 1;
            }
            break;
        case MUTABLE_MRANDOM:
            for (int i = 0; i < LED_BUSES; ++i)
            {
                colors[i] = rand_color();
            }
            break;
        default:
            if (color == MUTABLE_CYCLE)
            {
                color = 1;
            }
            else if (color == MUTABLE_SRANDOM)
            {
                color = rand_color();
            }
            memset(colors, color, LED_BUSES);
            break;
    }

    // initialize timer0
    if (blink_mode == BLINK_RUN)
    {
        TCNT0 = 0;  // about 15Hz to switch led bus
    } 
    else
    {
        TCNT0 = INIT_TCNT0;  // about 1kHz for bus switching
    }

    sei();  // Enable interrupts
}

// timer0 overflow interrupt
ISR (TIMER0_OVF_vect)
{
    static bool flag = true;
    static int c = 0;

    if (flag)
    {
        tick_led_buses(c == 0);
        PORTB |= colors[c];
    }
    else
    {
        PORTB &= ~(0b111);
        tick_led_buses(c == 0);
        if (++c >= LED_BUSES)
        {
            c = 0;
        }
    }

    flag = !flag;

    // reset timer0 counter
    if (get_blink_mode() == BLINK_RUN)
    {
        TCNT0 = 0;  // about 15Hz to switch led bus
    }
    else
    {
        TCNT0 = INIT_TCNT0;  // about 1kHz for bus switching
    }
}

void tick_led_buses(bool reset)
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

// timer1 compare match interrupt
ISR (TIMER1_COMPA_vect)
{
    static uint8_t colors_copy[LED_BUSES];

    switch(get_blink_mode())
    {
        case BLINK_BLINK:
            mode ^= BLINK_OFF_FLAG;
            if (mode & BLINK_OFF_FLAG)
            {
                memcpy(colors_copy, colors, LED_BUSES);
                memset(colors, BLACK, LED_BUSES);
                return;
            }
            else
            {
                memcpy(colors, colors_copy, LED_BUSES);
            }
            break;
        case BLINK_STRIP:
            if (!(mode & BLINK_OFF_FLAG))
            {
                memcpy(colors_copy, colors, LED_BUSES);
                mode |= BLINK_OFF_FLAG;
            } 
            else
            {
                memcpy(colors, colors_copy, LED_BUSES);
            }
            break;
    }
    switch(get_color_mode())
    {
        case MUTABLE_CYCLE:
        case MUTABLE_ROLL:
            for (int i = 0; i < LED_BUSES; ++i)
            {
                colors[i] = colors[i] % 7 + 1;
            }
            break;
        case MUTABLE_MRANDOM:
            for (int i = 0; i < LED_BUSES; ++i)
            {
                colors[i] = rand_color();
            }
            break;
        case MUTABLE_SRANDOM:
            memset(colors, rand_color(), LED_BUSES);
            break;
        default:
            break;
    }
    if (get_blink_mode() == BLINK_STRIP)
    {
        static bool first_off = true;
        memcpy(colors_copy, colors, LED_BUSES);
        for (int i = first_off ? 0 : 1; i < LED_BUSES; i += 2)
        {
            colors[i] = BLACK;
        }
        first_off = !first_off;
    }
}

// timer2 overflow interrupt
ISR (TIMER2_OVF_vect)
{
    ++clocks_ms;
    TCNT2 = INIT_TCNT2;
}

// INT0 interrupt
ISR (INT0_vect)
{
    static uint32_t last_ms = 0;

    if ((clocks_ms - last_ms) > 50)
    {
        PORTD ^= 0b10000000;
    }

    last_ms = clocks_ms;
}
