#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define CRANKSHAFT_DDR   DDRB
#define CRANKSHAFT_PORT  PORTB
#define CRANKSHAFT_PINR  PINB
#define CRANKSHAFT_BIT   PB0  /* D8 */

#define IGNITION_DDR     DDRB
#define IGNITION_PORT    PORTB
#define IGNITION_BIT     PB1  /* D9 */

#define POT_ANALOG_CH    1    /* A1 -> ADC1 */

#define LCD_ADDR 0x27

int THETA_0 = 45;
const uint16_t IGNITION_ON = 200; 

/* Volatile variables shared with ISRs */
volatile uint16_t n1 = 0, n2 = 0, x = 0;
volatile bool capture_flag = false;
volatile bool display_flag = false;
volatile bool calculate_flag = false;
volatile bool waiting_for_ovf = false;
float Speed = 0.0;
int POT_IGNITION = 0, THETA_IGNITION = 0, THETA_DELAY = 0;
float f = 0.0;
volatile uint16_t count_ADC = 0;
int prescaler = 64;
float T_cycle = 0.0, T_delay = 0.0;
volatile uint32_t delta_N = 0;
volatile uint32_t prev_delta_N = 0;
volatile uint32_t N_delay = 0; 
volatile uint8_t ignition_state = 0; 
volatile uint16_t delay_ovf_count = 0; 
volatile uint16_t delay_final_ticks = 0; 

/* UART (for optional debug) */
void uart_init(uint32_t baud) {
    uint16_t ubrr = (uint16_t)(F_CPU/16/baud - 1);
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr & 0xFF;
    UCSR0B = (1<<TXEN0); /* enable TX only */
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); /* 8N1 */
}
void uart_putc(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}
void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}
void uart_printf(const char *fmt, ...) {
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_print(buf);
}

void ADC_init() {
    ADMUX = (1 << REFS0) | (POT_ANALOG_CH & 0x0F);
    ADCSRA = (1 << ADEN) | (1<<ADPS2) | (1<<ADPS1);
}
uint16_t ADC_read_channel(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); 
    return ADC; 
}

void TWI_init(void) {
    TWSR = 0x00;    
    TWBR = 72;
    TWCR = (1<<TWEN);  
}
uint8_t TWI_start_write(uint8_t addr_write) {
    /* send START */
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);
    while (!(TWCR & (1<<TWINT)));

    TWDR = (addr_write & 0xFF);
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWSR & 0xF8;
}
void TWI_write_byte(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}
void TWI_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}


#define LCD_PCF_ADDR (LCD_ADDR << 1) 

#define LCD_BL  (1<<4)
#define LCD_EN  (1<<5)
#define LCD_RW  (1<<6)
#define LCD_RS  (1<<7)

void lcd_write4(uint8_t nibble, uint8_t control) {
    uint8_t data = (nibble & 0xF0) | control; 
    /* EN=1 */
    TWI_start_write(LCD_PCF_ADDR);
    TWI_write_byte(data | LCD_EN);
    TWI_stop();
    _delay_us(1);
    /* EN=0 */
    TWI_start_write(LCD_PCF_ADDR);
    TWI_write_byte(data & ~LCD_EN);
    TWI_stop();
    _delay_us(40);
}

void lcd_send_byte(uint8_t value, uint8_t mode) {
    uint8_t control = LCD_BL;
    if (mode) control |= LCD_RS; /* data mode */
    /* send high nibble first (value & 0xF0) */
    lcd_write4(value & 0xF0, control);
    /* then low nibble (shifted to high) */
    lcd_write4((value << 4) & 0xF0, control);
}

void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
}
void lcd_data(uint8_t data) {
    lcd_send_byte(data, 1);
}

void lcd_init(void) {
    TWI_init();
    _delay_ms(50);
    lcd_write4(0x30, LCD_BL); _delay_ms(5);
    lcd_write4(0x30, LCD_BL); _delay_ms(5);
    lcd_write4(0x30, LCD_BL); _delay_ms(2);
    lcd_write4(0x20, LCD_BL); _delay_ms(2);

    lcd_cmd(0x28);
    lcd_cmd(0x08); 
    lcd_cmd(0x01); _delay_ms(2);
    lcd_cmd(0x06); 
    lcd_cmd(0x0C);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + row_offsets[row]));
}

void lcd_print_str(const char *s) {
    while (*s) {
        lcd_data((uint8_t)*s++);
    }
}

void lcd_print_float(float val, uint8_t prec) {
    char buf[16];
    dtostrf(val, 0, prec, buf);
    lcd_print_str(buf);
}

long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ISR(TIMER0_COMPA_vect) {
    count_ADC++;
    if (count_ADC >= 1000) {
        count_ADC = 0;
        display_flag = true;
    }
}

ISR(TIMER1_OVF_vect) {
    x++;
}


ISR(TIMER1_CAPT_vect) {
    if (!capture_flag) {
        n1 = ICR1;
        x = 0;
        capture_flag = true;
    } else {
        n2 = ICR1;
        capture_flag = false;
        calculate_flag = true;
    }


    if (ignition_state == 0) {
        uint32_t current_N_delay = N_delay;

        if (current_N_delay == 0) {
            IGNITION_PORT &= ~(1 << IGNITION_BIT); 
            ignition_state = 2;
            TCNT2 = 0;
            OCR2A = 49; 
            TCCR2B = (1 << CS22); 
        } else {
            if (current_N_delay > 255) {
                delay_ovf_count = current_N_delay / 256;
                delay_final_ticks = current_N_delay % 256;
            } else {
                delay_ovf_count = 0;
                delay_final_ticks = current_N_delay;
            }
            ignition_state = 1;
            TCNT2 = 0;
            if (delay_ovf_count > 0) {
                OCR2A = 255;
                waiting_for_ovf = true;
            } else {
                OCR2A = delay_final_ticks - 1;
                waiting_for_ovf = false;
            }
            TCCR2B = (1 << CS22);
        }
    }
}

ISR(TIMER2_COMPA_vect) {
    static uint16_t current_ovf = 0;

    if (ignition_state == 1) {
        if (waiting_for_ovf) {
            if (current_ovf < delay_ovf_count) {
                current_ovf++;
            } else {
                waiting_for_ovf = false;
                current_ovf = 0;
                OCR2A = delay_final_ticks - 1;
                TCNT2 = 0;
            }
        } else {

            IGNITION_PORT &= ~(1 << IGNITION_BIT); 
            ignition_state = 2;
            TCNT2 = 0;
            OCR2A = 49; 
        }
    } else if (ignition_state == 2) {
        TCCR2B = 0; /* stop Timer2 */
        IGNITION_PORT |= (1 << IGNITION_BIT);
        ignition_state = 0;
        current_ovf = 0;
        waiting_for_ovf = false;
    }
}

void peripherals_setup(void) {
    cli();
    CRANKSHAFT_DDR &= ~(1 << CRANKSHAFT_BIT);

    IGNITION_DDR |= (1 << IGNITION_BIT);
    IGNITION_PORT |= (1 << IGNITION_BIT);

    ADC_init();

    TCCR0A = (1 << WGM01); 
    TCCR0B = (1 << CS01) | (1 << CS00); 
    OCR0A = 249; 
    TIMSK0 = (1 << OCIE0A);

    TCCR1A = 0x00;
    TCCR1B = (1 << CS11) | (1 << CS10);
    TCCR1B &= ~(1 << ICES1);
    TCNT1 = 0;
    TIMSK1 = (1 << TOIE1) | (1 << ICIE1); 


    TCCR2A = (1 << WGM21); 
    TCCR2B = 0x00;
    TIMSK2 = (1 << OCIE2A); 

    lcd_init();

    uart_init(115200);

    sei();
}

int main(void) {
    peripherals_setup();

    while (1) {
        /* Read POT via ADC (A1) */
        POT_IGNITION = ADC_read_channel(POT_ANALOG_CH);
        THETA_IGNITION = (int)map_val(POT_IGNITION, 0, 1023, 10, 45);
        THETA_DELAY = THETA_0 - THETA_IGNITION;

        if (calculate_flag) {
            calculate_flag = false;

            uint32_t local_delta_N = (uint32_t)x * 65536UL + (uint32_t)n2 - (uint32_t)n1;
            delta_N = local_delta_N;
            if (delta_N == 0) delta_N = 1;
            f = (float)(F_CPU / (prescaler * (float)delta_N));
            Speed = 60.0 * f;
            T_cycle = 1.0 / f;
            prev_delta_N = delta_N;
        }

        if (prev_delta_N > 0) {
            T_delay = (THETA_DELAY / 360.0) * T_cycle;
            float ticks_per_sec = (float)F_CPU / (float)prescaler;
            uint32_t localN = (uint32_t)(T_delay * ticks_per_sec + 0.5f);
            N_delay = localN;
        } else {
            N_delay = 0;
        }

        if (display_flag) {
            display_flag = false;
            lcd_cmd(0x01); _delay_ms(2);
            lcd_set_cursor(0, 0);
            lcd_print_str("Freq:");
            char buf[16];
            sprintf(buf, "f", f);
            lcd_set_cursor(5, 0);
            lcd_print_str(buf);
            lcd_set_cursor(8,0);
            lcd_print_str(" Hz");
            lcd_set_cursor(0,1);
            lcd_print_str("Speed:");
            sprintf(buf, "", Speed);
            lcd_set_cursor(6,1);
            lcd_print_str(buf);
            lcd_set_cursor(12,1);
            lcd_print_str(" RPM");

            /* Optional UART debug */
            uart_printf("f=%.2f Hz, Speed=%.2f RPM\r\n", f, Speed);
        }
    }
    return 0;
}
