#include <Arduino.h>
#define CRANKSHAFT_SENSOR_PIN 8
#define IGNITION_PIN 9
#define POT_VALUE A1
int THETA_0 = 45;
const uint16_t IGNITION_ON = 200; 
volatile uint16_t n1 = 0, n2 = 0, x = 0;
volatile bool capture_flag = false;
volatile bool display_flag = false;
volatile bool calculate_flag = false;
volatile bool waiting_for_ovf = false;
float Speed = 0.0;
int POT_IGNITION, THETA_IGNITION, THETA_DELAY;
float f;
volatile uint16_t count_ADC;
int prescaler = 64;
float T_cycle = 0.0, T_delay = 0.0;
volatile uint16_t delta_N = 0;
volatile uint16_t prev_delta_N = 0;
volatile uint32_t N_delay = 0; 
volatile uint8_t ignition_state = 0; 
volatile uint16_t delay_ovf_count = 0; 
volatile uint16_t delay_final_ticks = 0; 

ISR(TIMER0_COMPA_vect) {
    count_ADC++;
    if (count_ADC == 1000) {
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
            digitalWrite(IGNITION_PIN, LOW);
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
            digitalWrite(IGNITION_PIN, LOW);
            ignition_state = 2;
            TCNT2 = 0;
            OCR2A = 49; 
        }
    } else if (ignition_state == 2) { 
        TCCR2B = 0; 
        digitalWrite(IGNITION_PIN, HIGH); 
        ignition_state = 0; 
        current_ovf = 0;
        waiting_for_ovf = false;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(CRANKSHAFT_SENSOR_PIN, INPUT);
    pinMode(IGNITION_PIN, OUTPUT);
    pinMode(POT_VALUE, INPUT);
    digitalWrite(IGNITION_PIN, HIGH); 

    cli();
    TCNT0 = 0;
    TCCR0A = (1 << WGM01); 
    TCCR0B = (1 << CS01) | (1 << CS00); 
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);

   
    TCCR1A = 0;
    TCCR1B = (1 << CS11) | (1 << CS10); 
    TCCR1B &= ~(1 << ICES1); 
    TCNT1 = 0;
    TIMSK1 = (1 << TOIE1) | (1 << ICIE1);

   
    TCCR2A = (1 << WGM21); 
    TCCR2B = 0; 
    TIMSK2 = (1 << OCIE2A); 
    sei();
}

void loop() {
    POT_IGNITION = analogRead(POT_VALUE);
    THETA_IGNITION = map(POT_IGNITION, 0, 1023, 10, 45);
    THETA_DELAY = THETA_0 - THETA_IGNITION;

    if (calculate_flag) {
        calculate_flag = 0;
        delta_N = (x * 65536UL + n2 - n1);
        f = (uint16_t)(F_CPU / (prescaler * (float)delta_N));
        Speed = 60.0 * f;
        T_cycle = 1.0 / f;
        prev_delta_N = delta_N;
    }

 
    if (prev_delta_N > 0) {
        T_delay = (THETA_DELAY / 360.0) * T_cycle;
        N_delay = (uint32_t)(T_delay * (F_CPU / prescaler));
    }

  
  if (display_flag) {
        display_flag = 0;
        Serial.print("n1: "); Serial.print(n1);
        Serial.print(" | n2: "); Serial.print(n2);
        Serial.print(" | delay_final_ticks: "); Serial.print(delay_final_ticks);
        Serial.print(" | THETA_DELAY: "); Serial.print(THETA_DELAY);
        Serial.print(" | delta_N: "); Serial.print(delta_N);
        Serial.print(" | N_delay: "); Serial.print(N_delay);
        Serial.print(" | Speed: "); Serial.print(Speed); Serial.println(" RPM");
        Serial.println("----------------------");
    }
}