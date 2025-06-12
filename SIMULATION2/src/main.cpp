#include <Arduino.h>
#define CRANKSHAFT_SENSOR_PIN 8
#define IGNITION_PIN 9
#define POT_VALUE A1
int THETA_0 = 60;
const uint16_t IGNITION_ON = 400; // Thời gian mức HIGH: 200µs
volatile uint16_t n1 = 0, n2 = 0, x = 0;
volatile bool capture_flag = false;
volatile bool display_flag = false;
volatile bool calculate_flag = false;
volatile bool waiting_for_ovf = false; // Cờ chờ overflow
volatile float Speed = 0.0, f = 0.0;
int POT_IGNITION, THETA_IGNITION, THETA_DELAY;
volatile uint16_t count_ADC;
int prescaler = 64;
volatile float T_cycle, T_delay;
volatile uint16_t delta_N = 0;
volatile uint16_t prev_delta_N = 0;
volatile uint32_t N_delay = 0; // Để chứa giá trị lớn
// Biến mới cho state machine
volatile uint8_t ignition_state = 0; // 0: IDLE, 1: WAITING_DELAY, 2: GENERATING_PULSE
volatile uint16_t delay_ovf_count = 0; // Số lần tràn cho N_delay
volatile uint16_t delay_final_ticks = 0; // Số ticks còn lại cho N_delay


ISR(TIMER0_COMPA_vect) {
    count_ADC++;
    if (count_ADC == 100) {
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

    // Kiểm tra trạng thái để kích hoạt delay và xung
    if (ignition_state == 0) { // IDLE
        uint32_t current_N_delay = N_delay; // Lấy N_delay hiện tại từ loop

        if (current_N_delay == 0) {
            // Nếu N_delay = 0, tạo xung ngay lập tức
            digitalWrite(IGNITION_PIN, HIGH);
            ignition_state = 2; // GENERATING_PULSE
            TCNT2 = 0;
            OCR2A = 49; // 200µs = 50 ticks với prescaler 64 (tính từ 0 nên là 49)
            TCCR2B = (1 << CS22); // Bật Timer2, prescaler 64
        } else {
            // Tính số lần tràn và ticks cho N_delay
            if (current_N_delay > 255) {
                delay_ovf_count = current_N_delay / 256;
                delay_final_ticks = current_N_delay % 256;
            } else {
                delay_ovf_count = 0;
                delay_final_ticks = current_N_delay;
            }

            ignition_state = 1; // WAITING_DELAY
            TCNT2 = 0;
            if (delay_ovf_count > 0) {
                OCR2A = 255; // Đếm đến 255 cho các lần tràn
                waiting_for_ovf = true;
            } else {
                OCR2A = delay_final_ticks - 1; // Đếm ticks còn lại (từ 0 nên trừ 1)
                waiting_for_ovf = false;
            }
            TCCR2B = (1 << CS22); // Bật Timer2
        }
    }
}

// Ngắt Timer2 để điều khiển delay và xung
ISR(TIMER2_COMPA_vect) {
    static uint16_t current_ovf = 0;

    if (ignition_state == 1) { // WAITING_DELAY
        if (waiting_for_ovf) {
            if (current_ovf < delay_ovf_count) {
                current_ovf++; // Đếm số lần tràn
            } else {
                waiting_for_ovf = false;
                current_ovf = 0;
                OCR2A = delay_final_ticks - 1; // Đặt số ticks còn lại
                TCNT2 = 0;
            }
        } else {
            // Hết N_delay, bắt đầu tạo xung
            digitalWrite(IGNITION_PIN, HIGH);
            ignition_state = 2; // GENERATING_PULSE
            TCNT2 = 0;
            OCR2A = 49; // 200µs = 50 ticks
        }
    } else if (ignition_state == 2) { // GENERATING_PULSE
        TCCR2B = 0; // Dừng Timer2
        digitalWrite(IGNITION_PIN, LOW);
        ignition_state = 0; // Quay lại IDLE
        current_ovf = 0;
        waiting_for_ovf = false;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(CRANKSHAFT_SENSOR_PIN, INPUT);
    pinMode(IGNITION_PIN, OUTPUT);
    pinMode(POT_VALUE, INPUT);
    digitalWrite(IGNITION_PIN, LOW);

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
    THETA_IGNITION = map(POT_IGNITION, 0, 1023, 0, 60);
    THETA_DELAY = THETA_0 - THETA_IGNITION;

    if (calculate_flag) {
        calculate_flag = 0;
        delta_N = (x * 65536UL + n2 - n1);
        f = (float)(250000 / (float)delta_N);
        Speed = 60.0 * f;
        T_cycle = 1.0 / f;
        prev_delta_N = delta_N;
    }

    // Tính N_delay
    if (prev_delta_N > 0) {
        T_delay = (THETA_DELAY / 360.0) * T_cycle;
        N_delay = (uint32_t)(T_delay * 250000);
        // Hoặc đơn giản hơn: N_delay = (uint32_t)((THETA_DELAY / 360.0) * prev_delta_N);
    }

    // Hiển thị thông tin
    if (display_flag) {
        display_flag = 0;
        Serial.print("IGNITION_ANGLE: "); Serial.print(THETA_IGNITION);
        Serial.print(" | DELAY: "); Serial.print(THETA_DELAY);
        Serial.print(" | Frequency: "); Serial.print(f);Serial.print(" Hz");
        Serial.print(" | Speed: "); Serial.print(Speed); Serial.println(" RPM");
        Serial.println("----------------------");
    }
}