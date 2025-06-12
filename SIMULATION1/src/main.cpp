#include <Arduino.h>
#define SPEED_PIN 8
#define TDC_PIN 10
#define POT_PIN A0
#define PRESCALER 256
const int ADC_MIN = 102;  // 0.5V
const int ADC_MAX = 1023; // 5.0V
const int MIN_RPM = 150;
const int MAX_RPM = 15000;
volatile uint16_t count_ADC; 
volatile uint16_t count = 0;
volatile uint16_t times = 0;
volatile float TIMER_START;
volatile float TICKS_COUNT;
volatile float pot_value;
volatile float duty_value;
volatile float voltage;
volatile float period_signal;  
volatile float T_on;
volatile float T_off;
volatile float T_rising;
volatile float rpm = 0;
volatile uint16_t overflow1 = 0;
volatile bool adc_flag = false;
volatile bool display_flag = false;

ISR (TIMER0_COMPA_vect)
{
    count_ADC++;
    if(count_ADC == 10)
    {
        int times_2 = 10;
        int overflow2 = 10;
        times += times_2;
        overflow1 += overflow2;
        count++;
        adc_flag = true;
        count_ADC = 0;
        display_flag = true;
    }
}

ISR (TIMER1_COMPA_vect) {
    digitalWrite(SPEED_PIN, HIGH);
}

ISR (TIMER1_COMPB_vect) {
    digitalWrite(SPEED_PIN, LOW);
}

ISR (TIMER1_OVF_vect) 
{
    TCNT1 = TIMER_START;  
    digitalWrite(TDC_PIN, HIGH); // TDC lên HIGH tại 0 độ
    TCNT2 = 0;                   // Reset Timer2
    TCCR2B = (1 << CS22) | (1 << CS21);        // Bật Timer2 với prescaler 256
}

ISR (TIMER2_COMPA_vect)
{
    digitalWrite(TDC_PIN, LOW);  // TDC xuống LOW sau 100 µs
    TCCR2B = 0;                  // Tắt Timer2
}

long calculateCounter(long n) {
    return (uint32_t) (60 * F_CPU) / (PRESCALER * n);
}

void setup() {
    Serial.begin(9600);
    pinMode(SPEED_PIN, OUTPUT);   
    pinMode(TDC_PIN, OUTPUT);    
    digitalWrite(TDC_PIN, LOW);
    pinMode(POT_PIN, INPUT); 
    cli();
    // Setup Timer0
    TCNT0 = 0;
    TCCR0A = (1 << WGM01); 
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
    // Setup Timer1
    TCCR1A = 0;
    TCCR1B = (1 << CS12);  
    TCNT1 = TIMER_START;  
    OCR1A = 0; 
    OCR1B = 0; 
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1);
    // Setup Timer2
    TCCR2A = 0;        
    TCCR2B = 0;            
    TCNT2 = 0;
    OCR2A = 124;            
    TIMSK2 = (1 << OCIE2A); // Bật ngắt so sánh OCR2A
    sei();  
}

void loop() 
{
    if(adc_flag == true)
    {
        adc_flag = false;
        pot_value = analogRead(POT_PIN);
        voltage = (pot_value * 5.0) / 1023.0;
        if(pot_value < ADC_MIN){
            rpm = 0;
        }
        else{
            rpm = map(pot_value, ADC_MIN, ADC_MAX, MIN_RPM, MAX_RPM);
        }
        if(rpm > 0){
            period_signal = (60.0 / rpm) * 1000000.0;
            T_on = period_signal * (5.0 / 360.0); 
            T_rising = period_signal * (300.0 / 360.0);
            T_off = period_signal - T_on - T_rising;
            TICKS_COUNT = calculateCounter(rpm);
            TIMER_START = 65535 - TICKS_COUNT;
            uint32_t rising_ticks = (uint32_t) (TIMER_START + 300 * TICKS_COUNT / 360);
            uint32_t falling_ticks = (uint32_t) (TIMER_START + 305 * TICKS_COUNT / 360) ;
            noInterrupts();
            OCR1A = rising_ticks;
            OCR1B = falling_ticks;
            interrupts(); 
        }
        else{
            T_on = 0;
            T_off = 0;
            T_rising = 0;
            OCR1A = 0;
            OCR1B = 0;
        }
    }
    if(display_flag == true)
    {
        display_flag = false;
        Serial.print("Number of read ADC value after "); Serial.print(times);
        Serial.print(" ms: "); Serial.println(count);
        Serial.print("Voltage: "); Serial.print(voltage);
        if(rpm == 0){
            digitalWrite(SPEED_PIN, LOW);
            Serial.println(" V | Engine stop | Speed: 0 RPM");
            Serial.println("-----------------------");
        }
        else{
            Serial.print(" | Speed: "); Serial.print(rpm);Serial.print(" rpm");
            Serial.print(" | T_cycle: "); Serial.print(period_signal); Serial.println(" us");
            Serial.println("------------------------");
        }
    }
}