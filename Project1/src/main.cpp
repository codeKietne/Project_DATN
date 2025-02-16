#include <Arduino.h>
#define SIGNAL_PIN 8   
#define CONTROL_PIN 10
#define POT_PIN A0     
const float CLK_FREQ = 16000000.0; 
const int PRESCALER = 8; 
const int PRESCALER_T0 = 64;          
const int MIN_RPM = 150;       
const int MAX_RPM = 15000; 
int count_ADC; 
volatile float period_signal;  
volatile float pot_value;
volatile float rpm;
volatile float T_on;
volatile float T_off;
volatile float T_rising;
volatile bool signal_state = LOW;
volatile bool control_state = LOW;
volatile bool adc_flag = false;
volatile bool display_flag = false;

ISR (TIMER0_COMPA_vect)
{
    count_ADC++;
    if(count_ADC >= 80 )
  {
    adc_flag = true ;
    count_ADC = 0;
  }
}
ISR(TIMER1_COMPA_vect) 
{
    signal_state = !signal_state;  // Đảo trạng thái xung
    digitalWrite(SIGNAL_PIN, signal_state);
    if (signal_state) 
    {
      OCR1A = (CLK_FREQ / PRESCALER) * (T_on / 1000000.0) - 1;  // Khi HIGH, chờ đúng Ton
      control_state = LOW;
    } 
    else 
    {
      OCR1A = (CLK_FREQ / PRESCALER) * (T_off / 1000000.0) - 1;  // Khi LOW, chờ đúng Toff
      control_state = HIGH;
    }
    digitalWrite(CONTROL_PIN, control_state);
}

void setup() 
{
    Serial.begin(9600);
    pinMode(SIGNAL_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(CONTROL_PIN, OUTPUT);
    cli();
    //Setup Timer0
    TCNT0 = 0;
    TCCR0A = 0;
    TCCR0B = 0;
    OCR0A = 0;
    TIMSK0 = 0;

    TCCR0A = (1 << WGM01); 
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A =  ((uint8_t)CLK_FREQ / PRESCALER_T0 / 20) - 1; // 50ms = 1/20 Hz
    TIMSK0 = (1 << OCIE0A); 
    //Setup Timer1
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
    TIMSK1 = 0;

    TCCR1B = (1 << WGM12) | (1 << CS11);
    TIMSK1 = (1 << OCIE1A);
    sei(); 
}

void loop() 
{
  if(adc_flag == true)
  {
    adc_flag = false;
    pot_value = analogRead(POT_PIN);
    rpm = map(pot_value, 0, 1023, MIN_RPM, MAX_RPM);
    
    period_signal = (60.0 / rpm) * 1000000.0;
    T_on = period_signal * (5.0 / 360.0); 
    T_rising = period_signal * (300.0 / 360.0);
    T_off = period_signal - T_on - T_rising;
    OCR1A = (unsigned int) ((uint32_t)CLK_FREQ / PRESCALER) * (T_rising / 1000000.0) - 1;
    display_flag = true;
  }

  if(display_flag == true)
  {
    display_flag = false;
    Serial.print("Speed: ");
    Serial.print(rpm);
    Serial.print(" RPM | Ton: ");
    Serial.print(T_on);
    Serial.print(" us | Toff: ");
    Serial.print(T_off);
    Serial.print(" us | T_rising: ");
    Serial.println(T_rising);
  }
}


