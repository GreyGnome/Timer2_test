#include <avr/interrupt.h>
#include "digitalWriteFast.h"
#include <EnableInterrupt.h>
#include <wiring_private.h>

#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
#define CLOCK_OUT 12
#define CLOCK_ON 4
#define CLOCK_FAST 7
#define CLOCK_RESET 8

#define OVF_IN 2
#define COMPA_IN 3

//     RESET   (D 22)  PC6  1|    |28  PC5  (D 19  A5  SCL ADC5)
//             (D  0)  PD0  2|    |27  PC4  (D 18  A4  SDA ADC4)
//             (D  1)  PD1  3|    |26  PC3  (D 17  A3  ADC3)
//       INT0  (D  2)  PD2  4|    |25  PC2  (D 16  A2  ADC2)
//  OC2B INT1  (D  3)  PD3  5|    |24  PC1  (D 15  A1  ADC1)
//             (D  4)  PD4  6|    |23  PC0  (D 14  A0  ADC0)
//                     VCC  7|    |22  GND
//                     GND  8|    |21  AREF
//      TOSC1  (D 20)  PB6  9|    |20  AVCC
//             (D 21)  PB7 10|    |19  PB5  (D 13  SCK) 
//       OC0B  (D  5)  PD5 11|    |18  PB4  (D 12  MISO) 
//  AIN0 OC0A  (D  6)  PD6 12|x   |17  PB3  (D 11  MOSI OC2A)
//  AIN1       (D  7)  PD7 13|clk |16  PB2  (D 10  SS OC1A)
//             (D  8)  PB0 14|    |15  PB1  (D  9     OC1B)
//                           +----+

uint8_t counter;
volatile boolean overflow_signal=false;
volatile boolean compa_signal=false;

void ovf_in() {
  overflow_signal=true;
  counter=0;
}

void compa_in() {
  compa_signal=true;
}

volatile uint8_t my_millis=0;
ISR(TIMER0_COMPB_vect) {
  my_millis++;
}

uint8_t current_millis=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  pinMode(CLOCK_OUT, OUTPUT);
  digitalWriteFast(CLOCK_OUT, LOW);
  pinMode(CLOCK_ON, INPUT_PULLUP);
  pinMode(CLOCK_FAST, INPUT_PULLUP);
  pinMode(CLOCK_RESET, INPUT_PULLUP);
  pinMode(5, OUTPUT); // == OC0B
  enableInterrupt(OVF_IN, ovf_in, RISING);
  enableInterrupt(COMPA_IN, compa_in, RISING);
  Serial.println("Ready");
  // Utilize the timer0, which is already running for millis().
  OCR0B=0x00;  // 0xFF is constant high. 0x00 shows a single pulse.
               // This takes place every 1 ms as measured on a scope, so using
               // the compare interrupt as ms timer is good.
  sbi(TCCR0A, COM0B1); // Turn on OC0B (Arduino pin 5)
  sbi(TIMSK0, OCIE0B);
}

uint32_t delta_millis=0;
uint32_t last_event_millis=0;
boolean clock_on = false;
boolean clock_fast = false;
boolean clock_reset = false;

// These mark how often repeats will take place
const uint16_t HIGH_DURATION = 10;
const uint16_t LOW_DURATION = 390;
const uint16_t FAST_HIGH_DURATION = 10;
const uint16_t FAST_LOW_DURATION = 40;
const uint16_t RESET_DURATION = 500;

const uint8_t CLOCK_HIGH=0;
const uint8_t CLOCK_LOW=1;
const uint8_t CLOCK_FAST_HIGH=2;
const uint8_t CLOCK_FAST_LOW=3;
const uint8_t CLOCK_RESET_TIME=4;

uint16_t events_ms[] = { 0, 0, 0, 0, 0 };

void update_events() {
  uint16_t ms; // use register(s). Using the array would mean repeated memory reads.
               // (I'm actually not sure how it handles 16 bit values. But let's try.)
  current_millis=my_millis;

  if (clock_on) {
    ms = events_ms[CLOCK_HIGH];
    if (ms == 0) {
      digitalWriteFast(CLOCK_OUT, HIGH);
    }
    if (ms > HIGH_DURATION) {
      digitalWriteFast(CLOCK_OUT, LOW);
    }
    if (ms > LOW_DURATION) {
      counter++;
      Serial.print("Clock pulse: ");
      Serial.println(counter);
      ms = 0;
      clock_on = false;
      events_ms[CLOCK_HIGH] = ms;
      return;
    }
    ms++;
    events_ms[CLOCK_HIGH] = ms;
  }
  if (clock_fast) {
    ms = events_ms[CLOCK_FAST_HIGH];
    if (ms == 0) {
      digitalWriteFast(CLOCK_OUT, HIGH);
    }
    if (ms > FAST_HIGH_DURATION) {
      digitalWriteFast(CLOCK_OUT, LOW);
    }
    if (ms > FAST_LOW_DURATION) {
      counter++;
      Serial.print("Clock fast: ");
      Serial.println(counter);
      ms=0;
      events_ms[CLOCK_FAST_HIGH] = ms;
      clock_fast = false;
      return;
    }
    ms++;
    events_ms[CLOCK_FAST_HIGH] = ms;
  }
  if (clock_reset) {
    ms = events_ms[CLOCK_RESET_TIME];
    if (ms > RESET_DURATION) {
      counter = 0;
      Serial.println("Reset counter");
      ms = 0;
      clock_reset = false;
      events_ms[CLOCK_RESET_TIME] = ms;
      return;
    }
    ms++;
    events_ms[CLOCK_RESET_TIME] = ms;
  }
}

void loop() {
  if (overflow_signal) {
    Serial.println("Overflow!");
    overflow_signal=false;
  }
  if (compa_signal) {
    Serial.println("COMPA!");
    compa_signal=false;
  }
  if (my_millis != current_millis) update_events();
  if (! digitalReadFast(CLOCK_ON)) {
    if (! clock_on) { clock_on = true; digitalWriteFast(CLOCK_OUT, HIGH); }
  }
  if (! digitalReadFast(CLOCK_FAST)) {
    if (! clock_fast) { clock_fast = true; digitalWriteFast(CLOCK_OUT, HIGH); }
  }
  if (! digitalReadFast(CLOCK_RESET)) {
    if (! clock_reset) { clock_reset = true; }
  }
}
