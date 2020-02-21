#include <avr/interrupt.h>
#include "digitalWriteFast.h"
#include <EnableInterrupt.h>

#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
#define CLOCK_OUT 13
#define CLOCK_ON 4
#define CLOCK_FAST 5
#define CLOCK_RESET 6

#define OVF_IN 2
#define COMPA_IN 3

uint8_t counter;
volatile boolean overflow_signal=false;
volatile boolean compa_signal=false;

void ovf_in() {
  overflow_signal=true;
}

void compa_in() {
  compa_signal=true;
}

uint8_t current_millis=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  pinModeFast(CLOCK_OUT, OUTPUT);
  digitalWriteFast(CLOCK_OUT, LOW);
  pinMode(CLOCK_ON, INPUT_PULLUP);
  pinMode(CLOCK_FAST, INPUT_PULLUP);
  pinMode(CLOCK_RESET, INPUT_PULLUP);
  pinMode(CLOCK_FAST, INPUT);
  pinMode(CLOCK_RESET, INPUT);
  enableInterrupt(OVF_IN, ovf_in, RISING);
  enableInterrupt(COMPA_IN, compa_in, RISING);
  Serial.println("Ready");
  current_millis=TCNT0;
}

uint32_t delta_millis=0;
uint32_t last_event_millis=0;
boolean clock_on = false;
boolean clock_fast = false;
boolean clock_reset = false;

// These mark how often repeats will take place
const uint8_t HIGH_DURATION 10;
const uint8_t LOW_DURATION 290;
const uint8_t FAST_HIGH_DURATION 10;
const uint8_t FAST_LOW_DURATION 40;
const uint8_t RESET_DURATION 500;

const uint8_t CLOCK_HIGH=0;
const uint8_t CLOCK_LOW=1;
const uint8_t CLOCK_FAST_HIGH=2;
const uint8_t CLOCK_FAST_LOW=3;
const uint8_t CLOCK_RESET_TIME=4;

uint16_t events_ms = { 0, 0, 0, 0, 0 };

void update_events() {
  uint8_t ms; // use a register. Using the array would mean repeated memory reads.
  current_millis=TCNT0;

  if (clock_on) {
    ms = events_ms[CLOCK_HIGH]
    if (ms == 0) {
      digitalWriteFast(CLOCK_OUT, HIGH);
      ms++;
    }
    if (ms > HIGH_DURATION) {
      digitalWriteFast(CLOCK_OUT, LOW);
      ms++
    }
    if (ms > LOW_DURATION) {
      counter++;
      Serial.print("Clock pulse: ");
      Serial.println(counter);
      events_ms[CLOCK_HIGH] = 0;
      clock_on = false;
      return;
    }
    events_ms[CLOCK_HIGH] = ms;
  }
  if (clock_fast) {
    ms = events_ms[CLOCK_FAST_HIGH];

    if (ms == 0) {
      digitalWriteFast(CLOCK_OUT, HIGH);
      ms++;
    }
    if (ms > FAST_HIGH_DURATION) {
      digitalWriteFast(CLOCK_OUT, LOW);
      ms++;
    }
    if (ms > FAST_LOW_DURATION) {
      counter++;
      Serial.print("Clock pulse: ");
      Serial.println(counter);
      events_ms[CLOCK_FAST_HIGH] = 0;
      clock_fast = false;
      return;
    }
    events_ms[CLOCK_FAST_HIGH] = ms;
  }
  if (clock_reset) {
    ms = events_ms[CLOCK_RESET_TIME];
    if (ms > RESET_DURATION) {
      counter = 0;
      Serial.println("Reset counter");
      events_ms[CLOCK_RESET_TIME] = 0;
      clock_reset = false;
      return;
    }
    ms++;
    events_ms[CLOCK_RESET_TIME] = ms;
  }
}

void loop() {
  if (TCNT0 != current_millis) update_events();
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
