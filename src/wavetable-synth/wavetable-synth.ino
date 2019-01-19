#include <TimerOne.h>

#include "phase-inc-table.h"
#include "wavetables.h"

// interface pins
const uint8_t PIN_FREQ_IN = A0;
const uint8_t PIN_READY_LED = 12;
const uint8_t PIN_SIGNAL_OUT = 9;

// constants
const uint16_t CPU_FREQ = 1000000;
const uint16_t SAMPLE_FREQ = 40000; 

// global vars
uint16_t phase_inc;
uint16_t *wavetable;

void setup() {
  Serial.begin(115200);

  setup_sample_timer(SAMPLE_FREQ);
  setup_signal_output(PIN_SIGNAL_OUT);
  setup_freq_input(PIN_FREQ_IN);
  setup_ready_led(PIN_READY_LED);

  set_output_waveform(3);

  write_ready_led(true);
}

void loop() {
  uint16_t freq_in_val = analogRead(PIN_FREQ_IN);
  set_signal_freq(freq_in_val);
}

uint16_t calc_sample() {
  // increment the phase accumulator appropriately based on the wavetable length (number of samples)
  static uint16_t phase = 0;
  phase = inc_phase(phase, phase_inc, WAVETABLE_LEN - 1);

  // read a sample from the wavetable using the current phase accumulator value
  uint16_t sample = pgm_read_word_near(wavetable + phase);

  return sample;
}

void on_sample_tick() {
  uint16_t sample = calc_sample();

  write_sample(sample);
}

void set_signal_freq(uint16_t freq_in_val) {
  phase_inc = pgm_read_word_near(PHASE_INC + freq_in_val);
}

void set_output_waveform(uint8_t waveform_val) {
  if (waveform_val == 1) {
    wavetable = SINE;
  } else if (waveform_val == 2) {
    wavetable = TRI;
  } else if (waveform_val == 3) {
    wavetable = SAW;
  }
}

void setup_freq_input(uint8_t pin) {
  pinMode(pin, OUTPUT);
}

void setup_ready_led(uint8_t pin) {
  pinMode(pin, OUTPUT);
}

void setup_sample_timer(uint16_t sample_freq) {
  uint16_t sample_period = (1.0 / sample_freq) * CPU_FREQ;
  Timer1.initialize(sample_period);
  Timer1.attachInterrupt(on_sample_tick);
}

void setup_signal_output(uint8_t pin) {
  // enable PWM on output pin
  Timer1.pwm(pin, 0);
}

void write_ready_led(bool isOn) {
  const uint8_t led_val = isOn ? HIGH : LOW;
  digitalWrite(PIN_READY_LED, led_val);
}

void write_sample(uint16_t sample) {
  // map 0-65535 -> 0-1023 by dividing by 64 (right-shift 6 bits)
  uint16_t duty_val = sample >> 6;
  Timer1.setPwmDuty(PIN_SIGNAL_OUT, duty_val);
}

uint16_t inc_phase(uint16_t curr_val, uint16_t step, uint16_t max_val) {
  // increment value, wrapping around back to 0 if value exceeds max
  if (curr_val + step > max_val) {
    return curr_val + step - max_val;
  } else {
    return curr_val + step;
  }
}
