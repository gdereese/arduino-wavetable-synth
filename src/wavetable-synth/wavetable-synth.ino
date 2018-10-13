#include "wavetables.h"

// defines
#define SAMP_TIMER_INTR TIMER1_COMPA_vect
#define SIG_OUT_PINS PORTD
#define SIG_OUT_REG DDRD

// interface pins
const uint8_t PIN_FREQ_IN = A0;
const uint8_t PIN_READY_LED = 12;

// constants
const uint16_t FREQ_MIN = 20;
const uint16_t FREQ_MAX = 10000;
const uint16_t SAMPLE_FREQ = 40000; 

// global vars
uint16_t phase_inc;
uint16_t *wavetable;

void setup() {
  Serial.begin(115200);

  setup_sample_timer(SAMPLE_FREQ);
  setup_signal_output();
  setup_freq_input(PIN_FREQ_IN);
  setup_ready_led(PIN_READY_LED);

  set_waveform(3);

  write_ready_led(true);
}

void loop() {
  uint16_t freq = map(analogRead(PIN_FREQ_IN), 0, 1023, FREQ_MIN, FREQ_MAX);
  set_signal_freq(freq);
}

ISR(SAMP_TIMER_INTR) {
  uint16_t sample = calc_sample();

  write_sample(sample);
}

uint16_t calc_sample() {
  static uint16_t phase = 0;
  phase = inc_phase(phase, phase_inc, WAVETABLE_LEN - 1);

  uint16_t sample = pgm_read_word_near(wavetable + phase);

  return sample;
}

void set_signal_freq(uint16_t freq) {
  phase_inc = ((float)freq / SAMPLE_FREQ) * WAVETABLE_LEN;
}

void set_waveform(uint8_t waveform) {
  if (waveform == 1) {
    wavetable = SINE;
  } else if (waveform == 2) {
    wavetable = TRI;
  } else if (waveform == 3) {
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
  cli();

  const uint32_t base_clk_freq = 16000000;
  const uint8_t clk_prescaler = 1;

  // Timer 1 @ sample_freq Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = base_clk_freq / (sample_freq * clk_prescaler) - 1;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void setup_signal_output() {
  // setup all pins in register for output
  SIG_OUT_REG = 255;
}

void write_ready_led(bool isOn) {
  const uint8_t led_val = isOn ? HIGH : LOW;
  digitalWrite(PIN_READY_LED, led_val);
}

void write_sample(uint16_t sample) {
  // map 0-65535 -> 0-255 by dividing by 256 (right-shift 8 bits)
  SIG_OUT_PINS = sample >> 8;
}

uint16_t inc_phase(uint16_t curr_val, uint16_t step, uint16_t max_val) {
  if (curr_val + step > max_val) {
    return curr_val + step - max_val;
  } else {
    return curr_val + step;
  }
}
