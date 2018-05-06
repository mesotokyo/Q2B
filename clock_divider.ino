#include <MsTimer2.h>

// define pins - interrupt
#define CLOCK_RESET 2
#define CLOCK_IN    3

// define pins - external digital input
#define CH1_MUTE    11
#define CH2_MUTE    1
#define CH1_SIDE    6

// define pins - switch input
#define CH1_INV     0
#define CH2_INV     14
#define TRIG_MODE   5
#define TRIPLET     7
#define DOUBLE      8
#define HALF        9

// define pins - digital output
#define CH1_OUT     4
#define CH2_OUT     10
#define CLOCK_OUT   12
#define EOC_OUT     13

// define pins - analog input
#define CH1_DIV     A1
#define CH2_DIV     A2
#define CH1_ROT     A3
#define CH2_ROT     A4
#define BPM_GAUGE   A5

// note settings
#define NOTE_UNIT 32
#define RESOLUTION 24

// LED gate time
#define LED_ON_TIME 15.0

// define state valiables
volatile int clock_count = 0;
volatile unsigned long prev_tick = 0.0;
volatile int triplet_mode = 0;

/* BPM settings: 
 * a pulse interval is 60 / (BPM * DIVIDE_FACTOR).
 * when BPM == 480 and DIVIDE_FACTOR == 8,
 *  a pulse interval is 60/(480*8) [s] -> 60000/3840 [ms] -> 15.625 [ms]
 */
#define CLOCK_BPM_MIN 60.0
#define CLOCK_BPM_MAX 240.0
#define CLOCK_BPM_RANGE 180.0 /* (BPM_MAX - BPM_MIN) */

// first 10 degree and last 10 degree is headroom (rotation degrees is 300)
#define GAUGE_MARGINS (1023.0 * 3.0 / 300.0)

// setup pin mode
void init_pins() {
  // input pins with no-pullup
  pinMode(CLOCK_RESET, INPUT);
  pinMode(CLOCK_IN, INPUT);
  pinMode(CH1_MUTE, INPUT);
  pinMode(CH2_MUTE, INPUT);
  pinMode(CH1_SIDE, INPUT);

  // input pins with pullup
  pinMode(CH1_INV, INPUT_PULLUP);
  pinMode(CH2_INV, INPUT_PULLUP);
  pinMode(TRIG_MODE, INPUT_PULLUP);
  pinMode(TRIPLET, INPUT_PULLUP);
  pinMode(DOUBLE, INPUT_PULLUP);
  pinMode(HALF, INPUT_PULLUP);

  // output pins
  pinMode(CH1_OUT, OUTPUT);
  pinMode(CH2_OUT, OUTPUT);
  pinMode(CLOCK_OUT, OUTPUT);
  pinMode(EOC_OUT, OUTPUT);
  digitalWrite(CH1_OUT, HIGH);
  digitalWrite(CH2_OUT, HIGH);
  digitalWrite(CLOCK_OUT, HIGH);
  digitalWrite(EOC_OUT, HIGH);

  // analog pins
  pinMode(CH1_DIV, INPUT);
  pinMode(CH2_DIV, INPUT);
  pinMode(CH1_ROT, INPUT);
  pinMode(CH2_ROT, INPUT);
  pinMode(BPM_GAUGE, INPUT);
}

#define ON 1
#define OFF 0

void ch1_state(int state) {
  digitalWrite(CH1_OUT, !state);
}
void ch2_state(int state) {
  digitalWrite(CH2_OUT, !state);
}
void eoc_state(int state) {
  digitalWrite(EOC_OUT, !state);
}
void clock_state(int state) {
  digitalWrite(CLOCK_OUT, !state);
}

void all_output_off() {
  ch1_state(OFF);
  ch2_state(OFF);
  clock_state(OFF);
  eoc_state(OFF);
}

// interrupt handler for clock_in
void on_clock_in() {
  clock_count++;
  clock_state(ON);
  update_state();
}

// interrupt handler for clock_reset
void on_clock_reset() {
  clock_count = 0;
  clock_state(ON);
  update_state();
}

void update_clock() {
  int gauge = analogRead(BPM_GAUGE);

  // calculate BPM
  if (gauge < GAUGE_MARGINS) {
    // internal clock is disabled
    return;
  }
  float regularized_gauge = 1.0;
  if (gauge <= (1023.0 - GAUGE_MARGINS)) {
    regularized_gauge = (gauge - GAUGE_MARGINS) / (1023.0 - 2.0 * GAUGE_MARGINS);
  }

  float bpm = CLOCK_BPM_MIN + regularized_gauge * CLOCK_BPM_RANGE;

  // 60 sec = 60000 msec
  float next_trigger = 60000.0 / (bpm * RESOLUTION);
  unsigned long current_t = millis();
  if (current_t - prev_tick < next_trigger) {
    return;
  }

  prev_tick = current_t;
  clock_count++;
  update_state();
}

void update_state() {
  // check clock cycle
  if (clock_count >= RESOLUTION * 4) {
    clock_count = 0;
    eoc_state(ON);
  }

  // check clock and tripletmode
  int triplet = (digitalRead(TRIPLET) == LOW);
  int clock_unit;
  if (triplet) {
    if (digitalRead(DOUBLE) == LOW) {
      clock_unit = 2;
    } else if (digitalRead(HALF) == LOW) {
      clock_unit = 8;
    } else {
      clock_unit = 4;
    }
  } else {
    if (digitalRead(DOUBLE) == LOW) {
      clock_unit = 3;
    } else if (digitalRead(HALF) == LOW) {
      clock_unit = 12;
    } else {
      clock_unit = 6;
    }
  }
  int shift_base = RESOLUTION * 4 / clock_unit;
  int on_base_clock = (clock_count % clock_unit == 0);
  if (on_base_clock) {
    clock_state(ON);
  }

  // get ch.1/2's divides
  // convert 1024 step value to 6 step (0 to 5) value
  int ch1_div = analogRead(CH1_DIV) / 171;
  int ch2_div = analogRead(CH2_DIV) / 171;
  int ch1_unit, ch2_unit;
  if (0) {
    ch1_unit = (RESOLUTION * 4) / (int)(pow(3, ch1_div));
    ch2_unit = (RESOLUTION * 4) / (int)(pow(3, ch2_div));
  } else {
    ch1_unit = (RESOLUTION * 4) >> ch1_div;
    ch2_unit = (RESOLUTION * 4) >> ch2_div;
  }

  // get ch.1/2's rotates
  // convert 1024 step value to RESOLUTION step value
  int ch1_shift, ch2_shift;
  if (0) {
    ch1_shift = (shift_base / int(pow(3, ch1_div))) * analogRead(CH1_ROT) / 1023;
    ch2_shift = (shift_base / int(pow(3, ch2_div))) * analogRead(CH2_ROT) / 1023;
  } else {
    ch1_shift = (shift_base >> ch1_div) * analogRead(CH1_ROT) / 1024;
    ch2_shift = (shift_base >> ch2_div) * analogRead(CH2_ROT) / 1024;
  }

  int ch1_mute = digitalRead(CH1_MUTE);
  int ch2_mute = digitalRead(CH2_MUTE);

  if (digitalRead(CH1_INV) == HIGH) {
    // Normal mode
    if (clock_count % ch1_unit == ch1_shift * clock_unit
        && !ch1_mute) {
      ch1_state(ON);
    }
  } else {
    // Invert mode
    if (on_base_clock
        && !ch1_mute
        && clock_count % ch1_unit != ch1_shift * clock_unit) {
      ch1_state(ON);
    }
  }

  if (digitalRead(CH2_INV) == HIGH) {
    // Normal mode
    if ((clock_count % ch2_unit == ch2_shift * clock_unit
         || digitalRead(CH1_SIDE))
        && !ch2_mute) {
      ch2_state(ON);
    }
  } else {
    // Invert mode
    if (on_base_clock
        && !ch2_mute
        && (clock_count % ch2_unit != ch2_shift * clock_unit
            || digitalRead(CH1_SIDE))) {
      ch2_state(ON);
    }
  }
  MsTimer2::start();
}

void on_timer2() {
  all_output_off();
  MsTimer2::stop();
}

void setup() {
  // setup pin mode
  init_pins();

  // setup interrupt for CLOCK_IN and CLOCK_RESET
  // on TRIGGER mode, use RISING trigger
  attachInterrupt(digitalPinToInterrupt(CLOCK_IN), on_clock_in, RISING);
  attachInterrupt(digitalPinToInterrupt(CLOCK_RESET), on_clock_reset, RISING);

  /* start timer */
  prev_tick = millis();
  MsTimer2::set(LED_ON_TIME, on_timer2);
}

void loop() {
  // put your main code here, to run repeatedly:
  update_clock();
}

