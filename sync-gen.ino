// atmega328p datasheet - https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

#include <avr/sleep.h>
#include <limits.h>

// comment out below for fake-progressive scanning
#define INTERLACED

// constants
#define CPU_TICKS_PER_US (F_CPU / 1000000)
#define USEC_PER_KHZ 1000

// macros
#define KHZ_TO_USEC(khz) (1 / khz * USEC_PER_KHZ)
#define USEC_TO_TICKS(us) ((us * CPU_TICKS_PER_US) - 1)
#define KHZ_TO_TICKS(khz) USEC_TO_TICKS(KHZ_TO_USEC(khz))

// ntsc standards
#define NTSC_SCAN_LINES_PER_FRAME 525
#define NTSC_SCAN_LINES_PER_FIELD 263
#define NTSC_HORIZONTAL_FREQ_KHZ 15.734264
#define NTSC_HORIZONTAL_PERIOD_US KHZ_TO_USEC(NTSC_HORIZONTAL_FREQ_KHZ) // 63.5555625608
#define NTSC_VERTICAL_FREQ_HZ 59.94
#define NTSC_HSYNC_PERIOD_USEC 4.7
#define NTSC_VSYNC_PERIOD_USEC ((NTSC_HORIZONTAL_PERIOD_US / 2) - NTSC_HSYNC_PERIOD_USEC)
#define NTSC_VSYNCEQ_PERIOD_USEC 2.3
#define NTSC_BACK_PORCH_PERIOD_USEC 4.7
#define NTSC_FRONT_PORCH_PERIOD_USEC 1.65
#define NTSC_ACTIVE_VIDEO_PERIOD_USEC (NTSC_HORIZONTAL_PERIOD_US - NTSC_HSYNC_PERIOD_USEC - NTSC_BACK_PORCH_PERIOD_USEC - NTSC_FRONT_PORCH_PERIOD_USEC)
#define NTSC_FRAMES_PER_SEC 29.97

#define NTSC_SCAN_LINE_PERIOD_TICKS USEC_TO_TICKS(NTSC_HORIZONTAL_PERIOD_US)
#define NTSC_HALF_SCAN_LINE_PERIOD_TICKS (NTSC_SCAN_LINE_PERIOD_TICKS / 2)
#define NTSC_HSYNC_PERIOD_TICKS USEC_TO_TICKS(NTSC_HSYNC_PERIOD_USEC)
#define NTSC_VSYNC_PERIOD_TICKS USEC_TO_TICKS(NTSC_VSYNC_PERIOD_USEC)
#define NTSC_VBLANK_FIELD_LINE_START 1
#define NTSC_VBLANK_FIELD_LINE_END 21
#define NTSC_VSYNC_FIELD_LINE_START 1
#define NTSC_VSYNC_FIELD_LINE_END 9
#define NTSC_ACTIVE_VIDEO_DELAY_TICKS 0
#define NTSC_ACTIVE_VIDEO_PERIOD_TICKS USEC_TO_TICKS(NTSC_ACTIVE_VIDEO_PERIOD_USEC)
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_START (NTSC_VBLANK_FIELD_LINE_END + 1)
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_END (NTSC_ACTIVE_VIDEO_FIELD_LINE_START + (NTSC_SCAN_LINES_PER_FIELD - NTSC_VBLANK_FIELD_LINE_END - NTSC_VBLANK_FIELD_LINE_START + 1))
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_MID (NTSC_ACTIVE_VIDEO_FIELD_LINE_START + ((NTSC_ACTIVE_VIDEO_FIELD_LINE_END - NTSC_ACTIVE_VIDEO_FIELD_LINE_START) / 2))

// pins
// pins 9 and 10 are used by timer 1. we use pin 9 - output of waveform generation mode.
#define PIN_CSYNC 9 // hsync or csync. see 'hsync_instead_of_csync' below. active low.
#define PIN_VSYNC 7 // vsync. active low, inactive high.
#define PIN_LUMA 6  // for testing.

// macros
#define VSYNC_INACTIVE PORTD |= _BV(PB7) // bitWrite(PORTD, PIN_VSYNC, 1)
#define VSYNC_ACTIVE PORTD &= ~_BV(PB7)  // bitWrite(PORTD, PIN_VSYNC, 0)
#define LUMA_HIGH PORTD |= _BV(PB6)      // bitWrite(PORTD, PIN_LUMA, 1)
#define LUMA_LOW PORTD &= ~_BV(PB6)      // bitWrite(PORTD, PIN_LUMA, 0)

// settings
volatile bool hsync_instead_of_csync = false; // this can be set by a digital input pin!

// sync gen state
volatile uint16_t scan_line = 1;
volatile uint16_t field = 1;
volatile uint16_t field_line = 1;
volatile bool is_half_line = false;
volatile bool is_active_video_line = false;

void setup()
{

  // is this neccessary?
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  // initiatize pins for output
  pinMode(PIN_CSYNC, OUTPUT);
  pinMode(PIN_VSYNC, OUTPUT);
  pinMode(PIN_LUMA, OUTPUT);

  // timers:

  // disable interrupts
  cli();

  // timer/counter 0 - start
  {
    // disable timer/counter 0

    TIMSK0 = 0; // timer/counter 0 - interrupt mask - this prevents jitter!
    TCCR0A = 0; // timer/counter 0 - control register a
    TCCR0B = 0; // timer/counter 0 - control register a
    OCR0A = 0;  // timer/counter 0 - compare-a interrupt
    OCR0B = 0;  // timer/counter 0 - compare-b interrupt
    TCNT0 = 0;  // timer/counter 0 - value
  }
  // timer/counter 0 - end

  // timer/counter 1 - start
  {
    // timer/counter 1 control register a - https://onlinedocs.microchip.com/oxy/GUID-80B1922D-872B-40C8-A8A5-0CBE009FD908-en-US-3/GUID-853E47EF-C46F-422D-AD77-A76D833D0760.html
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(WGM11);
    // timer/counter 1 control register b - https://onlinedocs.microchip.com/oxy/GUID-80B1922D-872B-40C8-A8A5-0CBE009FD908-en-US-3/GUID-07C6751D-0319-41A4-AC9F-9B0AFDA21A07.html
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

    // wave generation mode - https://onlinedocs.microchip.com/oxy/GUID-80B1922D-872B-40C8-A8A5-0CBE009FD908-en-US-3/GUID-853E47EF-C46F-422D-AD77-A76D833D0760.html#BITFIELD_HVM_1FS_MR__GUID-F8FE6078-0B34-4167-918B-8B1A67F372A5
    // - WGM13 and WGM12 and WGM11 -> fast pwm, TOP = ICR1, update of OCR1A @ BOTTOM
    // compare mode for fast pwm - https://onlinedocs.microchip.com/oxy/GUID-80B1922D-872B-40C8-A8A5-0CBE009FD908-en-US-3/GUID-853E47EF-C46F-422D-AD77-A76D833D0760.html#BITFIELD_K1J_2BS_MR__TABLE_H2S_CCS_MR
    // - COM1A1 and COM1A0 -> inverting mode
    // CS10 -> no prescaling

    ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;    // timer/counter 1 - overflow interrupt
    OCR1A = NTSC_HSYNC_PERIOD_TICKS;       // timer/counter 1 - compare-a interrupt
    OCR1B = NTSC_ACTIVE_VIDEO_DELAY_TICKS; // timer/counter 1 - compare-b interrupt
    TCNT1 = 0;                             // timer/counter 1 - value
    TIMSK1 = _BV(TOIE1) | _BV(OCIE1B);     // timer/counter 1 interrupts - enable timer 1 overflow (TOIE1) and compare-b (OCIE1B)
  }
  // timer/counter 1 - end

  // enable interrupts
  sei();
}

// timer/counter 1 overflow interrupt
ISR(TIMER1_OVF_vect)
{

  if (field_line >= NTSC_VSYNC_FIELD_LINE_START && field_line <= NTSC_VSYNC_FIELD_LINE_END)
  {
    VSYNC_ACTIVE;
  }
  else
  {
    VSYNC_INACTIVE;
  }

#ifdef INTERLACED

  if ((scan_line == NTSC_SCAN_LINES_PER_FIELD))
  {
    is_half_line = !is_half_line;
    ICR1 = NTSC_HALF_SCAN_LINE_PERIOD_TICKS;
  }
  else if (!hsync_instead_of_csync && (field_line >= NTSC_VSYNC_FIELD_LINE_START && field_line <= NTSC_VSYNC_FIELD_LINE_END))
  {
    is_half_line = !is_half_line;
    ICR1 = NTSC_HALF_SCAN_LINE_PERIOD_TICKS;
  }
  else
  {
    is_half_line = false;
    ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;
  }

  if (!is_half_line)
  {
    scan_line++;
  }

#else

  ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;

  if (scan_line > NTSC_SCAN_LINES_PER_FIELD)
  {
    scan_line = 1;
  }
  else
  {
    scan_line++;
  }

#endif

  if (scan_line > NTSC_SCAN_LINES_PER_FRAME)
  {
    scan_line = 1;
  }

  if (scan_line > NTSC_SCAN_LINES_PER_FIELD)
  {
    field = 2;
    field_line = scan_line - NTSC_SCAN_LINES_PER_FIELD;
  }
  else
  {
    field = 1;
    field_line = scan_line;
  }

  if (!hsync_instead_of_csync && (field_line >= NTSC_VSYNC_FIELD_LINE_START && field_line <= NTSC_VSYNC_FIELD_LINE_END))
  {
    OCR1A = NTSC_VSYNC_PERIOD_TICKS;
  }
  else
  {
    OCR1A = NTSC_HSYNC_PERIOD_TICKS;
  }

  is_active_video_line = field_line >= NTSC_ACTIVE_VIDEO_FIELD_LINE_START && field_line <= NTSC_ACTIVE_VIDEO_FIELD_LINE_END;
}

// timer/counter 1 compare-b interrupt
ISR(TIMER1_COMPB_vect)
{

  if (is_active_video_line)
  {
    // how can we do this without delays!?
    _delay_us(4.7 + 4.7); // (roughly) delay hsync us + bporch us
    interlacing_test(false);
  }
}

void interlacing_test(bool luma_only_field_1)
{
  // this is just for demos sake to test progressive vs interlaced
  // unfortunately a delay is currently the only way to push out data in the field line "luma window"
  // perhaps there is a way to have a "pixel clock" that only can write within this window?

  // interlacing test:
  //  - when INTERLACED is defined and luma_only_field_1 is FALSE, the video will NOT flicker because we are drawing luma for BOTH fields per frame
  //  - when INTERLACED is defined and luma_only_field_1 is FALSE, the video will flicker because we are only drawing luma for ONE field per frame
  //  - when INTERLACED is NOT defined, the video will NOT flicker because we are only drawing luma one field but TWICE per frame

  int box_height = 50;
  int first_line = NTSC_ACTIVE_VIDEO_FIELD_LINE_MID - box_height;
  int last_line = NTSC_ACTIVE_VIDEO_FIELD_LINE_MID + box_height;

  if (field_line >= first_line && field_line <= last_line && field_line % 8 == 0 && (!luma_only_field_1 || field == 1))
  {
    _delay_us(10);
    LUMA_HIGH;
    _delay_us(25);
    LUMA_LOW;
    return;
  }
}

void loop()
{
  // is this neccessary?
  sleep_cpu();
}