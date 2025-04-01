// atmega328p datasheet - https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

#include <avr/sleep.h>
#include <limits.h>

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
#define NTSC_ACTIVE_VIDEO_PERIOD_USEC (NTSC_HORIZONTAL_PERIOD_US / 2)
#define NTSC_FRAMES_PER_SEC 29.97

#define NTSC_SCAN_LINE_PERIOD_TICKS USEC_TO_TICKS(NTSC_HORIZONTAL_PERIOD_US)
#define NTSC_HALF_SCAN_LINE_PERIOD_TICKS (NTSC_SCAN_LINE_PERIOD_TICKS / 2)
#define NTSC_HSYNC_PERIOD_TICKS USEC_TO_TICKS(NTSC_HSYNC_PERIOD_USEC)
#define NTSC_VSYNC_PERIOD_TICKS USEC_TO_TICKS(NTSC_VSYNC_PERIOD_USEC)
#define NTSC_VBLANK_FIELD_LINE_START 1
#define NTSC_VBLANK_FIELD_LINE_END 21
#define NTSC_VSYNC_FIELD_LINE_START 1
#define NTSC_VSYNC_FIELD_LINE_END 9
#define NTSC_ACTIVE_VIDEO_DELAY_TICKS NTSC_HSYNC_PERIOD_TICKS
#define NTSC_ACTIVE_VIDEO_PERIOD_TICKS USEC_TO_TICKS(NTSC_ACTIVE_VIDEO_PERIOD_USEC)
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_START (NTSC_VBLANK_FIELD_LINE_END + 1)
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_END (NTSC_ACTIVE_VIDEO_FIELD_LINE_START + (NTSC_SCAN_LINES_PER_FIELD - NTSC_VBLANK_FIELD_LINE_END - NTSC_VBLANK_FIELD_LINE_START + 1))
#define NTSC_ACTIVE_VIDEO_FIELD_LINE_MID (NTSC_ACTIVE_VIDEO_FIELD_LINE_START + ((NTSC_ACTIVE_VIDEO_FIELD_LINE_END - NTSC_ACTIVE_VIDEO_FIELD_LINE_START) / 2))

// output
// pins 9 and 10 are used by timer 1
// we use pin 9 - output of waveform generation mode.
#define PIN_CSYNC 9
// vsync - pin 7
#define PORT_VSYNC PORTD
#define PIN_VSYNC PD7
// luma - pin 6
#define PORT_LUMA PORTD
#define PIN_LUMA PD6

// macros
// we write directly to the PORTs as it's the fastest instruction possible
// using bitWrite and ESPECIALLY digitalWrite is slower
#define VSYNC_INACTIVE PORT_VSYNC |= _BV(PIN_VSYNC) // bitWrite(PORTD, PIN_VSYNC, 1)
#define VSYNC_ACTIVE PORT_VSYNC &= ~_BV(PIN_VSYNC)  // bitWrite(PORTD, PIN_VSYNC, 0)
#define LUMA_HIGH PORT_LUMA |= _BV(PIN_LUMA)        // bitWrite(PORTD, PIN_LUMA, 1)
#define LUMA_LOW PORT_LUMA &= ~_BV(PIN_LUMA)        // bitWrite(PORTD, PIN_LUMA, 0)

// settings
volatile bool interlacing = true;

// sync gen state
volatile uint16_t scan_line = 1;
volatile uint16_t field = 1;
volatile uint16_t field_line = 1;
volatile uint16_t half_line = 0;
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

    ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;              // timer/counter 1 - overflow interrupt
    OCR1A = NTSC_HSYNC_PERIOD_TICKS;                 // timer/counter 1 - compare-a interrupt
    OCR1B = NTSC_ACTIVE_VIDEO_PERIOD_TICKS;          // timer/counter 1 - compare-b interrupt
    TCNT1 = 0;                                       // timer/counter 1 - value
    TIMSK1 = _BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1B); // timer/counter 1 interrupts - enable timer 1 overflow (TOIE1) and compare-b (OCIE1B)
  }
  // timer/counter 1 - end

  // enable interrupts
  sei();
}

void loop()
{
  // is this neccessary?
  sleep_cpu();
}

// timer/counter 1 overflow interrupt
ISR(TIMER1_OVF_vect)
{
  if (interlacing)
  {
    if (scan_line == NTSC_SCAN_LINES_PER_FIELD)
    {
      ICR1 = NTSC_HALF_SCAN_LINE_PERIOD_TICKS;
    }
    else
    {
      ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;
    }

    OCR1A = NTSC_HSYNC_PERIOD_TICKS;

    if (scan_line != NTSC_SCAN_LINES_PER_FIELD)
    {
      scan_line++;

      if (scan_line == NTSC_SCAN_LINES_PER_FRAME)
      {
        OCR1A = NTSC_VSYNC_PERIOD_TICKS;

        scan_line = 1;
      }
    }
    else
    {
      if (half_line == 1)
      {
        scan_line++;
      }
      else
      {
        OCR1A = NTSC_VSYNC_PERIOD_TICKS;
      }
      
      half_line ^= 1;
    }
  }
  else
  {
    ICR1 = NTSC_SCAN_LINE_PERIOD_TICKS;
    OCR1A = NTSC_HSYNC_PERIOD_TICKS;

    if (scan_line == NTSC_SCAN_LINES_PER_FIELD)
    {
      OCR1A = NTSC_VSYNC_PERIOD_TICKS;

      scan_line = 1;
    }
    else
    {
      scan_line++;
    }
  }

  if (scan_line >= NTSC_SCAN_LINES_PER_FRAME)
  {
    scan_line = 1;
  }

  if (scan_line > NTSC_SCAN_LINES_PER_FIELD)
  {
    field = 2;
    field_line = (scan_line - NTSC_SCAN_LINES_PER_FIELD - 1);
  }
  else
  {
    field = 1;
    field_line = scan_line;
  }

  is_active_video_line = field_line >= NTSC_ACTIVE_VIDEO_FIELD_LINE_START && field_line <= NTSC_ACTIVE_VIDEO_FIELD_LINE_END;
}

// timer/counter 1 compare-a interrupt
ISR(TIMER1_COMPA_vect)
{
  if (is_active_video_line)
  {
    // is there a way to do this WITHOUT delays!?
    //_delay_us(4.7 + 4.7); // (roughly) delay hsync us + bporch us

    // pass interlacing_test(value): false to not pick out fields
    // pass interlacing_test(value): 1 for field-1 -or- 2 for field-2
    // depending on the value of the interlacing bool variable, you will see flickering and thinner or thicker lines
    // and the lines will clearly move, confirming interlacing is working
    interlacing_test(false);
  }
}

// timer/counter 1 compare-b interrupt
ISR(TIMER1_COMPB_vect)
{
  LUMA_LOW;
}

void interlacing_test(uint16_t field_for_luma)
{
  // this is just for demos sake to test progressive vs interlaced

  // is there a way to do this WITHOUT delays!?

  // interlacing test:
  //  - when interlacing is true and field_for_luma is 0/false:
  //    - the video will NOT flicker because we are drawing luma for BOTH fields per frame
  //    - the bars will be thicker becuase we are drawing BOTH fields
  //  - when interlacing is true and field_for_luma is 1 or 2:
  //    - the video will flicker because we are only drawing luma during field 1 or field 2
  //    - the bars will be thiner becuase we are only drawing half the fields
  //    - the bars will clearly move on the screen as well
  //  - when interlacing is false and field_for_luma is 1:
  //    - the video will NOT flicker because we are drawing field 1 but twice per frame
  //    - the bars will be thicker becuase we are drawing field 1 but twice per frame

  int box_height = 50;
  int first_line = NTSC_ACTIVE_VIDEO_FIELD_LINE_START; // NTSC_ACTIVE_VIDEO_FIELD_LINE_MID - box_height;
  int last_line = NTSC_ACTIVE_VIDEO_FIELD_LINE_END;    // NTSC_ACTIVE_VIDEO_FIELD_LINE_MID + box_height;

  if (field_line >= first_line && field_line <= last_line && field_line % 8 == 0 && (!field_for_luma || field == field_for_luma))
  {
    LUMA_HIGH;
    return;
  }
}
