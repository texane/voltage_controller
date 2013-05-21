/* theory of operation

   By default, caps c1,c2 are connected in parallel and a
   microcontroller measures the voltage accross them. When
   the voltage does not vary for more than a configured delay,
   the microcontroller switch to series mode to dump their
   charge into a battery for a configured delay.

   by pushing a plus or minus button, the user can setup the
   time a constant voltage is considered to trigger the timing
   logic. the values can be saved into the EEPROM.

   condition a
   when c1,c2 connected in parallel and v(c1,c2) does not
   change for parallel_ms, (ie. delta v(c1,c2) <= 1v), then
   connect c1 and c2 in serie. parallel_ms ranges from 10ms
   to 1000ms, with a 10ms resolution.

   condition b
   when c1,c2 connected in serie, the connection must be
   closed after serie_ms. serie_ms can range from 10ms to
   500ms.

   condition c
   a LCD displays the opto pulses per minutes (ppm). it also
   shows the output voltage between x- and x+.

   condition d
   timing setup must be user controlled. one switch selects
   between parallel_ms or serie_ms setup. setup should be
   set in persistent memory.
 */


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#define CONFIG_DEBUG 1
#define CONFIG_LCD 1
#define CONFIG_UART 1


#if CONFIG_LCD

/* note: lcd model MC21605A6W */
/* note: DB0:3 and RW must be grounded */

#define LCD_POS_DB 0x02
#define LCD_PORT_DB PORTD
#define LCD_DIR_DB DDRD
#define LCD_MASK_DB (0x0f << LCD_POS_DB)

#define LCD_POS_EN 0x06
#define LCD_PORT_EN PORTD
#define LCD_DIR_EN DDRD
#define LCD_MASK_EN (0x01 << LCD_POS_EN)

#define LCD_POS_RS 0x07
#define LCD_PORT_RS PORTD
#define LCD_DIR_RS DDRD
#define LCD_MASK_RS (0x01 << LCD_POS_RS)

static inline void wait_50_ns(void)
{
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_500_ns(void)
{
  /* 8 cycles at 16mhz */
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_50_us(void)
{
  /* 800 cycles at 16mhz */
  uint8_t x;
  for (x = 0; x < 100; ++x) wait_500_ns();
}

static inline void wait_2_ms(void)
{
  wait_50_us();
  wait_50_us();
  wait_50_us();
  wait_50_us();
}

static inline void wait_50_ms(void)
{
  /* FIXME: was _delay_ms(50), but not working */
  uint8_t x;
  for (x = 0; x < 25; ++x) wait_2_ms();
}

static inline void lcd_pulse_en(void)
{
  /* assume EN low */
  LCD_PORT_EN |= LCD_MASK_EN;
  wait_50_us();
  LCD_PORT_EN &= ~LCD_MASK_EN;
  wait_2_ms();
}

static inline void lcd_write_db4(uint8_t x)
{
  /* configured in 4 bits mode */

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) << LCD_POS_DB;
  lcd_pulse_en();

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x & 0xf) << LCD_POS_DB;
  lcd_pulse_en();
}

static inline void lcd_write_db8(uint8_t x)
{
  /* configured in 8 bits mode */

  /* only hi nibble transmitted, (0:3) grounded */
  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) << LCD_POS_DB;
  lcd_pulse_en();
}

static inline void lcd_setup(void)
{
  LCD_DIR_DB |= LCD_MASK_DB;
  LCD_DIR_RS |= LCD_MASK_RS;
  LCD_DIR_EN |= LCD_MASK_EN;

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_RS &= ~LCD_MASK_RS;
  LCD_PORT_EN &= ~LCD_MASK_EN;

  /* small delay for the lcd to boot */
  wait_50_ms();

  /* datasheet init sequence */

#define LCD_MODE_BLINK (1 << 0)
#define LCD_MODE_CURSOR (1 << 1)
#define LCD_MODE_DISPLAY (1 << 2)

  lcd_write_db8(0x30);
  wait_2_ms();
  wait_2_ms();
  wait_500_ns();

  lcd_write_db8(0x30);
  wait_2_ms();

  lcd_write_db4(0x32);
  wait_2_ms();

  lcd_write_db4(0x28);
  wait_2_ms();

  lcd_write_db4((1 << 3) | LCD_MODE_DISPLAY);
  wait_2_ms();

  lcd_write_db4(0x01);
  wait_2_ms();

  lcd_write_db4(0x0f);
  wait_2_ms();
}

static inline void lcd_clear(void)
{
  /* clear lcd */
  lcd_write_db4(0x01);
  wait_2_ms();
}

static inline void lcd_home(void)
{
  /* set cursor to home */
  lcd_write_db4(0x02);
  wait_2_ms();
}

static inline void lcd_set_ddram(uint8_t addr)
{
  lcd_write_db4((1 << 7) | addr);
  wait_2_ms();
}

static inline void lcd_goto_xy(uint8_t x, uint8_t y)
{
  /* assume 0 <= x < 8 */
  /* assume 0 <= y < 2 */

  /* from datasheet: */
  /* first line is 0x00 to 0x27 */
  /* second line is 0x40 to 0x67 */
  static const uint8_t row[] = { 0x00, 0x40 };
  lcd_set_ddram(row[y] | x);
}

static void lcd_write(const uint8_t* s, unsigned int n)
{
  wait_50_ns();

  LCD_PORT_RS |= LCD_MASK_RS;
  for (; n; --n, ++s)
  {
    lcd_write_db4(*s);
    wait_2_ms();
  }
  LCD_PORT_RS &= ~LCD_MASK_RS;
}

#endif /* CONFIG_LCD */


#if CONFIG_UART /* uart */

static inline void set_baud_rate(long baud)
{
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

static void uart_setup(void)
{
  /* #define CONFIG_FOSC (F_CPU * 2) */
  /* const uint16_t x = CONFIG_FOSC / (16 * BAUDS) - 1; */
#if 0 /* (bauds == 9600) */
  const uint16_t x = 206;
#elif 0 /* (bauds == 115200) */
  const uint16_t x = 16;
#elif 0 /* (bauds == 500000) */
  const uint16_t x = 3;
#elif 0 /* (bauds == 1000000) */
  const uint16_t x = 1;
#endif

  set_baud_rate(9600);

  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);

  UCSR0B = 1 << TXEN0;

  /* default to 8n1 framing */
  UCSR0C = (3 << 1);
}

static void uart_write(const uint8_t* s, uint8_t n)
{
  for (; n; --n, ++s)
  {
    /* wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) ;
    UDR0 = *s;
  }

  /* wait for last byte to be sent */
  while ((UCSR0A & (1 << 6)) == 0) ;
}

static void uart_write_cstring(const char* s)
{
  uint8_t n;
  for (n = 0; s[n]; ++n) ;
  uart_write((const uint8_t*)s, n);
}

#endif /* CONFIG_UART */


static inline uint8_t nibble(uint32_t x, uint8_t i)
{
  return (x >> (i * 4)) & 0xf;
}

static inline uint8_t hex(uint8_t x)
{
  return (x >= 0xa) ? 'a' + x - 0xa : '0' + x;
}

__attribute__((unused))
static uint8_t* uint32_to_string(uint32_t x)
{
  static uint8_t buf[8];

  buf[7] = hex(nibble(x, 0));
  buf[6] = hex(nibble(x, 1));
  buf[5] = hex(nibble(x, 2));
  buf[4] = hex(nibble(x, 3));
  buf[3] = hex(nibble(x, 4));
  buf[2] = hex(nibble(x, 5));
  buf[1] = hex(nibble(x, 6));
  buf[0] = hex(nibble(x, 7));

  return buf;
}

static uint8_t* uint8_to_string(uint8_t x)
{
  static uint8_t buf[2];

  buf[1] = hex(nibble(x, 0));
  buf[0] = hex(nibble(x, 1));

  return buf;
}

static uint8_t* uint10_to_string(uint16_t x)
{
  static uint8_t buf[3];

  buf[2] = hex(nibble(x, 0));
  buf[1] = hex(nibble(x, 1));
  buf[0] = hex(nibble(x, 2));

  return buf;
}

static unsigned int ulong_to_string(unsigned long n, char* s)
{
  unsigned char buf[8];
  unsigned long i = 0;
  unsigned int len = 0;
  if (n == 0) { *s = '0'; return 1; }
  while (n > 0) { buf[i++] = n % 10; n /= 10; }
  for (; i > 0; i--, ++len) s[len] = '0' + buf[i - 1];
  return len;
}

__attribute__((unused))
static unsigned int double_to_string(double x, const uint8_t** s)
{
  static char buf[32];
  uint8_t digits = 3;
  double rounding = 0.5;
  unsigned int len = 0;
  uint8_t i;
  unsigned long int_part;
  double remainder;

  if (x < 0.0) { buf[len++] = '-'; x = -x; }

  for (i = 0; i < digits; ++i) rounding /= 10.0;
  x += rounding;

  int_part = (unsigned long)x;
  remainder = x - (double)int_part;

  len += ulong_to_string(int_part, buf + len);

  if (digits > 0) buf[len++] = '.';

  while ((digits--) > 0)
  {
    unsigned long xx;
    remainder *= 10.0;
    xx = (unsigned long)remainder;
    len += ulong_to_string(xx, buf + len);
    remainder -= xx; 
  }

  *s = (const uint8_t*)buf;

  return len;
}


#if 0 /* USE_AVR_EEPROM */

/* total size: 1Kb */
/* page size: 4 bytes */
/* page count: 256 */

static void eeprom_write(uint8_t pos, uint8_t* buf, uint8_t n)
{
  /* pos the page position */
  /* n the page count */

  uint8_t i;

  for (i = 0; i < n; ++i)
  {
    /* datasheet, section 27.7.5 */

    /* load the command 0x11 */

    /* load the addr high byte */
    /* load the addr low byte */

    /* load data[0,1,2,3] */
    /* latch data[0,1,2,3] */

    /* program the page */
    /* set bs1 to 0 */
    /* negative pulse wr */
    /* wait until rdy */
  }

}

static void eeprom_read(uint8_t pos, uint8_t* buf, uint8_t n)
{
  uint8_t i;

  for (i = 0; i < n; ++i)
  {
    /* datasheet, section 27.7.7 */

    /* load the command 0x3 */

    /* load the addr high byte */
    /* load the addr low byte */

    /* set oe to 0 */
    /* set bs1 to 0 */

    /* read data */

    /* set oe to 1 */
  }
}

#endif /* USE_AVR_EEPROM */


/* adc */

#define ADC_POS_VCAP 0
#define ADC_MASK_VCAP (1 << ADC_POS_VCAP)
#define ADC_POS_VOUT 1
#define ADC_MASK_VOUT (1 << ADC_POS_VOUT)
#define ADC_COMMON_DDR DDRC
#define ADC_COMMON_MASK (ADC_MASK_VCAP | ADC_MASK_VOUT)

#define ADC_CONV_VOLT(__x) (((__x) * 1024) / 5)

static void adc_setup(void)
{
  /* chapter 23 */

  ADC_COMMON_DDR &= ~ADC_COMMON_MASK;

#if 1 /* fadc to 125khz */

  /* 10 bits resolution requires a clock between 50khz and 200khz. */
  /* a prescaler is used to derive fadc from fcpu. the prescaler */
  /* starts counting when ADEN is set, and is always reset when */
  /* ADEN goes low. */

  /* 16mhz / 200khz = 80. prescaler set to 128, thus fadc = 125khz */
  /* a normal conversion takes 13 FADC cycles */
  /* thus, actual sampling rate of 125000 / 13 = 9615 samples per second */

  ADCSRA = (7 << ADPS0);

#else /* force fadc to 1mhz */

  ADCSRA = (4 << ADPS0);

#endif /* fadc */

  ADCSRB = 0;

  /* aref, internal vref off, channel 0 */
  ADMUX = 1 << REFS0;

  /* disable digital input 0 to reduce power consumption, cf 23.9.5 */
  DIDR0 = ADC_COMMON_MASK;
}

__attribute__((unused))
static void adc_wait_25(void)
{
  /* wait for 25 adc cycles */
  /* FADC = FCPU / 128, thus waits for 128 * 25 = 3200 cpu cycles */
  uint8_t x = 0xff;
  for (; x; --x) __asm__ __volatile__ ("nop\n\t");
}

__attribute__((unused))
static void adc_start_single_conversion(void)
{
  ADCSRB = 0;
  ADCSRA |= (1 << ADEN);
}

__attribute__((unused))
static void adc_stop(void)
{
  ADCSRA &= ~(1 << ADEN);
}

static void adc_read(uint8_t chan, uint16_t* x)
{
  uint8_t l;

  /* select channel */
  ADMUX &= ~0xf;
  ADMUX |= chan;

  /* start conversion */
  ADCSRA |= (1 << ADSC);

  while (ADCSRA & (1 << ADSC)) ;

  /* read adcl first */
  l = ADCL;
  *x = ((((uint16_t)ADCH) << 8) | (uint16_t)l) & 0x3ff;
}

static void adc_read_vcap(uint16_t* val)
{
  adc_read(ADC_POS_VCAP, val);
}

static void adc_read_vout(uint16_t* val)
{
  adc_read(ADC_POS_VOUT, val);
}


/* application */

/* timer configuration */

#define TIMER_FREQ 100
#define TIMER_MS_TO_TICKS(__x) \
  (((uint32_t)(__x) * (uint32_t)TIMER_FREQ) / (uint32_t)1000)
#define TIMER_TICKS_TO_MS(__x) \
  (((uint32_t)(__x) * (uint32_t)1000) / (uint32_t)TIMER_FREQ)

/* opto control pins */

#define OPTO_COMMON_DDR DDRB
#define OPTO_COMMON_PORT PORTB
#define OPTO_PARALLEL_POS 0
#define OPTO_PARALLEL_MASK (1 << OPTO_PARALLEL_POS)
#define OPTO_SERIES_POS 1
#define OPTO_SERIES_MASK (1 << OPTO_SERIES_POS)
#define OPTO_COMMON_MASK (OPTO_PARALLEL_MASK | OPTO_SERIES_MASK)

static void opto_setup(void)
{
  OPTO_COMMON_DDR |= OPTO_COMMON_MASK;
  OPTO_COMMON_PORT &= ~OPTO_COMMON_MASK;
}

static void opto_enable_parallel(void)
{
  OPTO_COMMON_PORT |= OPTO_PARALLEL_MASK;
}

static void opto_disable_parallel(void)
{
  OPTO_COMMON_PORT &= ~OPTO_PARALLEL_MASK;
}

static void opto_enable_series(void)
{
  OPTO_COMMON_PORT |= OPTO_SERIES_MASK;
}

static void opto_disable_series(void)
{
  OPTO_COMMON_PORT &= ~OPTO_SERIES_MASK;
}

/* buttons ATMEGA328P pin configuration */

#define BUT_COMMON_DDR DDRC
#define BUT_COMMON_PIN PINC
#define BUT_COMMON_PORT PORTC

#define BUT_SAVE_POS 2
#define BUT_MODE_POS 3
#define BUT_PLUS_POS 4
#define BUT_MINUS_POS 5

#define BUT_SAVE_MASK (1 << BUT_SAVE_POS)
#define BUT_MODE_MASK (1 << BUT_MODE_POS)
#define BUT_PLUS_MASK (1 << BUT_PLUS_POS)
#define BUT_MINUS_MASK (1 << BUT_MINUS_POS)

#define BUT_COMMON_MASK \
  (BUT_SAVE_MASK | BUT_MODE_MASK | BUT_PLUS_MASK | BUT_MINUS_MASK)

/* button counters */

static volatile uint8_t but_states;

static void but_setup(void)
{
  /* enable pullups and set as input */
  BUT_COMMON_PORT |= BUT_COMMON_MASK;
  BUT_COMMON_DDR &= ~BUT_COMMON_MASK;

  /* reset button states */
  but_states = 0;
}

static inline void but_update_one
(uint8_t pre, uint8_t cur, uint8_t* counter, uint8_t mask)
{
  const uint8_t is_pressed = (cur & mask) == 0;

  if (is_pressed && ((pre & mask) == (cur & mask)))
  {
    if ((++*counter) == TIMER_MS_TO_TICKS(125))
    {
      but_states |= mask;
      *counter = 0;
    }
  }
  else
  {
    but_states &= ~mask;
    *counter = 0;
  }
}

static void but_update_states(void)
{
  static uint8_t save_counter = 0;
  static uint8_t mode_counter = 0;
  static uint8_t plus_counter = 0;
  static uint8_t minus_counter = 0;

  static uint8_t pre = 0;

  const uint8_t cur = BUT_COMMON_PIN & BUT_COMMON_MASK;

  but_update_one(pre, cur, &save_counter, BUT_SAVE_MASK);
  but_update_one(pre, cur, &mode_counter, BUT_MODE_MASK);
  but_update_one(pre, cur, &plus_counter, BUT_PLUS_MASK);
  but_update_one(pre, cur, &minus_counter, BUT_MINUS_MASK);

  /* affect current value to previous */
  pre = cur;
}

static uint8_t but_read_states(void)
{
  const uint8_t x = but_states;
  but_states &= ~(x);
  return x;
}

static uint8_t but_is_pressed(uint8_t x, uint8_t m)
{
  return x & m;
}

/* user configured ticks */

#define CONF_MODE_PARALLEL 0
#define CONF_MODE_SERIES 1

static uint8_t conf_parallel_ticks;
static uint8_t conf_series_ticks;
static uint8_t conf_lcd_ticks;

#define CONF_EEPROM_MAGIC 0xdeadbeef
#define CONF_EEPROM_ADDR ((void*)42) /* max: 256 */

static void conf_load(void)
{
  /* load configuration value from eeprom */

  uint8_t buf[8];

  eeprom_read_block(buf, CONF_EEPROM_ADDR, sizeof(buf));

  if (*(uint32_t*)buf != CONF_EEPROM_MAGIC)
  {
    /* default if eeprom not written */

#if CONFIG_DEBUG
    uart_write_cstring("bad_eeprom_magic\r\n");
#endif /* CONFIG_DEBUG */

    conf_parallel_ticks = TIMER_MS_TO_TICKS(10);
    conf_series_ticks = TIMER_MS_TO_TICKS(500);
  }
  else
  {
    /* check ranges */

    conf_parallel_ticks = *(uint8_t*)(buf + 4);
    conf_series_ticks = *(uint8_t*)(buf + 5);

#if CONFIG_DEBUG
    uart_write_cstring("good_eeprom_magic: ");
    uart_write(uint8_to_string(conf_parallel_ticks), 2);
    uart_write_cstring(" ");
    uart_write(uint8_to_string(conf_series_ticks), 2);
    uart_write_cstring("\r\n");
#endif /* CONFIG_DEBUG */

    if (conf_parallel_ticks < TIMER_MS_TO_TICKS(10))
      conf_parallel_ticks = TIMER_MS_TO_TICKS(10);
    else if (conf_parallel_ticks > TIMER_MS_TO_TICKS(1000))
      conf_parallel_ticks = TIMER_MS_TO_TICKS(1000);

    if (conf_series_ticks < TIMER_MS_TO_TICKS(10))
      conf_series_ticks = TIMER_MS_TO_TICKS(10);
    else if (conf_series_ticks > TIMER_MS_TO_TICKS(500))
      conf_series_ticks = TIMER_MS_TO_TICKS(500);
  }

  /* not loaded from eeprom */
  conf_lcd_ticks = TIMER_MS_TO_TICKS(1000);
}

static void conf_store(void)
{
  uint8_t buf[8];

  *(uint32_t*)buf = CONF_EEPROM_MAGIC;
  *(uint8_t*)(buf + 4) = conf_parallel_ticks;
  *(uint8_t*)(buf + 5) = conf_series_ticks;

  eeprom_write_block(buf, CONF_EEPROM_ADDR, sizeof(buf));
}

/* timer1 interrupt handler */

static volatile uint8_t timer_ticks = 0;

ISR(TIMER1_COMPA_vect)
{
  /* update button state */
  but_update_states();

  /* prevent overflow */
  if (timer_ticks == 0xff) return ;
  ++timer_ticks;
}

static void timer_enable(void)
{
  /* 16 bits timer1 is used */
  /* interrupt at TIMER_FREQ hz */
  /* fcpu / (64 * 2500) = 100 hz */

  /* stop timer */
  TCCR0B = 0;

  /* CTC mode, overflow when OCR1A reached */
  TCCR1A = 0;
  OCR1A = 2500;
  TCNT1 = 0;
  TCCR1C = 0;

  /* interrupt on OCIE0A match */
  TIMSK1 = 1 << 1;

  /* reset timer tick counter */
  timer_ticks = 0;

  /* start timer */
  /* prescaler set to 64 */
  TCCR1B = (1 << 3) | (3 << 0);
}

static void timer_disable(void)
{
  TCCR0B = 0;
}


/* absolute difference */

static uint16_t abs_diff(uint16_t a, uint16_t b)
{
  if (a > b) return a - b;
  return b - a;
}


/* printing routines */

static void print_voltages(uint16_t vcap, uint16_t vout)
{
  lcd_goto_xy(0, 0);
  lcd_write((const uint8_t*)"        ", 8);

  lcd_goto_xy(0, 0);
  lcd_write(uint10_to_string(vcap), 3);

  lcd_goto_xy(4, 0);
  lcd_write(uint10_to_string(vout), 3);
}

static void print_ppm(uint8_t ppm)
{
  lcd_goto_xy(0, 1);
  lcd_write(uint8_to_string(ppm), 2);
}

static void print_mode(uint8_t mode, uint8_t value)
{
  static const char* s[] = { "parallel", "series  " };

  lcd_goto_xy(0, 0);
  lcd_write((const uint8_t*)s[mode], 8);

  lcd_goto_xy(0, 1);
  lcd_write(uint8_to_string(value), 2);
}


/* main */

int main(void)
{
  uint8_t min_value;
  uint8_t max_value;
  uint8_t* value;
  uint8_t mode;
  uint8_t but;
  uint16_t vcap;
  uint16_t vout = 0;
  uint16_t prev_vcap;
  uint8_t opto_pulses;

#if CONFIG_LCD
  /* lcd */
  lcd_setup();
  lcd_clear();
  lcd_home();
#endif

#if CONFIG_UART
  uart_setup();
#endif

  adc_setup();
  adc_start_single_conversion();

  conf_load();
  opto_setup();
  but_setup();
  timer_disable();

  sei();

  /* voltage controller logic */

  opto_pulses = 0;

  while (1)
  {
  do_parallel_mode:
    opto_enable_parallel();
    adc_read_vcap(&prev_vcap);
    timer_enable();

    print_voltages(prev_vcap, vout);

    while (1)
    {
      but = but_read_states();
      if (but_is_pressed(but, BUT_MODE_MASK))
      {
	opto_disable_parallel();
	timer_disable();
	goto do_buttons;
      }

      adc_read_vcap(&vcap);

      /* apply 100 to 5 volts scaling, 1 volt becomes 0.05 */
      if (abs_diff(vcap, prev_vcap) >= ADC_CONV_VOLT(0.05))
      {
	goto do_parallel_mode;
      }

      /* voltage stable long enough  */
      if (timer_ticks >= conf_parallel_ticks)
      {
	opto_disable_parallel();
	timer_disable();
	break ;
      }
    }

    ++opto_pulses;

    /* do_series_mode: */
    opto_enable_series();
    timer_enable();
    while (timer_ticks < conf_series_ticks)
    {
      adc_read_vcap(&vcap);
      adc_read_vout(&vout);
      print_voltages(vcap, vout);
    }
    opto_disable_series();

    /* do_update_ppm: */
    print_ppm(opto_pulses);

    but = but_read_states();
  do_buttons:
    if (but_is_pressed(but, BUT_MODE_MASK))
    {
      mode = CONF_MODE_PARALLEL;
      print_mode(CONF_MODE_PARALLEL, conf_parallel_ticks);

#if CONFIG_DEBUG
      uart_write_cstring("enter\r\n");
#endif /* CONFIG_DEBUG */

      while (1)
      {
	but = but_read_states();

	/* toggle mode */
	if (but_is_pressed(but, BUT_MODE_MASK))
	{
#if CONFIG_DEBUG
	  uart_write_cstring("mode\r\n");
#endif /* CONFIG_DEBUG */
	  if (mode == CONF_MODE_PARALLEL) mode = CONF_MODE_SERIES;
	  else mode = CONF_MODE_PARALLEL;
	}
	
	/* update values */
	if (mode == CONF_MODE_PARALLEL)
	{
	  value = &conf_parallel_ticks;
	  min_value = TIMER_MS_TO_TICKS(10);
	  max_value = TIMER_MS_TO_TICKS(1000);
	}
	else
	{
	  value = &conf_series_ticks;
	  min_value = TIMER_MS_TO_TICKS(10);
	  max_value = TIMER_MS_TO_TICKS(500);
	}

	if (but_is_pressed(but, BUT_MINUS_MASK))
	{
	  if (*value >= (min_value + TIMER_MS_TO_TICKS(10)))
	    *value -= TIMER_MS_TO_TICKS(10);
	  else
	    *value = min_value;

#if CONFIG_DEBUG
	  uart_write_cstring("minus: ");
	  uart_write(uint8_to_string(*value), 2);
	  uart_write_cstring("\r\n");
#endif /* CONFIG_DEBUG */
	}

	if (but_is_pressed(but, BUT_PLUS_MASK))
	{
	  if (*value <= (max_value - TIMER_MS_TO_TICKS(10)))
	    *value += TIMER_MS_TO_TICKS(10);
	  else
	    *value = max_value;

#if CONFIG_DEBUG
	  uart_write_cstring("plus: ");
	  uart_write(uint8_to_string(*value), 2);
	  uart_write_cstring("\r\n");
#endif /* CONFIG_DEBUG */
	}

	/* update if something has changed */
	if (but_is_pressed(but, BUT_COMMON_MASK))
	{
	  print_mode(mode, *value);
	}

	/* save values and leaves */
	if (but_is_pressed(but, BUT_SAVE_MASK))
	{
#if CONFIG_DEBUG
	  uart_write_cstring("save\r\n");
#endif /* CONFIG_DEBUG */
	  conf_store();
	  break ;
	}
      }
    }
  }

  return 0;
}
