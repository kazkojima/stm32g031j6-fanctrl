#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include "stts75.h"
#include "delay.h"
#include <stdio.h>

#define DEBUG

// slow but enough
static uint32_t udiv16(uint32_t n, uint16_t d)
{
  int i;
  uint32_t q, r;
  q = 0; r = 0;
  for (i = 31; i >= 0; --i) {
    r <<= 1;
    r = (r & ~1) | ((n >> i) & 1);
    if (r >= d) {
      r = r - d;
      q = q | (1 << i);
    }
  }
  return q;
}

static void usart_setup(void)
{
#ifdef DEBUG
  /* Setup GPIO pin GPIO_USART2_TX/GPIO2 on GPIO port A for transmit. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
  /* Setup USART2 TX pin as alternate function 1. */
  gpio_set_af(GPIOA, GPIO_AF1, GPIO2);

  /* Enable clocks for USART2. */
  rcc_periph_clock_enable(RCC_USART2);

  /* Setup UART parameters. */
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART2);
#endif
}

static void i2c_setup(void)
{
  /* Enable clocks for I2C2. */
  rcc_periph_clock_enable(RCC_I2C2);

  /* Set alternate functions for the SCL and SDA pins of I2C2. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11|GPIO12);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO11|GPIO12);
  /* Setup I2C2 SCL/SDA pin as alternate function 6. */
  gpio_set_af(GPIOA, GPIO_AF6, GPIO11|GPIO12);

  /* Disable the I2C before changing any configuration. */
  i2c_peripheral_disable(I2C2);

  /* APB1 16MHz */
  i2c_set_speed(I2C2, i2c_speed_fm_400k, 16);

  /* If everything is configured -> enable the peripheral. */
  i2c_peripheral_enable(I2C2);
}

static void adc_setup(void)
{
  rcc_periph_clock_enable(RCC_ADC);

  /* Make sure the ADC doesn't run during config. */
  adc_power_off(ADC1);

  /* We configure everything for one single conversion. */
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  /* We want to read the temperature sensor, so we have to enable it. */
  adc_enable_temperature_sensor();
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_039DOT5);
  adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);

  adc_power_on(ADC1);
}

// 684 = 16000000/23400
#define DIV_PWM_23KHz 684

static void timer_setup(void)
{
#ifndef DEBUG
  /* Setup GPIO PA2 (pin 4) for TIM2 CH3 output. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
  /* Setup TIM2 CH3 pin as alternate function 1. */
  gpio_set_af(GPIOA, GPIO_AF2, GPIO2);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO2);

  rcc_periph_clock_enable(RCC_TIM2);
  timer_set_prescaler(TIM2, 0/*rcc_apb1_frequency / 1000000 - 1*/);
  timer_continuous_mode(TIM2);
  timer_set_period(TIM2, DIV_PWM_23KHz);
  timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
  timer_enable_oc_preload(TIM2, TIM_OC3);
  timer_set_oc_value(TIM2, TIM_OC3, 10);
  timer_enable_oc_output(TIM2, TIM_OC3);

  timer_enable_counter(TIM2);
#endif
}

static int16_t pr_prev;

static void pwm_set(int th)
{
#ifndef DEBUG
  int16_t pr, pwm;
  if (th > 8*16)
    pr = 8*16;
  else if (th <= 1*16)
    pr = 1*16;
  else
    pr = th;
  // Dumping
  pr = (pr + 7*pr_prev) >> 3;
  pr_prev = pr;
  pwm = (DIV_PWM_23KHz * pr) >> 7;
  timer_set_oc_value(TIM2, TIM_OC3, pwm);
#endif
}

static void my_usart_print_uint16(uint32_t usart, uint16_t value)
{
#ifdef DEBUG
  int8_t i;
  uint8_t *xdigits = "0123456789abcdef";
  char buffer[4];

  buffer[0] = xdigits[(value >> 12) & 0xf];
  buffer[1] = xdigits[(value >> 8) & 0xf];
  buffer[2] = xdigits[(value >> 4) & 0xf];
  buffer[3] = xdigits[(value >> 0) & 0xf];

  for (i = 0; i < 4; ++i)
    usart_send_blocking(usart, buffer[i]);
#endif
}

static void my_usart_print_string(uint32_t usart, char *str)
{
#ifdef DEBUG
  uint8_t c;
  while ((c = *str++) != 0) {
    usart_send_blocking(usart, c);
  }
#endif
}

uint16_t ts_cal1, ts_cal2;
// TS_OFS_TEMP=TS_CAL1_TEMP*16=480 in theory, though it looks
// the higher value is reasonable. ???
#define TS_OFS_TEMP 750

int16_t convert_ambient(uint16_t a)
{
  uint16_t td = ts_cal2 - ts_cal1;
  // Approx. 100/td by k/4096
  uint32_t k = udiv16(4096*100, td);
  return ((k * (a - ts_cal1) >> (12-4)) + TS_OFS_TEMP) << 4;
}
		       
int main(void)
{
  uint8_t channel_array[16];
  uint16_t ambient;
  uint16_t temperature;

  rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_16MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  usart_setup();
  adc_setup();
  i2c_setup();
  timer_setup();
  delay_setup();

  ts_cal1 = *(uint16_t *) 0x1fff75a8;
  ts_cal2 = *(uint16_t *) 0x1fff75ca;

  /* LED for boot progress */
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
  gpio_clear(GPIOC, GPIO14);

  /* Send a message on USART2. */
  my_usart_print_string(USART2, "stm\r\n");

  /* Select the channel we want to convert. 12=ambient_sensor. */
  channel_array[0] = ADC_CHANNEL_TEMP;
  adc_set_regular_sequence(ADC1, 1, channel_array);

  /* Set STTS75. */
  stts75_write_config(I2C2, STTS75_SENSOR0);
  //stts75_write_temp_os(I2C2, STTS75_SENSOR0, 0x1a00); /* 26 degrees */
  //stts75_write_temp_hyst(I2C2, STTS75_SENSOR0, 0x1a00);
 
  while (1) {
    adc_start_conversion_regular(ADC1);
    /* Wait for end of conversion. */
    while (!adc_eoc(ADC1))
      delay_us(1000);
    ambient = convert_ambient(adc_read_regular(ADC1));
    temperature = stts75_read_temperature(I2C2, STTS75_SENSOR0);
    pwm_set((temperature - ambient) >> 4);
    my_usart_print_string(USART2, "ambient:");
    my_usart_print_uint16(USART2, ambient);
    my_usart_print_string(USART2, "\r\n");
    my_usart_print_string(USART2, "tpm1075:");
    my_usart_print_uint16(USART2, temperature);
    my_usart_print_string(USART2, "\r\n");
    for (int i = 0; i < 20; i++)
      delay_us(50000);
  }
}

