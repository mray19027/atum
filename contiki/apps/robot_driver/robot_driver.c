/*
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-test-pwm Test the CC2538 PWM driver
 *
 * Demonstrates the use of the CC2538 PWM driver
 *
 * @{
 *
 * \file
 *         A quick program for testing the CC2538 PWM driver
 * \author
 *         Javier Sanchez <jsanchez@zolertia.com>
 *         Antonio Lignan <alinan@zolertia.com>
 */
#include "contiki.h"
#include "cpu.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/sys-ctrl.h"
#include "uart.h"
#include "pwm.h"
#include "systick.h"
#include "lpm.h"
#include "dev/ioc.h"
#include <stdio.h>
#include <stdint.h>

#include "dev/cc2538-sensors.h"
#include "dev/button-sensor.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
// #include "dev/als-sensor.h"
#include "dev/serial-line.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static uint8_t active;
static struct etimer kicker_timer;

/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t timer;
  uint8_t ab;
  uint8_t port;
  uint8_t pin;
  uint8_t duty;
  uint8_t off_state;
  uint32_t freq;
} pwm_config_t;
/*---------------------------------------------------------------------------*/
#define MAX_PWM  5
static pwm_config_t pwm_num[MAX_PWM] = {
  {
    .timer = PWM_TIMER_1,
    .ab = PWM_TIMER_A,
    .port = GPIO_B_NUM,
    .pin = 5,
    .freq = 490,
    .off_state = PWM_OFF_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_1,
    .ab = PWM_TIMER_B,
    .port = GPIO_B_NUM,
    .pin = 6,
    .freq = 490,
    .off_state = PWM_ON_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_2,
    .ab = PWM_TIMER_A,
    .port = GPIO_B_NUM,
    .pin = 3,
    .freq = 490,
    .off_state = PWM_OFF_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_2,
    .ab = PWM_TIMER_B,
    .port = GPIO_B_NUM,
    .pin = 4,
    .freq = 490,
    .off_state = PWM_ON_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_3,
    .ab = PWM_TIMER_A,
    .port = GPIO_C_NUM,
    .pin = 2,
    .freq = 50,
    .off_state = PWM_OFF_WHEN_STOP,
  }
};
static uint8_t pwm_en[MAX_PWM];
/*---------------------------------------------------------------------------*/
#if DEBUG
static const char *
gpt_name(uint8_t timer)
{
  switch(timer) {
  case PWM_TIMER_0:
    return "UNUSED";
  case PWM_TIMER_1:
    return "LEFT";
  case PWM_TIMER_2:
    return "RIGHT";
  case PWM_TIMER_3:
    return "KICKER";
  default:
    return "Unknown";
  }
}
#endif

void change_freq(uint8_t i, float duty) {
  pwm_stop(pwm_num[i].timer, pwm_num[i].ab, pwm_num[i].port, pwm_num[i].pin, 0);
  pwm_disable(pwm_num[i].timer, pwm_num[i].ab, pwm_num[i].port,  pwm_num[i].pin);
  pwm_enable(pwm_num[i].freq, duty, pwm_num[i].timer, pwm_num[i].ab);
  pwm_start(pwm_num[i].timer, pwm_num[i].ab, pwm_num[i].port, pwm_num[i].pin);
}

uint8_t buffer[5];
uint8_t counter;

int test(unsigned char c) {
  if (counter==0) {
    if ((uint8_t)c == 0x38 || (uint8_t)c == 0x40) {
      buffer[0] = (uint8_t)c;
      counter++;
    }
  } else if (counter>=4) {
    buffer[4] = (uint8_t)c;
    // if (buffer[4] == 0x39) {
      if (buffer[1]==3) { //ID = 0
        if (buffer[2]==90) {

          // change_freq(4, 11.7);
          // etimer_set(&kicker_timer, CLOCK_SECOND * 0.5);
          // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&kicker_timer));
          // change_freq(4, 3);
          // etimer_set(&kicker_timer, CLOCK_SECOND * 0.5);
          // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&kicker_timer));
          // change_freq(4, 7.35);

        } else if (buffer[2]==97) {
          change_freq(0, 0);
          change_freq(1, buffer[3]);
        } else if (buffer[2]==98) {
          change_freq(1, 0);
          change_freq(0, buffer[3]);
        } else if (buffer[2]==99) {
          change_freq(3, 0);
          change_freq(2, buffer[3]);
        } else if (buffer[2]==100) {
          change_freq(2, 0);
          change_freq(3, buffer[3]);
        }
        leds_toggle(LEDS_RED);
      }
    // }
    counter = 0;
  } else {
    buffer[counter] = (uint8_t)c;
    counter++;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc2538_pwm_test, "cc2538 pwm test");
AUTOSTART_PROCESSES(&cc2538_pwm_test);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2538_pwm_test, ev, data)
{
  PROCESS_BEGIN();

  counter = 0;

  uint8_t i;
  memset(pwm_en, 0, MAX_PWM);

  uart_init(0);
  uart_set_input(0, test);

  PRINTF("\nStarting the test\n");

  for(i = 0; i < MAX_PWM; i++) {
    if(pwm_enable(pwm_num[i].freq, 0,
                  pwm_num[i].timer, pwm_num[i].ab) == PWM_SUCCESS) {
      pwm_en[i] = 1;
      PRINTF("%s (%u) configuration OK\n", gpt_name(pwm_num[i].timer),
             pwm_num[i].ab);
    }
    if((pwm_en[i]) &&
       (pwm_start(pwm_num[i].timer, pwm_num[i].ab,
                  pwm_num[i].port, pwm_num[i].pin) != PWM_SUCCESS)) {
      pwm_en[i] = 0;
      PRINTF("%s (%u) failed to start \n", gpt_name(pwm_num[i].timer),
             pwm_num[i].ab);
    }
  }

  while (1) {

    PROCESS_YIELD();

    if(ev == sensors_event) {
      if(data == &button_select_sensor) {

      }
    }
    // leds_toggle(LEDS_GREEN);
    // etimer_set(&et, CLOCK_SECOND);
    // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    // PRINTF("Toggling LED\n");
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
