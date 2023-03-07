/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * servodeck.c - Deck driver for a servo motor
 * Connect power to VCOM (pin 9 right), ground to GND (pin 10 left), signal to TX (pin 1 right)
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "deck.h"
#include "param.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "stabilizer.h"// my code obtain servo ratio from stabilizer

// HW defines
#define SERVO_TIM_PERIF       RCC_APB1Periph_TIM5
#define SERVO_TIM             TIM5
#define SERVO_TIM_DBG         DBGMCU_TIM2_STOP
#define SERVO_TIM_SETCOMPARE  TIM_SetCompare2
#define SERVO_TIM_GETCAPTURE  TIM_GetCapture2

#define SERVO_GPIO_PERIF         RCC_AHB1Periph_GPIOA
#define SERVO_GPIO_PORT          GPIOA
#define SERVO_GPIO_PIN           GPIO_Pin_2 // TIM5_CH3
#define SERVO_GPIO_AF_PIN        GPIO_PinSource2
#define SERVO_GPIO_AF            GPIO_AF_TIM5

#define SERVO_PWM_BITS      (8)
#define SERVO_PWM_PERIOD    ((1<<SERVO_PWM_BITS) - 1)
#define SERVO_PWM_PRESCALE  (0)

/* This should be calculated.. */
#define SERVO_BASE_FREQ (329500)

static bool isInit;
static xTimerHandle timer;
// static uint8_t ratio = 237; // ~ 1.5ms pulse width (= neutral position)
// static uint8_t ratio = getservoRatio(); // ~ 1.5ms pulse width (= neutral position)

// measured:
// 243: 1.0ms (lower bound for most servos)
// 237: 1.5ms (neutral for most servos)
// 230: 2.0ms (upper bound for most servos)

static uint16_t freq = 50;


 

static void servoSetRatio(uint8_t ratio)
{
  TIM_SetCompare3(SERVO_TIM, ratio);
}

static void servoSetFreq(uint16_t freq)
{
  TIM_PrescalerConfig(SERVO_TIM, (SERVO_BASE_FREQ/freq), TIM_PSCReloadMode_Update);
}

static void servoTimer(xTimerHandle timer)
{

  servoSetRatio(getservoRatio());
  servoSetFreq(freq);
}

static void servoDeckInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(SERVO_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(SERVO_TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
  GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(SERVO_GPIO_PORT, SERVO_GPIO_AF_PIN, SERVO_GPIO_AF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OC3Init(SERVO_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

  // Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(SERVO_TIM, ENABLE);
  TIM_SetCompare3(SERVO_TIM, 0x00);

  // Enable the timer
  TIM_Cmd(SERVO_TIM, ENABLE);


  // use a timer to update based on parameters
  timer = xTimerCreate("servoTimer", M2T(50), pdTRUE, NULL, servoTimer);
  xTimerStart(timer, 100);

  isInit = true;
}

static const DeckDriver servo_deck = {
  // .vid = 0xBC,
  // .pid = 0x04,
  .name = "bcServo",

  .usedPeriph = DECK_USING_TIMER5,
  .usedGpio = DECK_USING_TX2,

  .init = servoDeckInit,
};

DECK_DRIVER(servo_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcServo, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(servo)
// PARAM_ADD(PARAM_UINT8, ratio, &ratio)
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(servo)
