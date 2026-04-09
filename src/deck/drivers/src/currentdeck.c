/*
 * Driver for the Current Sensor
 */
#define DEBUG_MODULE "CS"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "system.h"
#include "queue.h"
#include "deck.h"
#include "uart2.h"
#include "stabilizer.h"
#include "pm.h"
#include "currentdeck.h"

static bool isInit = false;
static TaskHandle_t xHandle = NULL;


static float reading_last = 0.0f; 


void currentTask(void *param)
{
    systemWaitStart();
    adcInit();

    while (1)
    {
        vTaskDelay(M2T(1));
        reading_last = analogReadVoltage(DECK_GPIO_SCK);
    }
}

static void currentInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Currnet Initialize.\n");

    xTaskCreate(currentTask, "CURRENT_TASK",
                configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

    isInit = true;
}

void currentDeckForceInit(void)
{
  currentInit(NULL);
}

static bool currentTest()
{
    if (!isInit)
        return false;

    DEBUG_PRINT("Test passed.\n");

    return true;
}

static const DeckDriver current_sampling = {
    .vid = 0,
    .pid = 0,
    .name = "currentdeck",
    .usedGpio = DECK_USING_PA5,
    // .usedPeriph = 0,
    .init = currentInit,
    .test = currentTest,
};

DECK_DRIVER(current_sampling);

// PARAM_GROUP_START(current_sampling)
// PARAM_GROUP_STOP(current_sampling)

LOG_GROUP_START(current_log)
LOG_ADD(LOG_FLOAT, adcread, & reading_last)
LOG_GROUP_STOP(current_log)