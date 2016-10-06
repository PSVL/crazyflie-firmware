/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * grideye.c - AMG88XX Grid-EYE driver.
 */

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"

#include "i2cdev.h"

#define GRIDEYE_DEFAULT_ADDRESS 0b1101000

#define GRIDEYE_RA_OPERATING_MODE 0x00
#define GRIDEYE_RA_SOFTWARE_RESET 0x01
#define GRIDEYE_RA_FRAME_RATE     0x02

#define GRIDEYE_RA_THERM_LOW      0x0E
#define GRIDEYE_RA_THERM_HIGH     0x0F

#define GRIDEYE_RA_PIXEL_0_LOW    0x80
#define GRIDEYE_RA_PIXEL_0_HIGH   0x81

#define GRIDEYE_RA_PIXEL_63_LOW   0xFE
#define GRIDEYE_RA_PIXEL_63_HIGH  0xFF

#define GRIDEYE_PIXEL_COUNT 64
#define GRIDEYE_RES_BITS    12
#define GRIDEYE_SIGN_BIT    (1 << (GRIDEYE_RES_BITS - 1))
#define GRIDEYE_DATA_MASK   (GRIDEYE_SIGN_BIT - 1)

#define GRIDEYE_THERM_SCALE 0.0625f
#define GRIDEYE_PIXEL_SCALE 0.25f

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

static void grideyeInit(DeckInfo *info);
static bool grideyeTest(void);
static bool grideyeTestConnection();
static void grideyeTask(void *param);

static float pixel_to_temp(uint8_t pixel, float scale);

static void grideyeInit(DeckInfo *info)
{
  if (isInit)
    return;

  I2Cx = I2C1_DEV;
  i2cdevInit(I2Cx);
  devAddr = GRIDEYE_DEFAULT_ADDRESS;

  xTaskCreate(grideyeTask, "gridTask", 2*configMINIMAL_STACK_SIZE, NULL, 3, NULL);

  isInit = true;
}

static bool grideyeTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus = grideyeTestConnection();
  testStatus = true;

  return testStatus;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
static bool grideyeTestConnection()
{
  bool ret = true;
  uint8_t output = 0;
  i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, 0x20);
  vTaskDelay(M2T(1));
  i2cdevReadByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, &output);
  ret &= (output == 0x20);
  i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, 0x00);
  vTaskDelay(M2T(1));
  i2cdevReadByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, &output);
  ret &= (output == 0x00);

  // Set frame rate to 1 hz
  //i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_FRAME_RATE, 0x01);
  //vTaskDelay(M2T(1));
  return ret;
}

static void grideyeTask(void *param)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    float celsiusTherm = pixel_to_temp(GRIDEYE_RA_THERM_LOW, GRIDEYE_THERM_SCALE);

    float celsiusMax = -900.0f;
    float celsiusMin =  900.0f;
    for (uint16_t i = GRIDEYE_RA_PIXEL_0_LOW;
        i < GRIDEYE_RA_PIXEL_63_HIGH; i+=2) {

      float celsius = pixel_to_temp(i, GRIDEYE_PIXEL_SCALE);
      if (celsiusMax < celsius) {
        celsiusMax = celsius;
      }
      if (celsiusMin > celsius) {
        celsiusMin = celsius;
      }
      //if (i % 16 == 0) {
      //  DEBUG_PRINT("\n");
      //}
      //DEBUG_PRINT("%2.2f ", celsius);
    }
    //DEBUG_PRINT("\n%2.2f %2.2f %2.2f\n\n", celsiusTherm, celsiusMax, celsiusMin);
    vTaskDelayUntil(&xLastWakeTime, M2T(1000));
  }
}

static float pixel_to_temp(uint8_t reg, float scale)
{
  // Read high and low registers
  uint8_t pixel[2] = {};
  i2cdevRead(I2Cx, devAddr, reg, 2, (uint8_t*)&pixel);

  int16_t temp = ((pixel[1] << 8) | pixel[0]) & GRIDEYE_DATA_MASK;
  // Temperature data is in 2's compliment
  if ((temp & GRIDEYE_SIGN_BIT) != 0) {
    temp -= GRIDEYE_SIGN_BIT;
  }
  return temp * scale;
}

static const DeckDriver grideye_deck = {
  .vid = 0xBC,
  .pid = 0xFF,
  .name = "bcGRIDEYE",
  .usedGpio = 0, // FIXME: set the used pins

  .init = grideyeInit,
  .test = grideyeTest,
};

DECK_DRIVER(grideye_deck);

