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

#define DEBUG_MODULE "GridEYE"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "debug.h"

#include "i2cdev.h"

#include "grideye.h"

#define GRIDEYE_DEFAULT_ADDRESS 0b1101000

#define GRIDEYE_RA_OPERATING_MODE 0x00
#define GRIDEYE_RA_SOFTWARE_RESET 0x01
#define GRIDEYE_RA_FRAME_RATE     0x02

#define GRIDEYE_RA_PIXEL_0_LOW    0x80
#define GRIDEYE_RA_PIXEL_0_HIGH   0x81

#define GRIDEYE_PIXEL_COUNT 64

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

void grideyeInit(I2C_Dev *i2cPort)
{
  if (isInit)
    return;

  I2Cx = i2cPort;
  devAddr = GRIDEYE_DEFAULT_ADDRESS;

  isInit = true;
}

bool grideyeTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus = grideyeTestConnection();

  return testStatus;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool grideyeTestConnection()
{
  bool ret = true;
  uint8_t output = 0;
  i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, 0x20);
  vTaskDelay(M2T(1));
  i2cdevReadByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, &output);
  ret &= (output == 0x20);
  DEBUG_PRINT("Mode: %d\n", output);
  i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, 0x00);
  vTaskDelay(M2T(1));
  i2cdevReadByte(I2Cx, devAddr, GRIDEYE_RA_OPERATING_MODE, &output);
  ret &= (output == 0x00);
  DEBUG_PRINT("Mode: %d\n", output);

  // Set frame rate to 1 hz
  i2cdevWriteByte(I2Cx, devAddr, GRIDEYE_RA_FRAME_RATE, 0x01);
  vTaskDelay(M2T(1));
  i2cdevReadByte(I2Cx, devAddr, GRIDEYE_RA_FRAME_RATE, &output);
  DEBUG_PRINT("Frame Rate: %d\n", output);
  return ret;
}

void grideyeReadData()
{
  for (uint8_t i = 0; i < GRIDEYE_PIXEL_COUNT*2; i=i+2) {
    uint8_t lowByte  = 0;
    uint8_t highByte = 0;

    i2cdevReadByte(I2Cx, devAddr, i, &lowByte);
    i2cdevReadByte(I2Cx, devAddr, i+1, &highByte);
    int16_t temp = (highByte << 8) | (lowByte);
    // Temperature data is in 2's compliment
    if (highByte != 0) {
      temp = -(2048 - temp);
    }
    float celsius = temp * 0.25f;
    DEBUG_PRINT("%.2f ", celsius);
    if (i % 8 == 0 && i != 0) {
      DEBUG_PRINT("\n");
    }
  }
}

