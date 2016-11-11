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
#include "crtp.h"

#include "grideye.h"

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

#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

static void grideyeInit(DeckInfo *info);
static bool grideyeTest(void);
static bool grideyeTestConnection();
static void grideyeTask(void *param);

static void send_pixel_packet(uint8_t index, uint8_t* data);
static uint8_t read_pixel(uint8_t reg);
static float pixel_to_temp(uint8_t pixel, float scale);

typedef struct {
  uint8_t row;
  uint8_t pixels[16];
} __attribute__ ((packed)) pixelPayload_t;

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

enum {
  ROTATION_NONE,
  ROTATION_90_CW,
  ROTATION_180_CW,
  ROTATION_270_CW,
};

static void grideyeTask(void *param)
{
  systemWaitStart();
  TickType_t xLastWakeTime;
  static uint8_t pixels[8*8] = {};
  static const uint8_t rotation = ROTATION_270_CW;

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    for (uint16_t i = GRIDEYE_RA_PIXEL_0_LOW;
        i < GRIDEYE_RA_PIXEL_63_HIGH; i+=2) {

      uint8_t index = (i - GRIDEYE_RA_PIXEL_0_LOW) >> 1;
      uint8_t row    = index / 8;
      uint8_t column = index % 8;
      uint8_t temp;
      switch (rotation) {
        case ROTATION_90_CW:
          temp = column;
          column = 7 - row;
          row = temp;
          break;
        case ROTATION_180_CW:
          column = 7 - column;
          row = 7 - row;
          break;
        case ROTATION_270_CW:
          temp = row;
          row = 7 - column;
          column = temp;
          break;
        case ROTATION_NONE:
        default:
          break;
      }
      index = row * 8 + column;
      pixels[index] = read_pixel(i);
      // TODO: Need a sleep in here to fly
      //send_pixel_packet(((i-GRIDEYE_RA_PIXEL_0_LOW)>>1), (uint8_t)celsius);
    }
    for (uint8_t i = 0; i < 4; i++) {
      send_pixel_packet(i*16, &(pixels[i*16]));
    }
    vTaskDelayUntil(&xLastWakeTime, M2T(1000));
  }
}

void grideyeGetSetpoint(setpoint_t *setpoint, const state_t* state)
{
  // TODO: Implement setpoint modification
}

static void send_pixel_packet(uint8_t index, uint8_t* data)
{
  CRTPPacket packet = {};
  packet.port = CRTP_PORT_IMAGE;
  packet.data[0] = (uint8_t)(index / 8.0f);
  memcpy(packet.data+1, data, sizeof(pixelPayload_t)-1);
  packet.size = sizeof(pixelPayload_t)+1;
  crtpSendPacket(&packet);
}

static uint8_t read_pixel(uint8_t reg)
{
  // Read high and low registers
  uint8_t pixel[2] = {};
  i2cdevRead(I2Cx, devAddr, reg, 2, (uint8_t*)&pixel);
  uint16_t ret = (uint16_t)((pixel[1] << 8) | pixel[0]) & GRIDEYE_DATA_MASK;
  return (uint8_t)(ret);
}

static float pixel_to_temp(uint8_t reg, float scale)
{
  int16_t temp = read_pixel(reg);
  // Temperature data is in 2's compliment
  if ((temp & GRIDEYE_SIGN_BIT) != 0) {
    temp -= GRIDEYE_SIGN_BIT;
  }
  return temp * scale;
}

#pragma GCC diagnostic pop

static const DeckDriver grideye_deck = {
  .vid = 0xBC,
  .pid = 0xFE,
  .name = "bcGRIDEYE",
  .usedGpio = 0, // FIXME: set the used pins

  .init = grideyeInit,
  .test = grideyeTest,
};

DECK_DRIVER(grideye_deck);

