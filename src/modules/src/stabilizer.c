/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 *
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "ext_position.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "crtp.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static void stabilizerTask(void* param);
static void sendLoggingData(void);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    //getExtPosition(&state);
#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &sensorData, &state, &setpoint, tick);
    powerDistribution(&control);
    sendLoggingData();

    tick++;
  }
}

void sendLoggingData()
{
  static float acc_avg[3] = {};
  static float gyro_avg[3] = {};
  static float mag_avg[3] = {};
  static float pos_avg[3] = {};
  static float vel_avg[3] = {};
  static float baro_avg = 0.0f;
  static uint8_t samples = 0;

  for (uint8_t i = 0; i < 3; i++) {
    acc_avg[i]  += sensorData.acc.axis[i];
    gyro_avg[i] += sensorData.gyro.axis[i];
    mag_avg[i]  += sensorData.mag.axis[i];
  }
  pos_avg[0]    += state.position.x;
  pos_avg[1]    += state.position.y;
  pos_avg[2]    += state.position.z;
  vel_avg[0]    += state.velocity.x;
  vel_avg[1]    += state.velocity.y;
  vel_avg[2]    += state.velocity.z;
  baro_avg      += sensorData.baro.asl;
  samples++;

  if (RATE_DO_EXECUTE(100, xTaskGetTickCount())) {
    float acc_out[3];
    float gyro_out[3];
    float mag_out[3];
    float pos_out[3];
    float vel_out[3];
    float baro_out;

    if (samples > 0) {
      for (uint8_t i = 0; i < 3; i++) {
        acc_out[i]  = acc_avg[i]  / samples;
        gyro_out[i] = gyro_avg[i] / samples;
        mag_out[i]  = mag_avg[i]  / samples;
        pos_out[i]  = pos_avg[i]  / samples;
        vel_out[i]  = vel_avg[i]  / samples;
        acc_avg[i]  = 0.0f;
        gyro_avg[i] = 0.0f;
        mag_avg[i]  = 0.0f;
        pos_out[i]  = 0.0f;
        vel_out[i]  = 0.0f;
      }
      baro_out = baro_avg / samples;
      baro_avg = 0.0f;
      samples = 0;
    }

    CRTPPacket p = {};
    p.port = CRTP_PORT_IMU;
    memcpy(p.data, acc_out, sizeof(float)*3);
    memcpy(p.data+sizeof(float)*3, gyro_out, sizeof(float)*3);
    p.size = sizeof(float)*6;
    crtpSendPacket(&p);

    CRTPPacket d = {};
    d.port = CRTP_PORT_MAG_BARO;
    memcpy(d.data, mag_out, sizeof(float)*3);
    memcpy(d.data+sizeof(float)*3, &baro_out, sizeof(float));
    d.size = sizeof(float)*4;
    crtpSendPacket(&d);

    CRTPPacket c = {};
    c.port = CRTP_PORT_STATE;
    memcpy(d.data, pos_out, sizeof(float)*3);
    memcpy(d.data+sizeof(float)*3, vel_out, sizeof(float));
    d.size = sizeof(float)*6;
    crtpSendPacket(&c);
  }
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)
