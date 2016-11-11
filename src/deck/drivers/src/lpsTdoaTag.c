/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoaTag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>

#include "log.h"
#include "lpsTdoaTag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];

static lpsAlgoOptions_t* options;

static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];

static double frameTimeInMasterClock = 0.0;
static double localClockCorrection = 1.0;

#define MASTER 0

#define CAP_timer

static uint64_t timestampToUint64(uint8_t *ts) {
  dwTime_t timestamp = {.full = 0};
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static uint64_t truncateToTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFFFul;
}

static void rxcallback(dwDevice_t *dev) {
  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  uint8_t anchor = rxPacket.sourceAddress & 0xff;

  if (anchor < LOCODECK_NR_OF_ANCHORS) {
    rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    if (anchor == MASTER) {
      frameTimeInMasterClock = truncateToTimeStamp(timestampToUint64(packet->timestamps[MASTER]) - timestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]));
      double frameTimeInLocalClock = truncateToTimeStamp(arrival.full - arrivals[MASTER].full);

      localClockCorrection = 1.0;
      if (frameTimeInLocalClock != 0.0) {
        localClockCorrection = frameTimeInMasterClock / frameTimeInLocalClock;
      }
    } else {
      double frameTimeInAnchorClock = truncateToTimeStamp(timestampToUint64(packet->timestamps[MASTER]) - timestampToUint64(rxPacketBuffer[anchor].timestamps[MASTER]));

      double anchorClockCorrection = 1.0;
      if (frameTimeInAnchorClock != 0.0) {
        anchorClockCorrection = frameTimeInMasterClock / frameTimeInAnchorClock;
      }

        float tdoaDistDiff;

        int64_t txAn_X = timestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]);
        int64_t txA0_X = timestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]);
        int64_t rxAn_0 = timestampToUint64(rxPacketBuffer[MASTER].timestamps[anchor]);
        int64_t rxA0_n = timestampToUint64(packet->timestamps[MASTER]);

        int64_t rxT_0  = arrivals[MASTER].full;
        int64_t rxT_n  = arrival.full;
        int64_t txAn_X2 = timestampToUint64(packet->timestamps[anchor]);


        int64_t tA0_n = (((truncateToTimeStamp(rxA0_n - txAn_X) * anchorClockCorrection) - truncateToTimeStamp(txA0_X - rxAn_0))) / 2.0;
        int64_t tT =  truncateToTimeStamp(rxT_n - rxT_0) * localClockCorrection - (tA0_n + truncateToTimeStamp(txAn_X2 - rxA0_n) * anchorClockCorrection);

        tdoaDistDiff = SPEED_OF_LIGHT * tT / LOCODECK_TS_FREQ;
        // Sanity check distances in case of missed packages
        if (tdoaDistDiff < 300 && tdoaDistDiff > -300)
        {
          uwbTdoaDistDiff[anchor] = tdoaDistDiff;
        }
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      setRadioInReceiveMode(dev);
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
      break;
    default:
      ASSERT_FAILED();
  }

  return MAX_TIMEOUT;
}

static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;
}

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
};


LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d01, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d02, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d03, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d04, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d05, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d06, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d07, &uwbTdoaDistDiff[7])
LOG_GROUP_STOP(tdoa)
