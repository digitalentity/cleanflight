/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/fc_msp_box.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "sensors/diagnostics.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#define BOX_SUFFIX ';'
#define BOX_SUFFIX_LEN 1

static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM", 0 },
    { BOXANGLE, "ANGLE", 1 },
    { BOXHORIZON, "HORIZON", 2 },
    { BOXNAVALTHOLD, "NAV ALTHOLD", 3 },   // old BARO
    { BOXHEADINGHOLD, "HEADING HOLD", 5 },
    { BOXHEADFREE, "HEADFREE", 6 },
    { BOXHEADADJ, "HEADADJ", 7 },
    { BOXCAMSTAB, "CAMSTAB", 8 },
    { BOXNAVRTH, "NAV RTH", 10 },         // old GPS HOME
    { BOXNAVPOSHOLD, "NAV POSHOLD", 11 },     // old GPS HOLD
    { BOXMANUAL, "MANUAL", 12 },
    { BOXBEEPERON, "BEEPER", 13 },
    { BOXLEDLOW, "LEDLOW", 15 },
    { BOXLIGHTS, "LIGHTS", 16 },
    { BOXOSD, "OSD SW", 19 },
    { BOXTELEMETRY, "TELEMETRY", 20 },
    { BOXAUTOTUNE, "AUTO TUNE", 21 },
    { BOXBLACKBOX, "BLACKBOX", 26 },
    { BOXFAILSAFE, "FAILSAFE", 27 },
    { BOXNAVWP, "NAV WP", 28 },
    { BOXAIRMODE, "AIR MODE", 29 },
    { BOXHOMERESET, "HOME RESET", 30 },
    { BOXGCSNAV, "GCS NAV", 31 },
    //{ BOXHEADINGLOCK, "HEADING LOCK", 32 },
    { BOXSURFACE, "SURFACE", 33 },
    { BOXFLAPERON, "FLAPERON", 34 },
    { BOXTURNASSIST, "TURN ASSIST", 35 },
    { BOXNAVLAUNCH, "NAV LAUNCH", 36 },
    { BOXAUTOTRIM, "SERVO AUTOTRIM", 37 },
    { BOXKILLSWITCH, "KILLSWITCH", 38 },
    { BOXCAMERA1, "CAMERA CONTROL 1", 39 },
    { BOXCAMERA2, "CAMERA CONTROL 2", 40 },
    { BOXCAMERA3, "CAMERA CONTROL 3", 41 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
uint8_t activeBoxIdCount = 0;

const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

const box_t *findBoxByPermanentId(uint8_t permenantId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

bool serializeBoxNamesReply(sbuf_t *dst)
{
    // First run of the loop - calculate total length of the reply
    int replyLengthTotal = 0;
    for (int i = 0; i < activeBoxIdCount; i++) {
        const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
        if (box) {
            replyLengthTotal += strlen(box->boxName) + BOX_SUFFIX_LEN;
        }
    }

    // Check if we have enough space to send a reply
    if (sbufBytesRemaining(dst) < replyLengthTotal) {
        return false;
    }

    for (int i = 0; i < activeBoxIdCount; i++) {
        const int activeBoxId = activeBoxIds[i];
        const box_t *box = findBoxByActiveBoxId(activeBoxId);
        if (box) {
            const int len = strlen(box->boxName);
            sbufWriteData(dst, box->boxName, len);
            sbufWriteU8(dst, BOX_SUFFIX);
        }
    }

    return true;
}

void serializeBoxReply(sbuf_t *dst)
{
    for (int i = 0; i < activeBoxIdCount; i++) {
        const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
        if (!box) {
            continue;
        }
        sbufWriteU8(dst, box->permanentId);
    }
}

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    activeBoxIdCount = 0;
    activeBoxIds[activeBoxIdCount++] = BOXARM;

    if (sensors(SENSOR_ACC)) {
        activeBoxIds[activeBoxIdCount++] = BOXANGLE;
        activeBoxIds[activeBoxIdCount++] = BOXHORIZON;

#ifdef USE_FLM_TURN_ASSIST
        activeBoxIds[activeBoxIdCount++] = BOXTURNASSIST;
#endif
    }

    if (!feature(FEATURE_AIRMODE)) {
        activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;
    }

    activeBoxIds[activeBoxIdCount++] = BOXHEADINGHOLD;

    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
        activeBoxIds[activeBoxIdCount++] = BOXHEADADJ;
    }

    if (feature(FEATURE_SERVO_TILT))
        activeBoxIds[activeBoxIdCount++] = BOXCAMSTAB;

#ifdef USE_GPS
    if (sensors(SENSOR_BARO) || (STATE(FIXED_WING) && feature(FEATURE_GPS))) {
        activeBoxIds[activeBoxIdCount++] = BOXNAVALTHOLD;
        activeBoxIds[activeBoxIdCount++] = BOXSURFACE;
    }

    if (feature(FEATURE_GPS) && sensors(SENSOR_ACC) && imuHasHeadingEnabled()) {
        activeBoxIds[activeBoxIdCount++] = BOXNAVPOSHOLD;
        activeBoxIds[activeBoxIdCount++] = BOXNAVRTH;
        activeBoxIds[activeBoxIdCount++] = BOXNAVWP;
        activeBoxIds[activeBoxIdCount++] = BOXHOMERESET;
        activeBoxIds[activeBoxIdCount++] = BOXGCSNAV;
    }
#endif

    if (STATE(FIXED_WING)) {
        activeBoxIds[activeBoxIdCount++] = BOXMANUAL;
        if (!feature(FEATURE_FW_LAUNCH)) {
           activeBoxIds[activeBoxIdCount++] = BOXNAVLAUNCH;
        }
        activeBoxIds[activeBoxIdCount++] = BOXAUTOTRIM;
#if defined(AUTOTUNE_FIXED_WING)
        activeBoxIds[activeBoxIdCount++] = BOXAUTOTUNE;
#endif
    }

#ifdef USE_SERVOS
    /*
     * FLAPERON mode active only in case of airplane and custom airplane. Activating on
     * flying wing can cause bad thing
     */
    if (STATE(FLAPERON_AVAILABLE)) {
        activeBoxIds[activeBoxIdCount++] = BOXFLAPERON;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;

#ifdef USE_LIGHTS
    activeBoxIds[activeBoxIdCount++] = BOXLIGHTS;
#endif

#ifdef USE_LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        activeBoxIds[activeBoxIdCount++] = BOXLEDLOW;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXOSD;

#ifdef USE_TELEMETRY
    if (feature(FEATURE_TELEMETRY) && telemetryConfig()->telemetry_switch)
        activeBoxIds[activeBoxIdCount++] = BOXTELEMETRY;
#endif

#ifdef USE_BLACKBOX
    if (feature(FEATURE_BLACKBOX)){
        activeBoxIds[activeBoxIdCount++] = BOXBLACKBOX;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXKILLSWITCH;
    activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;

#ifdef USE_RCDEVICE
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA1;
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA2;
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA3;
#endif
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)
#define CHECK_ACTIVE_BOX(condition, index)    do { if (IS_ENABLED(condition)) { activeBoxes[index] = 1; } } while(0)

void packBoxModeFlags(boxBitmask_t * mspBoxModeFlags)
{
    uint8_t activeBoxes[CHECKBOX_ITEM_COUNT];
    memset(activeBoxes, 0, sizeof(activeBoxes));

    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)),           BOXANGLE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)),         BOXHORIZON);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADING_MODE)),         BOXHEADINGHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)),        BOXHEADFREE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)),     BOXHEADADJ);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)),     BOXCAMSTAB);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(MANUAL_MODE)),          BOXMANUAL);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)),    BOXBEEPERON);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)),      BOXLEDLOW);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLIGHTS)),      BOXLIGHTS);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)),         BOXOSD);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)),   BOXTELEMETRY);
    CHECK_ACTIVE_BOX(IS_ENABLED(ARMING_FLAG(ARMED)),                BOXARM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)),    BOXBLACKBOX);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)),        BOXFAILSAFE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_ALTHOLD_MODE)),     BOXNAVALTHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_POSHOLD_MODE)),     BOXNAVPOSHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_RTH_MODE)),         BOXNAVRTH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_WP_MODE)),          BOXNAVWP);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)),     BOXAIRMODE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGCSNAV)),      BOXGCSNAV);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXSURFACE)),     BOXSURFACE);
#ifdef USE_FLM_FLAPERON
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FLAPERON)),             BOXFLAPERON);
#endif
#ifdef USE_FLM_TURN_ASSIST
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(TURN_ASSISTANT)),       BOXTURNASSIST);
#endif
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_LAUNCH_MODE)),      BOXNAVLAUNCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(AUTO_TUNE)),            BOXAUTOTUNE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAUTOTRIM)),    BOXAUTOTRIM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXKILLSWITCH)),  BOXKILLSWITCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHOMERESET)),   BOXHOMERESET);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA1)),     BOXCAMERA1);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA2)),     BOXCAMERA2);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA3)),     BOXCAMERA3);

    memset(mspBoxModeFlags, 0, sizeof(boxBitmask_t));
    for (uint32_t i = 0; i < activeBoxIdCount; i++) {
        if (activeBoxes[activeBoxIds[i]]) {
            bitArraySet(mspBoxModeFlags->bits, i);
        }
    }
}

uint16_t packSensorStatus(void)
{
    // Sensor bits
    uint16_t sensorStatus =
            IS_ENABLED(sensors(SENSOR_ACC))     << 0 |
            IS_ENABLED(sensors(SENSOR_BARO))    << 1 |
            IS_ENABLED(sensors(SENSOR_MAG))     << 2 |
            IS_ENABLED(sensors(SENSOR_GPS))     << 3 |
            IS_ENABLED(sensors(SENSOR_RANGEFINDER))   << 4 |
            //IS_ENABLED(sensors(SENSOR_OPFLOW))  << 5 |
            IS_ENABLED(sensors(SENSOR_PITOT))   << 6;

    // Hardware failure indication bit
    if (!isHardwareHealthy()) {
        sensorStatus |= 1 << 15;        // Bit 15 of sensor bit field indicates hardware failure
    }

    return sensorStatus;
}
