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
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(USE_NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/log.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/settings.h"

#include "flight/imu.h"
#include "flight/secondary_imu.h"

#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"
#include "sensors/opflow.h"

navigationPosEstimator_t posEstimator;

PG_REGISTER_WITH_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig, PG_POSITION_ESTIMATION_CONFIG, 5);

PG_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig,
        // Inertial position estimator parameters
        .automatic_mag_declination = SETTING_INAV_AUTO_MAG_DECL_DEFAULT,
        .reset_altitude_type = SETTING_INAV_RESET_ALTITUDE_DEFAULT,
        .reset_home_type = SETTING_INAV_RESET_HOME_DEFAULT,
        .gravity_calibration_tolerance = SETTING_INAV_GRAVITY_CAL_TOLERANCE_DEFAULT,  // 5 cm/s/s calibration error accepted (0.5% of gravity)
        .use_gps_velned = SETTING_INAV_USE_GPS_VELNED_DEFAULT,                        // "Disabled" is mandatory with gps_dyn_model = Pedestrian
        .use_gps_no_baro = SETTING_INAV_USE_GPS_NO_BARO_DEFAULT,                      // Use GPS altitude if no baro is available on all aircrafts
        .allow_dead_reckoning = SETTING_INAV_ALLOW_DEAD_RECKONING_DEFAULT,

        .max_surface_altitude = SETTING_INAV_MAX_SURFACE_ALTITUDE_DEFAULT,

        .w_xyz_acc_p = SETTING_INAV_W_XYZ_ACC_P_DEFAULT,

        .w_z_baro_p = SETTING_INAV_W_Z_BARO_P_DEFAULT,

        .w_z_surface_p = SETTING_INAV_W_Z_SURFACE_P_DEFAULT,
        .w_z_surface_v = SETTING_INAV_W_Z_SURFACE_V_DEFAULT,

        .w_z_gps_p = SETTING_INAV_W_Z_GPS_P_DEFAULT,
        .w_z_gps_v = SETTING_INAV_W_Z_GPS_V_DEFAULT,

        .w_xy_gps_p = SETTING_INAV_W_XY_GPS_P_DEFAULT,
        .w_xy_gps_v = SETTING_INAV_W_XY_GPS_V_DEFAULT,

        .w_xy_flow_p = SETTING_INAV_W_XY_FLOW_P_DEFAULT,
        .w_xy_flow_v = SETTING_INAV_W_XY_FLOW_V_DEFAULT,

        .w_z_res_v = SETTING_INAV_W_Z_RES_V_DEFAULT,
        .w_xy_res_v = SETTING_INAV_W_XY_RES_V_DEFAULT,

        .w_acc_bias = SETTING_INAV_W_ACC_BIAS_DEFAULT,

        .max_eph_epv = SETTING_INAV_MAX_EPH_EPV_DEFAULT,
        .baro_epv = SETTING_INAV_BARO_EPV_DEFAULT
);

#define resetTimer(tim, currentTimeUs) { (tim)->deltaTime = 0; (tim)->lastTriggeredTime = currentTimeUs; }
#define getTimerDeltaMicros(tim) ((tim)->deltaTime)
static bool updateTimer(navigationTimer_t * tim, timeUs_t interval, timeUs_t currentTimeUs)
{
    if ((currentTimeUs - tim->lastTriggeredTime) >= interval) {
        tim->deltaTime = currentTimeUs - tim->lastTriggeredTime;
        tim->lastTriggeredTime = currentTimeUs;
        return true;
    }
    else {
        return false;
    }
}

static bool shouldResetReferenceAltitude(void)
{
    switch ((nav_reset_type_e)positionEstimationConfig()->reset_altitude_type) {
        case NAV_RESET_NEVER:
            return false;
        case NAV_RESET_ON_FIRST_ARM:
            return !ARMING_FLAG(ARMED) && !ARMING_FLAG(WAS_EVER_ARMED);
        case NAV_RESET_ON_EACH_ARM:
            return !ARMING_FLAG(ARMED);
    }

    return false;
}

#if defined(USE_GPS)
/* Why is this here: Because GPS will be sending at quiet a nailed rate (if not overloaded by junk tasks at the brink of its specs)
 * but we might read out with timejitter because Irq might be off by a few us so we do a +-10% margin around the time between GPS
 * datasets representing the most common Hz-rates today. You might want to extend the list or find a smarter way.
 * Don't overload your GPS in its config with trash, choose a Hz rate that it can deliver at a sustained rate.
 * (c) CrashPilot1000
 */
static timeUs_t getGPSDeltaTimeFilter(timeUs_t dTus)
{
    if (dTus >= 225000 && dTus <= 275000) return HZ2US(4);       //  4Hz Data 250ms
    if (dTus >= 180000 && dTus <= 220000) return HZ2US(5);       //  5Hz Data 200ms
    if (dTus >=  90000 && dTus <= 110000) return HZ2US(10);      // 10Hz Data 100ms
    if (dTus >=  45000 && dTus <=  55000) return HZ2US(20);      // 20Hz Data  50ms
    if (dTus >=  30000 && dTus <=  36000) return HZ2US(30);      // 30Hz Data  33ms
    if (dTus >=  23000 && dTus <=  27000) return HZ2US(40);      // 40Hz Data  25ms
    if (dTus >=  18000 && dTus <=  22000) return HZ2US(50);      // 50Hz Data  20ms
    return dTus;                                                 // Filter failed. Set GPS Hz by measurement
}

#if defined(NAV_GPS_GLITCH_DETECTION)
static bool detectGPSGlitch(timeUs_t currentTimeUs)
{
    static timeUs_t previousTime = 0;
    static fpVector3_t lastKnownGoodPosition;
    static fpVector3_t lastKnownGoodVelocity;

    bool isGlitching = false;

    if (previousTime == 0) {
        isGlitching = false;
    }
    else {
        fpVector3_t predictedGpsPosition;
        float gpsDistance;
        float dT = US2S(currentTimeUs - previousTime);

        /* We predict new position based on previous GPS velocity and position */
        predictedGpsPosition.x = lastKnownGoodPosition.x + lastKnownGoodVelocity.x * dT;
        predictedGpsPosition.y = lastKnownGoodPosition.y + lastKnownGoodVelocity.y * dT;

        /* New pos is within predefined radius of predicted pos, radius is expanded exponentially */
        gpsDistance = sqrtf(sq(predictedGpsPosition.x - lastKnownGoodPosition.x) + sq(predictedGpsPosition.y - lastKnownGoodPosition.y));
        if (gpsDistance <= (INAV_GPS_GLITCH_RADIUS + 0.5f * INAV_GPS_GLITCH_ACCEL * dT * dT)) {
            isGlitching = false;
        }
        else {
            isGlitching = true;
        }
    }

    if (!isGlitching) {
        previousTime = currentTimeUs;
        lastKnownGoodPosition = posEstimator.gps.pos;
        lastKnownGoodVelocity = posEstimator.gps.vel;
    }

    return isGlitching;
}
#endif

/**
 * Update GPS topic
 *  Function is called on each GPS update
 */
void onNewGPSData(void)
{
    static timeUs_t lastGPSNewDataTime;
    static int32_t previousLat;
    static int32_t previousLon;
    static int32_t previousAlt;
    static bool isFirstGPSUpdate = true;

    gpsLocation_t newLLH;
    const timeUs_t currentTimeUs = micros();

    newLLH.lat = gpsSol.llh.lat;
    newLLH.lon = gpsSol.llh.lon;
    newLLH.alt = gpsSol.llh.alt;

    if (sensors(SENSOR_GPS)) {
        if (!STATE(GPS_FIX)) {
            isFirstGPSUpdate = true;
            return;
        }

        if ((currentTimeUs - lastGPSNewDataTime) > MS2US(INAV_GPS_TIMEOUT_MS)) {
            isFirstGPSUpdate = true;
        }

        /* Automatic magnetic declination calculation - do this once */
        if(STATE(GPS_FIX_HOME)){
            static bool magDeclinationSet = false;
            if (positionEstimationConfig()->automatic_mag_declination && !magDeclinationSet) {
                const float declination = geoCalculateMagDeclination(&newLLH); 
                imuSetMagneticDeclination(declination);
#ifdef USE_SECONDARY_IMU
                secondaryImuSetMagneticDeclination(declination);
#endif
                magDeclinationSet = true;
            }
        }
        /* Process position update if GPS origin is already set, or precision is good enough */
        // FIXME: Add HDOP check for acquisition of GPS origin
        /* Set GPS origin or reset the origin altitude - keep initial pre-arming altitude at zero */
        if (!posControl.gpsOrigin.valid) {
            geoSetOrigin(&posControl.gpsOrigin, &newLLH, GEO_ORIGIN_SET);
        }
        else if (shouldResetReferenceAltitude()) {
            /* If we were never armed - keep altitude at zero */
            geoSetOrigin(&posControl.gpsOrigin, &newLLH, GEO_ORIGIN_RESET_ALTITUDE);
        }

        if (posControl.gpsOrigin.valid) {
            /* Convert LLH position to local coordinates */
            geoConvertGeodeticToLocal(&posEstimator.gps.pos, &posControl.gpsOrigin, &newLLH, GEO_ALT_ABSOLUTE);

            /* If not the first update - calculate velocities */
            if (!isFirstGPSUpdate) {
                float dT = US2S(getGPSDeltaTimeFilter(currentTimeUs - lastGPSNewDataTime));

                /* Use VELNED provided by GPS if available, calculate from coordinates otherwise */
                float gpsScaleLonDown = constrainf(cos_approx((ABS(gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelNE) {
                    posEstimator.gps.vel.x = gpsSol.velNED[0];
                    posEstimator.gps.vel.y = gpsSol.velNED[1];
                }
                else {
                    posEstimator.gps.vel.x = (posEstimator.gps.vel.x + (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lat - previousLat) / dT)) / 2.0f;
                    posEstimator.gps.vel.y = (posEstimator.gps.vel.y + (gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lon - previousLon) / dT)) / 2.0f;
                }

                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelD) {
                    posEstimator.gps.vel.z = - gpsSol.velNED[2];   // NEU
                }
                else {
                    posEstimator.gps.vel.z = (posEstimator.gps.vel.z + (gpsSol.llh.alt - previousAlt) / dT) / 2.0f;
                }

#if defined(NAV_GPS_GLITCH_DETECTION)
                /* GPS glitch protection. We have local coordinates and local velocity for current GPS update. Check if they are sane */
                if (detectGPSGlitch(currentTimeUs)) {
                    posEstimator.gps.glitchRecovery = false;
                    posEstimator.gps.glitchDetected = true;
                }
                else {
                    /* Store previous glitch flag in glitchRecovery to indicate a valid reading after a glitch */
                    posEstimator.gps.glitchRecovery = posEstimator.gps.glitchDetected;
                    posEstimator.gps.glitchDetected = false;
                }
#endif

                /* FIXME: use HDOP/VDOP */
                if (gpsSol.flags.validEPE) {
                    posEstimator.gps.eph = gpsSol.eph;
                    posEstimator.gps.epv = gpsSol.epv;
                }
                else {
                    posEstimator.gps.eph = INAV_GPS_DEFAULT_EPH;
                    posEstimator.gps.epv = INAV_GPS_DEFAULT_EPV;
                }

                /* Indicate a last valid reading of Pos/Vel */
                posEstimator.gps.lastUpdateTime = currentTimeUs;
            }

            previousLat = gpsSol.llh.lat;
            previousLon = gpsSol.llh.lon;
            previousAlt = gpsSol.llh.alt;
            isFirstGPSUpdate = false;

            lastGPSNewDataTime = currentTimeUs;
        }
    }
    else {
        posEstimator.gps.lastUpdateTime = 0;
    }
}
#endif

#if defined(USE_BARO)
/**
 * Read BARO and update alt/vel topic
 *  Function is called from TASK_BARO
 */
void updatePositionEstimator_BaroTopic(timeUs_t currentTimeUs)
{
    static float initialBaroAltitudeOffset = 0.0f;
    float newBaroAlt = baroCalculateAltitude();

    /* If we are required - keep altitude at zero */
    if (shouldResetReferenceAltitude()) {
        initialBaroAltitudeOffset = newBaroAlt;
    }

    if (sensors(SENSOR_BARO) && baroIsCalibrationComplete()) {
        const timeUs_t baroDtUs = currentTimeUs - posEstimator.baro.lastUpdateTime;

        posEstimator.baro.alt = newBaroAlt - initialBaroAltitudeOffset;
        posEstimator.baro.epv = positionEstimationConfig()->baro_epv;
        posEstimator.baro.lastUpdateTime = currentTimeUs;

        if (baroDtUs <= MS2US(INAV_BARO_TIMEOUT_MS)) {
            pt1FilterApply3(&posEstimator.baro.avgFilter, posEstimator.baro.alt, US2S(baroDtUs));
        }
    }
    else {
        posEstimator.baro.alt = 0;
        posEstimator.baro.lastUpdateTime = 0;
    }
}
#endif

#if defined(USE_PITOT)
/**
 * Read Pitot and update airspeed topic
 *  Function is called at main loop rate, updates happen at reduced rate
 */
void updatePositionEstimator_PitotTopic(timeUs_t currentTimeUs)
{
    posEstimator.pitot.airspeed = pitot.airSpeed;
    posEstimator.pitot.lastUpdateTime = currentTimeUs;
}
#endif

/**
 * Update IMU topic
 *  Function is called at main loop rate
 */
static void restartGravityCalibration(void)
{
    zeroCalibrationStartS(&posEstimator.imu.gravityCalibration, CALIBRATING_GRAVITY_TIME_MS, positionEstimationConfig()->gravity_calibration_tolerance, false);
}

static bool gravityCalibrationComplete(void)
{
    return zeroCalibrationIsCompleteS(&posEstimator.imu.gravityCalibration);
}

static void updateIMUEstimationWeight(const float dt)
{
    bool isAccClipped = accIsClipped();

    // If accelerometer measurement is clipped - drop the acc weight to zero
    // and gradually restore weight back to 1.0 over time
    if (isAccClipped) {
        posEstimator.imu.accWeightFactor = 0.0f;
    }
    else {
        const float relAlpha = dt / (dt + INAV_ACC_CLIPPING_RC_CONSTANT);
        posEstimator.imu.accWeightFactor = posEstimator.imu.accWeightFactor * (1.0f - relAlpha) + 1.0f * relAlpha;
    }

    // DEBUG_VIBE[0-3] are used in IMU
    DEBUG_SET(DEBUG_VIBE, 4, posEstimator.imu.accWeightFactor * 1000);
}

float navGetAccelerometerWeight(void)
{
    const float accWeightScaled = posEstimator.imu.accWeightFactor * positionEstimationConfig()->w_xyz_acc_p;
    DEBUG_SET(DEBUG_VIBE, 5, accWeightScaled * 1000);

    return accWeightScaled;
}

static void updateIMUTopic(timeUs_t currentTimeUs)
{
    const float dt = US2S(currentTimeUs - posEstimator.imu.lastUpdateTime);
    posEstimator.imu.lastUpdateTime = currentTimeUs;

    if (!isImuReady()) {
        posEstimator.imu.accelNEU.x = 0;
        posEstimator.imu.accelNEU.y = 0;
        posEstimator.imu.accelNEU.z = 0;

        restartGravityCalibration();
    }
    else {
        /* Update acceleration weight based on vibration levels and clipping */
        updateIMUEstimationWeight(dt);

        fpVector3_t accelBF;

        /* Read acceleration data in body frame */
        accelBF.x = imuMeasuredAccelBF.x;
        accelBF.y = imuMeasuredAccelBF.y;
        accelBF.z = imuMeasuredAccelBF.z;

        /* Correct accelerometer bias */
        accelBF.x -= posEstimator.imu.accelBias.x;
        accelBF.y -= posEstimator.imu.accelBias.y;
        accelBF.z -= posEstimator.imu.accelBias.z;

        /* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
        imuTransformVectorBodyToEarth(&accelBF);

        /* Read acceleration data in NEU frame from IMU */
        posEstimator.imu.accelNEU.x = accelBF.x;
        posEstimator.imu.accelNEU.y = accelBF.y;
        posEstimator.imu.accelNEU.z = accelBF.z;

        /* When unarmed, assume that accelerometer should measure 1G. Use that to correct accelerometer gain */
        if (!ARMING_FLAG(ARMED) && !gravityCalibrationComplete()) {
            zeroCalibrationAddValueS(&posEstimator.imu.gravityCalibration, posEstimator.imu.accelNEU.z);

            if (gravityCalibrationComplete()) {
                zeroCalibrationGetZeroS(&posEstimator.imu.gravityCalibration, &posEstimator.imu.calibratedGravityCMSS);
                LOG_D(POS_ESTIMATOR, "Gravity calibration complete (%d)", (int)lrintf(posEstimator.imu.calibratedGravityCMSS));
            }
        }

        /* If calibration is incomplete - report zero acceleration */
        if (gravityCalibrationComplete()) {
            posEstimator.imu.accelNEU.z -= posEstimator.imu.calibratedGravityCMSS;
        }
        else {
            posEstimator.imu.accelNEU.x = 0;
            posEstimator.imu.accelNEU.y = 0;
            posEstimator.imu.accelNEU.z = 0;
        }

#if defined(NAV_BLACKBOX)
        /* Update blackbox values */
        navAccNEU[X] = posEstimator.imu.accelNEU.x;
        navAccNEU[Y] = posEstimator.imu.accelNEU.y;
        navAccNEU[Z] = posEstimator.imu.accelNEU.z;
#endif
    }
}

float updateEPE(const float oldEPE, const float dt, const float newEPE, const float w)
{
    return oldEPE + (newEPE - oldEPE) * w * dt;
}

static bool navIsAccelerationUsable(void)
{
    return true;
}

static bool navIsHeadingUsable(void)
{
    if (sensors(SENSOR_GPS)) {
        // If we have GPS - we need true IMU north (valid heading)
        return isImuHeadingValid();
    }
    else {
        // If we don't have GPS - we may use whatever we have, other sensors are operating in body frame
        return isImuHeadingValid() || positionEstimationConfig()->allow_dead_reckoning;
    }
}

static uint32_t calculateCurrentValidityFlags(timeUs_t currentTimeUs)
{
    /* Figure out if we have valid position data from our data sources */
    uint32_t newFlags = 0;

    if (sensors(SENSOR_GPS) && posControl.gpsOrigin.valid &&
        ((currentTimeUs - posEstimator.gps.lastUpdateTime) <= MS2US(INAV_GPS_TIMEOUT_MS)) &&
        (posEstimator.gps.eph < positionEstimationConfig()->max_eph_epv)) {
        if (posEstimator.gps.epv < positionEstimationConfig()->max_eph_epv) {
            newFlags |= EST_GPS_XY_VALID | EST_GPS_Z_VALID;
        }
        else {
            newFlags |= EST_GPS_XY_VALID;
        }
    }

    if (sensors(SENSOR_BARO) && ((currentTimeUs - posEstimator.baro.lastUpdateTime) <= MS2US(INAV_BARO_TIMEOUT_MS))) {
        newFlags |= EST_BARO_VALID;
    }

    if (sensors(SENSOR_RANGEFINDER) && ((currentTimeUs - posEstimator.surface.lastUpdateTime) <= MS2US(INAV_SURFACE_TIMEOUT_MS))) {
        newFlags |= EST_SURFACE_VALID;
    }

    if (sensors(SENSOR_OPFLOW) && posEstimator.flow.isValid && ((currentTimeUs - posEstimator.flow.lastUpdateTime) <= MS2US(INAV_FLOW_TIMEOUT_MS))) {
        newFlags |= EST_FLOW_VALID;
    }

    if (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv) {
        newFlags |= EST_XY_VALID;
    }

    if (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv) {
        newFlags |= EST_Z_VALID;
    }

    return newFlags;
}

static void estimationPredict(estimationContext_t * ctx)
{
    const float accWeight = navGetAccelerometerWeight();

    /* Prediction step: Z-axis */
    if ((ctx->newFlags & EST_Z_VALID)) {
        posEstimator.est.pos.z += posEstimator.est.vel.z * ctx->dt;
        posEstimator.est.pos.z += posEstimator.imu.accelNEU.z * sq(ctx->dt) / 2.0f * accWeight;
        posEstimator.est.vel.z += posEstimator.imu.accelNEU.z * ctx->dt * sq(accWeight);
    }

    /* Prediction step: XY-axis */
    if ((ctx->newFlags & EST_XY_VALID)) {
        // Predict based on known velocity
        posEstimator.est.pos.x += posEstimator.est.vel.x * ctx->dt;
        posEstimator.est.pos.y += posEstimator.est.vel.y * ctx->dt;

        // If heading is valid, accelNEU is valid as well. Account for acceleration
        if (navIsHeadingUsable() && navIsAccelerationUsable()) {
            posEstimator.est.pos.x += posEstimator.imu.accelNEU.x * sq(ctx->dt) / 2.0f * accWeight;
            posEstimator.est.pos.y += posEstimator.imu.accelNEU.y * sq(ctx->dt) / 2.0f * accWeight;
            posEstimator.est.vel.x += posEstimator.imu.accelNEU.x * ctx->dt * sq(accWeight);
            posEstimator.est.vel.y += posEstimator.imu.accelNEU.y * ctx->dt * sq(accWeight);
        }
    }
}

static bool estimationCalculateCorrection_Z(estimationContext_t * ctx)
{
    if (ctx->newFlags & EST_BARO_VALID) {
        timeUs_t currentTimeUs = micros();

        if (!ARMING_FLAG(ARMED)) {
            posEstimator.state.baroGroundAlt = posEstimator.est.pos.z;
            posEstimator.state.isBaroGroundValid = true;
            posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
        }
        else {
            if (posEstimator.est.vel.z > 15) {
                if (currentTimeUs > posEstimator.state.baroGroundTimeout) {
                    posEstimator.state.isBaroGroundValid = false;
                }
            }
            else {
                posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
            }
        }

        // We might be experiencing air cushion effect - use sonar or baro groung altitude to detect it
        bool isAirCushionEffectDetected = ARMING_FLAG(ARMED) &&
                                            (((ctx->newFlags & EST_SURFACE_VALID) && posEstimator.surface.alt < 20.0f && posEstimator.state.isBaroGroundValid) ||
                                             ((ctx->newFlags & EST_BARO_VALID) && posEstimator.state.isBaroGroundValid && posEstimator.baro.alt < posEstimator.state.baroGroundAlt));

        // Altitude
        const float baroAltResidual = (isAirCushionEffectDetected ? posEstimator.state.baroGroundAlt : posEstimator.baro.alt) - posEstimator.est.pos.z;
        ctx->estPosCorr.z += baroAltResidual * positionEstimationConfig()->w_z_baro_p * ctx->dt;
        ctx->estVelCorr.z += baroAltResidual * sq(positionEstimationConfig()->w_z_baro_p) * ctx->dt;

        // If GPS is available - also use GPS climb rate
        if (ctx->newFlags & EST_GPS_Z_VALID) {
            // Trust GPS velocity only if residual/error is less than 2.5 m/s, scale weight according to gaussian distribution
            const float gpsRocResidual = posEstimator.gps.vel.z - posEstimator.est.vel.z;
            const float gpsRocScaler = bellCurve(gpsRocResidual, 250.0f);
            ctx->estVelCorr.z += gpsRocResidual * positionEstimationConfig()->w_z_gps_v * gpsRocScaler * ctx->dt;
        }

        ctx->newEPV = updateEPE(posEstimator.est.epv, ctx->dt, posEstimator.baro.epv, positionEstimationConfig()->w_z_baro_p);

        // Accelerometer bias
        if (!isAirCushionEffectDetected) {
            ctx->accBiasCorr.z -= baroAltResidual * sq(positionEstimationConfig()->w_z_baro_p);
        }

        return true;
    }
    else if ((STATE(FIXED_WING_LEGACY) || positionEstimationConfig()->use_gps_no_baro) && (ctx->newFlags & EST_GPS_Z_VALID)) {
        // If baro is not available - use GPS Z for correction on a plane
        // Reset current estimate to GPS altitude if estimate not valid
        if (!(ctx->newFlags & EST_Z_VALID)) {
            ctx->estPosCorr.z += posEstimator.gps.pos.z - posEstimator.est.pos.z;
            ctx->estVelCorr.z += posEstimator.gps.vel.z - posEstimator.est.vel.z;
            ctx->newEPV = posEstimator.gps.epv;
        }
        else {
            // Altitude
            const float gpsAltResudual = posEstimator.gps.pos.z - posEstimator.est.pos.z;

            ctx->estPosCorr.z += gpsAltResudual * positionEstimationConfig()->w_z_gps_p * ctx->dt;
            ctx->estVelCorr.z += gpsAltResudual * sq(positionEstimationConfig()->w_z_gps_p) * ctx->dt;
            ctx->estVelCorr.z += (posEstimator.gps.vel.z - posEstimator.est.vel.z) * positionEstimationConfig()->w_z_gps_v * ctx->dt;
            ctx->newEPV = updateEPE(posEstimator.est.epv, ctx->dt, MAX(posEstimator.gps.epv, gpsAltResudual), positionEstimationConfig()->w_z_gps_p);

            // Accelerometer bias
            ctx->accBiasCorr.z -= gpsAltResudual * sq(positionEstimationConfig()->w_z_gps_p);
        }

        return true;
    }

    DEBUG_SET(DEBUG_ALTITUDE, 0, posEstimator.est.pos.z);       // Position estimate
    DEBUG_SET(DEBUG_ALTITUDE, 2, posEstimator.baro.alt);        // Baro altitude
    DEBUG_SET(DEBUG_ALTITUDE, 4, posEstimator.gps.pos.z);       // GPS altitude
    DEBUG_SET(DEBUG_ALTITUDE, 6, accGetVibrationLevel());       // Vibration level
    DEBUG_SET(DEBUG_ALTITUDE, 1, posEstimator.est.vel.z);       // Vertical speed estimate
    DEBUG_SET(DEBUG_ALTITUDE, 3, posEstimator.imu.accelNEU.z);  // Vertical acceleration on earth frame
    DEBUG_SET(DEBUG_ALTITUDE, 5, posEstimator.gps.vel.z);       // GPS vertical speed
    DEBUG_SET(DEBUG_ALTITUDE, 7, accGetClipCount());            // Clip count

    return false;
}

static bool estimationCalculateCorrection_XY_GPS(estimationContext_t * ctx)
{
    if (ctx->newFlags & EST_GPS_XY_VALID) {
        /* If GPS is valid and our estimate is NOT valid - reset it to GPS coordinates and velocity */
        if (!(ctx->newFlags & EST_XY_VALID)) {
            ctx->estPosCorr.x += posEstimator.gps.pos.x - posEstimator.est.pos.x;
            ctx->estPosCorr.y += posEstimator.gps.pos.y - posEstimator.est.pos.y;
            ctx->estVelCorr.x += posEstimator.gps.vel.x - posEstimator.est.vel.x;
            ctx->estVelCorr.y += posEstimator.gps.vel.y - posEstimator.est.vel.y;
            ctx->newEPH = posEstimator.gps.eph;
        }
        else {
            const float gpsPosXResidual = posEstimator.gps.pos.x - posEstimator.est.pos.x;
            const float gpsPosYResidual = posEstimator.gps.pos.y - posEstimator.est.pos.y;
            const float gpsVelXResidual = posEstimator.gps.vel.x - posEstimator.est.vel.x;
            const float gpsVelYResidual = posEstimator.gps.vel.y - posEstimator.est.vel.y;
            const float gpsPosResidualMag = sqrtf(sq(gpsPosXResidual) + sq(gpsPosYResidual));

            //const float gpsWeightScaler = scaleRangef(bellCurve(gpsPosResidualMag, INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
            const float gpsWeightScaler = 1.0f;

            const float w_xy_gps_p = positionEstimationConfig()->w_xy_gps_p * gpsWeightScaler;
            const float w_xy_gps_v = positionEstimationConfig()->w_xy_gps_v * sq(gpsWeightScaler);

            // Coordinates
            ctx->estPosCorr.x += gpsPosXResidual * w_xy_gps_p * ctx->dt;
            ctx->estPosCorr.y += gpsPosYResidual * w_xy_gps_p * ctx->dt;

            // Velocity from coordinates
            ctx->estVelCorr.x += gpsPosXResidual * sq(w_xy_gps_p) * ctx->dt;
            ctx->estVelCorr.y += gpsPosYResidual * sq(w_xy_gps_p) * ctx->dt;

            // Velocity from direct measurement
            ctx->estVelCorr.x += gpsVelXResidual * w_xy_gps_v * ctx->dt;
            ctx->estVelCorr.y += gpsVelYResidual * w_xy_gps_v * ctx->dt;

            // Accelerometer bias
            ctx->accBiasCorr.x -= gpsPosXResidual * sq(w_xy_gps_p);
            ctx->accBiasCorr.y -= gpsPosYResidual * sq(w_xy_gps_p);

            /* Adjust EPH */
            ctx->newEPH = updateEPE(posEstimator.est.eph, ctx->dt, MAX(posEstimator.gps.eph, gpsPosResidualMag), w_xy_gps_p);
        }

        return true;
    }

    return false;
}

/**
 * Calculate next estimate using IMU and apply corrections from reference sensors (GPS, BARO etc)
 *  Function is called at main loop rate
 */
static void updateEstimatedTopic(timeUs_t currentTimeUs)
{
    estimationContext_t ctx;

    /* Calculate dT */
    ctx.dt = US2S(currentTimeUs - posEstimator.est.lastUpdateTime);
    posEstimator.est.lastUpdateTime = currentTimeUs;

    /* If IMU is not ready we can't estimate anything */
    if (!isImuReady()) {
        posEstimator.est.eph = positionEstimationConfig()->max_eph_epv + 0.001f;
        posEstimator.est.epv = positionEstimationConfig()->max_eph_epv + 0.001f;
        posEstimator.flags = 0;
        return;
    }

    /* Calculate new EPH and EPV for the case we didn't update postion */
    ctx.newEPH = posEstimator.est.eph * ((posEstimator.est.eph <= positionEstimationConfig()->max_eph_epv) ? 1.0f + ctx.dt : 1.0f);
    ctx.newEPV = posEstimator.est.epv * ((posEstimator.est.epv <= positionEstimationConfig()->max_eph_epv) ? 1.0f + ctx.dt : 1.0f);
    ctx.newFlags = calculateCurrentValidityFlags(currentTimeUs);
    vectorZero(&ctx.estPosCorr);
    vectorZero(&ctx.estVelCorr);
    vectorZero(&ctx.accBiasCorr);

    /* AGL estimation - separate process, decouples from Z coordinate */
    estimationCalculateAGL(&ctx);

    /* Prediction stage: X,Y,Z */
    estimationPredict(&ctx);

    /* Correction stage: Z */
    const bool estZCorrectOk =
        estimationCalculateCorrection_Z(&ctx);

    /* Correction stage: XY: GPS, FLOW */
    // FIXME: Handle transition from FLOW to GPS and back - seamlessly fly indoor/outdoor
    const bool estXYCorrectOk =
        estimationCalculateCorrection_XY_GPS(&ctx) ||
        estimationCalculateCorrection_XY_FLOW(&ctx);

    // If we can't apply correction or accuracy is off the charts - decay velocity to zero
    if (!estXYCorrectOk || ctx.newEPH > positionEstimationConfig()->max_eph_epv) {
        ctx.estVelCorr.x = (0.0f - posEstimator.est.vel.x) * positionEstimationConfig()->w_xy_res_v * ctx.dt;
        ctx.estVelCorr.y = (0.0f - posEstimator.est.vel.y) * positionEstimationConfig()->w_xy_res_v * ctx.dt;
    }

    if (!estZCorrectOk || ctx.newEPV > positionEstimationConfig()->max_eph_epv) {
        ctx.estVelCorr.z = (0.0f - posEstimator.est.vel.z) * positionEstimationConfig()->w_z_res_v * ctx.dt;
    }

    // Apply corrections
    vectorAdd(&posEstimator.est.pos, &posEstimator.est.pos, &ctx.estPosCorr);
    vectorAdd(&posEstimator.est.vel, &posEstimator.est.vel, &ctx.estVelCorr);

    /* Correct accelerometer bias */
    if (positionEstimationConfig()->w_acc_bias > 0.0f) {
        const float accelBiasCorrMagnitudeSq = sq(ctx.accBiasCorr.x) + sq(ctx.accBiasCorr.y) + sq(ctx.accBiasCorr.z);
        if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) {
            /* transform error vector from NEU frame to body frame */
            imuTransformVectorEarthToBody(&ctx.accBiasCorr);

            /* Correct accel bias */
            posEstimator.imu.accelBias.x += ctx.accBiasCorr.x * positionEstimationConfig()->w_acc_bias * ctx.dt;
            posEstimator.imu.accelBias.y += ctx.accBiasCorr.y * positionEstimationConfig()->w_acc_bias * ctx.dt;
            posEstimator.imu.accelBias.z += ctx.accBiasCorr.z * positionEstimationConfig()->w_acc_bias * ctx.dt;
        }
    }

    /* Update uncertainty */
    posEstimator.est.eph = ctx.newEPH;
    posEstimator.est.epv = ctx.newEPV;

    // Keep flags for further usage
    posEstimator.flags = ctx.newFlags;
}

/**
 * Examine estimation error and update navigation system if estimate is good enough
 *  Function is called at main loop rate, but updates happen less frequently - at a fixed rate
 */
static void publishEstimatedTopic(timeUs_t currentTimeUs)
{
    static navigationTimer_t posPublishTimer;

    /* IMU operates in decidegrees while INAV operates in deg*100 */
    updateActualHeading(navIsHeadingUsable(), DECIDEGREES_TO_CENTIDEGREES(attitude.values.yaw));

    /* Position and velocity are published with INAV_POSITION_PUBLISH_RATE_HZ */
    if (updateTimer(&posPublishTimer, HZ2US(INAV_POSITION_PUBLISH_RATE_HZ), currentTimeUs)) {
        /* Publish position update */
        if (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv) {
            // FIXME!!!!!
            updateActualHorizontalPositionAndVelocity(true, true, posEstimator.est.pos.x, posEstimator.est.pos.y, posEstimator.est.vel.x, posEstimator.est.vel.y);
        }
        else {
            updateActualHorizontalPositionAndVelocity(false, false, posEstimator.est.pos.x, posEstimator.est.pos.y, 0, 0);
        }

        /* Publish altitude update and set altitude validity */
        if (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv) {
            navigationEstimateStatus_e aglStatus = (posEstimator.est.aglQual == SURFACE_QUAL_LOW) ? EST_USABLE : EST_TRUSTED;
            updateActualAltitudeAndClimbRate(true, posEstimator.est.pos.z, posEstimator.est.vel.z, posEstimator.est.aglAlt, posEstimator.est.aglVel, aglStatus);
        }
        else {
            updateActualAltitudeAndClimbRate(false, posEstimator.est.pos.z, 0, posEstimator.est.aglAlt, 0, EST_NONE);
        }

#if defined(NAV_BLACKBOX)
        navEPH = posEstimator.est.eph;
        navEPV = posEstimator.est.epv;
#endif
    }
}

#if defined(NAV_GPS_GLITCH_DETECTION)
bool isGPSGlitchDetected(void)
{
    return posEstimator.gps.glitchDetected;
}
#endif

/**
 * Initialize position estimator
 *  Should be called once before any update occurs
 */
void initializePositionEstimator(void)
{
    int axis;

    posEstimator.est.eph = positionEstimationConfig()->max_eph_epv + 0.001f;
    posEstimator.est.epv = positionEstimationConfig()->max_eph_epv + 0.001f;

    posEstimator.imu.lastUpdateTime = 0;
    posEstimator.gps.lastUpdateTime = 0;
    posEstimator.baro.lastUpdateTime = 0;
    posEstimator.surface.lastUpdateTime = 0;

    posEstimator.est.aglAlt = 0;
    posEstimator.est.aglVel = 0;

    posEstimator.est.flowCoordinates[X] = 0;
    posEstimator.est.flowCoordinates[Y] = 0;

    posEstimator.imu.accWeightFactor = 0;

    restartGravityCalibration();

    for (axis = 0; axis < 3; axis++) {
        posEstimator.imu.accelBias.v[axis] = 0;
        posEstimator.est.pos.v[axis] = 0;
        posEstimator.est.vel.v[axis] = 0;
    }

    pt1FilterInit(&posEstimator.baro.avgFilter, INAV_BARO_AVERAGE_HZ, 0.0f);
    pt1FilterInit(&posEstimator.surface.avgFilter, INAV_SURFACE_AVERAGE_HZ, 0.0f);
}

/**
 * Update estimator
 *  Update rate: loop rate (>100Hz)
 */
void updatePositionEstimator(void)
{
    static bool isInitialized = false;

    if (!isInitialized) {
        initializePositionEstimator();
        isInitialized = true;
    }

    const timeUs_t currentTimeUs = micros();

    /* Read updates from IMU, preprocess */
    updateIMUTopic(currentTimeUs);

    /* Update estimate */
    updateEstimatedTopic(currentTimeUs);

    /* Publish estimate */
    publishEstimatedTopic(currentTimeUs);
}

bool navIsCalibrationComplete(void)
{
    return gravityCalibrationComplete();
}

#endif
