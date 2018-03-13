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
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/opflow.h"
#include "sensors/sensors.h"


/**
 * Model-identification based position estimator
 * Based on INAV position estimator for PX4 by Anton Babushkin <anton.babushkin@me.com>
 * @author Konstantin Sharlaimov <konstantin.sharlaimov@gmail.com>
 */
#define INAV_GPS_DEFAULT_EPH                200.0f  // 2m GPS HDOP  (gives about 1.6s of dead-reckoning if GPS is temporary lost)
#define INAV_GPS_DEFAULT_EPV                500.0f  // 5m GPS VDOP

#define INAV_GPS_ACCEPTANCE_EPE             500.0f  // 5m acceptance radius

#define INAV_ACC_BIAS_ACCEPTANCE_VALUE      (GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway

#define INAV_GPS_GLITCH_RADIUS              250.0f  // 2.5m GPS glitch radius
#define INAV_GPS_GLITCH_ACCEL               1000.0f // 10m/s/s max possible acceleration for GPS glitch detection

#define INAV_POSITION_PUBLISH_RATE_HZ       50      // Publish position updates at this rate
#define INAV_PITOT_UPDATE_RATE              10

#define INAV_GPS_TIMEOUT_MS                 1500    // GPS timeout
#define INAV_BARO_TIMEOUT_MS                200     // Baro timeout
#define INAV_SURFACE_TIMEOUT_MS             300     // Surface timeout    (missed 3 readings in a row)

#define INAV_HISTORY_BUF_SIZE               (INAV_POSITION_PUBLISH_RATE_HZ / 2)     // Enough to hold 0.5 sec historical data

typedef struct {
    timeUs_t    lastTriggeredTime;
    timeUs_t    deltaTime;
} navigationTimer_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
#if defined(NAV_GPS_GLITCH_DETECTION)
    bool        glitchDetected;
    bool        glitchRecovery;
#endif
    t_fp_vector pos;            // GPS position in NEU coordinate system (cm)
    t_fp_vector vel;            // GPS velocity (cms)
    float       eph;
    float       epv;
} navPositionEstimatorGPS_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw barometric altitude (cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       airspeed;            // airspeed (cm/s)
} navPositionEstimatorPITOT_t;

typedef enum {
    SURFACE_QUAL_LOW,   // Surface sensor signal lost long ago - most likely surface distance is incorrect
    SURFACE_QUAL_MID,   // Surface sensor is not available but we can somewhat trust the estimate
    SURFACE_QUAL_HIGH   // All good
} navAGLEstimateQuality_e;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw altitude measurement (cm)
    float       reliability;
} navPositionEstimatorSURFACE_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    bool        isValid;
    float       quality;
    float       flowRate[2];
    float       bodyRate[2];
} navPositionEstimatorFLOW_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)

    // 3D position, velocity and confidence
    t_fp_vector pos;
    t_fp_vector vel;
    float       eph;
    float       epv;

    // AGL
    navAGLEstimateQuality_e aglQual;
    float                   aglOffset;  // Offset between surface and pos.Z
    float                   aglAlt;
    float                   aglVel;
} navPositionEstimatorESTIMATE_t;

typedef struct {
    timeUs_t    baroGroundTimeout;
    float       baroGroundAlt;
    bool        isBaroGroundValid;
} navPositionEstimatorSTATE_t;

typedef struct {
    t_fp_vector     accelNEU;
    t_fp_vector     accelBias;
    bool            gravityCalibrationComplete;
} navPosisitonEstimatorIMU_t;

typedef struct {
    // Data sources
    navPositionEstimatorGPS_t   gps;
    navPositionEstimatorBARO_t  baro;
    navPositionEstimatorSURFACE_t surface;
    navPositionEstimatorPITOT_t pitot;
    navPositionEstimatorFLOW_t  flow;

    // IMU data
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    navPositionEstimatorESTIMATE_t  est;

    // Extra state variables
    navPositionEstimatorSTATE_t state;
} navigationPosEstimator_t;

static navigationPosEstimator_t posEstimator;

PG_REGISTER_WITH_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig, PG_POSITION_ESTIMATION_CONFIG, 2);

PG_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig,
        // Inertial position estimator parameters
        .automatic_mag_declination = 1,
        .reset_altitude_type = NAV_RESET_ALTITUDE_ON_FIRST_ARM,
        .gravity_calibration_tolerance = 5,     // 5 cm/s/s calibration error accepted (0.5% of gravity)
        .use_gps_velned = 1,         // "Disabled" is mandatory with gps_dyn_model = Pedestrian

        .max_surface_altitude = 200,

        .w_z_baro_p = 0.35f,

        .w_z_surface_p = 3.500f,
        .w_z_surface_v = 6.100f,

        .w_z_gps_p = 0.2f,
        .w_z_gps_v = 0.5f,

        .w_xy_gps_p = 1.0f,
        .w_xy_gps_v = 2.0f,

        .w_z_res_v = 0.5f,
        .w_xy_res_v = 0.5f,

        .w_acc_bias = 0.01f,

        .max_eph_epv = 1000.0f,
        .baro_epv = 100.0f
);

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    posEstimator.est.pos.A[axis] += posEstimator.est.vel.A[axis] * dt + acc * dt * dt / 2.0f;
    posEstimator.est.vel.A[axis] += acc * dt;
}

static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posEstimator.est.pos.A[axis] += ewdt;
    posEstimator.est.vel.A[axis] += w * ewdt;
}

static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    posEstimator.est.vel.A[axis] += e * w * dt;
}

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
    switch (positionEstimationConfig()->reset_altitude_type) {
        case NAV_RESET_ALTITUDE_NEVER:
            return false;
        case NAV_RESET_ALTITUDE_ON_FIRST_ARM:
            return !ARMING_FLAG(ARMED) && !ARMING_FLAG(WAS_EVER_ARMED);
        case NAV_RESET_ALTITUDE_ON_EACH_ARM:
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
    static t_fp_vector lastKnownGoodPosition;
    static t_fp_vector lastKnownGoodVelocity;

    bool isGlitching = false;

    if (previousTime == 0) {
        isGlitching = false;
    }
    else {
        t_fp_vector predictedGpsPosition;
        float gpsDistance;
        float dT = US2S(currentTimeUs - previousTime);

        /* We predict new position based on previous GPS velocity and position */
        predictedGpsPosition.V.X = lastKnownGoodPosition.V.X + lastKnownGoodVelocity.V.X * dT;
        predictedGpsPosition.V.Y = lastKnownGoodPosition.V.Y + lastKnownGoodVelocity.V.Y * dT;

        /* New pos is within predefined radius of predicted pos, radius is expanded exponentially */
        gpsDistance = sqrtf(sq(predictedGpsPosition.V.X - lastKnownGoodPosition.V.X) + sq(predictedGpsPosition.V.Y - lastKnownGoodPosition.V.Y));
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

#if defined(NAV_AUTO_MAG_DECLINATION)
        /* Automatic magnetic declination calculation - do this once */
        static bool magDeclinationSet = false;
        if (positionEstimationConfig()->automatic_mag_declination && !magDeclinationSet) {
            mag.magneticDeclination = geoCalculateMagDeclination(&newLLH) * 10.0f; // heading is in 0.1deg units
            magDeclinationSet = true;
        }
#endif

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
            geoConvertGeodeticToLocal(&posControl.gpsOrigin, &newLLH, & posEstimator.gps.pos, GEO_ALT_ABSOLUTE);

            /* If not the first update - calculate velocities */
            if (!isFirstGPSUpdate) {
                float dT = US2S(getGPSDeltaTimeFilter(currentTimeUs - lastGPSNewDataTime));

                /* Use VELNED provided by GPS if available, calculate from coordinates otherwise */
                float gpsScaleLonDown = constrainf(cos_approx((ABS(gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelNE) {
                    posEstimator.gps.vel.V.X = gpsSol.velNED[0];
                    posEstimator.gps.vel.V.Y = gpsSol.velNED[1];
                }
                else {
                    posEstimator.gps.vel.V.X = (posEstimator.gps.vel.V.X + (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lat - previousLat) / dT)) / 2.0f;
                    posEstimator.gps.vel.V.Y = (posEstimator.gps.vel.V.Y + (gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lon - previousLon) / dT)) / 2.0f;
                }

                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelD) {
                    posEstimator.gps.vel.V.Z = - gpsSol.velNED[2];   // NEU
                }
                else {
                    posEstimator.gps.vel.V.Z = (posEstimator.gps.vel.V.Z + (gpsSol.llh.alt - previousAlt) / dT) / 2.0f;
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
    imuUpdateGPSCOG();
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
        posEstimator.baro.alt = newBaroAlt - initialBaroAltitudeOffset;
        posEstimator.baro.epv = positionEstimationConfig()->baro_epv;
        posEstimator.baro.lastUpdateTime = currentTimeUs;
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
static void updatePitotTopic(timeUs_t currentTimeUs)
{
    static navigationTimer_t pitotUpdateTimer;

    if (updateTimer(&pitotUpdateTimer, HZ2US(INAV_PITOT_UPDATE_RATE), currentTimeUs)) {
        float newTAS = pitotCalculateAirSpeed();
        if (sensors(SENSOR_PITOT) && pitotIsCalibrationComplete()) {
            posEstimator.pitot.airspeed = newTAS;
        }
        else {
            posEstimator.pitot.airspeed = 0;
        }
    }
}
#endif

#ifdef USE_RANGEFINDER
/**
 * Read surface and update alt/vel topic
 *  Function is called from TASK_RANGEFINDER at arbitrary rate - as soon as new measurements are available
 */
#define RANGEFINDER_RELIABILITY_RC_CONSTANT     (0.47802f)
#define RANGEFINDER_RELIABILITY_LIGHT_THRESHOLD (0.15f)
#define RANGEFINDER_RELIABILITY_LOW_THRESHOLD   (0.33f)
#define RANGEFINDER_RELIABILITY_HIGH_THRESHOLD  (0.75f)

void updatePositionEstimator_SurfaceTopic(timeUs_t currentTimeUs, float newSurfaceAlt)
{
    const float dt = US2S(currentTimeUs - posEstimator.surface.lastUpdateTime);
    float newReliabilityMeasurement = 0;

    posEstimator.surface.lastUpdateTime = currentTimeUs;

    if (newSurfaceAlt >= 0) {
        if (newSurfaceAlt <= positionEstimationConfig()->max_surface_altitude) {
            newReliabilityMeasurement = 1.0f;
            posEstimator.surface.alt = newSurfaceAlt;
        }
        else {
            newReliabilityMeasurement = 0.0f;
        }
    }
    else {
        // Negative values - out of range or failed hardware
        newReliabilityMeasurement = 0.0f;
    }

    /* Reliability is a measure of confidence of rangefinder measurement. It's increased with each valid sample and decreased with each invalid sample */
    if (dt > 0.5f) {
        posEstimator.surface.reliability = 0.0f;
    }
    else {
        const float relAlpha = dt / (dt + RANGEFINDER_RELIABILITY_RC_CONSTANT);
        posEstimator.surface.reliability = posEstimator.surface.reliability * (1.0f - relAlpha) + newReliabilityMeasurement * relAlpha;
    }
}
#endif

#ifdef USE_OPTICAL_FLOW
/**
 * Read optical flow topic
 *  Function is called by OPFLOW task as soon as new update is available
 */
void updatePositionEstimator_OpticalFlowTopic(timeUs_t currentTimeUs)
{
    posEstimator.flow.lastUpdateTime = currentTimeUs;
    posEstimator.flow.isValid = opflow.isHwHealty && (opflow.flowQuality == OPFLOW_QUALITY_VALID);
    posEstimator.flow.flowRate[X] = opflow.flowRate[X];
    posEstimator.flow.flowRate[Y] = opflow.flowRate[Y];
    posEstimator.flow.bodyRate[X] = opflow.bodyRate[X];
    posEstimator.flow.bodyRate[Y] = opflow.bodyRate[Y];
}
#endif

/**
 * Update IMU topic
 *  Function is called at main loop rate
 */
static void updateIMUTopic(void)
{
    static float calibratedGravityCMSS = GRAVITY_CMSS;
    static timeMs_t gravityCalibrationTimeout = 0;

    if (!isImuReady()) {
        posEstimator.imu.accelNEU.V.X = 0;
        posEstimator.imu.accelNEU.V.Y = 0;
        posEstimator.imu.accelNEU.V.Z = 0;

        gravityCalibrationTimeout = millis();
        posEstimator.imu.gravityCalibrationComplete = false;
    }
    else {
        t_fp_vector accelBF;

        /* Read acceleration data in body frame */
        accelBF.V.X = imuMeasuredAccelBF.V.X;
        accelBF.V.Y = imuMeasuredAccelBF.V.Y;
        accelBF.V.Z = imuMeasuredAccelBF.V.Z;

        /* Correct accelerometer bias */
        accelBF.V.X -= posEstimator.imu.accelBias.V.X;
        accelBF.V.Y -= posEstimator.imu.accelBias.V.Y;
        accelBF.V.Z -= posEstimator.imu.accelBias.V.Z;

        /* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
        imuTransformVectorBodyToEarth(&accelBF);

        /* Read acceleration data in NEU frame from IMU */
        posEstimator.imu.accelNEU.V.X = accelBF.V.X;
        posEstimator.imu.accelNEU.V.Y = accelBF.V.Y;
        posEstimator.imu.accelNEU.V.Z = accelBF.V.Z;

        /* When unarmed, assume that accelerometer should measure 1G. Use that to correct accelerometer gain */
        if (!ARMING_FLAG(ARMED) && !posEstimator.imu.gravityCalibrationComplete) {
            // Slowly converge on calibrated gravity while level
            const float gravityOffsetError = posEstimator.imu.accelNEU.V.Z - calibratedGravityCMSS;
            calibratedGravityCMSS += gravityOffsetError * 0.0025f;

            if (ABS(gravityOffsetError) < positionEstimationConfig()->gravity_calibration_tolerance) {  // Error should be within 0.5% of calibrated gravity
                if ((millis() - gravityCalibrationTimeout) > 250) {
                    posEstimator.imu.gravityCalibrationComplete = true;
                }
            }
            else {
                gravityCalibrationTimeout = millis();
            }
        }

        /* If calibration is incomplete - report zero acceleration */
        if (posEstimator.imu.gravityCalibrationComplete) {
            posEstimator.imu.accelNEU.V.Z -= calibratedGravityCMSS;
        }
        else {
            posEstimator.imu.accelNEU.V.X = 0;
            posEstimator.imu.accelNEU.V.Y = 0;
            posEstimator.imu.accelNEU.V.Z = 0;
        }

#if defined(NAV_BLACKBOX)
        /* Update blackbox values */
        navAccNEU[X] = posEstimator.imu.accelNEU.A[X];
        navAccNEU[Y] = posEstimator.imu.accelNEU.A[Y];
        navAccNEU[Z] = posEstimator.imu.accelNEU.A[Z];
#endif
    }
}

static float updateEPE(const float oldEPE, const float dt, const float newEPE, const float w)
{
    return oldEPE + (newEPE - oldEPE) * w * dt;
}

static bool navIsHeadingUsable(void) 
{
    // If we have GPS - we need true IMU north (valid heading)
    return isImuHeadingValid();
}

/**
 * Calculate next estimate using IMU and apply corrections from reference sensors (GPS, BARO etc)
 *  Function is called at main loop rate
 */
static void updateEstimatedTopic(timeUs_t currentTimeUs)
{
    /* Calculate dT */
    float dt = US2S(currentTimeUs - posEstimator.est.lastUpdateTime);
    posEstimator.est.lastUpdateTime = currentTimeUs;

    /* If IMU is not ready we can't estimate anything */
    if (!isImuReady()) {
        posEstimator.est.eph = positionEstimationConfig()->max_eph_epv + 0.001f;
        posEstimator.est.epv = positionEstimationConfig()->max_eph_epv + 0.001f;
        return;
    }

    /* Calculate new EPH and EPV for the case we didn't update postion */
    float newEPH = posEstimator.est.eph;
    float newEPV = posEstimator.est.epv;

    if (newEPH <= positionEstimationConfig()->max_eph_epv) {
        newEPH *= 1.0f + dt;
    }

    if (newEPV <= positionEstimationConfig()->max_eph_epv) {
        newEPV *= 1.0f + dt;
    }

    /* Figure out if we have valid position data from our data sources */
#if defined(NAV_GPS_GLITCH_DETECTION)
    //isGPSValid = isGPSValid && !posEstimator.gps.glitchDetected;
#endif
    const bool isGPSValid = sensors(SENSOR_GPS) &&
                            posControl.gpsOrigin.valid &&
                            ((currentTimeUs - posEstimator.gps.lastUpdateTime) <= MS2US(INAV_GPS_TIMEOUT_MS)) &&
                            (posEstimator.gps.eph < positionEstimationConfig()->max_eph_epv);
    const bool isGPSZValid = isGPSValid && (posEstimator.gps.epv < positionEstimationConfig()->max_eph_epv);
    const bool isBaroValid = sensors(SENSOR_BARO) && ((currentTimeUs - posEstimator.baro.lastUpdateTime) <= MS2US(INAV_BARO_TIMEOUT_MS));
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    const bool isSurfaceValid = sensors(SENSOR_RANGEFINDER) && ((currentTimeUs - posEstimator.surface.lastUpdateTime) <= MS2US(INAV_SURFACE_TIMEOUT_MS));
#endif

    /* Do some preparations to data */
    if (isBaroValid) {
        if (!ARMING_FLAG(ARMED)) {
            posEstimator.state.baroGroundAlt = posEstimator.est.pos.V.Z;
            posEstimator.state.isBaroGroundValid = true;
            posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
        }
        else {
            if (posEstimator.est.vel.V.Z > 15) {
                if (currentTimeUs > posEstimator.state.baroGroundTimeout) {
                    posEstimator.state.isBaroGroundValid = false;
                }
            }
            else {
                posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
            }
        }
    }
    else {
        posEstimator.state.isBaroGroundValid = false;
    }

#if defined(USE_BARO)
    /* We might be experiencing air cushion effect - use sonar or baro groung altitude to detect it */
    bool isAirCushionEffectDetected = ARMING_FLAG(ARMED) &&
                                        ((isSurfaceValid && posEstimator.surface.alt < 20.0f && posEstimator.state.isBaroGroundValid) ||
                                         (isBaroValid && posEstimator.state.isBaroGroundValid && posEstimator.baro.alt < posEstimator.state.baroGroundAlt));
#endif

    /* Validate EPV for GPS and calculate altitude/climb rate correction flags */
    const bool useGpsZPos = STATE(FIXED_WING) && !sensors(SENSOR_BARO) && isGPSValid && isGPSZValid;
    const bool useGpsZVel = isGPSValid && isGPSZValid;

    /* Estimate validity */
    const bool isEstXYValid = (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv);
    const bool isEstZValid = (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv);

    /* Prediction step: Z-axis */
    if (isEstZValid) {
        inavFilterPredict(Z, dt, posEstimator.imu.accelNEU.V.Z);
    }

    /* Prediction step: XY-axis */
    if (isEstXYValid) {
        if (navIsHeadingUsable()) {
            inavFilterPredict(X, dt, posEstimator.imu.accelNEU.V.X);
            inavFilterPredict(Y, dt, posEstimator.imu.accelNEU.V.Y);
        }
        else {
            inavFilterPredict(X, dt, 0.0f);
            inavFilterPredict(Y, dt, 0.0f);
        }
    }

    /* Accelerometer bias correction */
    const bool updateAccBias = (positionEstimationConfig()->w_acc_bias > 0);
    t_fp_vector accelBiasCorr = { { 0, 0, 0} };

    /* Correction step: Z-axis */
    if (useGpsZPos || isBaroValid) {
        float gpsWeightScaler = 1.0f;

        /* Handle Z-axis reset */
        if (!isEstZValid && useGpsZPos) {
            posEstimator.est.pos.V.Z = posEstimator.gps.pos.V.Z;
            posEstimator.est.vel.V.Z = posEstimator.gps.vel.V.Z;
            newEPV = posEstimator.gps.epv;
        }
        else {
#if defined(USE_BARO)
            /* Apply BARO correction to altitude */
            if (isBaroValid) {
                const float baroResidual = (isAirCushionEffectDetected ? posEstimator.state.baroGroundAlt : posEstimator.baro.alt) - posEstimator.est.pos.V.Z;
                inavFilterCorrectPos(Z, dt, baroResidual, positionEstimationConfig()->w_z_baro_p);
                newEPV = updateEPE(posEstimator.est.epv, dt, posEstimator.baro.epv, positionEstimationConfig()->w_z_baro_p);

                /* accelerometer bias correction for baro */
                if (updateAccBias && !isAirCushionEffectDetected) {
                    accelBiasCorr.V.Z -= baroResidual * sq(positionEstimationConfig()->w_z_baro_p);
                }
            }
#endif

            /* Apply GPS correction to altitude */
            if (useGpsZPos) {
                const float gpsResidualZ = posEstimator.gps.pos.V.Z - posEstimator.est.pos.V.Z;
                inavFilterCorrectPos(Z, dt, gpsResidualZ, positionEstimationConfig()->w_z_gps_p * gpsWeightScaler);
                newEPV = updateEPE(posEstimator.est.epv, dt, MAX(posEstimator.gps.epv, gpsResidualZ), positionEstimationConfig()->w_z_gps_p);

                if (updateAccBias) {
                    accelBiasCorr.V.Z -= gpsResidualZ * sq(positionEstimationConfig()->w_z_gps_p);
                }
            }

            /* Apply GPS correction to climb rate */
            if (useGpsZVel) {
                const float gpsResidualZVel = posEstimator.gps.vel.V.Z - posEstimator.est.vel.V.Z;
                inavFilterCorrectVel(Z, dt, gpsResidualZVel, positionEstimationConfig()->w_z_gps_v * sq(gpsWeightScaler));
            }
        }
    }
    else {
        inavFilterCorrectVel(Z, dt, 0.0f - posEstimator.est.vel.V.Z, positionEstimationConfig()->w_z_res_v);
    }

    /* Correction step: XY-axis */
    /* GPS */
    if (isGPSValid) {
        /* If GPS is valid and our estimate is NOT valid - reset it to GPS coordinates and velocity */
        if (!isEstXYValid) {
            posEstimator.est.pos.V.X = posEstimator.gps.pos.V.X;
            posEstimator.est.pos.V.Y = posEstimator.gps.pos.V.Y;
            posEstimator.est.vel.V.X = posEstimator.gps.vel.V.X;
            posEstimator.est.vel.V.Y = posEstimator.gps.vel.V.Y;
            newEPH = posEstimator.gps.eph;
        }
        else {
            const float gpsResidualX = posEstimator.gps.pos.V.X - posEstimator.est.pos.V.X;
            const float gpsResidualY = posEstimator.gps.pos.V.Y - posEstimator.est.pos.V.Y;
            const float gpsResidualXVel = posEstimator.gps.vel.V.X - posEstimator.est.vel.V.X;
            const float gpsResidualYVel = posEstimator.gps.vel.V.Y - posEstimator.est.vel.V.Y;
            const float gpsResidualXYMagnitude = sqrtf(sq(gpsResidualX) + sq(gpsResidualY));

            //const float gpsWeightScaler = scaleRangef(bellCurve(gpsResidualXYMagnitude, INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
            const float gpsWeightScaler = 1.0f;

            const float w_xy_gps_p = positionEstimationConfig()->w_xy_gps_p * gpsWeightScaler;
            const float w_xy_gps_v = positionEstimationConfig()->w_xy_gps_v * sq(gpsWeightScaler);

            inavFilterCorrectPos(X, dt, gpsResidualX, w_xy_gps_p);
            inavFilterCorrectPos(Y, dt, gpsResidualY, w_xy_gps_p);

            inavFilterCorrectVel(X, dt, gpsResidualXVel, w_xy_gps_v);
            inavFilterCorrectVel(Y, dt, gpsResidualYVel, w_xy_gps_v);

            /* Adjust EPH */
            newEPH = updateEPE(posEstimator.est.eph, dt, MAX(posEstimator.gps.eph, gpsResidualXYMagnitude), positionEstimationConfig()->w_xy_gps_p);
        }
    }
    else {
        inavFilterCorrectVel(X, dt, 0.0f - posEstimator.est.vel.V.X, positionEstimationConfig()->w_xy_res_v);
        inavFilterCorrectVel(Y, dt, 0.0f - posEstimator.est.vel.V.Y, positionEstimationConfig()->w_xy_res_v);
    }

    /* Correct accelerometer bias */
    if (updateAccBias) {
        const float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.V.X) + sq(accelBiasCorr.V.Y) + sq(accelBiasCorr.V.Z);
        if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) {
            /* transform error vector from NEU frame to body frame */
            imuTransformVectorEarthToBody(&accelBiasCorr);

            /* Correct accel bias */
            posEstimator.imu.accelBias.V.X += accelBiasCorr.V.X * positionEstimationConfig()->w_acc_bias * dt;
            posEstimator.imu.accelBias.V.Y += accelBiasCorr.V.Y * positionEstimationConfig()->w_acc_bias * dt;
            posEstimator.imu.accelBias.V.Z += accelBiasCorr.V.Z * positionEstimationConfig()->w_acc_bias * dt;
        }
    }

    /* Update uncertainty */
    posEstimator.est.eph = newEPH;
    posEstimator.est.epv = newEPV;

    /* AGL estimation */
#ifdef USE_RANGEFINDER
    if (isSurfaceValid) {   // If surface topic is updated in timely manner - do something smart
        navAGLEstimateQuality_e newAglQuality = posEstimator.est.aglQual;
        bool resetSurfaceEstimate = false;
        switch (posEstimator.est.aglQual) {
            case SURFACE_QUAL_LOW:
                if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_HIGH_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_HIGH;
                    resetSurfaceEstimate = true;
                }
                else if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_LOW_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_LOW;
                }
                else {
                    newAglQuality = SURFACE_QUAL_LOW;
                }
                break;

            case SURFACE_QUAL_MID:
                if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_HIGH_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_HIGH;
                }
                else if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_LOW_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_MID;
                }
                else {
                    newAglQuality = SURFACE_QUAL_LOW;
                }
                break;

            case SURFACE_QUAL_HIGH:
                if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_HIGH_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_HIGH;
                }
                else if (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_LOW_THRESHOLD) {
                    newAglQuality = SURFACE_QUAL_MID;
                }
                else {
                    newAglQuality = SURFACE_QUAL_LOW;
                }
                break;
        }

        posEstimator.est.aglQual = newAglQuality;

        if (resetSurfaceEstimate) {
            posEstimator.est.aglAlt = posEstimator.surface.alt;
            if (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv) {
                posEstimator.est.aglVel = posEstimator.est.vel.V.Z;
                posEstimator.est.aglOffset = posEstimator.est.pos.V.Z - posEstimator.surface.alt;
            }
            else {
                posEstimator.est.aglVel = 0;
                posEstimator.est.aglOffset = 0;
            }
        }

        // Update estimate
        posEstimator.est.aglAlt = posEstimator.est.aglAlt + posEstimator.est.aglVel * dt + posEstimator.imu.accelNEU.V.Z * dt * dt * 0.5f;
        posEstimator.est.aglVel = posEstimator.est.aglVel + posEstimator.imu.accelNEU.V.Z * dt;

        // Apply correction
        if (posEstimator.est.aglQual == SURFACE_QUAL_HIGH) {
            // Correct estimate from rangefinder
            const float surfaceResidual = posEstimator.surface.alt - posEstimator.est.aglAlt;
            const float bellCurveScaler = scaleRangef(bellCurve(surfaceResidual, 50.0f), 0.0f, 1.0f, 0.1f, 1.0f);

            posEstimator.est.aglAlt += surfaceResidual * positionEstimationConfig()->w_z_surface_p * bellCurveScaler * posEstimator.surface.reliability * dt;
            posEstimator.est.aglVel += surfaceResidual * positionEstimationConfig()->w_z_surface_v * sq(bellCurveScaler) * sq(posEstimator.surface.reliability) * dt;

            // Update estimate offset
            if ((posEstimator.est.aglQual == SURFACE_QUAL_HIGH) && (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv)) {
                posEstimator.est.aglOffset = posEstimator.est.pos.V.Z - posEstimator.surface.alt;
            }
        }
        else if (posEstimator.est.aglQual == SURFACE_QUAL_MID) {
            // Correct estimate from altitude fused from rangefinder and global altitude
            const float estAltResidual = (posEstimator.est.pos.V.Z - posEstimator.est.aglOffset) - posEstimator.est.aglAlt;
            const float surfaceResidual = posEstimator.surface.alt - posEstimator.est.aglAlt;
            const float surfaceWeightScaler = scaleRangef(bellCurve(surfaceResidual, 50.0f), 0.0f, 1.0f, 0.1f, 1.0f) * posEstimator.surface.reliability;
            const float mixedResidual = surfaceResidual * surfaceWeightScaler + estAltResidual * (1.0f - surfaceWeightScaler);
            
            posEstimator.est.aglAlt += mixedResidual * positionEstimationConfig()->w_z_surface_p * dt;
            posEstimator.est.aglVel += mixedResidual * positionEstimationConfig()->w_z_surface_v * dt;
        }
        else {  // SURFACE_QUAL_LOW
            // In this case rangefinder can't be trusted - simply use global altitude
            posEstimator.est.aglAlt = posEstimator.est.pos.V.Z - posEstimator.est.aglOffset;
            posEstimator.est.aglVel = posEstimator.est.vel.V.Z;
        }
    }
    else {
        posEstimator.est.aglAlt = posEstimator.est.pos.V.Z - posEstimator.est.aglOffset;
        posEstimator.est.aglVel = posEstimator.est.vel.V.Z;
        posEstimator.est.aglQual = SURFACE_QUAL_LOW;
    }

#if defined(NAV_BLACKBOX)
        DEBUG_SET(DEBUG_AGL, 0, posEstimator.surface.reliability * 1000);
        DEBUG_SET(DEBUG_AGL, 1, posEstimator.est.aglQual);
        DEBUG_SET(DEBUG_AGL, 2, posEstimator.est.aglAlt);
        DEBUG_SET(DEBUG_AGL, 3, posEstimator.est.aglVel);
#endif

#else
    posEstimator.est.aglAlt = posEstimator.est.pos.V.Z;
    posEstimator.est.aglVel = posEstimator.est.vel.V.Z;
    posEstimator.est.aglQual = SURFACE_QUAL_LOW;
#endif
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
            updateActualHorizontalPositionAndVelocity(true, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, posEstimator.est.vel.V.X, posEstimator.est.vel.V.Y);
        }
        else {
            updateActualHorizontalPositionAndVelocity(false, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, 0, 0);
        }

        /* Publish altitude update and set altitude validity */
        if (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv) {
            navigationEstimateStatus_e aglStatus = (posEstimator.est.aglQual == SURFACE_QUAL_LOW) ? EST_USABLE : EST_TRUSTED;
            updateActualAltitudeAndClimbRate(true, posEstimator.est.pos.V.Z, posEstimator.est.vel.V.Z, posEstimator.est.aglAlt, posEstimator.est.aglVel, aglStatus);
        }
        else {
            updateActualAltitudeAndClimbRate(false, posEstimator.est.pos.V.Z, 0, posEstimator.est.aglAlt, 0, EST_NONE);
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

    posEstimator.gps.lastUpdateTime = 0;
    posEstimator.baro.lastUpdateTime = 0;
    posEstimator.surface.lastUpdateTime = 0;

    posEstimator.est.aglAlt = 0;
    posEstimator.est.aglVel = 0;

    posEstimator.imu.gravityCalibrationComplete = false;

    for (axis = 0; axis < 3; axis++) {
        posEstimator.imu.accelBias.A[axis] = 0;
        posEstimator.est.pos.A[axis] = 0;
        posEstimator.est.vel.A[axis] = 0;
    }
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

    /* Periodic sensor updates */
#if defined(USE_PITOT)
    updatePitotTopic(currentTimeUs);
#endif

    /* Read updates from IMU, preprocess */
    updateIMUTopic();

    /* Update estimate */
    updateEstimatedTopic(currentTimeUs);

    /* Publish estimate */
    publishEstimatedTopic(currentTimeUs);
}

bool navIsCalibrationComplete(void)
{
    return posEstimator.imu.gravityCalibrationComplete;
}

#endif
