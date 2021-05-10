
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/wind_estimator.h"

#include "navigation/navigation.h"

#include "sensors/battery.h"

#include <stdint.h>

#if defined(USE_ADC) && defined(USE_GPS)

/* INPUTS:
 *   - heading degrees
 *   - horizontalWindSpeed
 *   - windHeading degrees
 * OUTPUT:
 *   returns same unit as horizontalWindSpeed
 */
static float forwardWindSpeed(float heading, float horizontalWindSpeed, float windHeading) {
    return horizontalWindSpeed * cos_approx(DEGREES_TO_RADIANS(windHeading - heading));
}

#ifdef USE_WIND_ESTIMATOR
/* INPUTS:
 *   - forwardSpeed (same unit as horizontalWindSpeed)
 *   - heading degrees
 *   - horizontalWindSpeed (same unit as forwardSpeed)
 *   - windHeading degrees
 * OUTPUT:
 *   returns degrees
 */
static float windDriftCompensationAngle(float forwardSpeed, float heading, float horizontalWindSpeed, float windHeading) {
    return RADIANS_TO_DEGREES(asin_approx(-horizontalWindSpeed * sin_approx(DEGREES_TO_RADIANS(windHeading - heading)) / forwardSpeed));
}

/* INPUTS:
 *   - forwardSpeed (same unit as horizontalWindSpeed)
 *   - heading degrees
 *   - horizontalWindSpeed (same unit as forwardSpeed)
 *   - windHeading degrees
 * OUTPUT:
 *   returns (same unit as forwardSpeed and horizontalWindSpeed)
 */
static float windDriftCorrectedForwardSpeed(float forwardSpeed, float heading, float horizontalWindSpeed, float windHeading) {
    return forwardSpeed * cos_approx(DEGREES_TO_RADIANS(windDriftCompensationAngle(forwardSpeed, heading, horizontalWindSpeed, windHeading)));
}

/* INPUTS:
 *   - forwardSpeed (same unit as horizontalWindSpeed)
 *   - heading degrees
 *   - horizontalWindSpeed (same unit as forwardSpeed)
 *   - windHeading degrees
 * OUTPUT:
 *   returns (same unit as forwardSpeed and horizontalWindSpeed)
 */
static float windCompensatedForwardSpeed(float forwardSpeed, float heading, float horizontalWindSpeed, float windHeading) {
    return windDriftCorrectedForwardSpeed(forwardSpeed, heading, horizontalWindSpeed, windHeading) + forwardWindSpeed(heading, horizontalWindSpeed, windHeading);
}
#endif

// returns degrees
static int8_t RTHInitialAltitudeChangePitchAngle(float altitudeChange) {
    return altitudeChange < 0 ? navConfig()->fw.max_dive_angle : -navConfig()->fw.max_climb_angle;
}

// pitch in degrees
// output in Watt
static float estimatePitchPower(float pitch) {
    int16_t altitudeChangeThrottle = (int16_t)pitch * navConfig()->fw.pitch_to_throttle;
    altitudeChangeThrottle = constrain(altitudeChangeThrottle, navConfig()->fw.min_throttle, navConfig()->fw.max_throttle);
    const float altitudeChangeThrToCruiseThrRatio = (float)(altitudeChangeThrottle - getThrottleIdleValue()) / (navConfig()->fw.cruise_throttle - getThrottleIdleValue());
    return (float)heatLossesCompensatedPower(batteryMetersConfig()->idle_power + batteryMetersConfig()->cruise_power * altitudeChangeThrToCruiseThrRatio) / 100;
}

// altitudeChange is in m
// verticalWindSpeed is in m/s
// cruise_speed is in cm/s
// output is in seconds
static float estimateRTHAltitudeChangeTime(float altitudeChange, float verticalWindSpeed) {
    // Assuming increase in throttle keeps air speed at cruise speed
    const float estimatedVerticalSpeed = (float)navConfig()->fw.cruise_speed / 100 * sin_approx(DEGREES_TO_RADIANS(RTHInitialAltitudeChangePitchAngle(altitudeChange))) + verticalWindSpeed;
    return altitudeChange / estimatedVerticalSpeed;
}

// altitudeChange is in m
// horizontalWindSpeed is in m/s
// windHeading is in degrees
// verticalWindSpeed is in m/s
// cruise_speed is in cm/s
// output is in meters
static float estimateRTHAltitudeChangeGroundDistance(float altitudeChange, float horizontalWindSpeed, float windHeading, float verticalWindSpeed) {
    // Assuming increase in throttle keeps air speed at cruise speed
    const float estimatedHorizontalSpeed = (float)navConfig()->fw.cruise_speed / 100 * cos_approx(DEGREES_TO_RADIANS(RTHInitialAltitudeChangePitchAngle(altitudeChange))) + forwardWindSpeed(DECIDEGREES_TO_DEGREES((float)attitude.values.yaw), horizontalWindSpeed, windHeading);
    return estimateRTHAltitudeChangeTime(altitudeChange, verticalWindSpeed) * estimatedHorizontalSpeed;
}

// altitudeChange is in m
// verticalWindSpeed is in m/s
// output is in Wh
static float estimateRTHInitialAltitudeChangeEnergy(float altitudeChange, float verticalWindSpeed) {
    const float RTHInitialAltitudeChangePower = estimatePitchPower(altitudeChange < 0 ? navConfig()->fw.max_dive_angle : -navConfig()->fw.max_climb_angle);
    return RTHInitialAltitudeChangePower * estimateRTHAltitudeChangeTime(altitudeChange, verticalWindSpeed) / 3600;
}

// horizontalWindSpeed is in m/s
// windHeading is in degrees
// verticalWindSpeed is in m/s
// altitudeChange is in m
// returns distance in m
// *heading is in degrees
static float estimateRTHDistanceAndHeadingAfterAltitudeChange(float altitudeChange, float horizontalWindSpeed, float windHeading, float verticalWindSpeed, float *heading) {
    float estimatedAltitudeChangeGroundDistance = estimateRTHAltitudeChangeGroundDistance(altitudeChange, horizontalWindSpeed, windHeading, verticalWindSpeed);
    if (navConfig()->general.flags.rth_climb_first && (altitudeChange > 0)) {
        float headingDiff = DEGREES_TO_RADIANS(DECIDEGREES_TO_DEGREES((float)attitude.values.yaw) - GPS_directionToHome);
        float triangleAltitude = GPS_distanceToHome * sin_approx(headingDiff);
        float triangleAltitudeToReturnStart = estimatedAltitudeChangeGroundDistance - GPS_distanceToHome * cos_approx(headingDiff);
        const float reverseHeadingDiff = RADIANS_TO_DEGREES(atan2_approx(triangleAltitude, triangleAltitudeToReturnStart));
        *heading = CENTIDEGREES_TO_DEGREES(wrap_36000(DEGREES_TO_CENTIDEGREES(180 + reverseHeadingDiff + DECIDEGREES_TO_DEGREES((float)attitude.values.yaw))));
        return sqrt(sq(triangleAltitude) + sq(triangleAltitudeToReturnStart));
    } else {
        *heading = GPS_directionToHome;
        return GPS_distanceToHome - estimatedAltitudeChangeGroundDistance;
    }
}

// distanceToHome is in meters
// output in Watt
static float estimateRTHEnergyAfterInitialClimb(float distanceToHome, float speedToHome) {
    const float timeToHome = distanceToHome / speedToHome; // seconds
    const float altitudeChangeDescentToHome = CENTIMETERS_TO_METERS(navConfig()->general.flags.rth_alt_control_mode == NAV_RTH_AT_LEAST_ALT_LINEAR_DESCENT ? MAX(0, getEstimatedActualPosition(Z) - getFinalRTHAltitude()) : 0);
    const float pitchToHome = MIN(RADIANS_TO_DEGREES(atan2_approx(altitudeChangeDescentToHome, distanceToHome)), navConfig()->fw.max_dive_angle);
    return estimatePitchPower(pitchToHome) * timeToHome / 3600;
}

// returns Wh
static float calculateRemainingEnergyBeforeRTH(bool takeWindIntoAccount) {

    const float RTH_initial_altitude_change = MAX(0, (getFinalRTHAltitude() - getEstimatedActualPosition(Z)) / 100);

    float RTH_heading; // degrees
#ifdef USE_WIND_ESTIMATOR
    uint16_t windHeading; // centidegrees
    const float horizontalWindSpeed = takeWindIntoAccount ? getEstimatedHorizontalWindSpeed(&windHeading) / 100 : 0; // m/s
    const float windHeadingDegrees = CENTIDEGREES_TO_DEGREES((float)windHeading);
    const float verticalWindSpeed = getEstimatedWindSpeed(Z) / 100;

    const float RTH_distance = estimateRTHDistanceAndHeadingAfterAltitudeChange(RTH_initial_altitude_change, horizontalWindSpeed, windHeadingDegrees, verticalWindSpeed, &RTH_heading);
    const float RTH_speed = windCompensatedForwardSpeed((float)navConfig()->fw.cruise_speed / 100, RTH_heading, horizontalWindSpeed, windHeadingDegrees);
#else
    UNUSED(takeWindIntoAccount);
    const float RTH_distance = estimateRTHDistanceAndHeadingAfterAltitudeChange(RTH_initial_altitude_change, 0, 0, 0, &RTH_heading);
    const float RTH_speed = (float)navConfig()->fw.cruise_speed / 100;
#endif

    DEBUG_SET(DEBUG_REM_FLIGHT_TIME, 0, lrintf(RTH_initial_altitude_change * 100));
    DEBUG_SET(DEBUG_REM_FLIGHT_TIME, 1, lrintf(RTH_distance * 100));
    DEBUG_SET(DEBUG_REM_FLIGHT_TIME, 2, lrintf(RTH_speed * 100));
#ifdef USE_WIND_ESTIMATOR
    DEBUG_SET(DEBUG_REM_FLIGHT_TIME, 3, lrintf(horizontalWindSpeed * 100));
#endif

    if (RTH_speed <= 0)
        return -2; // wind is too strong to return at cruise throttle (TODO: might be possible to take into account min speed thr boost)

#ifdef USE_WIND_ESTIMATOR
    const float energy_to_home = estimateRTHInitialAltitudeChangeEnergy(RTH_initial_altitude_change, verticalWindSpeed) + estimateRTHEnergyAfterInitialClimb(RTH_distance, RTH_speed); // Wh
#else
    const float energy_to_home = estimateRTHInitialAltitudeChangeEnergy(RTH_initial_altitude_change, 0) + estimateRTHEnergyAfterInitialClimb(RTH_distance, RTH_speed); // Wh
#endif
    const float energy_margin_abs = (currentBatteryProfile->capacity.value - currentBatteryProfile->capacity.critical) * batteryMetersConfig()->rth_energy_margin / 100000; // Wh
    const float remaining_energy_before_rth = getBatteryRemainingCapacity() / 1000 - energy_margin_abs - energy_to_home; // Wh

    if (remaining_energy_before_rth < 0) // No energy left = No time left
        return 0;

    return remaining_energy_before_rth;
}

// returns seconds
float calculateRemainingFlightTimeBeforeRTH(bool takeWindIntoAccount) {

    const float remainingEnergyBeforeRTH = calculateRemainingEnergyBeforeRTH(takeWindIntoAccount);

    // error: return error code directly
    if (remainingEnergyBeforeRTH < 0)
        return remainingEnergyBeforeRTH;

    const float averagePower = (float)calculateAveragePower() / 100;

    if (averagePower == 0)
        return -1;

    const float time_before_rth = remainingEnergyBeforeRTH * 3600 / averagePower;

    return time_before_rth;
}

// returns meters
float calculateRemainingDistanceBeforeRTH(bool takeWindIntoAccount) {

    // Fixed wing only for now
    if (!(STATE(FIXED_WING_LEGACY) || ARMING_FLAG(ARMED))) {
        return -1;
    }

#ifdef USE_WIND_ESTIMATOR
    if (takeWindIntoAccount && !isEstimatedWindSpeedValid()) {
        return -1;
    }
#endif

    // check requirements
    const bool areBatterySettingsOK = feature(FEATURE_VBAT) && feature(FEATURE_CURRENT_METER) && batteryWasFullWhenPluggedIn();
    const bool areRTHEstimatorSettingsOK = batteryMetersConfig()->cruise_power > 0 && currentBatteryProfile->capacity.unit == BAT_CAPACITY_UNIT_MWH &&currentBatteryProfile->capacity.value > 0 && navConfig()->fw.cruise_speed > 0;
    const bool isNavigationOK = navigationPositionEstimateIsHealthy() && isImuHeadingValid();

    if (!(areBatterySettingsOK && areRTHEstimatorSettingsOK && isNavigationOK)) {
        return -1;
    }

    const float remainingFlightTimeBeforeRTH = calculateRemainingFlightTimeBeforeRTH(takeWindIntoAccount);

    // error: return error code directly
    if (remainingFlightTimeBeforeRTH < 0)
        return remainingFlightTimeBeforeRTH;

    return remainingFlightTimeBeforeRTH * calculateAverageSpeed();
}

#endif
