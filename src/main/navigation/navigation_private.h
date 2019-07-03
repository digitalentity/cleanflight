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

#pragma once

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR    1.113195f  // MagicEarthNumber from APM

#if defined(USE_NAV)

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"
#include "fc/runtime_config.h"

#define MIN_POSITION_UPDATE_RATE_HZ         5       // Minimum position update rate at which XYZ controllers would be applied
#define NAV_THROTTLE_CUTOFF_FREQENCY_HZ     4       // low-pass filter on throttle output
#define NAV_FW_CONTROL_MONITORING_RATE      2
#define NAV_FW_PITCH_CUTOFF_FREQENCY_HZ     2       // low-pass filter on Z (pitch angle) for fixed wing
#define NAV_FW_ROLL_CUTOFF_FREQUENCY_HZ     10      // low-pass filter on roll correction for fixed wing
#define NAV_DTERM_CUT_HZ                    10.0f
#define NAV_ACCELERATION_XY_MAX             980.0f  // cm/s/s       // approx 45 deg lean angle

#define INAV_SURFACE_MAX_DISTANCE           40

typedef enum {
    NAV_POS_UPDATE_NONE                 = 0,
    NAV_POS_UPDATE_Z                    = 1 << 1,
    NAV_POS_UPDATE_XY                   = 1 << 0,
    NAV_POS_UPDATE_HEADING              = 1 << 2,
    NAV_POS_UPDATE_BEARING              = 1 << 3,
    NAV_POS_UPDATE_BEARING_TAIL_FIRST   = 1 << 4,
} navSetWaypointFlags_t;

typedef enum {
    ROC_TO_ALT_RESET,
    ROC_TO_ALT_NORMAL
} climbRateToAltitudeControllerMode_e;

typedef enum {
    EST_NONE = 0,       // No valid sensor present
    EST_USABLE = 1,     // Estimate is usable but may be inaccurate
    EST_TRUSTED = 2     // Estimate is usable and based on actual sensor data
} navigationEstimateStatus_e;

typedef enum {
    NAV_HOME_INVALID = 0,
    NAV_HOME_VALID_XY = 1 << 0,
    NAV_HOME_VALID_Z = 1 << 1,
    NAV_HOME_VALID_HEADING = 1 << 2,
    NAV_HOME_VALID_ALL = NAV_HOME_VALID_XY | NAV_HOME_VALID_Z | NAV_HOME_VALID_HEADING,
} navigationHomeFlags_t;

typedef struct navigationFlags_s {
    bool horizontalPositionDataNew;
    bool verticalPositionDataNew;
    bool headingDataNew;

    bool horizontalPositionDataConsumed;
    bool verticalPositionDataConsumed;

    navigationEstimateStatus_e estAltStatus;        // Indicates that we have a working altitude sensor (got at least one valid reading from it)
    navigationEstimateStatus_e estPosStatus;        // Indicates that GPS is working (or not)
    navigationEstimateStatus_e estVelStatus;        // Indicates that GPS is working (or not)
    navigationEstimateStatus_e estAglStatus;
    navigationEstimateStatus_e estHeadingStatus;    // Indicate valid heading - wither mag or GPS at certain speed on airplane

    bool isAdjustingPosition;
    bool isAdjustingAltitude;
    bool isAdjustingHeading;

    // Behaviour modifiers
    bool isGCSAssistedNavigationEnabled;    // Does iNav accept WP#255 - follow-me etc.
    bool isGCSAssistedNavigationReset;      // GCS control was disabled - indicate that so code could take action accordingly
    bool isTerrainFollowEnabled;            // Does iNav use rangefinder for terrain following (adjusting baro altitude target according to rangefinders readings)

    bool forcedRTHActivated;
} navigationFlags_t;

typedef enum {
    PID_DTERM_FROM_ERROR            = 1 << 0,
    PID_ZERO_INTEGRATOR             = 1 << 1,
    PID_SHRINK_INTEGRATOR           = 1 << 2,
} pidControllerFlags_e;

typedef struct {
    fpVector3_t pos;
    fpVector3_t vel;
} navEstimatedPosVel_t;

typedef struct {
    // Local estimated states
    navEstimatedPosVel_t    abs;
    navEstimatedPosVel_t    agl;
    int32_t                 yaw;

    // Service values
    float                   sinYaw;
    float                   cosYaw;
    float                   surfaceMin;
    float                   velXY;
} navigationEstimatedState_t;

typedef struct {
    fpVector3_t pos;
    fpVector3_t vel;
    int32_t     yaw;
} navigationDesiredState_t;

typedef enum {
    NAV_FSM_EVENT_NONE = 0,
    NAV_FSM_EVENT_TIMEOUT,

    NAV_FSM_EVENT_SUCCESS,
    NAV_FSM_EVENT_ERROR,

    NAV_FSM_EVENT_SWITCH_TO_IDLE,
    NAV_FSM_EVENT_SWITCH_TO_ALTHOLD,
    NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D,
    NAV_FSM_EVENT_SWITCH_TO_RTH,
    NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME,
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT,
    NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING,
    NAV_FSM_EVENT_SWITCH_TO_LAUNCH,

    NAV_FSM_EVENT_STATE_SPECIFIC_1,             // State-specific event
    NAV_FSM_EVENT_STATE_SPECIFIC_2,             // State-specific event
    NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING = NAV_FSM_EVENT_STATE_SPECIFIC_1,
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND = NAV_FSM_EVENT_STATE_SPECIFIC_1,
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED = NAV_FSM_EVENT_STATE_SPECIFIC_2,

    NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D,
    NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D,
    NAV_FSM_EVENT_SWITCH_TO_CRUISE_ADJ,
    NAV_FSM_EVENT_COUNT,
} navigationFSMEvent_t;

// This enum is used to keep values in blackbox logs stable, so we can
// freely change navigationFSMState_t.
typedef enum {
    NAV_PERSISTENT_ID_UNDEFINED                                 = 0,

    NAV_PERSISTENT_ID_IDLE                                      = 1,

    NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE                        = 2,
    NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS                       = 3,

    NAV_PERSISTENT_ID_UNUSED_1                                  = 4,  // was NAV_STATE_POSHOLD_2D_INITIALIZE
    NAV_PERSISTENT_ID_UNUSED_2                                  = 5,  // was NAV_STATE_POSHOLD_2D_IN_PROGRESS

    NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE                     = 6,
    NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS                    = 7,

    NAV_PERSISTENT_ID_RTH_INITIALIZE                            = 8,
    NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT                     = 9,
    NAV_PERSISTENT_ID_RTH_HEAD_HOME                             = 10,
    NAV_PERSISTENT_ID_RTH_HOVER_PRIOR_TO_LANDING                = 11,
    NAV_PERSISTENT_ID_RTH_HOVER_ABOVE_HOME                      = 29,
    NAV_PERSISTENT_ID_RTH_LANDING                               = 12,
    NAV_PERSISTENT_ID_RTH_FINISHING                             = 13,
    NAV_PERSISTENT_ID_RTH_FINISHED                              = 14,

    NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE                       = 15,
    NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION                       = 16,
    NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS                      = 17,
    NAV_PERSISTENT_ID_WAYPOINT_REACHED                          = 18,
    NAV_PERSISTENT_ID_WAYPOINT_NEXT                             = 19,
    NAV_PERSISTENT_ID_WAYPOINT_FINISHED                         = 20,
    NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND                         = 21,

    NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE              = 22,
    NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS             = 23,
    NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED                = 24,

    NAV_PERSISTENT_ID_LAUNCH_INITIALIZE                         = 25,
    NAV_PERSISTENT_ID_LAUNCH_WAIT                               = 26,
    NAV_PERSISTENT_ID_UNUSED_3                                  = 27, // was NAV_STATE_LAUNCH_MOTOR_DELAY
    NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS                        = 28,

    NAV_PERSISTENT_ID_CRUISE_2D_INITIALIZE                      = 29,
    NAV_PERSISTENT_ID_CRUISE_2D_IN_PROGRESS                     = 30,
    NAV_PERSISTENT_ID_CRUISE_2D_ADJUSTING                       = 31,

    NAV_PERSISTENT_ID_CRUISE_3D_INITIALIZE                      = 32,
    NAV_PERSISTENT_ID_CRUISE_3D_IN_PROGRESS                     = 33,
    NAV_PERSISTENT_ID_CRUISE_3D_ADJUSTING                       = 34,
} navigationPersistentId_e;

typedef enum {
    NAV_STATE_UNDEFINED = 0,

    NAV_STATE_IDLE,

    NAV_STATE_ALTHOLD_INITIALIZE,
    NAV_STATE_ALTHOLD_IN_PROGRESS,

    NAV_STATE_POSHOLD_3D_INITIALIZE,
    NAV_STATE_POSHOLD_3D_IN_PROGRESS,

    NAV_STATE_RTH_INITIALIZE,
    NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,
    NAV_STATE_RTH_HEAD_HOME,
    NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
    NAV_STATE_RTH_HOVER_ABOVE_HOME,
    NAV_STATE_RTH_LANDING,
    NAV_STATE_RTH_FINISHING,
    NAV_STATE_RTH_FINISHED,

    NAV_STATE_WAYPOINT_INITIALIZE,
    NAV_STATE_WAYPOINT_PRE_ACTION,
    NAV_STATE_WAYPOINT_IN_PROGRESS,
    NAV_STATE_WAYPOINT_REACHED,
    NAV_STATE_WAYPOINT_NEXT,
    NAV_STATE_WAYPOINT_FINISHED,
    NAV_STATE_WAYPOINT_RTH_LAND,

    NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
    NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
    NAV_STATE_EMERGENCY_LANDING_FINISHED,

    NAV_STATE_LAUNCH_INITIALIZE,
    NAV_STATE_LAUNCH_WAIT,
    NAV_STATE_LAUNCH_IN_PROGRESS,

    NAV_STATE_CRUISE_2D_INITIALIZE,
    NAV_STATE_CRUISE_2D_IN_PROGRESS,
    NAV_STATE_CRUISE_2D_ADJUSTING,
    NAV_STATE_CRUISE_3D_INITIALIZE,
    NAV_STATE_CRUISE_3D_IN_PROGRESS,
    NAV_STATE_CRUISE_3D_ADJUSTING,

    NAV_STATE_COUNT,
} navigationFSMState_t;

typedef enum {
    /* Navigation controllers */
    NAV_CTL_ALT             = (1 << 0),     // Altitude controller
    NAV_CTL_POS             = (1 << 1),     // Position controller
    NAV_CTL_YAW             = (1 << 2),
    NAV_CTL_EMERG           = (1 << 3),
    NAV_CTL_LAUNCH          = (1 << 4),

    /* Navigation requirements for flight modes and controllers */
    NAV_REQUIRE_ANGLE       = (1 << 5),
    NAV_REQUIRE_ANGLE_FW    = (1 << 6),
    NAV_REQUIRE_MAGHOLD     = (1 << 7),
    NAV_REQUIRE_THRTILT     = (1 << 8),
    NAV_REQUIRE_THR_UNIDIR  = (1 << 9),
    NAV_REQUIRE_THR_BIDIR   = (1 << 10),

    /* Navigation autonomous modes */
    NAV_AUTO_RTH            = (1 << 11),
    NAV_AUTO_WP             = (1 << 12),

    /* Adjustments for navigation modes from RC input */
    NAV_RC_ALT              = (1 << 13),
    NAV_RC_POS              = (1 << 14),
    NAV_RC_YAW              = (1 << 15),

    /* Additional flags */
    NAV_CTL_LAND            = (1 << 16),
} navigationFSMStateFlags_t;

typedef struct {
    navigationPersistentId_e            persistentId;
    navigationFSMEvent_t                (*onEntry)(navigationFSMState_t previousState);
    uint32_t                            timeoutMs;
    navSystemStatus_State_e             mwState;
    navSystemStatus_Error_e             mwError;
    navigationFSMStateFlags_t           stateFlags;
    flightModeFlags_e                   mapToFlightModes;
    navigationFSMState_t                onEvent[NAV_FSM_EVENT_COUNT];
} navigationFSMStateDescriptor_t;

typedef struct {
    timeMs_t        lastCheckTime;
    fpVector3_t     initialPosition;
    float           minimalDistanceToHome;
} rthSanityChecker_t;

typedef struct {
    fpVector3_t                 targetPos;
    int32_t                     yaw;
    int32_t                     previousYaw;
    timeMs_t                    lastYawAdjustmentTime;
} navCruise_t;

typedef struct {
    navigationHomeFlags_t   homeFlags;
    navWaypointPosition_t   homePosition;       // Original home position and base altitude
    float                   rthInitialAltitude; // Altitude at start of RTH
    float                   rthFinalAltitude;   // Altitude at end of RTH approach
    float                   rthInitialDistance; // Distance when starting flight home
    fpVector3_t             homeTmpWaypoint;    // Temporary storage for home target
} rthState_t;

typedef enum {
    RTH_HOME_ENROUTE_INITIAL,       // Initial position for RTH approach
    RTH_HOME_ENROUTE_PROPORTIONAL,  // Prorpotional position for RTH approach
    RTH_HOME_ENROUTE_FINAL,         // Final position for RTH approach
    RTH_HOME_FINAL_HOVER,           // Final hover altitude (if rth_home_altitude is set)
    RTH_HOME_FINAL_LAND,            // Home position and altitude
} rthTargetMode_e;

typedef struct {
    /* Flags and navigation system state */
    navigationFSMState_t        navState;
    navigationPersistentId_e    navPersistentId;

    navigationFlags_t           flags;

    /* Navigation PID controllers + pre-computed flight parameters */
    navigationPIDControllers_t  pids;
    float                       posDecelerationTime;
    float                       posResponseExpo;

    /* Local system state, both actual (estimated) and desired (target setpoint)*/
    navigationEstimatedState_t  actualState;
    navigationDesiredState_t    desiredState;   // waypoint coordinates + velocity

    uint32_t                    lastValidPositionTimeMs;
    uint32_t                    lastValidAltitudeTimeMs;

    /* INAV GPS origin (position where GPS fix was first acquired) */
    gpsOrigin_t                 gpsOrigin;

    /* Home parameters (NEU coordinated), geodetic position of home (LLH) is stores in GPS_home variable */
    rthSanityChecker_t          rthSanityChecker;
    rthState_t                  rthState;

    /* Home parameters */
    uint32_t                    homeDistance;   // cm
    int32_t                     homeDirection;  // deg*100

    /* Cruise */
    navCruise_t                 cruise;

    /* Waypoint list */
    navWaypoint_t               waypointList[NAV_MAX_WAYPOINTS];
    bool                        waypointListValid;
    int8_t                      waypointCount;

    navWaypointPosition_t       activeWaypoint;     // Local position and initial bearing, filled on waypoint activation
    int8_t                      activeWaypointIndex;

    /* Internals & statistics */
    int16_t                     rcAdjustment[4];
    float                       totalTripDistance;
} navigationPosControl_t;

#if defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE)
PG_DECLARE_ARRAY(navWaypoint_t, NAV_MAX_WAYPOINTS, nonVolatileWaypointList);
#endif

extern navigationPosControl_t posControl;

/* Internally used functions */
const navEstimatedPosVel_t * navGetCurrentActualPositionAndVelocity(void);

float navPidApply2(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags);
float navPidApply3(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags, const float gainScaler);
void navPidReset(pidController_t *pid);
void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD, float _kFF, float _dTermLpfHz);

bool isThrustFacingDownwards(void);
uint32_t calculateDistanceToDestination(const fpVector3_t * destinationPos);
int32_t calculateBearingToDestination(const fpVector3_t * destinationPos);
void resetLandingDetector(void);
bool isLandingDetected(void);

navigationFSMStateFlags_t navGetCurrentStateFlags(void);

void setHomePosition(const fpVector3_t * pos, int32_t yaw, navSetWaypointFlags_t useMask, navigationHomeFlags_t homeFlags);
void setDesiredPosition(const fpVector3_t * pos, int32_t yaw, navSetWaypointFlags_t useMask);
void setDesiredSurfaceOffset(float surfaceOffset);
void setDesiredPositionToFarAwayTarget(int32_t yaw, int32_t distance, navSetWaypointFlags_t useMask);
void updateClimbRateToAltitudeController(float desiredClimbRate, climbRateToAltitudeControllerMode_e mode);

bool isWaypointReached(const navWaypointPosition_t * waypoint, const bool isWaypointHome);
bool isWaypointMissed(const navWaypointPosition_t * waypoint);
bool isApproachingLastWaypoint(void);
float getActiveWaypointSpeed(void);

void updateActualHeading(bool headingValid, int32_t newHeading);
void updateActualHorizontalPositionAndVelocity(bool estPosValid, bool estVelValid, float newX, float newY, float newVelX, float newVelY);
void updateActualAltitudeAndClimbRate(bool estimateValid, float newAltitude, float newVelocity, float surfaceDistance, float surfaceVelocity, navigationEstimateStatus_e surfaceStatus);

bool checkForPositionSensorTimeout(void);

bool isGPSGlitchDetected(void);

/* Multicopter-specific functions */
void setupMulticopterAltitudeController(void);

void resetMulticopterAltitudeController(void);
void resetMulticopterPositionController(void);
void resetMulticopterHeadingController(void);
void resetMulticopterBrakingMode(void);

bool adjustMulticopterAltitudeFromRCInput(void);
bool adjustMulticopterHeadingFromRCInput(void);
bool adjustMulticopterPositionFromRCInput(float rcPitchAdjustment, float rcRollAdjustment);

void applyMulticopterNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs);

void resetFixedWingLandingDetector(void);
void resetMulticopterLandingDetector(void);

bool isMulticopterLandingDetected(void);
bool isFixedWingLandingDetected(void);

void calculateMulticopterInitialHoldPosition(fpVector3_t * pos);

/* Fixed-wing specific functions */
void setupFixedWingAltitudeController(void);

void resetFixedWingAltitudeController(void);
void resetFixedWingPositionController(void);
void resetFixedWingHeadingController(void);

bool adjustFixedWingAltitudeFromRCInput(void);
bool adjustFixedWingHeadingFromRCInput(void);
bool adjustFixedWingPositionFromRCInput(void);

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs);

void calculateFixedWingInitialHoldPosition(fpVector3_t * pos);

/* Fixed-wing launch controller */
void resetFixedWingLaunchController(timeUs_t currentTimeUs);
bool isFixedWingLaunchDetected(void);
void enableFixedWingLaunchController(timeUs_t currentTimeUs);
bool isFixedWingLaunchFinishedOrAborted(void);
void abortFixedWingLaunch(void);
void applyFixedWingLaunchController(timeUs_t currentTimeUs);

#endif
