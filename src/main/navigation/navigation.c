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


#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_control.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"


// Multirotors:
#define MR_RTH_CLIMB_OVERSHOOT_CM   100  // target this amount of cm *above* the target altitude to ensure it is actually reached (Vz > 0 at target alt)
#define MR_RTH_CLIMB_MARGIN_MIN_CM  100  // start cruising home this amount of cm *before* reaching the cruise altitude (while continuing the ascend)
#define MR_RTH_CLIMB_MARGIN_PERCENT 15   // on high RTH altitudes use even bigger margin - percent of the altitude set
// Planes:
#define FW_RTH_CLIMB_OVERSHOOT_CM   100
#define FW_RTH_CLIMB_MARGIN_MIN_CM  100
#define FW_RTH_CLIMB_MARGIN_PERCENT 15

/*-----------------------------------------------------------
 * Compatibility for home position
 *-----------------------------------------------------------*/
gpsLocation_t GPS_home;
uint32_t      GPS_distanceToHome;        // distance to home point in meters
int16_t       GPS_directionToHome;       // direction to home point in degrees

radar_pois_t radar_pois[RADAR_MAX_POIS];

#if defined(USE_NAV)
#if defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE)
PG_REGISTER_ARRAY(navWaypoint_t, NAV_MAX_WAYPOINTS, nonVolatileWaypointList, PG_WAYPOINT_MISSION_STORAGE, 0);
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(navConfig_t, navConfig, PG_NAV_CONFIG, 4);

PG_RESET_TEMPLATE(navConfig_t, navConfig,
    .general = {

        .flags = {
            .use_thr_mid_for_althold = 0,
            .extra_arming_safety = NAV_EXTRA_ARMING_SAFETY_ON,
            .user_control_mode = NAV_GPS_ATTI,
            .rth_alt_control_mode = NAV_RTH_AT_LEAST_ALT,
            .rth_climb_first = 1,               // Climb first, turn after reaching safe altitude
            .rth_climb_ignore_emerg = 0,        // Ignore GPS loss on initial climb
            .rth_tail_first = 0,
            .disarm_on_landing = 0,
            .rth_allow_landing = NAV_RTH_ALLOW_LANDING_ALWAYS,
            .auto_overrides_motor_stop = 1,
        },

        // General navigation parameters
        .pos_failure_timeout = 5,               // 5 sec
        .waypoint_radius = 100,                 // 2m diameter
        .waypoint_safe_distance = 10000,        // centimeters - first waypoint should be closer than this
        .max_auto_speed = 300,                  // 3 m/s = 10.8 km/h
        .max_auto_climb_rate = 500,             // 5 m/s
        .max_manual_speed = 500,
        .max_manual_climb_rate = 200,
        .land_descent_rate = 200,               // centimeters/s
        .land_slowdown_minalt = 500,            // altitude in centimeters
        .land_slowdown_maxalt = 2000,           // altitude in meters
        .emerg_descent_rate = 500,              // centimeters/s
        .min_rth_distance = 500,                // centimeters, if closer than this land immediately
        .rth_altitude = 1000,                   // altitude in centimeters
        .rth_home_altitude = 0,                 // altitude in centimeters
        .rth_abort_threshold = 50000,           // centimeters - 500m should be safe for all aircraft
        .max_terrain_follow_altitude = 100,     // max altitude in centimeters in terrain following mode
    },

    // MC-specific
    .mc = {
        .max_bank_angle = 30,                   // degrees
        .hover_throttle = 1500,
        .auto_disarm_delay = 2000,              // milliseconds - time before disarming when auto disarm is enabled and landing is confirmed
        .braking_speed_threshold = 100,         // Braking can become active above 1m/s
        .braking_disengage_speed = 75,          // Stop when speed goes below 0.75m/s
        .braking_timeout = 2000,                // Timeout barking after 2s
        .braking_boost_factor = 100,            // A 100% boost by default
        .braking_boost_timeout = 750,           // Timout boost after 750ms
        .braking_boost_speed_threshold = 150,   // Boost can happen only above 1.5m/s
        .braking_boost_disengage_speed = 100,   // Disable boost at 1m/s
        .braking_bank_angle = 40,               // Max braking angle     
        .posDecelerationTime = 120,             // posDecelerationTime * 100
        .posResponseExpo = 10,                  // posResponseExpo * 100
    },

    // Fixed wing
    .fw = {
        .max_bank_angle = 35,                   // degrees
        .max_climb_angle = 20,                  // degrees
        .max_dive_angle = 15,                   // degrees
        .cruise_throttle = 1400,
        .cruise_speed = 0,                      // cm/s
        .max_throttle = 1700,
        .min_throttle = 1200,
        .pitch_to_throttle = 10,                // pwm units per degree of pitch (10pwm units ~ 1% throttle)
        .loiter_radius = 5000,                  // 50m

        //Fixed wing landing
        .land_dive_angle = 2,                   // 2 degrees dive by default

        // Fixed wing launch
        .launch_velocity_thresh = 300,          // 3 m/s
        .launch_accel_thresh = 1.9f * 981,      // cm/s/s (1.9*G)
        .launch_time_thresh = 40,               // 40ms
        .launch_throttle = 1700,
        .launch_idle_throttle = 1000,           // Motor idle or MOTOR_STOP
        .launch_motor_timer = 500,              // ms
        .launch_motor_spinup_time = 100,        // ms, time to gredually increase throttle from idle to launch
        .launch_min_time = 0,                   // ms, min time in launch mode
        .launch_timeout = 5000,                 // ms, timeout for launch procedure
        .launch_max_altitude = 0,               // cm, altitude where to consider launch ended
        .launch_climb_angle = 18,               // 18 degrees
        .launch_max_angle = 45,                 // 45 deg
        .cruise_yaw_rate  = 20,                 // 20dps
        .allow_manual_thr_increase = false
    }
);

navigationPosControl_t  posControl;
navSystemStatus_t       NAV_Status;

#if defined(NAV_BLACKBOX)
int16_t navCurrentState;
int16_t navActualVelocity[3];
int16_t navDesiredVelocity[3];
int16_t navActualHeading;
int16_t navDesiredHeading;
int32_t navTargetPosition[3];
int32_t navLatestActualPosition[3];
int16_t navActualSurface;
uint16_t navFlags;
uint16_t navEPH;
uint16_t navEPV;
int16_t navAccNEU[3];
#endif

static fpVector3_t * rthGetHomeTargetPosition(rthTargetMode_e mode);
static void updateDesiredRTHAltitude(void);
static void resetAltitudeController(bool useTerrainFollowing);
static void resetPositionController(void);
static void setupAltitudeController(void);
static void resetHeadingController(void);
void resetGCSFlags(void);

static void calculateAndSetActiveWaypoint(const navWaypoint_t * waypoint);
static void calculateAndSetActiveWaypointToLocalPosition(const fpVector3_t * pos);
void calculateInitialHoldPosition(fpVector3_t * pos);
void calculateFarAwayTarget(fpVector3_t * farAwayPos, int32_t yaw, int32_t distance);
void calculateNewCruiseTarget(fpVector3_t * origin, int32_t yaw, int32_t distance);
static bool isWaypointPositionReached(const fpVector3_t * pos, const bool isWaypointHome);

void initializeRTHSanityChecker(const fpVector3_t * pos);
bool validateRTHSanityChecker(void);

/*************************************************************************************************/
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_IDLE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_ADJUSTING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_ADJUSTING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HEAD_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_ABOVE_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_NEXT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_WAIT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS(navigationFSMState_t previousState);

static const navigationFSMStateDescriptor_t navFSM[NAV_STATE_COUNT] = {
    /** Idle state ******************************************************/
    [NAV_STATE_IDLE] = {
        .persistentId = NAV_PERSISTENT_ID_IDLE,
        .onEntry = navOnEnteringState_NAV_STATE_IDLE,
        .timeoutMs = 0,
        .stateFlags = 0,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_LAUNCH]            = NAV_STATE_LAUNCH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    /** ALTHOLD mode ***************************************************/
    [NAV_STATE_ALTHOLD_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_ANGLE_FW | NAV_REQUIRE_THRTILT | NAV_REQUIRE_THR_BIDIR,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_ALTHOLD_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_ALTHOLD_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_ANGLE_FW | NAV_REQUIRE_THRTILT | NAV_REQUIRE_THR_BIDIR | NAV_RC_ALT,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_ALTHOLD_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    /** POSHOLD_3D mode ************************************************/
    [NAV_STATE_POSHOLD_3D_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT | NAV_REQUIRE_THR_BIDIR,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_POSHOLD_3D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_POSHOLD_3D_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT | NAV_REQUIRE_THR_BIDIR | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_POSHOLD_3D_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },
    /** CRUISE_2D mode ************************************************/
    [NAV_STATE_CRUISE_2D_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_2D_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_2D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_CRUISE_2D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_CRUISE_2D_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_2D_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_2D_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_CRUISE_2D_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_ADJ]        = NAV_STATE_CRUISE_2D_ADJUSTING,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

        [NAV_STATE_CRUISE_2D_ADJUSTING] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_2D_ADJUSTING,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_2D_ADJUSTING,
        .timeoutMs = 10,
        .stateFlags =  NAV_REQUIRE_ANGLE | NAV_RC_POS,
        .mapToFlightModes = NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_CRUISE_2D_IN_PROGRESS,
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_CRUISE_2D_ADJUSTING,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** CRUISE_3D mode ************************************************/
    [NAV_STATE_CRUISE_3D_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_3D_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_3D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_CRUISE_3D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_CRUISE_3D_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_3D_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_3D_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_CTL_ALT | NAV_REQUIRE_ANGLE | NAV_RC_POS | NAV_RC_YAW | NAV_RC_ALT,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_CRUISE_3D_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_ADJ]        = NAV_STATE_CRUISE_3D_ADJUSTING,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_CRUISE_3D_ADJUSTING] = {
        .persistentId = NAV_PERSISTENT_ID_CRUISE_3D_ADJUSTING,
        .onEntry = navOnEnteringState_NAV_STATE_CRUISE_3D_ADJUSTING,
        .timeoutMs = 10,
        .stateFlags =  NAV_CTL_ALT | NAV_REQUIRE_ANGLE | NAV_RC_POS | NAV_RC_ALT,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_CRUISE_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_CRUISE_3D_IN_PROGRESS,
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_CRUISE_3D_ADJUSTING,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** RTH_3D mode ************************************************/
    [NAV_STATE_RTH_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_INITIALIZE,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_INITIALIZE,      // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING]       = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_CLIMB_TO_SAFE_ALT] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,     // allow pos adjustment while climbind to safe alt
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_WAIT_FOR_RTH_ALT,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_HEAD_HOME] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_HEAD_HOME,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_HEAD_HOME,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_HEAD_HOME,           // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_HOVER_PRIOR_TO_LANDING,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
        .timeoutMs = 500,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_SETTLE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME] = NAV_STATE_RTH_HOVER_ABOVE_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_HOVER_ABOVE_HOME] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_HOVER_ABOVE_HOME,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_HOVER_ABOVE_HOME,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW | NAV_RC_ALT,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_HOVER_ABOVE_HOME,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_HOVER_ABOVE_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_LANDING] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_LANDING,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_LANDING,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_CTL_LAND | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_LANDING,         // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_FINISHING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_FINISHING] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_FINISHING,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_FINISHED] = {
        .persistentId = NAV_PERSISTENT_ID_RTH_FINISHED,
        .onEntry = navOnEnteringState_NAV_STATE_RTH_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LANDED,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_FINISHED,         // re-process state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** WAYPOINT mode ************************************************/
    [NAV_STATE_WAYPOINT_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_PRE_ACTION,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_PRE_ACTION] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_WP_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_REACHED,       // successfully reached waypoint
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_REACHED] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_REACHED,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_REACHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_REACHED,   // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_NEXT,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND] = NAV_STATE_WAYPOINT_RTH_LAND,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_RTH_LAND] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_CTL_LAND | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_RTH_LAND,   // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_NEXT] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_NEXT,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_NEXT,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_PRE_ACTION,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_FINISHED] = {
        .persistentId = NAV_PERSISTENT_ID_WAYPOINT_FINISHED,
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_WP_ENROUTE,
        .mwError = MW_NAV_ERROR_FINISH,
        .onEvent = {
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D]         = NAV_STATE_CRUISE_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D]         = NAV_STATE_CRUISE_3D_INITIALIZE,
        }
    },

    /** EMERGENCY LANDING ************************************************/
    [NAV_STATE_EMERGENCY_LANDING_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_EMERGENCY_LANDING,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_IDLE,   // ALTHOLD also bails out from emergency (to IDLE, AltHold will take over from there)
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_EMERGENCY_LANDING,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_IDLE,   // ALTHOLD also bails out from emergency (to IDLE, AltHold will take over from there)
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_FINISHED] = {
        .persistentId = NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED,
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_LANDED,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_INITIALIZE] = {
        .persistentId = NAV_PERSISTENT_ID_LAUNCH_INITIALIZE,
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_LAUNCH_WAIT,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_WAIT] = {
        .persistentId = NAV_PERSISTENT_ID_LAUNCH_WAIT,
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_WAIT,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_LAUNCH | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_LAUNCH_WAIT,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_LAUNCH_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_IN_PROGRESS] = {
        .persistentId = NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS,
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_LAUNCH | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_LAUNCH_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },
};

static navigationFSMStateFlags_t navGetStateFlags(navigationFSMState_t state)
{
    return navFSM[state].stateFlags;
}

static flightModeFlags_e navGetMappedFlightModes(navigationFSMState_t state)
{
    return navFSM[state].mapToFlightModes;
}

navigationFSMStateFlags_t navGetCurrentStateFlags(void)
{
    return navGetStateFlags(posControl.navState);
}

static bool navTerrainFollowingRequested(void)
{
    // Terrain following not supported on FIXED WING aircraft yet
    return !STATE(FIXED_WING) && IS_RC_MODE_ACTIVE(BOXSURFACE);
}

/*************************************************************************************************/
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_IDLE(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    resetAltitudeController(false);
    resetHeadingController();
    resetPositionController();

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);
    const bool terrainFollowingToggled = (posControl.flags.isTerrainFollowEnabled != navTerrainFollowingRequested());

    resetGCSFlags();

    // If surface tracking mode changed value - reset altitude controller
    if ((prevFlags & NAV_CTL_ALT) == 0 || terrainFollowingToggled) {
        resetAltitudeController(navTerrainFollowingRequested());
    }

    if (((prevFlags & NAV_CTL_ALT) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0) || terrainFollowingToggled) {
        setupAltitudeController();
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);  // This will reset surface offset
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If GCS was disabled - reset altitude setpoint
    if (posControl.flags.isGCSAssistedNavigationReset) {
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
        resetGCSFlags();
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);
    const bool terrainFollowingToggled = (posControl.flags.isTerrainFollowEnabled != navTerrainFollowingRequested());

    resetGCSFlags();

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    if ((prevFlags & NAV_CTL_ALT) == 0 || terrainFollowingToggled) {
        resetAltitudeController(navTerrainFollowingRequested());
        setupAltitudeController();
    }

    if (((prevFlags & NAV_CTL_ALT) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0) || terrainFollowingToggled) {
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);  // This will reset surface offset
    }

    if ((previousState != NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING) && (previousState != NAV_STATE_RTH_HOVER_ABOVE_HOME) && (previousState != NAV_STATE_RTH_LANDING)) {
        fpVector3_t targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If GCS was disabled - reset 2D pos setpoint
    if (posControl.flags.isGCSAssistedNavigationReset) {
        fpVector3_t targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
        resetGCSFlags();
    }

    return NAV_FSM_EVENT_NONE;
}
/////////////////

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if (!STATE(FIXED_WING)) { return NAV_FSM_EVENT_ERROR; } // Only on FW for now

    DEBUG_SET(DEBUG_CRUISE, 0, 1);
    if (checkForPositionSensorTimeout()) { return NAV_FSM_EVENT_SWITCH_TO_IDLE; }  // Switch to IDLE if we do not have an healty position. Try the next iteration.

    if (!(prevFlags & NAV_CTL_POS)) {
        resetPositionController();
    }

    posControl.cruise.yaw = posControl.actualState.yaw; // Store the yaw to follow
    posControl.cruise.previousYaw = posControl.cruise.yaw;
    posControl.cruise.lastYawAdjustmentTime = 0;

    return NAV_FSM_EVENT_SUCCESS; // Go to CRUISE_XD_IN_PROGRESS state
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_IN_PROGRESS(navigationFSMState_t previousState)
{
    const timeMs_t currentTimeMs = millis();

    if (checkForPositionSensorTimeout()) { return NAV_FSM_EVENT_SWITCH_TO_IDLE; } // Switch to IDLE if we do not have an healty position. Do the CRUISE init the next iteration.

    DEBUG_SET(DEBUG_CRUISE, 0, 2);
    DEBUG_SET(DEBUG_CRUISE, 2, 0);

    if (posControl.flags.isAdjustingPosition) {
        return NAV_FSM_EVENT_SWITCH_TO_CRUISE_ADJ;
    }

    // User is yawing. We record the desidered yaw and we change the desidered target in the meanwhile
    if (posControl.flags.isAdjustingHeading) {
        timeMs_t timeDifference = currentTimeMs - posControl.cruise.lastYawAdjustmentTime;
        if (timeDifference > 100) timeDifference = 0; // if adjustment was called long time ago, reset the time difference.
        // XXX: This was using rcCommand, changed to use the input instead
        // of the input
        // XXX2: Delete after code review
        float rateTarget = scaleRangef(rcControlGetInput()->yaw, RC_COMMAND_MIN, RC_COMMAND_MAX, -DEGREES_TO_CENTIDEGREES(navConfig()->fw.cruise_yaw_rate), DEGREES_TO_CENTIDEGREES(navConfig()->fw.cruise_yaw_rate));
        float centidegsPerIteration = rateTarget * timeDifference / 1000.0f;
        posControl.cruise.yaw = wrap_36000(posControl.cruise.yaw - centidegsPerIteration);
        DEBUG_SET(DEBUG_CRUISE, 1, CENTIDEGREES_TO_DEGREES(posControl.cruise.yaw));
        posControl.cruise.lastYawAdjustmentTime = currentTimeMs;
    }

    if (currentTimeMs - posControl.cruise.lastYawAdjustmentTime > 4000)
        posControl.cruise.previousYaw = posControl.cruise.yaw;

    uint32_t distance = gpsSol.groundSpeed * 60; // next WP to be reached in 60s [cm]

    if ((previousState == NAV_STATE_CRUISE_2D_INITIALIZE) || (previousState == NAV_STATE_CRUISE_2D_ADJUSTING)
            || (previousState == NAV_STATE_CRUISE_3D_INITIALIZE) || (previousState == NAV_STATE_CRUISE_3D_ADJUSTING)
            || posControl.flags.isAdjustingHeading) {
        calculateFarAwayTarget(&posControl.cruise.targetPos, posControl.cruise.yaw, distance);
        DEBUG_SET(DEBUG_CRUISE, 2, 1);
    } else if (calculateDistanceToDestination(&posControl.cruise.targetPos) <= (navConfig()->fw.loiter_radius * 1.10f)) { //10% margin
        calculateNewCruiseTarget(&posControl.cruise.targetPos, posControl.cruise.yaw, distance);
        DEBUG_SET(DEBUG_CRUISE, 2, 2);
    }

    setDesiredPosition(&posControl.cruise.targetPos, posControl.cruise.yaw, NAV_POS_UPDATE_XY);

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_2D_ADJUSTING(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    DEBUG_SET(DEBUG_CRUISE, 0, 3);

    // User is rolling, changing manually direction. Wait until it is done and then restore CRUISE
    if (posControl.flags.isAdjustingPosition) {
        posControl.cruise.yaw = posControl.actualState.yaw; //store current heading
        posControl.cruise.lastYawAdjustmentTime = millis();
        return NAV_FSM_EVENT_NONE;  // reprocess the state
    }

    resetPositionController();

    return NAV_FSM_EVENT_SUCCESS; // go back to the CRUISE_XD_IN_PROGRESS state
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_INITIALIZE(navigationFSMState_t previousState)
{
    if (!STATE(FIXED_WING)) { return NAV_FSM_EVENT_ERROR; } // only on FW for now

    navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(previousState);

    return navOnEnteringState_NAV_STATE_CRUISE_2D_INITIALIZE(previousState);
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_IN_PROGRESS(navigationFSMState_t previousState)
{
    navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(previousState);

    return navOnEnteringState_NAV_STATE_CRUISE_2D_IN_PROGRESS(previousState);
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_CRUISE_3D_ADJUSTING(navigationFSMState_t previousState)
{
    navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(previousState);

    return navOnEnteringState_NAV_STATE_CRUISE_2D_ADJUSTING(previousState);
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if ((posControl.flags.estHeadingStatus == EST_NONE) || (posControl.flags.estAltStatus == EST_NONE) || (posControl.flags.estPosStatus != EST_TRUSTED) || !STATE(GPS_FIX_HOME)) {
        // Heading sensor, altitude sensor and HOME fix are mandatory for RTH. If not satisfied - switch to emergency landing
        // If we are in dead-reckoning mode - also fail, since coordinates may be unreliable
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (STATE(FIXED_WING) && (posControl.homeDistance < navConfig()->general.min_rth_distance) && !posControl.flags.forcedRTHActivated) {
        // Prevent RTH from activating on airplanes if too close to home unless it's a failsafe RTH
        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
    }

    // If we have valid position sensor or configured to ignore it's loss at initial stage - continue
    if ((posControl.flags.estPosStatus >= EST_USABLE) || navConfig()->general.flags.rth_climb_ignore_emerg) {
        // Reset altitude and position controllers if necessary
        if ((prevFlags & NAV_CTL_POS) == 0) {
            resetPositionController();
        }

        // Reset altitude controller if it was not enabled or if we are in terrain follow mode
        if ((prevFlags & NAV_CTL_ALT) == 0 || posControl.flags.isTerrainFollowEnabled) {
            // Make sure surface tracking is not enabled - RTH uses global altitude, not AGL
            resetAltitudeController(false);
            setupAltitudeController();
        }

        // If close to home - reset home position and land
        if (posControl.homeDistance < navConfig()->general.min_rth_distance) {
            setHomePosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING, NAV_HOME_VALID_ALL);
            setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);

            return NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING;   // NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING
        }
        else {
            fpVector3_t targetHoldPos;

            if (STATE(FIXED_WING)) {
                // Airplane - climbout before turning around
                calculateFarAwayTarget(&targetHoldPos, posControl.actualState.yaw, 100000.0f);  // 1km away
            } else {
                // Multicopter, hover and climb
                calculateInitialHoldPosition(&targetHoldPos);

                // Initialize RTH sanity check to prevent fly-aways on RTH
                // For airplanes this is delayed until climb-out is finished
                initializeRTHSanityChecker(&targetHoldPos);
            }

            setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);

            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_CLIMB_TO_SAFE_ALT
        }
    }
    /* Position sensor failure timeout - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if ((posControl.flags.estHeadingStatus == EST_NONE)) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If we have valid pos sensor OR we are configured to ignore GPS loss
    if ((posControl.flags.estPosStatus >= EST_USABLE) || !checkForPositionSensorTimeout() || navConfig()->general.flags.rth_climb_ignore_emerg) {
        const uint8_t rthClimbMarginPercent = STATE(FIXED_WING) ? FW_RTH_CLIMB_MARGIN_PERCENT : MR_RTH_CLIMB_MARGIN_PERCENT;
        const float rthAltitudeMargin = MAX(FW_RTH_CLIMB_MARGIN_MIN_CM, (rthClimbMarginPercent/100.0) * fabsf(posControl.rthState.rthInitialAltitude - posControl.rthState.homePosition.pos.z));

        // If we reached desired initial RTH altitude or we don't want to climb first
        if (((navGetCurrentActualPositionAndVelocity()->pos.z - posControl.rthState.rthInitialAltitude) > -rthAltitudeMargin) || (!navConfig()->general.flags.rth_climb_first)) {

            // Delayed initialization for RTH sanity check on airplanes - allow to finish climb first as it can take some distance
            if (STATE(FIXED_WING)) {
                initializeRTHSanityChecker(&navGetCurrentActualPositionAndVelocity()->pos);
            }

            // Save initial home distance for future use
            posControl.rthState.rthInitialDistance = posControl.homeDistance;
            fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_ENROUTE_INITIAL);

            if (navConfig()->general.flags.rth_tail_first && !STATE(FIXED_WING)) {
                setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
            }
            else {
                setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
            }

            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_HEAD_HOME

        } else {

            fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_ENROUTE_INITIAL);

            /* For multi-rotors execute sanity check during initial ascent as well */
            if (!STATE(FIXED_WING) && !validateRTHSanityChecker()) {
                return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
            }

            // Climb to safe altitude and turn to correct direction
            if (STATE(FIXED_WING)) {
                tmpHomePos->z += FW_RTH_CLIMB_OVERSHOOT_CM;
                setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z);
            } else {
                // Until the initial climb phase is complete target slightly *above* the cruise altitude to ensure we actually reach
                // it in a reasonable time. Immediately after we finish this phase - target the original altitude.
                tmpHomePos->z += MR_RTH_CLIMB_OVERSHOOT_CM;

                if (navConfig()->general.flags.rth_tail_first) {
                    setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
                } else {
                    setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
                }
            }

            return NAV_FSM_EVENT_NONE;

        }
    }
    /* Position sensor failure timeout - land */
    else {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if ((posControl.flags.estHeadingStatus == EST_NONE)) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If we have position sensor - continue home
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_ENROUTE_PROPORTIONAL);

        if (isWaypointPositionReached(tmpHomePos, true)) {
            // Successfully reached position target - update XYZ-position
            setDesiredPosition(tmpHomePos, posControl.rthState.homePosition.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
            return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING
        }
        else if (!validateRTHSanityChecker()) {
            // Sanity check of RTH
            return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
        }
        else {
            setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z);
            return NAV_FSM_EVENT_NONE;
        }
    }
    /* Position sensor failure timeout - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if ((posControl.flags.estHeadingStatus == EST_NONE)) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If position ok OR within valid timeout - continue
    if ((posControl.flags.estPosStatus >= EST_USABLE) || !checkForPositionSensorTimeout()) {

        // Wait until target heading is reached (with 15 deg margin for error)
        if (STATE(FIXED_WING)) {
            resetLandingDetector();
            updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
            return navigationRTHAllowsLanding() ? NAV_FSM_EVENT_SUCCESS : NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME;
        }
        else {
            if (ABS(wrap_18000(posControl.rthState.homePosition.yaw - posControl.actualState.yaw)) < DEGREES_TO_CENTIDEGREES(15)) {
                resetLandingDetector();
                updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
                return navigationRTHAllowsLanding() ? NAV_FSM_EVENT_SUCCESS : NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME;
            }
            else if (!validateRTHSanityChecker()) {
                // Continue to check for RTH sanity during pre-landing hover
                return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
            }
            else {
                fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_ENROUTE_FINAL);
                setDesiredPosition(tmpHomePos, posControl.rthState.homePosition.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
                return NAV_FSM_EVENT_NONE;
            }
        }

    } else {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_ABOVE_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!(validateRTHSanityChecker() || (posControl.flags.estPosStatus >= EST_USABLE) || !checkForPositionSensorTimeout()))
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;

    fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_FINAL_HOVER);

    if (navConfig()->general.rth_home_altitude) {
        float timeToReachHomeAltitude = fabsf(tmpHomePos->z - navGetCurrentActualPositionAndVelocity()->pos.z) / navConfig()->general.max_auto_climb_rate;

        if (timeToReachHomeAltitude < 1) {
            // we almost reached the target home altitude so set the desired altitude to the desired home altitude
            setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z);
        } else {
            float altitudeChangeDirection = tmpHomePos->z > navGetCurrentActualPositionAndVelocity()->pos.z ? 1 : -1;
            updateClimbRateToAltitudeController(altitudeChangeDirection * navConfig()->general.max_auto_climb_rate, ROC_TO_ALT_NORMAL);
        }
    }
    else {
        setDesiredPosition(tmpHomePos, 0, NAV_POS_UPDATE_Z);
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!ARMING_FLAG(ARMED)) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else if (isLandingDetected()) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        if (!validateRTHSanityChecker()) {
            // Continue to check for RTH sanity during landing
            return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
        }

        float descentVelLimited = 0;

        // A safeguard - if surface altitude sensors is available and it is reading < 50cm altitude - drop to low descend speed
        if ((posControl.flags.estAglStatus == EST_TRUSTED) && posControl.actualState.agl.pos.z < 50.0f) {
            // land_descent_rate == 200 : descend speed = 30 cm/s, gentle touchdown
            // Do not allow descent velocity slower than -30cm/s so the landing detector works.
            descentVelLimited = MIN(-0.15f * navConfig()->general.land_descent_rate, -30.0f);
        }
        else {
            fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_FINAL_LAND);

            // Ramp down descent velocity from 100% at maxAlt altitude to 25% from minAlt to 0cm.
            float descentVelScaling = (navGetCurrentActualPositionAndVelocity()->pos.z - tmpHomePos->z - navConfig()->general.land_slowdown_minalt)
                / (navConfig()->general.land_slowdown_maxalt - navConfig()->general.land_slowdown_minalt) * 0.75f + 0.25f;  // Yield 1.0 at 2000 alt and 0.25 at 500 alt

            descentVelScaling = constrainf(descentVelScaling, 0.25f, 1.0f);

            // Do not allow descent velocity slower than -50cm/s so the landing detector works.
            descentVelLimited = MIN(-descentVelScaling * navConfig()->general.land_descent_rate, -50.0f);
        }

        updateClimbRateToAltitudeController(descentVelLimited, ROC_TO_ALT_NORMAL);

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (navConfig()->general.flags.disarm_on_landing) {
        disarm(DISARM_NAVIGATION);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHED(navigationFSMState_t previousState)
{
    // Stay in this state
    UNUSED(previousState);
    updateClimbRateToAltitudeController(-0.3f * navConfig()->general.land_descent_rate, ROC_TO_ALT_NORMAL);  // FIXME

    // Prevent I-terms growing when already landed
    pidResetErrorAccumulators();

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (posControl.waypointCount == 0 || !posControl.waypointListValid) {
        return NAV_FSM_EVENT_ERROR;
    }
    else {
        // Prepare controllers
        resetPositionController();

        // Make sure surface tracking is not enabled - RTH uses global altitude, not AGL
        resetAltitudeController(false);
        setupAltitudeController();

        posControl.activeWaypointIndex = 0;
        return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_PRE_ACTION
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION(navigationFSMState_t previousState)
{
    /* A helper function to do waypoint-specific action */
    UNUSED(previousState);

    switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
        case NAV_WP_ACTION_WAYPOINT:
            calculateAndSetActiveWaypoint(&posControl.waypointList[posControl.activeWaypointIndex]);
            return NAV_FSM_EVENT_SUCCESS;       // will switch to NAV_STATE_WAYPOINT_IN_PROGRESS

        case NAV_WP_ACTION_RTH:
        default:
            posControl.rthState.rthInitialDistance = posControl.homeDistance;
            initializeRTHSanityChecker(&navGetCurrentActualPositionAndVelocity()->pos);
            calculateAndSetActiveWaypointToLocalPosition(rthGetHomeTargetPosition(RTH_HOME_ENROUTE_INITIAL));
            return NAV_FSM_EVENT_SUCCESS;       // will switch to NAV_STATE_WAYPOINT_IN_PROGRESS
    };
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if ((posControl.flags.estPosStatus >= EST_USABLE) && (posControl.flags.estHeadingStatus >= EST_USABLE)) {
        switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
            case NAV_WP_ACTION_WAYPOINT:
            default:
                if (isWaypointReached(&posControl.activeWaypoint, false) || isWaypointMissed(&posControl.activeWaypoint)) {
                    return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_REACHED
                }
                else {
                    setDesiredPosition(&posControl.activeWaypoint.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
                    return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
                }
                break;

            case NAV_WP_ACTION_RTH:
                if (isWaypointReached(&posControl.activeWaypoint, true) || isWaypointMissed(&posControl.activeWaypoint)) {
                    return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_REACHED
                }
                else {
                    setDesiredPosition(&posControl.activeWaypoint.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
                    setDesiredPosition(rthGetHomeTargetPosition(RTH_HOME_ENROUTE_PROPORTIONAL), 0, NAV_POS_UPDATE_Z);
                    return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
                }
                break;
        }
    }
    /* No pos sensor available for NAV_WAIT_FOR_GPS_TIMEOUT_MS - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    else {
        return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
        case NAV_WP_ACTION_RTH:
            if (posControl.waypointList[posControl.activeWaypointIndex].p1 != 0) {
                return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND;
            }
            else {
                return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_WAYPOINT_NEXT
            }

        default:
            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_WAYPOINT_NEXT
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    const navigationFSMEvent_t landEvent = navOnEnteringState_NAV_STATE_RTH_LANDING(previousState);
    if (landEvent == NAV_FSM_EVENT_SUCCESS) {
        // Landing controller returned success - invoke RTH finishing state and finish the waypoint
        navOnEnteringState_NAV_STATE_RTH_FINISHING(previousState);
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_NEXT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    const bool isLastWaypoint = (posControl.waypointList[posControl.activeWaypointIndex].flag == NAV_WP_FLAG_LAST) ||
                          (posControl.activeWaypointIndex >= (posControl.waypointCount - 1));

    if (isLastWaypoint) {
        // Last waypoint reached
        return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED;
    }
    else {
        // Waypoint reached, do something and move on to next waypoint
        posControl.activeWaypointIndex++;
        return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_PRE_ACTION
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if ((posControl.flags.estPosStatus >= EST_USABLE) && (posControl.flags.estHeadingStatus >= EST_USABLE)) {
        return NAV_FSM_EVENT_NONE;
    }
    /* No pos sensor available for NAV_WAIT_FOR_GPS_TIMEOUT_MS - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    else {
        return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);

    // Emergency landing MAY use common altitude controller if vertical position is valid - initialize it
    // Make sure terrain following is not enabled
    resetAltitudeController(false);

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);

    // Prevent I-terms growing when already landed
    pidResetErrorAccumulators();

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE(navigationFSMState_t previousState)
{
    const timeUs_t currentTimeUs = micros();
    UNUSED(previousState);

    resetFixedWingLaunchController(currentTimeUs);

    return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_LAUNCH_WAIT
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_WAIT(navigationFSMState_t previousState)
{
    const timeUs_t currentTimeUs = micros();
    UNUSED(previousState);

    if (isFixedWingLaunchDetected()) {
        enableFixedWingLaunchController(currentTimeUs);
        return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_LAUNCH_IN_PROGRESS
    }

    //allow to leave NAV_LAUNCH_MODE if it has being enabled as feature by moving sticks with low throttle.
    if (feature(FEATURE_FW_LAUNCH)) {
        throttleStatus_e throttleStatus = calculateThrottleStatus();
        if ((throttleStatus == THROTTLE_LOW) && (areSticksDeflectedMoreThanPosHoldDeadband())) {
            abortFixedWingLaunch();
            return NAV_FSM_EVENT_SWITCH_TO_IDLE;
        }
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (isFixedWingLaunchFinishedOrAborted()) {
        return NAV_FSM_EVENT_SUCCESS;
    }

    return NAV_FSM_EVENT_NONE;
}

static void navUpdateBidirThrottleMode(void)
{
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (navStateFlags & NAV_REQUIRE_THR_BIDIR) {
        rcControlSetBidirThrottleMode(RC_CONTROL_BIDIR_THR_ON);
    } else if (navStateFlags & NAV_REQUIRE_THR_UNIDIR) {
        rcControlSetBidirThrottleMode(RC_CONTROL_BIDIR_THR_OFF);
    } else {
        rcControlSetBidirThrottleMode(RC_CONTROL_BIDIR_THR_USER);
    }
}

static navigationFSMState_t navSetNewFSMState(navigationFSMState_t newState)
{
    navigationFSMState_t previousState;

    previousState = posControl.navState;
    if (posControl.navState != newState) {
        posControl.navState = newState;
        posControl.navPersistentId = navFSM[newState].persistentId;
        navUpdateBidirThrottleMode();
    }
    return previousState;
}

static void navProcessFSMEvents(navigationFSMEvent_t injectedEvent)
{
    const timeMs_t currentMillis = millis();
    navigationFSMState_t previousState;
    static timeMs_t lastStateProcessTime = 0;

    /* If timeout event defined and timeout reached - switch state */
    if ((navFSM[posControl.navState].timeoutMs > 0) && (navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT] != NAV_STATE_UNDEFINED) &&
            ((currentMillis - lastStateProcessTime) >= navFSM[posControl.navState].timeoutMs)) {
        /* Update state */
        previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT]);

        /* Call new state's entry function */
        while (navFSM[posControl.navState].onEntry) {
            navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

            if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[newEvent]);
            }
            else {
                break;
            }
        }

        lastStateProcessTime  = currentMillis;
    }

    /* Inject new event */
    if (injectedEvent != NAV_FSM_EVENT_NONE && navFSM[posControl.navState].onEvent[injectedEvent] != NAV_STATE_UNDEFINED) {
        /* Update state */
        previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[injectedEvent]);

        /* Call new state's entry function */
        while (navFSM[posControl.navState].onEntry) {
            navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

            if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[newEvent]);
            }
            else {
                break;
            }
        }

        lastStateProcessTime  = currentMillis;
    }

    /* Update public system state information */
    NAV_Status.mode = MW_GPS_MODE_NONE;

    if (ARMING_FLAG(ARMED)) {
        navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);

        if (navStateFlags & NAV_AUTO_RTH) {
            NAV_Status.mode = MW_GPS_MODE_RTH;
        }
        else if (navStateFlags & NAV_AUTO_WP) {
            NAV_Status.mode = MW_GPS_MODE_NAV;
        }
        else if (navStateFlags & NAV_CTL_EMERG) {
            NAV_Status.mode = MW_GPS_MODE_EMERG;
        }
        else if (navStateFlags & NAV_CTL_POS) {
            NAV_Status.mode = MW_GPS_MODE_HOLD;
        }
    }

    NAV_Status.state = navFSM[posControl.navState].mwState;
    NAV_Status.error = navFSM[posControl.navState].mwError;

    NAV_Status.flags = 0;
    if (posControl.flags.isAdjustingPosition)   NAV_Status.flags |= MW_NAV_FLAG_ADJUSTING_POSITION;
    if (posControl.flags.isAdjustingAltitude)   NAV_Status.flags |= MW_NAV_FLAG_ADJUSTING_ALTITUDE;

    NAV_Status.activeWpNumber = posControl.activeWaypointIndex + 1;
    NAV_Status.activeWpAction = 0;
    if ((posControl.activeWaypointIndex >= 0) && (posControl.activeWaypointIndex < NAV_MAX_WAYPOINTS)) {
        NAV_Status.activeWpAction = posControl.waypointList[posControl.activeWaypointIndex].action;
    }
}

static fpVector3_t * rthGetHomeTargetPosition(rthTargetMode_e mode)
{
    posControl.rthState.homeTmpWaypoint = posControl.rthState.homePosition.pos;

    switch (mode) {
        case RTH_HOME_ENROUTE_INITIAL:
            posControl.rthState.homeTmpWaypoint.z = posControl.rthState.rthInitialAltitude;
            break;

        case RTH_HOME_ENROUTE_PROPORTIONAL:
            {
                float rthTotalDistanceToTravel = posControl.rthState.rthInitialDistance - (STATE(FIXED_WING) ? navConfig()->fw.loiter_radius : 0);
                if (rthTotalDistanceToTravel >= 100) {
                    float ratioNotTravelled = constrainf(posControl.homeDistance / rthTotalDistanceToTravel, 0.0f, 1.0f);
                    posControl.rthState.homeTmpWaypoint.z = (posControl.rthState.rthInitialAltitude * ratioNotTravelled) + (posControl.rthState.rthFinalAltitude * (1.0f - ratioNotTravelled));
                }
                else {
                    posControl.rthState.homeTmpWaypoint.z = posControl.rthState.rthFinalAltitude;
                }
            }
            break;

        case RTH_HOME_ENROUTE_FINAL:
            posControl.rthState.homeTmpWaypoint.z = posControl.rthState.rthFinalAltitude;
            break;

        case RTH_HOME_FINAL_HOVER:
            if (navConfig()->general.rth_home_altitude) {
                posControl.rthState.homeTmpWaypoint.z = posControl.rthState.homePosition.pos.z + navConfig()->general.rth_home_altitude;
            }
            else {
                // If home altitude not defined - fall back to final ENROUTE altitude
                posControl.rthState.homeTmpWaypoint.z = posControl.rthState.rthFinalAltitude;
            }
            break;

        case RTH_HOME_FINAL_LAND:
            break;
    }

    return &posControl.rthState.homeTmpWaypoint;
}

/*-----------------------------------------------------------
 * Float point PID-controller implementation
 *-----------------------------------------------------------*/
// Implementation of PID with back-calculation I-term anti-windup
// Control System Design, Lecture Notes for ME 155A by Karl Johan Åström (p.228)
// http://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
float navPidApply3(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags, const float gainScaler)
{
    float newProportional, newDerivative, newFeedForward;
    float error = setpoint - measurement;

    /* P-term */
    newProportional = error * pid->param.kP * gainScaler;

    /* D-term */
    if (pid->reset) {
        pid->last_input = (pidFlags & PID_DTERM_FROM_ERROR) ? error : measurement;
        pid->reset = false;
    }

    if (pidFlags & PID_DTERM_FROM_ERROR) {
        /* Error-tracking D-term */
        newDerivative = (error - pid->last_input) / dt;
        pid->last_input = error;
    } else {
        /* Measurement tracking D-term */
        newDerivative = -(measurement - pid->last_input) / dt;
        pid->last_input = measurement;
    }

    if (pid->dTermLpfHz > 0) {
        newDerivative = pid->param.kD * pt1FilterApply4(&pid->dterm_filter_state, newDerivative, pid->dTermLpfHz, dt) * gainScaler;
    } else {
        newDerivative = pid->param.kD * newDerivative;
    }

    if (pidFlags & PID_ZERO_INTEGRATOR) {
        pid->integrator = 0.0f;
    }

    /*
     * Compute FeedForward parameter
     */
    newFeedForward = setpoint * pid->param.kFF * gainScaler;

    /* Pre-calculate output and limit it if actuator is saturating */
    const float outVal = newProportional + (pid->integrator * gainScaler) + newDerivative + newFeedForward;
    const float outValConstrained = constrainf(outVal, outMin, outMax);

    pid->proportional = newProportional;
    pid->integral = pid->integrator;
    pid->derivative = newDerivative;
    pid->feedForward = newFeedForward;
    pid->output_constrained = outValConstrained;

    /* Update I-term */
    if (!(pidFlags & PID_ZERO_INTEGRATOR)) {
        const float newIntegrator = pid->integrator + (error * pid->param.kI * gainScaler * dt) + ((outValConstrained - outVal) * pid->param.kT * dt);

        if (pidFlags & PID_SHRINK_INTEGRATOR) {
            // Only allow integrator to shrink
            if (fabsf(newIntegrator) < fabsf(pid->integrator)) {
                pid->integrator = newIntegrator;
            }
        }
        else {
            pid->integrator = newIntegrator;
        }
    }

    return outValConstrained;
}

float navPidApply2(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags)
{
    return navPidApply3(pid, setpoint, measurement, dt, outMin, outMax, pidFlags, 1.0f);
}

void navPidReset(pidController_t *pid)
{
    pid->reset = true;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->last_input = 0.0f;
    pid->feedForward = 0.0f;
    pt1FilterReset(&pid->dterm_filter_state, 0.0f);
    pid->output_constrained = 0.0f;
}

void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD, float _kFF, float _dTermLpfHz)
{
    pid->param.kP = _kP;
    pid->param.kI = _kI;
    pid->param.kD = _kD;
    pid->param.kFF = _kFF;

    if (_kI > 1e-6f && _kP > 1e-6f) {
        float Ti = _kP / _kI;
        float Td = _kD / _kP;
        pid->param.kT = 2.0f / (Ti + Td);
    }
    else {
        pid->param.kI = 0.0;
        pid->param.kT = 0.0;
    }
    pid->dTermLpfHz = _dTermLpfHz;
    navPidReset(pid);
}

/*-----------------------------------------------------------
 * Detects if thrust vector is facing downwards
 *-----------------------------------------------------------*/
bool isThrustFacingDownwards(void)
{
    // Tilt angle <= 80 deg; cos(80) = 0.17364817766693034885171662676931
    return (calculateCosTiltAngle() >= 0.173648178f);
}

/*-----------------------------------------------------------
 * Checks if position sensor (GPS) is failing for a specified timeout (if enabled)
 *-----------------------------------------------------------*/
bool checkForPositionSensorTimeout(void)
{
    if (navConfig()->general.pos_failure_timeout) {
        if ((posControl.flags.estPosStatus == EST_NONE) && ((millis() - posControl.lastValidPositionTimeMs) > (1000 * navConfig()->general.pos_failure_timeout))) {
            return true;
        }
        else {
            return false;
        }
    }
    else {
        // Timeout not defined, never fail
        return false;
    }
}

/*-----------------------------------------------------------
 * Processes an update to XY-position and velocity
 *-----------------------------------------------------------*/
void updateActualHorizontalPositionAndVelocity(bool estPosValid, bool estVelValid, float newX, float newY, float newVelX, float newVelY)
{
    posControl.actualState.abs.pos.x = newX;
    posControl.actualState.abs.pos.y = newY;
    posControl.actualState.abs.vel.x = newVelX;
    posControl.actualState.abs.vel.y = newVelY;

    posControl.actualState.agl.pos.x = newX;
    posControl.actualState.agl.pos.y = newY;
    posControl.actualState.agl.vel.x = newVelX;
    posControl.actualState.agl.vel.y = newVelY;

    posControl.actualState.velXY = sqrtf(sq(newVelX) + sq(newVelY));

    // CASE 1: POS & VEL valid
    if (estPosValid && estVelValid) {
        posControl.flags.estPosStatus = EST_TRUSTED;
        posControl.flags.estVelStatus = EST_TRUSTED;
        posControl.flags.horizontalPositionDataNew = 1;
        posControl.lastValidPositionTimeMs = millis();
    }
    // CASE 1: POS invalid, VEL valid
    else if (!estPosValid && estVelValid) {
        posControl.flags.estPosStatus = EST_USABLE;     // Pos usable, but not trusted
        posControl.flags.estVelStatus = EST_TRUSTED;
        posControl.flags.horizontalPositionDataNew = 1;
        posControl.lastValidPositionTimeMs = millis();
    }
    // CASE 3: can't use pos/vel data
    else {
        posControl.flags.estPosStatus = EST_NONE;
        posControl.flags.estVelStatus = EST_NONE;
        posControl.flags.horizontalPositionDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[X] = newX;
    navLatestActualPosition[Y] = newY;
    navActualVelocity[X] = constrain(newVelX, -32678, 32767);
    navActualVelocity[Y] = constrain(newVelY, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to Z-position and velocity
 *-----------------------------------------------------------*/
void updateActualAltitudeAndClimbRate(bool estimateValid, float newAltitude, float newVelocity, float surfaceDistance, float surfaceVelocity, navigationEstimateStatus_e surfaceStatus)
{
    posControl.actualState.abs.pos.z = newAltitude;
    posControl.actualState.abs.vel.z = newVelocity;

    posControl.actualState.agl.pos.z = surfaceDistance;
    posControl.actualState.agl.vel.z = surfaceVelocity;

    // Update altitude that would be used when executing RTH
    if (estimateValid) {
        updateDesiredRTHAltitude();

        // If we acquired new surface reference - changing from NONE/USABLE -> TRUSTED
        if ((surfaceStatus == EST_TRUSTED) && (posControl.flags.estAglStatus != EST_TRUSTED)) {
            // If we are in terrain-following modes - signal that we should update the surface tracking setpoint
            //      NONE/USABLE means that we were flying blind, now we should lock to surface
            //updateSurfaceTrackingSetpoint();
        }

        posControl.flags.estAglStatus = surfaceStatus;  // Could be TRUSTED or USABLE
        posControl.flags.estAltStatus = EST_TRUSTED;
        posControl.flags.verticalPositionDataNew = 1;
        posControl.lastValidAltitudeTimeMs = millis();
    }
    else {
        posControl.flags.estAltStatus = EST_NONE;
        posControl.flags.estAglStatus = EST_NONE;
        posControl.flags.verticalPositionDataNew = 0;
    }

    if (ARMING_FLAG(ARMED)) {
        if ((posControl.flags.estAglStatus == EST_TRUSTED) && surfaceDistance > 0) {
            if (posControl.actualState.surfaceMin > 0) {
                posControl.actualState.surfaceMin = MIN(posControl.actualState.surfaceMin, surfaceDistance);
            }
            else {
                posControl.actualState.surfaceMin = surfaceDistance;
            }
        }
    }
    else {
        posControl.actualState.surfaceMin = -1;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[Z] = navGetCurrentActualPositionAndVelocity()->pos.z;
    navActualVelocity[Z] = constrain(navGetCurrentActualPositionAndVelocity()->vel.z, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to estimated heading
 *-----------------------------------------------------------*/
void updateActualHeading(bool headingValid, int32_t newHeading)
{
    /* Update heading. Check if we're acquiring a valid heading for the
     * first time and update home heading accordingly.
     */
    navigationEstimateStatus_e newEstHeading = headingValid ? EST_TRUSTED : EST_NONE;
    if (newEstHeading >= EST_USABLE && posControl.flags.estHeadingStatus < EST_USABLE &&
        (posControl.rthState.homeFlags & (NAV_HOME_VALID_XY | NAV_HOME_VALID_Z)) &&
        (posControl.rthState.homeFlags & NAV_HOME_VALID_HEADING) == 0) {

        // Home was stored using the fake heading (assuming boot as 0deg). Calculate
        // the offset from the fake to the actual yaw and apply the same rotation
        // to the home point.
        int32_t fakeToRealYawOffset = newHeading - posControl.actualState.yaw;

        posControl.rthState.homePosition.yaw += fakeToRealYawOffset;
        if (posControl.rthState.homePosition.yaw < 0) {
            posControl.rthState.homePosition.yaw += DEGREES_TO_CENTIDEGREES(360);
        }
        if (posControl.rthState.homePosition.yaw >= DEGREES_TO_CENTIDEGREES(360)) {
            posControl.rthState.homePosition.yaw -= DEGREES_TO_CENTIDEGREES(360);
        }
        posControl.rthState.homeFlags |= NAV_HOME_VALID_HEADING;
    }
    posControl.actualState.yaw = newHeading;
    posControl.flags.estHeadingStatus = newEstHeading;

    /* Precompute sin/cos of yaw angle */
    posControl.actualState.sinYaw = sin_approx(CENTIDEGREES_TO_RADIANS(newHeading));
    posControl.actualState.cosYaw = cos_approx(CENTIDEGREES_TO_RADIANS(newHeading));

    posControl.flags.headingDataNew = 1;
}

/*-----------------------------------------------------------
 * Returns pointer to currently used position (ABS or AGL) depending on surface tracking status
 *-----------------------------------------------------------*/
const navEstimatedPosVel_t * navGetCurrentActualPositionAndVelocity(void)
{
    return posControl.flags.isTerrainFollowEnabled ? &posControl.actualState.agl : &posControl.actualState.abs;
}

/*-----------------------------------------------------------
 * Calculates distance and bearing to destination point
 *-----------------------------------------------------------*/
static uint32_t calculateDistanceFromDelta(float deltaX, float deltaY)
{
    return sqrtf(sq(deltaX) + sq(deltaY));
}

static int32_t calculateBearingFromDelta(float deltaX, float deltaY)
{
    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(deltaY, deltaX)));
}

uint32_t calculateDistanceToDestination(const fpVector3_t * destinationPos)
{
    const navEstimatedPosVel_t *posvel = navGetCurrentActualPositionAndVelocity();
    const float deltaX = destinationPos->x - posvel->pos.x;
    const float deltaY = destinationPos->y - posvel->pos.y;

    return calculateDistanceFromDelta(deltaX, deltaY);
}

int32_t calculateBearingToDestination(const fpVector3_t * destinationPos)
{
    const navEstimatedPosVel_t *posvel = navGetCurrentActualPositionAndVelocity();
    const float deltaX = destinationPos->x - posvel->pos.x;
    const float deltaY = destinationPos->y - posvel->pos.y;

    return calculateBearingFromDelta(deltaX, deltaY);
}

bool navCalculatePathToDestination(navDestinationPath_t *result, const fpVector3_t * destinationPos)
{
    if (posControl.flags.estPosStatus == EST_NONE ||
        posControl.flags.estHeadingStatus == EST_NONE) {

        return false;
    }

    const navEstimatedPosVel_t *posvel = navGetCurrentActualPositionAndVelocity();
    const float deltaX = destinationPos->x - posvel->pos.x;
    const float deltaY = destinationPos->y - posvel->pos.y;

    result->distance = calculateDistanceFromDelta(deltaX, deltaY);
    result->bearing = calculateBearingFromDelta(deltaX, deltaY);
    return true;
}

/*-----------------------------------------------------------
 * Check if waypoint is/was reached. Assume that waypoint-yaw stores initial bearing
 *-----------------------------------------------------------*/
bool isWaypointMissed(const navWaypointPosition_t * waypoint)
{
    int32_t bearingError = calculateBearingToDestination(&waypoint->pos) - waypoint->yaw;
    bearingError = wrap_18000(bearingError);

    return ABS(bearingError) > 10000; // TRUE if we passed the waypoint by 100 degrees
}

static bool isWaypointPositionReached(const fpVector3_t * pos, const bool isWaypointHome)
{
    // We consider waypoint reached if within specified radius
    const uint32_t wpDistance = calculateDistanceToDestination(pos);

    if (STATE(FIXED_WING) && isWaypointHome) {
        // Airplane will do a circular loiter over home and might never approach it closer than waypoint_radius - need extra check
        return (wpDistance <= navConfig()->general.waypoint_radius)
                || (wpDistance <= (navConfig()->fw.loiter_radius * 1.10f));  // 10% margin of desired circular loiter radius
    }
    else {
        return (wpDistance <= navConfig()->general.waypoint_radius);
    }
}

bool isWaypointReached(const navWaypointPosition_t * waypoint, const bool isWaypointHome)
{
    return isWaypointPositionReached(&waypoint->pos, isWaypointHome);
}

static void updateHomePositionCompatibility(void)
{
    geoConvertLocalToGeodetic(&GPS_home, &posControl.gpsOrigin, &posControl.rthState.homePosition.pos);
    GPS_distanceToHome = posControl.homeDistance / 100;
    GPS_directionToHome = posControl.homeDirection / 100;
}

// Backdoor for RTH estimator
float getFinalRTHAltitude(void)
{
    return posControl.rthState.rthFinalAltitude;
}

/*-----------------------------------------------------------
 * Reset home position to current position
 *-----------------------------------------------------------*/
static void updateDesiredRTHAltitude(void)
{
    if (ARMING_FLAG(ARMED)) {
        if (!(navGetStateFlags(posControl.navState) & NAV_AUTO_RTH)) {
            switch (navConfig()->general.flags.rth_alt_control_mode) {
                case NAV_RTH_NO_ALT:
                    posControl.rthState.rthInitialAltitude = posControl.actualState.abs.pos.z;
                    posControl.rthState.rthFinalAltitude = posControl.rthState.rthInitialAltitude;
                    break;

                case NAV_RTH_EXTRA_ALT: // Maintain current altitude + predefined safety margin
                    posControl.rthState.rthInitialAltitude = posControl.actualState.abs.pos.z + navConfig()->general.rth_altitude;
                    posControl.rthState.rthFinalAltitude = posControl.rthState.rthInitialAltitude;
                    break;

                case NAV_RTH_MAX_ALT:
                    posControl.rthState.rthInitialAltitude = MAX(posControl.rthState.rthInitialAltitude, posControl.actualState.abs.pos.z);
                    posControl.rthState.rthFinalAltitude = posControl.rthState.rthInitialAltitude;
                    break;

                case NAV_RTH_AT_LEAST_ALT:  // Climb to at least some predefined altitude above home
                    posControl.rthState.rthInitialAltitude = MAX(posControl.rthState.homePosition.pos.z + navConfig()->general.rth_altitude, posControl.actualState.abs.pos.z);
                    posControl.rthState.rthFinalAltitude = posControl.rthState.rthInitialAltitude;
                    break;

                case NAV_RTH_AT_LEAST_ALT_LINEAR_DESCENT:
                    posControl.rthState.rthInitialAltitude = MAX(posControl.rthState.homePosition.pos.z + navConfig()->general.rth_altitude, posControl.actualState.abs.pos.z);
                    posControl.rthState.rthFinalAltitude = posControl.rthState.homePosition.pos.z + navConfig()->general.rth_altitude;
                    break;

                case NAV_RTH_CONST_ALT:     // Climb/descend to predefined altitude above home
                default:
                    posControl.rthState.rthInitialAltitude = posControl.rthState.homePosition.pos.z + navConfig()->general.rth_altitude;
                    posControl.rthState.rthFinalAltitude = posControl.rthState.rthInitialAltitude;
            }
        }
    } else {
        posControl.rthState.rthInitialAltitude = posControl.actualState.abs.pos.z;
        posControl.rthState.rthFinalAltitude = posControl.actualState.abs.pos.z;
    }
}

/*-----------------------------------------------------------
 * RTH sanity test logic
 *-----------------------------------------------------------*/
void initializeRTHSanityChecker(const fpVector3_t * pos)
{
    const timeMs_t currentTimeMs = millis();

    posControl.rthSanityChecker.lastCheckTime = currentTimeMs;
    posControl.rthSanityChecker.initialPosition = *pos;
    posControl.rthSanityChecker.minimalDistanceToHome = calculateDistanceToDestination(&posControl.rthState.homePosition.pos);
}

bool validateRTHSanityChecker(void)
{
    const timeMs_t currentTimeMs = millis();
    bool checkResult = true;    // Between the checks return the "good" status

    // Ability to disable this
    if (navConfig()->general.rth_abort_threshold == 0) {
        return true;
    }

    // Check at 10Hz rate
    if ((currentTimeMs - posControl.rthSanityChecker.lastCheckTime) > 100) {
        const float currentDistanceToHome = calculateDistanceToDestination(&posControl.rthState.homePosition.pos);

        if (currentDistanceToHome < posControl.rthSanityChecker.minimalDistanceToHome) {
            posControl.rthSanityChecker.minimalDistanceToHome = currentDistanceToHome;
        }
        else if ((currentDistanceToHome - posControl.rthSanityChecker.minimalDistanceToHome) > navConfig()->general.rth_abort_threshold) {
            // If while doing RTH we got even farther away from home - RTH is doing something crazy
            checkResult = false;
        }

        posControl.rthSanityChecker.lastCheckTime = currentTimeMs;
    }

    return checkResult;
}

/*-----------------------------------------------------------
 * Reset home position to current position
 *-----------------------------------------------------------*/
void setHomePosition(const fpVector3_t * pos, int32_t yaw, navSetWaypointFlags_t useMask, navigationHomeFlags_t homeFlags)
{
    // XY-position
    if ((useMask & NAV_POS_UPDATE_XY) != 0) {
        posControl.rthState.homePosition.pos.x = pos->x;
        posControl.rthState.homePosition.pos.y = pos->y;
        if (homeFlags & NAV_HOME_VALID_XY) {
            posControl.rthState.homeFlags |= NAV_HOME_VALID_XY;
        } else {
            posControl.rthState.homeFlags &= ~NAV_HOME_VALID_XY;
        }
    }

    // Z-position
    if ((useMask & NAV_POS_UPDATE_Z) != 0) {
        posControl.rthState.homePosition.pos.z = pos->z;
        if (homeFlags & NAV_HOME_VALID_Z) {
            posControl.rthState.homeFlags |= NAV_HOME_VALID_Z;
        } else {
            posControl.rthState.homeFlags &= ~NAV_HOME_VALID_Z;
        }
    }

    // Heading
    if ((useMask & NAV_POS_UPDATE_HEADING) != 0) {
        // Heading
        posControl.rthState.homePosition.yaw = yaw;
        if (homeFlags & NAV_HOME_VALID_HEADING) {
            posControl.rthState.homeFlags |= NAV_HOME_VALID_HEADING;
        } else {
            posControl.rthState.homeFlags &= ~NAV_HOME_VALID_HEADING;
        }
    }

    posControl.homeDistance = 0;
    posControl.homeDirection = 0;

    // Update target RTH altitude as a waypoint above home
    updateDesiredRTHAltitude();

    updateHomePositionCompatibility();
    ENABLE_STATE(GPS_FIX_HOME);
}

static navigationHomeFlags_t navigationActualStateHomeValidity(void)
{
    navigationHomeFlags_t flags = 0;

    if (posControl.flags.estPosStatus >= EST_USABLE) {
        flags |= NAV_HOME_VALID_XY | NAV_HOME_VALID_Z;
    }

    if (posControl.flags.estHeadingStatus >= EST_USABLE) {
        flags |= NAV_HOME_VALID_HEADING;
    }

    return flags;
}

/*-----------------------------------------------------------
 * Update home position, calculate distance and bearing to home
 *-----------------------------------------------------------*/
void updateHomePosition(void)
{
    // Disarmed and have a valid position, constantly update home
    if (!ARMING_FLAG(ARMED)) {
        if (posControl.flags.estPosStatus >= EST_USABLE) {
            const navigationHomeFlags_t validHomeFlags = NAV_HOME_VALID_XY | NAV_HOME_VALID_Z;
            bool setHome = (posControl.rthState.homeFlags & validHomeFlags) != validHomeFlags;
            switch ((nav_reset_type_e)positionEstimationConfig()->reset_home_type) {
                case NAV_RESET_NEVER:
                    break;
                case NAV_RESET_ON_FIRST_ARM:
                    setHome |= !ARMING_FLAG(WAS_EVER_ARMED);
                    break;
                case NAV_RESET_ON_EACH_ARM:
                    setHome = true;
                    break;
            }
            if (setHome) {
                setHomePosition(&posControl.actualState.abs.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING, navigationActualStateHomeValidity());
            }
        }
    }
    else {
        static bool isHomeResetAllowed = false;

        // If pilot so desires he may reset home position to current position
        if (IS_RC_MODE_ACTIVE(BOXHOMERESET)) {
            if (isHomeResetAllowed && !FLIGHT_MODE(FAILSAFE_MODE) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE) && (posControl.flags.estPosStatus >= EST_USABLE)) {
                const navSetWaypointFlags_t homeUpdateFlags = STATE(GPS_FIX_HOME) ? (NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING) : (NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
                setHomePosition(&posControl.actualState.abs.pos, posControl.actualState.yaw, homeUpdateFlags, navigationActualStateHomeValidity());
                isHomeResetAllowed = false;
            }
        }
        else {
            isHomeResetAllowed = true;
        }

        // Update distance and direction to home if armed (home is not updated when armed)
        if (STATE(GPS_FIX_HOME)) {
            fpVector3_t * tmpHomePos = rthGetHomeTargetPosition(RTH_HOME_FINAL_LAND);
            posControl.homeDistance = calculateDistanceToDestination(tmpHomePos);
            posControl.homeDirection = calculateBearingToDestination(tmpHomePos);
            updateHomePositionCompatibility();
        }
    }
}

/*-----------------------------------------------------------
 * Update flight statistics
 *-----------------------------------------------------------*/
static void updateNavigationFlightStatistics(void)
{
    static timeMs_t previousTimeMs = 0;
    const timeMs_t currentTimeMs = millis();
    const timeDelta_t timeDeltaMs = currentTimeMs - previousTimeMs;
    previousTimeMs = currentTimeMs;

    if (ARMING_FLAG(ARMED)) {
        posControl.totalTripDistance += posControl.actualState.velXY * MS2S(timeDeltaMs);
    }
}

uint32_t getTotalTravelDistance(void)
{
    return lrintf(posControl.totalTripDistance);
}

/*-----------------------------------------------------------
 * Calculate platform-specific hold position (account for deceleration)
 *-----------------------------------------------------------*/
void calculateInitialHoldPosition(fpVector3_t * pos)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        calculateFixedWingInitialHoldPosition(pos);
    }
    else {
        calculateMulticopterInitialHoldPosition(pos);
    }
}

/*-----------------------------------------------------------
 * Set active XYZ-target and desired heading
 *-----------------------------------------------------------*/
void setDesiredPosition(const fpVector3_t * pos, int32_t yaw, navSetWaypointFlags_t useMask)
{
    // XY-position update is allowed only when not braking in NAV_CRUISE_BRAKING
    if ((useMask & NAV_POS_UPDATE_XY) != 0 && !STATE(NAV_CRUISE_BRAKING)) {
        posControl.desiredState.pos.x = pos->x;
        posControl.desiredState.pos.y = pos->y;
    }

    // Z-position
    if ((useMask & NAV_POS_UPDATE_Z) != 0) {
        updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);   // Reset RoC/RoD -> altitude controller
        posControl.desiredState.pos.z = pos->z;
    }

    // Heading
    if ((useMask & NAV_POS_UPDATE_HEADING) != 0) {
        // Heading
        posControl.desiredState.yaw = yaw;
    }
    else if ((useMask & NAV_POS_UPDATE_BEARING) != 0) {
        posControl.desiredState.yaw = calculateBearingToDestination(pos);
    }
    else if ((useMask & NAV_POS_UPDATE_BEARING_TAIL_FIRST) != 0) {
        posControl.desiredState.yaw = wrap_36000(calculateBearingToDestination(pos) - 18000);
    }
}

void calculateFarAwayTarget(fpVector3_t * farAwayPos, int32_t yaw, int32_t distance)
{
    farAwayPos->x = navGetCurrentActualPositionAndVelocity()->pos.x + distance * cos_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos->y = navGetCurrentActualPositionAndVelocity()->pos.y + distance * sin_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos->z = navGetCurrentActualPositionAndVelocity()->pos.z;
}

void calculateNewCruiseTarget(fpVector3_t * origin, int32_t yaw, int32_t distance)
{
    origin->x = origin->x + distance * cos_approx(CENTIDEGREES_TO_RADIANS(yaw));
    origin->y = origin->y + distance * sin_approx(CENTIDEGREES_TO_RADIANS(yaw));
    origin->z = origin->z;
}

/*-----------------------------------------------------------
 * NAV land detector
 *-----------------------------------------------------------*/
void resetLandingDetector(void)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        resetFixedWingLandingDetector();
    }
    else {
        resetMulticopterLandingDetector();
    }
}

bool isLandingDetected(void)
{
    bool landingDetected;

    if (STATE(FIXED_WING)) { // FIXED_WING
        landingDetected = isFixedWingLandingDetected();
    }
    else {
        landingDetected = isMulticopterLandingDetected();
    }

    return landingDetected;
}

/*-----------------------------------------------------------
 * Z-position controller
 *-----------------------------------------------------------*/
void updateClimbRateToAltitudeController(float desiredClimbRate, climbRateToAltitudeControllerMode_e mode)
{
    static timeUs_t lastUpdateTimeUs;
    timeUs_t currentTimeUs = micros();

    // Terrain following uses different altitude measurement
    const float altitudeToUse = navGetCurrentActualPositionAndVelocity()->pos.z;

    if (mode == ROC_TO_ALT_RESET) {
        lastUpdateTimeUs = currentTimeUs;
        posControl.desiredState.pos.z = altitudeToUse;
    }
    else {
        if (STATE(FIXED_WING)) {
            // Fixed wing climb rate controller is open-loop. We simply move the known altitude target
            float timeDelta = US2S(currentTimeUs - lastUpdateTimeUs);

            DEBUG_SET(DEBUG_FW_CLIMB_RATE_TO_ALTITUDE, 0, desiredClimbRate);
            DEBUG_SET(DEBUG_FW_CLIMB_RATE_TO_ALTITUDE, 1, timeDelta * 1000);

            if (timeDelta <= HZ2S(MIN_POSITION_UPDATE_RATE_HZ)) {
                posControl.desiredState.pos.z += desiredClimbRate * timeDelta;
                posControl.desiredState.pos.z = constrainf(posControl.desiredState.pos.z, altitudeToUse - 500, altitudeToUse + 500);    // FIXME: calculate sanity limits in a smarter way
            }
        }
        else {
            // Multicopter climb-rate control is closed-loop, it's possible to directly calculate desired altitude setpoint to yield the required RoC/RoD
            posControl.desiredState.pos.z = altitudeToUse + (desiredClimbRate / posControl.pids.pos[Z].param.kP);
        }

        lastUpdateTimeUs = currentTimeUs;
    }
}

static void resetAltitudeController(bool useTerrainFollowing)
{
    // Set terrain following flag
    posControl.flags.isTerrainFollowEnabled = useTerrainFollowing;

    if (STATE(FIXED_WING)) {
        resetFixedWingAltitudeController();
    }
    else {
        resetMulticopterAltitudeController();
    }
}

static void setupAltitudeController(void)
{
    if (STATE(FIXED_WING)) {
        setupFixedWingAltitudeController();
    }
    else {
        setupMulticopterAltitudeController();
    }
}

static bool adjustAltitudeFromRCInput(void)
{
    if (STATE(FIXED_WING)) {
        return adjustFixedWingAltitudeFromRCInput();
    }
    else {
        return adjustMulticopterAltitudeFromRCInput();
    }
}

/*-----------------------------------------------------------
 * Heading controller (pass-through to MAG mode)
 *-----------------------------------------------------------*/
static void resetHeadingController(void)
{
    if (STATE(FIXED_WING)) {
        resetFixedWingHeadingController();
    }
    else {
        resetMulticopterHeadingController();
    }
}

static bool adjustHeadingFromRCInput(void)
{
    if (STATE(FIXED_WING)) {
        return adjustFixedWingHeadingFromRCInput();
    }
    else {
        return adjustMulticopterHeadingFromRCInput();
    }
}

/*-----------------------------------------------------------
 * XY Position controller
 *-----------------------------------------------------------*/
static void resetPositionController(void)
{
    if (STATE(FIXED_WING)) {
        resetFixedWingPositionController();
    }
    else {
        resetMulticopterPositionController();
        resetMulticopterBrakingMode();
    }
}

static bool adjustPositionFromRCInput(void)
{
    bool retValue;

    if (STATE(FIXED_WING)) {
        retValue = adjustFixedWingPositionFromRCInput();
    }
    else {

        const float rcPitchAdjustment = rcControlGetInputAxisApplyingPosholdDeadband(PITCH);
        const float rcRollAdjustment = rcControlGetInputAxisApplyingPosholdDeadband(ROLL);

        retValue = adjustMulticopterPositionFromRCInput(rcPitchAdjustment, rcRollAdjustment);
    }

    return retValue;
}

/*-----------------------------------------------------------
 * WP controller
 *-----------------------------------------------------------*/
void resetGCSFlags(void)
{
    posControl.flags.isGCSAssistedNavigationReset = false;
    posControl.flags.isGCSAssistedNavigationEnabled = false;
}

void getWaypoint(uint8_t wpNumber, navWaypoint_t * wpData)
{
    /* Default waypoint to send */
    wpData->action = NAV_WP_ACTION_RTH;
    wpData->lat = 0;
    wpData->lon = 0;
    wpData->alt = 0;
    wpData->p1 = 0;
    wpData->p2 = 0;
    wpData->p3 = 0;
    wpData->flag = NAV_WP_FLAG_LAST;

    // WP #0 - special waypoint - HOME
    if (wpNumber == 0) {
        if (STATE(GPS_FIX_HOME)) {
            wpData->lat = GPS_home.lat;
            wpData->lon = GPS_home.lon;
            wpData->alt = GPS_home.alt;
        }
    }
    // WP #255 - special waypoint - directly get actualPosition
    else if (wpNumber == 255) {
        gpsLocation_t wpLLH;

        geoConvertLocalToGeodetic(&wpLLH, &posControl.gpsOrigin, &navGetCurrentActualPositionAndVelocity()->pos);

        wpData->lat = wpLLH.lat;
        wpData->lon = wpLLH.lon;
        wpData->alt = wpLLH.alt;
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS)) {
        if (wpNumber <= posControl.waypointCount) {
            *wpData = posControl.waypointList[wpNumber - 1];
        }
    }
}

void setWaypoint(uint8_t wpNumber, const navWaypoint_t * wpData)
{
    gpsLocation_t wpLLH;
    navWaypointPosition_t wpPos;

    // Pre-fill structure to convert to local coordinates
    wpLLH.lat = wpData->lat;
    wpLLH.lon = wpData->lon;
    wpLLH.alt = wpData->alt;

    // WP #0 - special waypoint - HOME
    if ((wpNumber == 0) && ARMING_FLAG(ARMED) && (posControl.flags.estPosStatus >= EST_USABLE) && posControl.gpsOrigin.valid && posControl.flags.isGCSAssistedNavigationEnabled) {
        // Forcibly set home position. Note that this is only valid if already armed, otherwise home will be reset instantly
        geoConvertGeodeticToLocal(&wpPos.pos, &posControl.gpsOrigin, &wpLLH, GEO_ALT_RELATIVE);
        setHomePosition(&wpPos.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING, NAV_HOME_VALID_ALL);
    }
    // WP #255 - special waypoint - directly set desiredPosition
    // Only valid when armed and in poshold mode
    else if ((wpNumber == 255) && (wpData->action == NAV_WP_ACTION_WAYPOINT) &&
             ARMING_FLAG(ARMED) && (posControl.flags.estPosStatus == EST_TRUSTED) && posControl.gpsOrigin.valid && posControl.flags.isGCSAssistedNavigationEnabled &&
             (posControl.navState == NAV_STATE_POSHOLD_3D_IN_PROGRESS)) {
        // Convert to local coordinates
        geoConvertGeodeticToLocal(&wpPos.pos, &posControl.gpsOrigin, &wpLLH, GEO_ALT_RELATIVE);

        navSetWaypointFlags_t waypointUpdateFlags = NAV_POS_UPDATE_XY;

        // If we received global altitude == 0, use current altitude
        if (wpData->alt != 0) {
            waypointUpdateFlags |= NAV_POS_UPDATE_Z;
        }

        if (wpData->p1 > 0 && wpData->p1 < 360) {
            waypointUpdateFlags |= NAV_POS_UPDATE_HEADING;
        }

        setDesiredPosition(&wpPos.pos, DEGREES_TO_CENTIDEGREES(wpData->p1), waypointUpdateFlags);
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS) && !ARMING_FLAG(ARMED)) {
        if (wpData->action == NAV_WP_ACTION_WAYPOINT || wpData->action == NAV_WP_ACTION_RTH) {
            // Only allow upload next waypoint (continue upload mission) or first waypoint (new mission)
            if (wpNumber == (posControl.waypointCount + 1) || wpNumber == 1) {
                posControl.waypointList[wpNumber - 1] = *wpData;
                posControl.waypointCount = wpNumber;
                posControl.waypointListValid = (wpData->flag == NAV_WP_FLAG_LAST);
            }
        }
    }
}

void resetWaypointList(void)
{
    /* Can only reset waypoint list if not armed */
    if (!ARMING_FLAG(ARMED)) {
        posControl.waypointCount = 0;
        posControl.waypointListValid = false;
    }
}

bool isWaypointListValid(void)
{
    return posControl.waypointListValid;
}

int getWaypointCount(void)
{
    return posControl.waypointCount;
}

#ifdef NAV_NON_VOLATILE_WAYPOINT_STORAGE
bool loadNonVolatileWaypointList(void)
{
    if (ARMING_FLAG(ARMED))
        return false;

    resetWaypointList();

    for (int i = 0; i < NAV_MAX_WAYPOINTS; i++) {
        // Load waypoint
        setWaypoint(i + 1, nonVolatileWaypointList(i));

        // Check if this is the last waypoint
        if (nonVolatileWaypointList(i)->flag == NAV_WP_FLAG_LAST)
            break;
    }

    // Mission sanity check failed - reset the list
    if (!posControl.waypointListValid) {
        resetWaypointList();
    }

    return posControl.waypointListValid;
}

bool saveNonVolatileWaypointList(void)
{
    if (ARMING_FLAG(ARMED) || !posControl.waypointListValid)
        return false;

    for (int i = 0; i < NAV_MAX_WAYPOINTS; i++) {
        getWaypoint(i + 1, nonVolatileWaypointListMutable(i));
    }

    saveConfigAndNotify();

    return true;
}
#endif

static void mapWaypointToLocalPosition(fpVector3_t * localPos, const navWaypoint_t * waypoint)
{
    gpsLocation_t wpLLH;

    wpLLH.lat = waypoint->lat;
    wpLLH.lon = waypoint->lon;
    wpLLH.alt = waypoint->alt;

    geoConvertGeodeticToLocal(localPos, &posControl.gpsOrigin, &wpLLH, GEO_ALT_RELATIVE);
}

static void calculateAndSetActiveWaypointToLocalPosition(const fpVector3_t * pos)
{
    posControl.activeWaypoint.pos = *pos;

    // Calculate initial bearing towards waypoint and store it in waypoint yaw parameter (this will further be used to detect missed waypoints)
    posControl.activeWaypoint.yaw = calculateBearingToDestination(pos);

    // Set desired position to next waypoint (XYZ-controller)
    setDesiredPosition(&posControl.activeWaypoint.pos, posControl.activeWaypoint.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
}

static void calculateAndSetActiveWaypoint(const navWaypoint_t * waypoint)
{
    fpVector3_t localPos;
    mapWaypointToLocalPosition(&localPos, waypoint);
    calculateAndSetActiveWaypointToLocalPosition(&localPos);
}

/**
 * Returns TRUE if we are in WP mode and executing last waypoint on the list, or in RTH mode, or in PH mode
 *  In RTH mode our only and last waypoint is home
 *  In PH mode our waypoint is hold position */
bool isApproachingLastWaypoint(void)
{
    if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
        if (posControl.waypointCount == 0) {
            /* No waypoints */
            return true;
        }
        else if ((posControl.activeWaypointIndex == (posControl.waypointCount - 1)) ||
                 (posControl.waypointList[posControl.activeWaypointIndex].flag == NAV_WP_FLAG_LAST)) {
            return true;
        }
        else {
            return false;
        }
    }
    else if (navGetStateFlags(posControl.navState) & NAV_CTL_POS) {
        // If POS controller is active we are in Poshold or RTH mode - assume last waypoint
        return true;
    }
    else {
        return false;
    }
}

float getActiveWaypointSpeed(void)
{
    if (posControl.flags.isAdjustingPosition) {
        // In manual control mode use different cap for speed
        return navConfig()->general.max_manual_speed;
    }
    else {
        uint16_t waypointSpeed = navConfig()->general.max_auto_speed;

        if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
            if (posControl.waypointCount > 0 && posControl.waypointList[posControl.activeWaypointIndex].action == NAV_WP_ACTION_WAYPOINT) {
                const float wpSpecificSpeed = posControl.waypointList[posControl.activeWaypointIndex].p1;
                if (wpSpecificSpeed >= 50.0f && wpSpecificSpeed <= navConfig()->general.max_auto_speed) {
                    waypointSpeed = wpSpecificSpeed;
                }
            }
        }

        return waypointSpeed;
    }
}

/*-----------------------------------------------------------
 * Process adjustments to alt, pos and yaw controllers
 *-----------------------------------------------------------*/
static void processNavigationRCAdjustments(void)
{
    /* Process pilot's RC input. Disable all pilot's input when in FAILSAFE_MODE */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if ((navStateFlags & NAV_RC_ALT) && (!FLIGHT_MODE(FAILSAFE_MODE))) {
        posControl.flags.isAdjustingAltitude = adjustAltitudeFromRCInput();
    }
    else {
        posControl.flags.isAdjustingAltitude = false;
    }

    if (navStateFlags & NAV_RC_POS) {
        if (!FLIGHT_MODE(FAILSAFE_MODE)) {
            posControl.flags.isAdjustingPosition = adjustPositionFromRCInput();
        }
        else {
            if (!STATE(FIXED_WING)) {
                resetMulticopterBrakingMode();
            }
        }
    }
    else {
        posControl.flags.isAdjustingPosition = false;
    }

    if ((navStateFlags & NAV_RC_YAW) && (!FLIGHT_MODE(FAILSAFE_MODE))) {
        posControl.flags.isAdjustingHeading = adjustHeadingFromRCInput();
    }
    else {
        posControl.flags.isAdjustingHeading = false;
    }
}

/*-----------------------------------------------------------
 * A main function to call position controllers at loop rate
 *-----------------------------------------------------------*/
void applyWaypointNavigationAndAltitudeHold(void)
{
    const timeUs_t currentTimeUs = micros();

#if defined(NAV_BLACKBOX)
    navFlags = 0;
    if (posControl.flags.estAltStatus == EST_TRUSTED)       navFlags |= (1 << 0);
    if (posControl.flags.estAglStatus == EST_TRUSTED)       navFlags |= (1 << 1);
    if (posControl.flags.estPosStatus == EST_TRUSTED)       navFlags |= (1 << 2);
    if (posControl.flags.isTerrainFollowEnabled)            navFlags |= (1 << 3);
#if defined(NAV_GPS_GLITCH_DETECTION)
    if (isGPSGlitchDetected())                              navFlags |= (1 << 4);
#endif
    if (posControl.flags.estHeadingStatus == EST_TRUSTED)   navFlags |= (1 << 5);
#endif

    // Reset all navigation requests - NAV controllers will set them if necessary
    DISABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);

    // No navigation when disarmed
    if (!ARMING_FLAG(ARMED)) {
        // If we are disarmed, abort forced RTH
        posControl.flags.forcedRTHActivated = false;
        return;
    }

    /* Reset flags */
    posControl.flags.horizontalPositionDataConsumed = 0;
    posControl.flags.verticalPositionDataConsumed = 0;

    /* Process controllers */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (STATE(FIXED_WING)) {
        applyFixedWingNavigationController(navStateFlags, currentTimeUs);
    }
    else {
        applyMulticopterNavigationController(navStateFlags, currentTimeUs);
    }

    /* Consume position data */
    if (posControl.flags.horizontalPositionDataConsumed)
        posControl.flags.horizontalPositionDataNew = 0;

    if (posControl.flags.verticalPositionDataConsumed)
        posControl.flags.verticalPositionDataNew = 0;


#if defined(NAV_BLACKBOX)
    if (posControl.flags.isAdjustingPosition)       navFlags |= (1 << 6);
    if (posControl.flags.isAdjustingAltitude)       navFlags |= (1 << 7);
    if (posControl.flags.isAdjustingHeading)        navFlags |= (1 << 8);

    navTargetPosition[X] = lrintf(posControl.desiredState.pos.x);
    navTargetPosition[Y] = lrintf(posControl.desiredState.pos.y);
    navTargetPosition[Z] = lrintf(posControl.desiredState.pos.z);
#endif
}

/*-----------------------------------------------------------
 * Set CF's FLIGHT_MODE from current NAV_MODE
 *-----------------------------------------------------------*/
void switchNavigationFlightModes(void)
{
    const flightModeFlags_e enabledNavFlightModes = navGetMappedFlightModes(posControl.navState);
    const flightModeFlags_e disabledFlightModes = (NAV_ALTHOLD_MODE | NAV_RTH_MODE | NAV_POSHOLD_MODE | NAV_WP_MODE | NAV_LAUNCH_MODE | NAV_CRUISE_MODE) & (~enabledNavFlightModes);
    DISABLE_FLIGHT_MODE(disabledFlightModes);
    ENABLE_FLIGHT_MODE(enabledNavFlightModes);
}

/*-----------------------------------------------------------
 * desired NAV_MODE from combination of FLIGHT_MODE flags
 *-----------------------------------------------------------*/
static bool canActivateAltHoldMode(void)
{
    return (posControl.flags.estAltStatus >= EST_USABLE);
}

static bool canActivatePosHoldMode(void)
{
    return (posControl.flags.estPosStatus >= EST_USABLE) && (posControl.flags.estVelStatus == EST_TRUSTED) && (posControl.flags.estHeadingStatus >= EST_USABLE);
}

static bool canActivateNavigationModes(void)
{
    return (posControl.flags.estPosStatus == EST_TRUSTED) && (posControl.flags.estVelStatus == EST_TRUSTED) && (posControl.flags.estHeadingStatus >= EST_USABLE);
}

static navigationFSMEvent_t selectNavEventFromBoxModeInput(void)
{
    static bool canActivateWaypoint = false;
    static bool canActivateLaunchMode = false;

    //We can switch modes only when ARMED
    if (ARMING_FLAG(ARMED)) {
        // Ask failsafe system if we can use navigation system
        if (failsafeBypassNavigation()) {
            return NAV_FSM_EVENT_SWITCH_TO_IDLE;
        }

        // Flags if we can activate certain nav modes (check if we have required sensors and they provide valid data)
        bool canActivateAltHold    = canActivateAltHoldMode();
        bool canActivatePosHold    = canActivatePosHoldMode();
        bool canActivateNavigation = canActivateNavigationModes();

        // LAUNCH mode has priority over any other NAV mode
        if (STATE(FIXED_WING)) {
            if (isNavLaunchEnabled()) {     // FIXME: Only available for fixed wing aircrafts now
                if (canActivateLaunchMode) {
                    canActivateLaunchMode = false;
                    return NAV_FSM_EVENT_SWITCH_TO_LAUNCH;
                }
                else if FLIGHT_MODE(NAV_LAUNCH_MODE) {
                    // Make sure we don't bail out to IDLE
                    return NAV_FSM_EVENT_NONE;
                }
            }
            else {
                // If we were in LAUNCH mode - force switch to IDLE only if the throttle is low
                if (FLIGHT_MODE(NAV_LAUNCH_MODE)) {
                    throttleStatus_e throttleStatus = calculateThrottleStatus();
                    if (throttleStatus != THROTTLE_LOW)
                        return NAV_FSM_EVENT_NONE;
                    else
                        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
                }
            }
        }

        // RTH/Failsafe_RTH can override MANUAL
        if (posControl.flags.forcedRTHActivated || (IS_RC_MODE_ACTIVE(BOXNAVRTH) && canActivatePosHold && canActivateNavigation && canActivateAltHold && STATE(GPS_FIX_HOME))) {
            // If we request forced RTH - attempt to activate it no matter what
            // This might switch to emergency landing controller if GPS is unavailable
            canActivateWaypoint = false;    // Block WP mode if we switched to RTH for whatever reason
            return NAV_FSM_EVENT_SWITCH_TO_RTH;
        }

        // MANUAL mode has priority over WP/PH/AH
        if (IS_RC_MODE_ACTIVE(BOXMANUAL)) {
            canActivateWaypoint = false;    // Block WP mode if we are in PASSTHROUGH mode
            return NAV_FSM_EVENT_SWITCH_TO_IDLE;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVWP)) {
            if ((FLIGHT_MODE(NAV_WP_MODE)) || (canActivateNavigation && canActivateWaypoint && canActivatePosHold && canActivateAltHold && STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED) && posControl.waypointListValid && (posControl.waypointCount > 0)))
                return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT;
        }
        else {
            // Arm the state variable if the WP BOX mode is not enabled
            canActivateWaypoint = true;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD)) {
            if (FLIGHT_MODE(NAV_POSHOLD_MODE) || (canActivatePosHold && canActivateAltHold))
                return NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D;
        }

        // PH has priority over CRUISE
        // CRUISE has priority on AH
        if (IS_RC_MODE_ACTIVE(BOXNAVCRUISE)) {

            if (IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) && ((FLIGHT_MODE(NAV_CRUISE_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE)) || (canActivatePosHold && canActivateAltHold)))
                return NAV_FSM_EVENT_SWITCH_TO_CRUISE_3D;

            if (FLIGHT_MODE(NAV_CRUISE_MODE) || (canActivatePosHold))
                return NAV_FSM_EVENT_SWITCH_TO_CRUISE_2D;

        }

        if (IS_RC_MODE_ACTIVE(BOXNAVALTHOLD)) {
            if ((FLIGHT_MODE(NAV_ALTHOLD_MODE)) || (canActivateAltHold))
                return NAV_FSM_EVENT_SWITCH_TO_ALTHOLD;
        }
    }
    else {
        canActivateWaypoint = false;

        // Launch mode can be activated if feature FW_LAUNCH is enabled or BOX is turned on prior to arming (avoid switching to LAUNCH in flight)
        canActivateLaunchMode = isNavLaunchEnabled();
    }

    return NAV_FSM_EVENT_SWITCH_TO_IDLE;
}

/*-----------------------------------------------------------
 * An indicator that throttle tilt compensation is forced
 *-----------------------------------------------------------*/
bool navigationRequiresThrottleTiltCompensation(void)
{
    return !STATE(FIXED_WING) && (navGetStateFlags(posControl.navState) & NAV_REQUIRE_THRTILT);
}

/*-----------------------------------------------------------
 * An indicator that ANGLE mode must be forced per NAV requirement
 *-----------------------------------------------------------*/
bool navigationRequiresAngleMode(void)
{
    const navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    return (currentState & NAV_REQUIRE_ANGLE) || ((currentState & NAV_REQUIRE_ANGLE_FW) && STATE(FIXED_WING));
}

/*-----------------------------------------------------------
 * An indicator that TURN ASSISTANCE is required for navigation
 *-----------------------------------------------------------*/
bool navigationRequiresTurnAssistance(void)
{
    const navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    if (STATE(FIXED_WING)) {
        // For airplanes turn assistant is always required when controlling position
        return (currentState & (NAV_CTL_POS | NAV_CTL_ALT));
    }
    else {
        return false;
    }
}

/**
 * An indicator that NAV is in charge of heading control (a signal to disable other heading controllers)
 */
int8_t navigationGetHeadingControlState(void)
{
    // For airplanes report as manual heading control
    if (STATE(FIXED_WING)) {
        return NAV_HEADING_CONTROL_MANUAL;
    }

    // For multirotors it depends on navigation system mode
    if (navGetStateFlags(posControl.navState) & NAV_REQUIRE_MAGHOLD) {
        if (posControl.flags.isAdjustingHeading) {
            return NAV_HEADING_CONTROL_MANUAL;
        }
        else {
            return NAV_HEADING_CONTROL_AUTO;
        }
    }
    else {
        return NAV_HEADING_CONTROL_NONE;
    }
}

bool navigationTerrainFollowingEnabled(void)
{
    return posControl.flags.isTerrainFollowEnabled;
}

navArmingBlocker_e navigationIsBlockingArming(bool *usedBypass)
{
    const bool navBoxModesEnabled = IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP) || IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) || (STATE(FIXED_WING) && IS_RC_MODE_ACTIVE(BOXNAVALTHOLD)) || (STATE(FIXED_WING) && IS_RC_MODE_ACTIVE(BOXNAVCRUISE));
    const bool navLaunchComboModesEnabled = isNavLaunchEnabled() && (IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP) || IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) || IS_RC_MODE_ACTIVE(BOXNAVCRUISE));

    if (usedBypass) {
        *usedBypass = false;
    }

    if (navConfig()->general.flags.extra_arming_safety == NAV_EXTRA_ARMING_SAFETY_OFF) {
        return NAV_ARMING_BLOCKER_NONE;
    }

    // Apply extra arming safety only if pilot has any of GPS modes configured
    if ((isUsingNavigationModes() || failsafeMayRequireNavigationMode()) && !((posControl.flags.estPosStatus >= EST_USABLE) && STATE(GPS_FIX_HOME))) {
        if (navConfig()->general.flags.extra_arming_safety == NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS &&
            (STATE(NAV_EXTRA_ARMING_SAFETY_BYPASSED) || rxGetChannelValue(YAW) > 1750)) {
            if (usedBypass) {
                *usedBypass = true;
            }
            return NAV_ARMING_BLOCKER_NONE;
        }
        return NAV_ARMING_BLOCKER_MISSING_GPS_FIX;
    }

    // Don't allow arming if any of NAV modes is active
    if (!ARMING_FLAG(ARMED) && navBoxModesEnabled && !navLaunchComboModesEnabled) {
        return NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE;
    }

    // Don't allow arming if first waypoint is farther than configured safe distance
    if (posControl.waypointCount > 0) {
        fpVector3_t startingWaypointPos;
        mapWaypointToLocalPosition(&startingWaypointPos, &posControl.waypointList[0]);

        const bool navWpMissionStartTooFar = calculateDistanceToDestination(&startingWaypointPos) > navConfig()->general.waypoint_safe_distance;

        if (navWpMissionStartTooFar) {
            return NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR;
        }
    }

    return NAV_ARMING_BLOCKER_NONE;
}

bool navigationPositionEstimateIsHealthy(void)
{
    return (posControl.flags.estPosStatus >= EST_USABLE) && STATE(GPS_FIX_HOME);
}

/**
 * Indicate ready/not ready status
 */
static void updateReadyStatus(void)
{
    static bool posReadyBeepDone = false;

    /* Beep out READY_BEEP once when position lock is firstly acquired and HOME set */
    if (navigationPositionEstimateIsHealthy() && !posReadyBeepDone) {
        beeper(BEEPER_READY_BEEP);
        posReadyBeepDone = true;
    }
}

void updateFlightBehaviorModifiers(void)
{
    if (posControl.flags.isGCSAssistedNavigationEnabled && !IS_RC_MODE_ACTIVE(BOXGCSNAV)) {
        posControl.flags.isGCSAssistedNavigationReset = true;
    }

    posControl.flags.isGCSAssistedNavigationEnabled = IS_RC_MODE_ACTIVE(BOXGCSNAV);
}

/**
 * Process NAV mode transition and WP/RTH state machine
 *  Update rate: RX (data driven or 50Hz)
 */
void updateWaypointsAndNavigationMode(void)
{
    /* Initiate home position update */
    updateHomePosition();

    /* Update flight statistics */
    updateNavigationFlightStatistics();

    /* Update NAV ready status */
    updateReadyStatus();

    // Update flight behaviour modifiers
    updateFlightBehaviorModifiers();

    // Process switch to a different navigation mode (if needed)
    navProcessFSMEvents(selectNavEventFromBoxModeInput());

    // Process pilot's RC input to adjust behaviour
    processNavigationRCAdjustments();

    // Map navMode back to enabled flight modes
    switchNavigationFlightModes();

#if defined(NAV_BLACKBOX)
    navCurrentState = (int16_t)posControl.navPersistentId;
#endif
}

/*-----------------------------------------------------------
 * NAV main control functions
 *-----------------------------------------------------------*/
void navigationUsePIDs(void)
{
    /** Multicopter PIDs */
    // Brake time parameter
    posControl.posDecelerationTime = (float)navConfig()->mc.posDecelerationTime / 100.0f;

    // Position controller expo (taret vel expo for MC)
    posControl.posResponseExpo = constrainf((float)navConfig()->mc.posResponseExpo / 100.0f, 0.0f, 1.0f);

    // Initialize position hold P-controller
    for (int axis = 0; axis < 2; axis++) {
        navPidInit(
            &posControl.pids.pos[axis], 
            (float)pidProfile()->bank_mc.pid[PID_POS_XY].P / 100.0f,
            0.0f,
            0.0f,
            0.0f,
            NAV_DTERM_CUT_HZ
        );

        navPidInit(&posControl.pids.vel[axis], (float)pidProfile()->bank_mc.pid[PID_VEL_XY].P / 20.0f,
                                               (float)pidProfile()->bank_mc.pid[PID_VEL_XY].I / 100.0f,
                                               (float)pidProfile()->bank_mc.pid[PID_VEL_XY].D / 100.0f,
                                               (float)pidProfile()->bank_mc.pid[PID_VEL_XY].FF / 100.0f,
                                               pidProfile()->navVelXyDTermLpfHz
        );
    }

    // Initialize altitude hold PID-controllers (pos_z, vel_z, acc_z
    navPidInit(
        &posControl.pids.pos[Z], 
        (float)pidProfile()->bank_mc.pid[PID_POS_Z].P / 100.0f,
        0.0f,
        0.0f,
        0.0f,
        NAV_DTERM_CUT_HZ
    );

    navPidInit(&posControl.pids.vel[Z], (float)pidProfile()->bank_mc.pid[PID_VEL_Z].P / 66.7f,
                                        (float)pidProfile()->bank_mc.pid[PID_VEL_Z].I / 20.0f,
                                        (float)pidProfile()->bank_mc.pid[PID_VEL_Z].D / 100.0f,
                                        0.0f,
                                        NAV_DTERM_CUT_HZ
    );

    // Initialize surface tracking PID
    navPidInit(&posControl.pids.surface, 2.0f,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         NAV_DTERM_CUT_HZ
    );

    /** Airplane PIDs */
    // Initialize fixed wing PID controllers
    navPidInit(&posControl.pids.fw_nav, (float)pidProfile()->bank_fw.pid[PID_POS_XY].P / 100.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_XY].I / 100.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_XY].D / 100.0f,
                                        0.0f,
                                        NAV_DTERM_CUT_HZ
    );

    navPidInit(&posControl.pids.fw_alt, (float)pidProfile()->bank_fw.pid[PID_POS_Z].P / 10.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_Z].I / 10.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_Z].D / 10.0f,
                                        0.0f,
                                        NAV_DTERM_CUT_HZ
    );
}

void navigationInit(void)
{
    /* Initial state */
    posControl.navState = NAV_STATE_IDLE;

    posControl.flags.horizontalPositionDataNew = 0;
    posControl.flags.verticalPositionDataNew = 0;
    posControl.flags.headingDataNew = 0;

    posControl.flags.estAltStatus = EST_NONE;
    posControl.flags.estPosStatus = EST_NONE;
    posControl.flags.estVelStatus = EST_NONE;
    posControl.flags.estHeadingStatus = EST_NONE;
    posControl.flags.estAglStatus = EST_NONE;

    posControl.flags.forcedRTHActivated = 0;
    posControl.waypointCount = 0;
    posControl.activeWaypointIndex = 0;
    posControl.waypointListValid = false;

    /* Set initial surface invalid */
    posControl.actualState.surfaceMin = -1.0f;

    /* Reset statistics */
    posControl.totalTripDistance = 0.0f;

    /* Use system config */
    navigationUsePIDs();
}

/*-----------------------------------------------------------
 * Access to estimated position/velocity data
 *-----------------------------------------------------------*/
float getEstimatedActualVelocity(int axis)
{
    return navGetCurrentActualPositionAndVelocity()->vel.v[axis];
}

float getEstimatedActualPosition(int axis)
{
    return navGetCurrentActualPositionAndVelocity()->pos.v[axis];
}

/*-----------------------------------------------------------
 * Ability to execute RTH on external event
 *-----------------------------------------------------------*/
void activateForcedRTH(void)
{
    abortFixedWingLaunch();
    posControl.flags.forcedRTHActivated = true;
    navProcessFSMEvents(selectNavEventFromBoxModeInput());
}

void abortForcedRTH(void)
{
    // Disable failsafe RTH and make sure we back out of navigation mode to IDLE
    // If any navigation mode was active prior to RTH it will be re-enabled with next RX update
    posControl.flags.forcedRTHActivated = false;
    navProcessFSMEvents(NAV_FSM_EVENT_SWITCH_TO_IDLE);
}

rthState_e getStateOfForcedRTH(void)
{
    /* If forced RTH activated and in AUTO_RTH or EMERG state */
    if (posControl.flags.forcedRTHActivated && (navGetStateFlags(posControl.navState) & (NAV_AUTO_RTH | NAV_CTL_EMERG))) {
        if (posControl.navState == NAV_STATE_RTH_FINISHED || posControl.navState == NAV_STATE_EMERGENCY_LANDING_FINISHED) {
            return RTH_HAS_LANDED;
        }
        else {
            return RTH_IN_PROGRESS;
        }
    }
    else {
        return RTH_IDLE;
    }
}

bool navigationIsExecutingAnEmergencyLanding(void)
{
    return navGetCurrentStateFlags() & NAV_CTL_EMERG;
}

bool navigationIsControllingThrottle(void)
{
    navigationFSMStateFlags_t stateFlags = navGetCurrentStateFlags();
    return ((stateFlags & (NAV_CTL_ALT | NAV_CTL_EMERG | NAV_CTL_LAUNCH | NAV_CTL_LAND)) && (getMotorStatus() != MOTOR_STOPPED_USER));
}

bool navigationIsFlyingAutonomousMode(void)
{
    navigationFSMStateFlags_t stateFlags = navGetCurrentStateFlags();
    return (stateFlags & (NAV_AUTO_RTH | NAV_AUTO_WP));
}

bool navigationRTHAllowsLanding(void)
{
    navRTHAllowLanding_e allow = navConfig()->general.flags.rth_allow_landing;
    return allow == NAV_RTH_ALLOW_LANDING_ALWAYS ||
        (allow == NAV_RTH_ALLOW_LANDING_FS_ONLY && FLIGHT_MODE(FAILSAFE_MODE));
}

bool FAST_CODE isNavLaunchEnabled(void)
{
    return IS_RC_MODE_ACTIVE(BOXNAVLAUNCH) || feature(FEATURE_FW_LAUNCH);
}

int32_t navigationGetHomeHeading(void)
{
    return posControl.rthState.homePosition.yaw;
}

// returns m/s
float calculateAverageSpeed() {
    float flightTime = getFlightTime();
    if (flightTime == 0.0f) return 0;
    return (float)getTotalTravelDistance() / (flightTime * 100);
}

const navigationPIDControllers_t* getNavigationPIDControllers(void) {
    return &posControl.pids;
}

bool isAdjustingPosition(void) {
    return posControl.flags.isAdjustingPosition;
}

bool isAdjustingHeading(void) {
    return posControl.flags.isAdjustingHeading;
}

int32_t getCruiseHeadingAdjustment(void) {
    return wrap_18000(posControl.cruise.yaw - posControl.cruise.previousYaw);
}

#else // NAV

#ifdef USE_GPS
/* Fallback if navigation is not compiled in - handle GPS home coordinates */
static float GPS_scaleLonDown;
static float GPS_totalTravelDistance = 0;

static void GPS_distance_cm_bearing(int32_t currentLat1, int32_t currentLon1, int32_t destinationLat2, int32_t destinationLon2, uint32_t *dist, int32_t *bearing)
{
    const float dLat = destinationLat2 - currentLat1; // difference of latitude in 1/10 000 000 degrees
    const float dLon = (float)(destinationLon2 - currentLon1) * GPS_scaleLonDown;

    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;

    *bearing = 9000.0f + RADIANS_TO_CENTIDEGREES(atan2_approx(-dLat, dLon));      // Convert the output radians to 100xdeg

    if (*bearing < 0)
        *bearing += 36000;
}

void onNewGPSData(void)
{
    static timeMs_t previousTimeMs = 0;
    const timeMs_t currentTimeMs = millis();
    const timeDelta_t timeDeltaMs = currentTimeMs - previousTimeMs;
    previousTimeMs = currentTimeMs;

    if (!(sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5))
        return;

    if (ARMING_FLAG(ARMED)) {
        /* Update home distance and direction */
        if (STATE(GPS_FIX_HOME)) {
            uint32_t dist;
            int32_t dir;
            GPS_distance_cm_bearing(gpsSol.llh.lat, gpsSol.llh.lon, GPS_home.lat, GPS_home.lon, &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = lrintf(dir / 100.0f);
        } else {
            GPS_distanceToHome = 0;
            GPS_directionToHome = 0;
        }

        /* Update trip distance */
        GPS_totalTravelDistance += gpsSol.groundSpeed * MS2S(timeDeltaMs);
    }
    else {
        // Set home position to current GPS coordinates
        ENABLE_STATE(GPS_FIX_HOME);
        GPS_home.lat = gpsSol.llh.lat;
        GPS_home.lon = gpsSol.llh.lon;
        GPS_home.alt = gpsSol.llh.alt;
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
        GPS_scaleLonDown = cos_approx((fabsf((float)gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f);
    }
}

int32_t getTotalTravelDistance(void)
{
    return lrintf(GPS_totalTravelDistance);
}

#endif

#endif  // NAV
