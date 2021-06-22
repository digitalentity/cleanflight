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

#include <stdbool.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "common/fp_pid.h"

#include "config/feature.h"

#include "flight/failsafe.h"

#include "io/gps.h"

/* GPS Home location data */
extern gpsLocation_t        GPS_home;
extern uint32_t             GPS_distanceToHome;        // distance to home point in meters
extern int16_t              GPS_directionToHome;       // direction to home point in degrees
extern fpVector3_t          original_rth_home;         // the original rth home - save it, since it could be replaced by safehome or HOME_RESET

extern bool autoThrottleManuallyIncreased;

/* Navigation system updates */
void onNewGPSData(void);
#if defined(USE_SAFE_HOME)

#define MAX_SAFE_HOMES 8

typedef struct {
    uint8_t enabled;
    int32_t lat;
    int32_t lon;
} navSafeHome_t;

typedef enum {
    SAFEHOME_USAGE_OFF = 0,    // Don't use safehomes
    SAFEHOME_USAGE_RTH = 1,    // Default - use safehome for RTH
    SAFEHOME_USAGE_RTH_FS = 2, // Use safehomes for RX failsafe only
} safehomeUsageMode_e;

PG_DECLARE_ARRAY(navSafeHome_t, MAX_SAFE_HOMES, safeHomeConfig);

extern int8_t safehome_index;                    // -1 if no safehome, 0 to MAX_SAFEHOMES -1 otherwise
extern uint32_t safehome_distance;               // distance to the nearest safehome
extern bool safehome_applied;                    // whether the safehome has been applied to home.

void resetSafeHomes(void);                       // remove all safehomes
bool findNearestSafeHome(void);                  // Find nearest safehome

#endif // defined(USE_SAFE_HOME)

#if defined(USE_NAV)
#if defined(USE_BLACKBOX)
#define NAV_BLACKBOX
#endif

#ifndef NAV_MAX_WAYPOINTS
#define NAV_MAX_WAYPOINTS 15
#endif

#define NAV_ACCEL_CUTOFF_FREQUENCY_HZ 2       // low-pass filter on XY-acceleration target

enum {
    NAV_GPS_ATTI    = 0,                    // Pitch/roll stick controls attitude (pitch/roll lean angles)
    NAV_GPS_CRUISE  = 1                     // Pitch/roll stick controls velocity (forward/right speed)
};

enum {
    NAV_LOITER_RIGHT = 0,                    // Loitering direction right
    NAV_LOITER_LEFT  = 1,                    // Loitering direction left
    NAV_LOITER_YAW   = 2
};

enum {
    NAV_RTH_NO_ALT                       = 0, // Maintain current altitude
    NAV_RTH_EXTRA_ALT                    = 1, // Maintain current altitude + predefined safety margin
    NAV_RTH_CONST_ALT                    = 2, // Climb/descend to predefined altitude
    NAV_RTH_MAX_ALT                      = 3, // Track maximum altitude and climb to it when RTH
    NAV_RTH_AT_LEAST_ALT                 = 4, // Climb to predefined altitude if below it
    NAV_RTH_AT_LEAST_ALT_LINEAR_DESCENT  = 5, // Climb to predefined altitude if below it,
                                              // descend linearly to reach home at predefined altitude if above it
};

enum {
    NAV_HEADING_CONTROL_NONE = 0,
    NAV_HEADING_CONTROL_AUTO,
    NAV_HEADING_CONTROL_MANUAL
};

typedef enum {
    NAV_RESET_NEVER = 0,
    NAV_RESET_ON_FIRST_ARM,
    NAV_RESET_ON_EACH_ARM,
} nav_reset_type_e;

typedef enum {
    NAV_RTH_ALLOW_LANDING_NEVER = 0,
    NAV_RTH_ALLOW_LANDING_ALWAYS = 1,
    NAV_RTH_ALLOW_LANDING_FS_ONLY = 2, // Allow landing only if RTH was triggered by failsafe
} navRTHAllowLanding_e;

typedef enum {
    NAV_EXTRA_ARMING_SAFETY_OFF = 0,
    NAV_EXTRA_ARMING_SAFETY_ON = 1,
    NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS = 2, // Allow disabling by holding THR+YAW low
} navExtraArmingSafety_e;

typedef enum {
    NAV_ARMING_BLOCKER_NONE = 0,
    NAV_ARMING_BLOCKER_MISSING_GPS_FIX = 1,
    NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE = 2,
    NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR = 3,
    NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR = 4,
} navArmingBlocker_e;

typedef enum {
    NOMS_OFF_ALWAYS,
    NOMS_OFF,
    NOMS_AUTO_ONLY,
    NOMS_ALL_NAV
} navOverridesMotorStop_e;

typedef enum {
    OFF,
    ON,
    ON_FW_SPIRAL,
} navRTHClimbFirst_e;

typedef struct positionEstimationConfig_s {
    uint8_t automatic_mag_declination;
    uint8_t reset_altitude_type; // from nav_reset_type_e
    uint8_t reset_home_type; // nav_reset_type_e
    uint8_t gravity_calibration_tolerance;    // Tolerance of gravity calibration (cm/s/s)
    uint8_t use_gps_velned;
    uint8_t allow_dead_reckoning;

    uint16_t max_surface_altitude;

    float w_z_baro_p;   // Weight (cutoff frequency) for barometer altitude measurements

    float w_z_surface_p;  // Weight (cutoff frequency) for surface altitude measurements
    float w_z_surface_v;  // Weight (cutoff frequency) for surface velocity measurements

    float w_z_gps_p;    // GPS altitude data is very noisy and should be used only on airplanes
    float w_z_gps_v;    // Weight (cutoff frequency) for GPS climb rate measurements

    float w_xy_gps_p;   // Weight (cutoff frequency) for GPS position measurements
    float w_xy_gps_v;   // Weight (cutoff frequency) for GPS velocity measurements

    float w_xy_flow_p;
    float w_xy_flow_v;

    float w_z_res_v;    // When velocity sources lost slowly decrease estimated velocity with this weight
    float w_xy_res_v;

    float w_acc_bias;   // Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.
    float w_xyz_acc_p;

    float max_eph_epv;  // Max estimated position error acceptable for estimation (cm)
    float baro_epv;     // Baro position error

    uint8_t use_gps_no_baro;
} positionEstimationConfig_t;

PG_DECLARE(positionEstimationConfig_t, positionEstimationConfig);

typedef struct navConfig_s {

    struct {
        struct {
            uint8_t use_thr_mid_for_althold;    // Don't remember throttle when althold was initiated, assume that throttle is at Thr Mid = zero climb rate
            uint8_t extra_arming_safety;        // from navExtraArmingSafety_e
            uint8_t user_control_mode;          // NAV_GPS_ATTI or NAV_GPS_CRUISE
            uint8_t rth_alt_control_mode;       // Controls the logic for choosing the RTH altitude
            uint8_t rth_climb_first;            // Controls the logic for initial RTH climbout
            uint8_t rth_tail_first;             // Return to home tail first
            uint8_t disarm_on_landing;          //
            uint8_t rth_allow_landing;          // Enable landing as last stage of RTH. Use constants in navRTHAllowLanding_e.
            uint8_t rth_climb_ignore_emerg;     // Option to ignore GPS loss on initial climb stage of RTH
            uint8_t rth_alt_control_override;   // Override RTH Altitude and Climb First settings using Pitch and Roll stick
            uint8_t nav_overrides_motor_stop;   // Autonomous modes override motor_stop setting and user command to stop motor
            uint8_t safehome_usage_mode;        // Controls when safehomes are used
        } flags;

        uint8_t  pos_failure_timeout;           // Time to wait before switching to emergency landing (0 - disable)
        uint16_t waypoint_radius;               // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
        uint16_t waypoint_safe_distance;        // Waypoint mission sanity check distance
        bool     waypoint_load_on_boot;         // load waypoints automatically during boot
        uint16_t max_auto_speed;                // autonomous navigation speed cm/sec
        uint16_t max_auto_climb_rate;           // max vertical speed limitation cm/sec
        uint16_t max_manual_speed;              // manual velocity control max horizontal speed
        uint16_t max_manual_climb_rate;         // manual velocity control max vertical speed
        uint16_t land_minalt_vspd;              // Final RTH landing descent rate under minalt
        uint16_t land_maxalt_vspd;              // RTH landing descent rate target at maxalt
        uint16_t land_slowdown_minalt;          // Altitude to stop lowering descent rate during RTH descend
        uint16_t land_slowdown_maxalt;          // Altitude to start lowering descent rate during RTH descend
        uint16_t emerg_descent_rate;            // emergency landing descent rate
        uint16_t rth_altitude;                  // altitude to maintain when RTH is active (depends on rth_alt_control_mode) (cm)
        uint16_t rth_home_altitude;             // altitude to go to during RTH after the craft reached home (cm)
        uint16_t min_rth_distance;              // 0 Disables. Minimal distance for RTH in cm, otherwise it will just autoland
        uint16_t rth_abort_threshold;           // Initiate emergency landing if during RTH we get this much [cm] away from home
        uint16_t max_terrain_follow_altitude;   // Max altitude to be used in SURFACE TRACKING mode
        uint16_t safehome_max_distance;         // Max distance that a safehome is from the arming point
        uint16_t max_altitude;                  // Max altitude when in AltHold mode (not Surface Following)
    } general;

    struct {
        uint8_t  max_bank_angle;                // multicopter max banking angle (deg)
        uint16_t auto_disarm_delay;             // multicopter safety delay for landing detector

#ifdef USE_MR_BRAKING_MODE
        uint16_t braking_speed_threshold;       // above this speed braking routine might kick in
        uint16_t braking_disengage_speed;       // below this speed braking will be disengaged
        uint16_t braking_timeout;               // Timeout for braking mode
        uint8_t  braking_boost_factor;          // Acceleration boost multiplier at max speed
        uint16_t braking_boost_timeout;         // Timeout for boost mode
        uint16_t braking_boost_speed_threshold; // Above this speed braking boost mode can engage
        uint16_t braking_boost_disengage_speed; // Below this speed braking boost will disengage
        uint8_t  braking_bank_angle;            // Max angle [deg] that MR is allowed duing braking boost phase
#endif

        uint8_t posDecelerationTime;            // Brake time parameter
        uint8_t posResponseExpo;                // Position controller expo (taret vel expo for MC)
        bool slowDownForTurning;             // Slow down during WP missions when changing heading on next waypoint
    } mc;

    struct {
        uint8_t  max_bank_angle;             // Fixed wing max banking angle (deg)
        uint8_t  max_climb_angle;            // Fixed wing max banking angle (deg)
        uint8_t  max_dive_angle;             // Fixed wing max banking angle (deg)
        uint16_t cruise_speed;               // Speed at cruise throttle (cm/s), used for time/distance left before RTH
        uint8_t control_smoothness;          // The amount of smoothing to apply to controls for navigation
        uint16_t pitch_to_throttle_smooth;    // How smoothly the autopilot makes pitch to throttle correction inside a deadband defined by pitch_to_throttle_thresh.
        uint8_t  pitch_to_throttle_thresh;   // Threshold from average pitch where momentary pitch_to_throttle correction kicks in. [decidegrees]
        uint16_t loiter_radius;              // Loiter radius when executing PH on a fixed wing
        int8_t land_dive_angle;
        uint16_t launch_velocity_thresh;     // Velocity threshold for swing launch detection
        uint16_t launch_accel_thresh;        // Acceleration threshold for launch detection (cm/s/s)
        uint16_t launch_time_thresh;         // Time threshold for launch detection (ms)
        uint16_t launch_motor_timer;         // Time to wait before setting launch_throttle (ms)
        uint16_t launch_idle_motor_timer;    // Time to wait before motor starts at_idle throttle (ms)
        uint16_t launch_motor_spinup_time;   // Time to speed-up motors from idle to launch_throttle (ESC desync prevention)
        uint16_t launch_end_time;            // Time to make the transition from launch angle to leveled and throttle transition from launch throttle to the stick position
        uint16_t launch_min_time;	           // Minimum time in launch mode to prevent possible bump of the sticks from leaving launch mode early
        uint16_t launch_timeout;             // Launch timeout to disable launch mode and swith to normal flight (ms)
        uint16_t launch_max_altitude;        // cm, altitude where to consider launch ended
        uint8_t  launch_climb_angle;         // Target climb angle for launch (deg)
        uint8_t  launch_max_angle;           // Max tilt angle (pitch/roll combined) to consider launch successful. Set to 180 to disable completely [deg]
        bool     launch_allow_throttle_low;  // Allow launch with throttle low
        uint8_t  cruise_yaw_rate;            // Max yaw rate (dps) when CRUISE MODE is enabled
        bool     allow_manual_thr_increase;
        bool    useFwNavYawControl;
        uint8_t yawControlDeadband;
    } fw;
} navConfig_t;

PG_DECLARE(navConfig_t, navConfig);

typedef struct gpsOrigin_s {
    bool    valid;
    float   scale;
    int32_t lat;    // Lattitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsOrigin_t;

typedef enum {
    NAV_WP_ACTION_WAYPOINT  = 0x01,
    NAV_WP_ACTION_HOLD_TIME = 0x03,
    NAV_WP_ACTION_RTH       = 0x04,
    NAV_WP_ACTION_SET_POI   = 0x05,
    NAV_WP_ACTION_JUMP      = 0x06,
    NAV_WP_ACTION_SET_HEAD  = 0x07,
    NAV_WP_ACTION_LAND      = 0x08
} navWaypointActions_e;

typedef enum {
    NAV_WP_HEAD_MODE_NONE  = 0,
    NAV_WP_HEAD_MODE_POI   = 1,
    NAV_WP_HEAD_MODE_FIXED = 2
} navWaypointHeadings_e;

typedef enum {
    NAV_WP_FLAG_HOME = 0x48,
    NAV_WP_FLAG_LAST = 0xA5
} navWaypointFlags_e;

typedef struct {
    uint8_t action;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t p1, p2, p3;
    uint8_t flag;
} navWaypoint_t;

typedef struct {
    navWaypointHeadings_e  mode;
    uint32_t heading; // fixed heading * 100 (SET_HEAD)
    fpVector3_t poi_pos; // POI location in local coordinates (SET_POI)
} navWapointHeading_t;

typedef struct radar_pois_s {
    gpsLocation_t gps;
    uint8_t state;
    uint16_t heading; // °
    uint16_t speed; // cm/s
    uint8_t lq; // from 0 t o 4
    uint16_t distance; // m
    int16_t altitude; // m
    int16_t direction; // °
} radar_pois_t;

#define RADAR_MAX_POIS 5

extern radar_pois_t radar_pois[RADAR_MAX_POIS];

typedef struct {
    fpVector3_t pos;
    int32_t     yaw;             // deg * 100
} navWaypointPosition_t;

typedef struct navDestinationPath_s {
    uint32_t distance; // meters * 100
    int32_t bearing; // deg * 100
} navDestinationPath_t;

typedef struct navigationPIDControllers_s {
    /* Multicopter PIDs */
    pidController_t   pos[XYZ_AXIS_COUNT];
    pidController_t vel[XYZ_AXIS_COUNT];
    pidController_t surface;

    /* Fixed-wing PIDs */
    pidController_t fw_alt;
    pidController_t fw_nav;
    pidController_t fw_heading;
} navigationPIDControllers_t;

/* MultiWii-compatible params for telemetry */
typedef enum {
    MW_GPS_MODE_NONE = 0,
    MW_GPS_MODE_HOLD,
    MW_GPS_MODE_RTH,
    MW_GPS_MODE_NAV,
    MW_GPS_MODE_EMERG = 15
} navSystemStatus_Mode_e;

typedef enum {
    MW_NAV_STATE_NONE = 0,                // None
    MW_NAV_STATE_RTH_START,               // RTH Start
    MW_NAV_STATE_RTH_ENROUTE,             // RTH Enroute
    MW_NAV_STATE_HOLD_INFINIT,            // PosHold infinit
    MW_NAV_STATE_HOLD_TIMED,              // PosHold timed
    MW_NAV_STATE_WP_ENROUTE,              // WP Enroute
    MW_NAV_STATE_PROCESS_NEXT,            // Process next
    MW_NAV_STATE_DO_JUMP,                 // Jump
    MW_NAV_STATE_LAND_START,              // Start Land (unused)
    MW_NAV_STATE_LAND_IN_PROGRESS,        // Land in Progress
    MW_NAV_STATE_LANDED,                  // Landed
    MW_NAV_STATE_LAND_SETTLE,             // Settling before land
    MW_NAV_STATE_LAND_START_DESCENT,      // Start descent
    MW_NAV_STATE_HOVER_ABOVE_HOME,        // Hover/Loitering above home
    MW_NAV_STATE_EMERGENCY_LANDING,       // Emergency landing
    MW_NAV_STATE_RTH_CLIMB,               // RTH Climb safe altitude
} navSystemStatus_State_e;

typedef enum {
    MW_NAV_ERROR_NONE = 0,            //All systems clear
    MW_NAV_ERROR_TOOFAR,              //Next waypoint distance is more than safety distance
    MW_NAV_ERROR_SPOILED_GPS,         //GPS reception is compromised - Nav paused - copter is adrift !
    MW_NAV_ERROR_WP_CRC,              //CRC error reading WP data from EEPROM - Nav stopped
    MW_NAV_ERROR_FINISH,              //End flag detected, navigation finished
    MW_NAV_ERROR_TIMEWAIT,            //Waiting for poshold timer
    MW_NAV_ERROR_INVALID_JUMP,        //Invalid jump target detected, aborting
    MW_NAV_ERROR_INVALID_DATA,        //Invalid mission step action code, aborting, copter is adrift
    MW_NAV_ERROR_WAIT_FOR_RTH_ALT,    //Waiting to reach RTH Altitude
    MW_NAV_ERROR_GPS_FIX_LOST,        //Gps fix lost, aborting mission
    MW_NAV_ERROR_DISARMED,            //NAV engine disabled due disarm
    MW_NAV_ERROR_LANDING              //Landing
} navSystemStatus_Error_e;

typedef enum {
    MW_NAV_FLAG_ADJUSTING_POSITION  = 1 << 0,
    MW_NAV_FLAG_ADJUSTING_ALTITUDE  = 1 << 1,
} navSystemStatus_Flags_e;

typedef struct {
    navSystemStatus_Mode_e  mode;
    navSystemStatus_State_e state;
    navSystemStatus_Error_e error;
    navSystemStatus_Flags_e flags;
    uint8_t                 activeWpNumber;
    navWaypointActions_e    activeWpAction;
} navSystemStatus_t;

void navigationUsePIDs(void);
void navigationInit(void);

/* Position estimator update functions */
void updatePositionEstimator_BaroTopic(timeUs_t currentTimeUs);
void updatePositionEstimator_OpticalFlowTopic(timeUs_t currentTimeUs);
void updatePositionEstimator_SurfaceTopic(timeUs_t currentTimeUs, float newSurfaceAlt);
void updatePositionEstimator_PitotTopic(timeUs_t currentTimeUs);

/* Navigation system updates */
void updateWaypointsAndNavigationMode(void);
void updatePositionEstimator(void);
void applyWaypointNavigationAndAltitudeHold(void);

/* Functions to signal navigation requirements to main loop */
bool navigationRequiresAngleMode(void);
bool navigationRequiresThrottleTiltCompensation(void);
bool navigationRequiresTurnAssistance(void);
int8_t navigationGetHeadingControlState(void);
// Returns wether arming is blocked by the navigation system.
// If usedBypass is provided, it will indicate wether any checks
// were bypassed due to user input.
navArmingBlocker_e navigationIsBlockingArming(bool *usedBypass);
bool navigationPositionEstimateIsHealthy(void);
bool navIsCalibrationComplete(void);
bool navigationTerrainFollowingEnabled(void);

/* Access to estimated position and velocity */
typedef struct {
    uint8_t altStatus;
    uint8_t posStatus;
    uint8_t velStatus;
    uint8_t aglStatus;
    fpVector3_t pos;
    fpVector3_t vel;
    float agl;
} navPositionAndVelocity_t;

float getEstimatedActualVelocity(int axis);
float getEstimatedActualPosition(int axis);
uint32_t getTotalTravelDistance(void);
void getEstimatedPositionAndVelocity(navPositionAndVelocity_t * pos);

/* Waypoint list access functions */
int getWaypointCount(void);
bool isWaypointListValid(void);
void getWaypoint(uint8_t wpNumber, navWaypoint_t * wpData);
void setWaypoint(uint8_t wpNumber, const navWaypoint_t * wpData);
void resetWaypointList(void);
bool loadNonVolatileWaypointList(void);
bool saveNonVolatileWaypointList(void);

float getFinalRTHAltitude(void);
int16_t fixedWingPitchToThrottleCorrection(int16_t pitch, timeUs_t currentTimeUs);

/* Geodetic functions */
typedef enum {
    GEO_ALT_ABSOLUTE,
    GEO_ALT_RELATIVE
} geoAltitudeConversionMode_e;

typedef enum {
    GEO_ORIGIN_SET,
    GEO_ORIGIN_RESET_ALTITUDE
} geoOriginResetMode_e;

typedef enum {
    NAV_WP_TAKEOFF_DATUM,
    NAV_WP_MSL_DATUM
} geoAltitudeDatumFlag_e;

// geoSetOrigin stores the location provided in llh as a GPS origin in the
// provided origin parameter. resetMode indicates wether all origin coordinates
// should be overwritten by llh (GEO_ORIGIN_SET) or just the altitude, leaving
// other fields untouched (GEO_ORIGIN_RESET_ALTITUDE).
void geoSetOrigin(gpsOrigin_t *origin, const gpsLocation_t *llh, geoOriginResetMode_e resetMode);
// geoConvertGeodeticToLocal converts the geodetic location given in llh to
// the local coordinate space and stores the result in pos. The altConv
// indicates wether the altitude in llh is relative to the default GPS
// origin (GEO_ALT_RELATIVE) or absolute (e.g. Earth frame)
// (GEO_ALT_ABSOLUTE). If origin is invalid pos is set to
// (0, 0, 0) and false is returned. It returns true otherwise.
bool geoConvertGeodeticToLocal(fpVector3_t *pos, const gpsOrigin_t *origin, const gpsLocation_t *llh, geoAltitudeConversionMode_e altConv);
// geoConvertGeodeticToLocalOrigin calls geoConvertGeodeticToLocal with the
// default GPS origin.
bool geoConvertGeodeticToLocalOrigin(fpVector3_t * pos, const gpsLocation_t *llh, geoAltitudeConversionMode_e altConv);
// geoConvertLocalToGeodetic converts a local point as provided in pos to
// geodetic coordinates using the provided GPS origin. It returns wether
// the provided origin is valid and the conversion could be performed.
bool geoConvertLocalToGeodetic(gpsLocation_t *llh, const gpsOrigin_t *origin, const fpVector3_t *pos);
float geoCalculateMagDeclination(const gpsLocation_t * llh); // degrees units
// Select absolute or relative altitude based on WP mission flag setting
geoAltitudeConversionMode_e waypointMissionAltConvMode(geoAltitudeDatumFlag_e datumFlag);

/* Distance/bearing calculation */
bool navCalculatePathToDestination(navDestinationPath_t *result, const fpVector3_t * destinationPos);

/* Failsafe-forced RTH mode */
void activateForcedRTH(void);
void abortForcedRTH(void);
rthState_e getStateOfForcedRTH(void);

/* Getter functions which return data about the state of the navigation system */
bool navigationInAutomaticThrottleMode(void);
bool launchAllowedWithThrottleLow(void);
bool navigationIsControllingThrottle(void);
bool isFixedWingAutoThrottleManuallyIncreased(void);
bool navigationIsFlyingAutonomousMode(void);
bool navigationIsExecutingAnEmergencyLanding(void);
bool navigationIsControllingAltitude(void);
/* Returns true iff navConfig()->general.flags.rth_allow_landing is NAV_RTH_ALLOW_LANDING_ALWAYS
 * or if it's NAV_RTH_ALLOW_LANDING_FAILSAFE and failsafe mode is active.
 */
bool navigationRTHAllowsLanding(void);
bool isWaypointMissionRTHActive(void);

bool isNavLaunchEnabled(void);
bool isFixedWingLaunchDetected(void);
bool isFixedWingLaunchFinishedOrAborted(void);
bool isFixedWingLaunchFinishedThrottleLow(void);
const char * fixedWingLaunchStateMessage(void);

float calculateAverageSpeed(void);

const navigationPIDControllers_t* getNavigationPIDControllers(void);

int32_t navigationGetHeadingError(void);
int32_t getCruiseHeadingAdjustment(void);
bool isAdjustingPosition(void);
bool isAdjustingHeading(void);

/* Returns the heading recorded when home position was acquired.
 * Note that the navigation system uses deg*100 as unit and angles
 * are in the [0, 360 * 100) interval.
 */
int32_t navigationGetHomeHeading(void);

/* Compatibility data */
extern navSystemStatus_t    NAV_Status;

extern int16_t navCurrentState;
extern int16_t navActualVelocity[3];
extern int16_t navDesiredVelocity[3];
extern int32_t navTargetPosition[3];
extern int32_t navLatestActualPosition[3];
extern int16_t navActualSurface;
extern uint16_t navFlags;
extern uint16_t navEPH;
extern uint16_t navEPV;
extern int16_t navAccNEU[3];

#else

#define navigationRequiresAngleMode() (0)
#define navigationGetHeadingControlState() (0)
#define navigationRequiresThrottleTiltCompensation() (0)
#define getEstimatedActualVelocity(axis) (0)
#define navigationIsControllingThrottle() (0)
#define navigationRTHAllowsLanding() (0)
#define navigationGetHomeHeading(0)

#endif
