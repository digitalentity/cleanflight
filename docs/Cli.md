# Command Line Interface (CLI)

INAV has a command line interface (CLI) that can be used to change settings and configure the FC.

## Accessing the CLI.

The CLI can be accessed via the GUI tool or via a terminal emulator connected to the CLI serial port.

1. Connect your terminal emulator to the CLI serial port (which, by default, is the same as the MSP serial port)
2. Use the baudrate specified by msp_baudrate (115200 by default).
3. Send a `#` character.

To save your settings type in 'save', saving will reboot the flight controller.

To exit the CLI without saving power off the flight controller or type in 'exit'.

To see a list of other commands type in 'help' and press return.

To dump your configuration (including the current profile), use the 'dump' or 'diff' command.

See the other documentation sections for details of the cli commands and settings that are available.

## Backup via CLI

Disconnect main power, connect to cli via USB/FTDI.

dump using cli

```
profile 0
dump
```

dump profiles using cli if you use them

```
profile 1
dump profile
profile 2
dump profile
```

copy screen output to a file and save it.

Alternatively, use the `diff` command to dump only those settings that differ from their default values (those that have been changed).


## Restore via CLI.

Use the cli `defaults` command first.

When restoring from backup it's a good idea to do a dump of the latest defaults so you know what has changed - if you do this each time a firmware release is created you will be able to see the cli changes between firmware versions. If you blindly restore your backup you would not benefit from these new defaults or may even end up with completely wrong settings in case some parameters changed semantics and/or value ranges.

It may be good idea to restore settings using the `diff` output rather than complete `dump`. This way you can have more control on what is restored and the risk of mistakenly restoring bad values if the semantics changes is minimised.

To perform the restore simply paste the saved commands in the Configurator CLI tab and then type `save`.

After restoring it's always a good idea to `dump` or `diff` the settings once again and compare the output with previous one to verify if everything is set as it should be.


## CLI Command Reference

| `Command`        | Description                                    |
|------------------|------------------------------------------------|
| `1wire <esc>`    | passthrough 1wire to the specified esc         |
| `adjrange`       | show/set adjustment ranges settings            |
| `aux`            | show/set aux settings                          |
| `beeper`         | show/set beeper (buzzer) usage (see docs/Buzzer.md) |
| `mmix`           | design custom motor mixer                      |
| `smix`           | design custom servo mixer                      |
| `color`          | configure colors                               |
| `defaults`       | reset to defaults and reboot                   |
| `dump`           | print configurable settings in a pastable form |
| `diff`           | print only settings that have been modified    |
| `exit`           |                                                |
| `feature`        | list or -val or val                            |
| `get`            | get variable value                             |
| `gpspassthrough` | passthrough gps to serial                      |
| `help`           |                                                |
| `led`            | configure leds                                 |
| `map`            | mapping of rc channel order                    |
| `motor`          | get/set motor output value                     |
| `msc`            | Enter USB Mass storage mode. See docs/USB_Mass_Storage_(MSC)_mode.md for usage information.|
| `play_sound`     | index, or none for next                        |
| `profile`        | index (0 to 2)                                 |
| `rxrange`        | configure rx channel ranges (end-points) |
| `save`           | save and reboot                                |
| `serial`         | Configure serial ports                         |
| `serialpassthrough <id> <baud> <mode>`| where `id` is the zero based port index, `baud` is a standard baud rate, and mode is `rx`, `tx`, or both (`rxtx`) |
| `set`            | name=value or blank or * for list              |
| `status`         | show system status                             |
| `temp_sensor`    | list or configure temperature sensor(s). See docs/Temperature sensors.md |
| `wp`             | list or configure waypoints. See more in docs/Navigation.md section NAV WP |
| `version`        |                                                |

### serial

The syntax of the `serial` command is `serial <id>  <functions> <msp-baudrate> <gps-baudrate> <telemetry-baudate> <peripheral-baudrate>`.

A shorter form is also supported to enable and disable functions using `serial <id> +n` and
`serial <id> -n`, where n is the a serial function identifier. The following values are available:

| Function              | Identifier    |
|-----------------------|---------------|
| MSP                   | 0             |
| GPS                   | 1             |
| TELEMETRY_FRSKY       | 2             |
| TELEMETRY_HOTT        | 3             |
| TELEMETRY_LTM         | 4             |
| TELEMETRY_SMARTPORT   | 5             |
| RX_SERIAL             | 6             |
| BLACKBOX              | 7             |
| TELEMETRY_MAVLINK     | 8             |
| TELEMETRY_IBUS        | 9             |
| RCDEVICE              | 10            |
| VTX_SMARTAUDIO        | 11            |
| VTX_TRAMP             | 12            |
| UAV_INTERCONNECT      | 13            |
| OPTICAL_FLOW          | 14            |
| LOG                   | 15            |
| RANGEFINDER           | 16            |
| VTX_FFPV              | 17            |

`serial` can also be used without any argument to print the current configuration of all the serial ports.

## CLI Variable Reference

|  Variable Name | Default Value | Description |
|  ------ | ------ | ------ |
|  looptime  | 1000 | This is the main loop time (in us). Changing this affects PID effect with some PID controllers (see PID section for details). A very conservative value of 3500us/285Hz should work for everyone. Setting it to zero does not limit loop time, so it will go as fast as possible. |
|  i2c_speed | 400KHZ | This setting controls the clock speed of I2C bus. 400KHZ is the default that most setups are able to use. Some noise-free setups may be overclocked to 800KHZ. Some sensor chips or setups with long wires may work unreliably at 400KHZ - user can try lowering the clock speed to 200KHZ or even 100KHZ. User need to bear in mind that lower clock speeds might require higher looptimes (lower looptime rate) |
|  cpu_underclock  | OFF | This option is only available on certain architectures (F3 CPUs at the moment). It makes CPU clock lower to reduce interference to long-range RC systems working at 433MHz |
|  gyro_sync  | OFF | This option enables gyro_sync feature. In this case the loop will be synced to gyro refresh rate. Loop will always wait for the newest gyro measurement. Maximum gyro refresh rate is determined by gyro_hardware_lpf  |
|  min_check  | 1100 | These are min/max values (in us) which, when a channel is smaller (min) or larger (max) than the value will activate various RC commands, such as arming, or stick configuration. Normally, every RC channel should be set so that min = 1000us, max = 2000us. On most transmitters this usually means 125% endpoints. Default check values are 100us above/below this value. |
|  max_check  | 1900 | These are min/max values (in us) which, when a channel is smaller (min) or larger (max) than the value will activate various RC commands, such as arming, or stick configuration. Normally, every RC channel should be set so that min = 1000us, max = 2000us. On most transmitters this usually means 125% endpoints. Default check values are 100us above/below this value. |
|  rssi_channel  | 0 | RX channel containing the RSSI signal |
|  rssi_min  | 0 | The minimum RSSI value sent by the receiver, in %. For example, if your receiver's minimum RSSI value shows as 42% in the configurator/OSD set this parameter to 42. See also rssi_max. Note that rssi_min can be set to a value bigger than rssi_max to invert the RSSI calculation (i.e. bigger values mean lower RSSI). |
|  rssi_max  | 100 | The maximum RSSI value sent by the receiver, in %. For example, if your receiver's maximum RSSI value shows as 83% in the configurator/OSD set this parameter to 83. See also rssi_min. |
|  rc_filter_frequency  | 50 | RC data biquad filter cutoff frequency. Lower cutoff frequencies result in smoother response at expense of command control delay. Practical values are 20-50. Set to zero to disable entirely and use unsmoothed RC stick values |
|  input_filtering_mode  | OFF | Filter out noise from OpenLRS Telemetry RX |
|  throttle_idle  | 15 |  The percentage of the throttle range (`max_throttle` - `min_command`) above `min_command` used for minimum / idle throttle. |
|  max_throttle  | 1850 | This is the maximum value (in us) sent to esc when armed. Default of 1850 are OK for everyone (legacy). For modern ESCs, higher values (c. 2000) may be more appropriate. If you have brushed motors, the value should be set to 2000. |
|  min_command  | 1000 | This is the PWM value sent to ESCs when they are not armed. If ESCs beep slowly when powered up, try decreasing this value. It can also be used for calibrating all ESCs at once. |
|  3d_deadband_low  | 1406 | Low value of throttle deadband for 3D mode (when stick is in the 3d_deadband_throttle range, the fixed values of 3d_deadband_low / _high are used instead) |
|  3d_deadband_high  | 1514 | High value of throttle deadband for 3D mode (when stick is in the deadband range, the value in 3d_neutral is used instead) |
|  3d_neutral  | 1460 | Neutral (stop) throttle value for 3D mode |
|  3d_deadband_throttle  | 50 | Throttle signal will be held to a fixed value when throttle is centered with an error margin defined in this parameter. |
|  motor_pwm_rate  | 400 | Output frequency (in Hz) for motor pins. Default is 400Hz for motor with motor_pwm_protocol set to STANDARD. For *SHOT (e.g. ONESHOT125) values of 1000 and 2000 have been tested by the development team and are supported. It may be possible to use higher values. For BRUSHED values of 8000 and above should be used. Setting to 8000 will use brushed mode at 8kHz switching frequency. Up to 32kHz is supported for brushed. Default is 16000 for boards with brushed motors. Note, that in brushed mode, minthrottle is offset to zero. For brushed mode, set max_throttle to 2000. |
|  motor_pwm_protocol  | STANDARD | Protocol that is used to send motor updates to ESCs. Possible values - STANDARD, ONESHOT125, ONESHOT42, MULTISHOT, DSHOT150, DSHOT300, DSHOT600, DSHOT1200, BRUSHED |
|  fixed_wing_auto_arm  | OFF | Auto-arm fixed wing aircraft on throttle above min_check, and disarming with stick commands are disabled, so power cycle is required to disarm. Requires enabled motorstop and no arm switch configured. |
|  disarm_kill_switch  | ON | Disarms the motors independently of throttle value. Setting to OFF reverts to the old behaviour of disarming only when the throttle is low. Only applies when arming and disarming with an AUX channel. |
|  switch_disarm_delay | 250 | Delay before disarming when requested by switch (ms) [0-1000] |
|  small_angle  | 25 | If the aircraft tilt angle exceed this value the copter will refuse to arm.  |
|  reboot_character  | 82 | Special character used to trigger reboot |
|  gps_provider  | UBLOX | Which GPS protocol to be used |
|  gps_sbas_mode  | NONE | Which SBAS mode to be used |
|  gps_dyn_model  | AIR_1G | GPS navigation model: Pedestrian, Air_1g, Air_4g. Default is AIR_1G. Use pedestrian with caution, can cause flyaways with fast flying. |
|  gps_auto_config  | ON | Enable automatic configuration of UBlox GPS receivers. |
|  gps_auto_baud  | ON | Automatic configuration of GPS baudrate(The specified baudrate in configured in ports will be used) when used with UBLOX GPS. When used with NAZA/DJI it will automatic detect GPS baudrate and change to it, ignoring the selected baudrate set in ports |
|  gps_min_sats  | 6 | Minimum number of GPS satellites in view to acquire GPS_FIX and consider GPS position valid. Some GPS receivers appeared to be very inaccurate with low satellite count. |
|  gps_ublox_use_galileo | OFF | Enable use of Galileo satellites. This is at the expense of other regional constellations, so benefit may also be regional. Requires M8N and Ublox firmware 3.x (or later) [OFF/ON].
|  inav_auto_mag_decl  | ON | Automatic setting of magnetic declination based on GPS position. When used manual magnetic declination is ignored. |
|  inav_gravity_cal_tolerance  | 5 | Unarmed gravity calibration tolerance level. Won't finish the calibration until estimated gravity error falls below this value. |
|  inav_use_gps_velned  | ON | Defined if iNav should use velocity data provided by GPS module for doing position and speed estimation. If set to OFF iNav will fallback to calculating velocity from GPS coordinates. Using native velocity data may improve performance on some GPS modules. Some GPS modules introduce significant delay and using native velocity may actually result in much worse performance. |
|  inav_reset_altitude | FIRST_ARM | Defines when relative estimated altitude is reset to zero. Variants - `NEVER` (once reference is acquired it's used regardless); `FIRST_ARM` (keep altitude at zero until firstly armed), `EACH_ARM` (altitude is reset to zero on each arming) |
|  inav_reset_home | EACH_ARM | Allows to chose when the home position is reset. Can help prevent resetting home position after accidental mid-air disarm. Possible values are: NEVER, FIRST_ARM and EACH_ARM |
|  inav_max_surface_altitude  | 200 | Max allowed altitude for surface following mode. [cm] |
|  inav_w_z_baro_p  | 0.350 | Weight of barometer measurements in estimated altitude and climb rate |
|  inav_w_z_gps_p  | 0.200 | Weight of GPS altitude measurements in estimated altitude. Setting is used only of airplanes |
|  inav_w_z_gps_v  | 0.500 | Weight of GPS climb rate measurements in estimated climb rate. Setting is used on both airplanes and multirotors. If GPS doesn't support native climb rate reporting (i.e. NMEA GPS) you may consider setting this to zero |
|  inav_w_xy_gps_p  | 1.000 | Weight of GPS coordinates in estimated UAV position and speed. |
|  inav_w_xy_gps_v  | 2.000 | Weight of GPS velocity data in estimated UAV speed |
|  inav_w_z_res_v  | 0.500 | Decay coefficient for estimated climb rate when baro/GPS reference for altitude is lost |
|  inav_w_xy_res_v  | 0.500 | Decay coefficient for estimated velocity when GPS reference for position is lost |
|  inav_w_acc_bias  | 0.010 | Weight for accelerometer drift estimation |
|  inav_max_eph_epv  | 1000.000 | Maximum uncertainty value until estimated position is considered valid and is used for navigation [cm] |
|  inav_baro_epv  | 100.000 | Uncertainty value for barometric sensor [cm] |
|  name  | Empty string | Craft name |
|  nav_disarm_on_landing  | OFF | If set to ON, iNav disarms the FC after landing |
|  nav_use_midthr_for_althold  | OFF | If set to OFF, the FC remembers your throttle stick position when enabling ALTHOLD and treats it as a neutral midpoint for holding altitude |
|  nav_extra_arming_safety  | ON | If set to ON drone won't arm if no GPS fix and any navigation mode like RTH or POSHOLD is configured. ALLOW_BYPASS allows the user to momentarily disable this check by holding yaw high (left stick held at the bottom right in mode 2) when switch arming is used |
|  nav_user_control_mode  | ATTI | Defines how Pitch/Roll input from RC receiver affects flight in POSHOLD mode: ATTI - pitch/roll controls attitude like in ANGLE mode; CRUISE - pitch/roll controls velocity in forward and right direction. |
|  nav_position_timeout  | 5 | If GPS fails wait for this much seconds before switching to emergency landing mode (0 - disable) |
|  nav_wp_radius  | 100 | Waypoint radius [cm]. Waypoint would be considered reached if machine is within this radius |
|  nav_wp_safe_distance  | 10000 | First waypoint in the mission should be closer than this value [cm]. A value of 0 disables this check. |
|  nav_auto_speed  | 300 | Maximum velocity firmware is allowed in full auto modes (RTH, WP) [cm/s] [Multirotor only] |
|  nav_auto_climb_rate  | 500 | Maximum climb/descent rate that UAV is allowed to reach during navigation modes. [cm/s] |
|  nav_manual_speed  | 500 | Maximum velocity firmware is allowed when processing pilot input for POSHOLD/CRUISE control mode [cm/s] [Multirotor only] |
|  nav_manual_climb_rate  | 200 | Maximum climb/descent rate firmware is allowed when processing pilot input for ALTHOLD control mode [cm/s] |
|  nav_landing_speed  | 200 | Vertical descent velocity during the RTH landing phase. [cm/s] |
|  nav_land_slowdown_minalt  | 500 | Defines at what altitude the descent velocity should start to be 25% of nav_landing_speed [cm] |
|  nav_land_slowdown_maxalt  | 2000 | Defines at what altitude the descent velocity should start to ramp down from 100% nav_landing_speed to 25% nav_landing_speed. [cm] |
|  nav_emerg_landing_speed  | 500 | Rate of descent UAV will try to maintain when doing emergency descent sequence [cm/s] |
|  nav_min_rth_distance  | 500 | Minimum distance from homepoint when RTH full procedure will be activated [cm]. Below this distance, the mode will activate at the current location and the final phase is executed (loiter / land). Above this distance, the full procedure is activated, which may include initial climb and flying directly to the homepoint before entering the loiter / land phase. |
|  nav_overrides_motor_stop | ON | Setting to OFF combined with MOTOR_STOP feature will allow user to stop motor when in autonomous modes. On most places this setting is likely to cause a stall. |
|  nav_rth_climb_first  | ON | If set to ON drone will climb to nav_rth_altitude first and head home afterwards. If set to OFF drone will head home instantly and climb on the way. |
|  nav_rth_tail_first  | OFF | If set to ON drone will return tail-first. Obviously meaningless for airplanes. |
|  nav_rth_allow_landing  | ALWAYS | If set to ON drone will land as a last phase of RTH. |
|  nav_rth_climb_ignore_emerg  | OFF | If set to ON, aircraft will execute initial climb regardless of position sensor (GPS) status. |
|  nav_rth_alt_mode  | AT_LEAST | Configure how the aircraft will manage altitude on the way home, see Navigation modes on wiki for more details |
|  nav_rth_altitude  | 1000 | Used in EXTRA, FIXED and AT_LEAST rth alt modes [cm] (Default 1000 means 10 meters) |
|  nav_rth_home_altitude  | 0 | Aircraft will climb/descend to this altitude after reaching home if landing is not enabled. Set to 0 to stay at `nav_rth_altitude` (default) [cm] |
|  nav_rth_home_wait  | 0 | Aircraft will wait after reaching home and at this same time climb/descend to nav_rth_home_altitude (if enabled) and then landing (if enabled). Set to 0 to disabled (default) [s] |
|  nav_rth_abort_threshold  | 50000 | RTH sanity checking feature will notice if distance to home is increasing during RTH and once amount of increase exceeds the threshold defined by this parameter, instead of continuing RTH machine will enter emergency landing, self-level and go down safely. Default is 500m which is safe enough for both multirotor machines and airplanes. [cm] |
|  nav_rth_home_offset_distance  | 0 | Distance offset from GPS established home to "safe" position used for RTH (cm, 0 disables) |
|  nav_rth_home_offset_direction | 0 | Direction offset from GPS established home to "safe" position used for RTH (degrees, 0=N, 90=E, 180=S, 270=W, requires non-zero offset distance) |
|  nav_mc_bank_angle  | 30 | Maximum banking angle (deg) that multicopter navigation is allowed to set. Machine must be able to satisfy this angle without loosing altitude |
|  nav_mc_hover_thr  | 1500 | Multicopter hover throttle hint for altitude controller. Should be set to approximate throttle value when drone is hovering. |
|  nav_mc_auto_disarm_delay  | 2000 |  |
|  nav_fw_cruise_thr  | 1400 | Cruise throttle in GPS assisted modes, this includes RTH. Should be set high enough to avoid stalling. This values gives INAV a base for throttle when flying straight, and it will increase or decrease throttle based on pitch of airplane and the parameters below. In addition it will increase throttle if GPS speed gets below 7m/s ( hardcoded )  |
|  nav_fw_cruise_speed  | 0 | Speed for the plane/wing at cruise throttle used for remaining flight time/distance estimation in cm/s |
|  nav_fw_allow_manual_thr_increase  | OFF | Enable the possibility to manually increase the throttle in auto throttle controlled modes for fixed wing |
|  nav_fw_min_thr  | 1200 | Minimum throttle for flying wing in GPS assisted modes |
|  nav_fw_max_thr  | 1700 | Maximum throttle for flying wing in GPS assisted modes |
|  nav_fw_bank_angle  | 20 | Max roll angle when rolling / turning in GPS assisted modes, is also restrained by global max_angle_inclination_rll |
|  nav_fw_climb_angle  | 20 | Max pitch angle when climbing in GPS assisted modes, is also restrained by global max_angle_inclination_pit |
|  nav_fw_dive_angle  | 15 | Max negative pitch angle when diving in GPS assisted modes, is also restrained by global max_angle_inclination_pit |
|  nav_fw_pitch2thr  | 10 | Amount of throttle applied related to pitch attitude in GPS assisted modes. Throttle = nav_fw_cruise_throttle - (nav_fw_pitch2thr * pitch_angle). (notice that pitch_angle is in degrees and is negative when climbing and positive when diving, and throttle value is constrained between nav_fw_min_thr and nav_fw_max_thr) |
|  nav_fw_loiter_radius  | 5000 | PosHold radius. 3000 to 7500 is a good value (30-75m) [cm] |
|  nav_fw_launch_velocity  | 300 | Forward velocity threshold for swing-launch detection [cm/s] |
|  nav_fw_launch_accel  | 1863 | Forward acceleration threshold for bungee launch of throw launch [cm/s/s], 1G = 981 cm/s/s |
|  nav_fw_launch_max_angle  | 45 | Max tilt angle (pitch/roll combined) to consider launch successful. Set to 180 to disable completely [deg] |
|  nav_fw_launch_detect_time  | 40 | Time for which thresholds have to breached to consider launch happened [ms] |
|  nav_fw_launch_thr  | 1700 | Launch throttle - throttle to be set during launch sequence (pwm units) |
|  nav_fw_launch_idle_thr       | 1000  | Launch idle throttle - throttle to be set before launch sequence is initiated. If set below minimum throttle it will force motor stop or at idle throttle (depending if the MOTOR_STOP is enabled). If set above minimum throttle it will force throttle to this value (if MOTOR_STOP is enabled it will be handled according to throttle stick position)	|
|  nav_fw_launch_motor_delay    | 500 | Delay between detected launch and launch sequence start and throttling up (ms) |
|  nav_fw_launch_spinup_time    | 100 | Time to bring power from minimum throttle to nav_fw_launch_thr - to avoid big stress on ESC and large torque from propeller |
|  nav_fw_launch_timeout  | 5000 | Maximum time for launch sequence to be executed. After this time LAUNCH mode will be turned off and regular flight mode will take over (ms) |
|  nav_fw_launch_max_altitude  | 0 | Altitude at which LAUNCH mode will be turned off and regular flight mode will take over. [cm] |
|  nav_fw_launch_climb_angle  | 18 | Climb angle for launch sequence (degrees), is also restrained by global max_angle_inclination_pit |
|  nav_fw_launch_min_time | 0 | Allow launch mode to execute at least this time (ms) and ignore stick movements [0-60000]. |
|  nav_fw_launch_max_altitude | 0 | Altitude (centimeters) at which LAUNCH mode will be turned off and regular flight mode will take over [0-60000]. |
|  nav_fw_land_dive_angle  | 2 | Dive angle that airplane will use during final landing phase. During dive phase, motor is stopped or IDLE and roll control is locked to 0 degrees |
|  nav_fw_cruise_yaw_rate  | 20 | Max YAW rate when NAV CRUISE mode is enabled (0=disable control via yaw stick) [dps]|
|  serialrx_provider  | SPEK1024 | When feature SERIALRX is enabled, this allows connection to several receivers which output data via digital interface resembling serial. See RX section. |
|  serialrx_halfduplex  | OFF | Allow serial receiver to operate on UART TX pin. With some receivers will allow control and telemetry over a single wire |
|  serialrx_inverted     | OFF | Reverse the serial inversion of the  serial RX protocol. When this value is OFF, each protocol will use its default signal (e.g. SBUS will use an inverted signal). Some OpenLRS receivers produce a non-inverted SBUS signal. This setting supports this type of receivers (including modified FrSKY). |
|  spektrum_sat_bind  | 0 | 0 = disabled. Used to bind the spektrum satellite to RX |
|  telemetry_switch  | OFF | Which aux channel to use to change serial output & baud rate (MSP / Telemetry). It disables automatic switching to Telemetry when armed. |
|  telemetry_inverted  | OFF | Determines if the telemetry protocol default signal inversion is reversed. This should be OFF in most cases unless a custom or hacked RX is used. |
|  frsky_unit  | METRIC | Not used? [METRIC/IMPERIAL] |
|  frsky_default_latitude  | 0.000 | D-Series telemetry only: OpenTX needs a valid set of coordinates to show compass value. A fake value defined in this setting is sent while no fix is acquired. |
|  frsky_default_longitude  | 0.000 | D-Series telemetry only: OpenTX needs a valid set of coordinates to show compass value. A fake value defined in this setting is sent while no fix is acquired. |
|  frsky_coordinates_format  | 0 | D-Series telemetry only: FRSKY_FORMAT_DMS (default), FRSKY_FORMAT_NMEA |
|  frsky_vfas_precision  | 0 | D-Series telemetry only: Set to 1 to send raw VBat value in 0.1V resolution for receivers that can handle it, or 0 (default) to use the standard method |
|  frsky_pitch_roll  | OFF | S.Port and D-Series telemetry: Send pitch and roll degrees*10 instead of raw accelerometer data |
|  smartport_fuel_unit  | MAH | S.Port telemetry only: Unit of the value sent with the `FUEL` ID (FrSky D-Series always sends precent). [PERCENT/MAH/MWH] |
|  telemetry_uart_unidir  | OFF | S.Port telemetry only: Turn UART into UNIDIR for usage on F1 and F4 target. See Telemetry.md for details |
|  report_cell_voltage  | OFF | S.Port, D-Series, and IBUS telemetry: Send the average cell voltage if set to ON |
|  hott_alarm_sound_interval  | 5 | Battery alarm delay in seconds for Hott telemetry |
|  smartport_fuel_unit  | MAH | S.Port and D-Series telemetry: Unit of the value sent with the `FUEL` ID. [PERCENT/MAH/MWH] |
|  ibus_telemetry_type  | 0 | Type compatibility ibus telemetry for transmitters. See Telemetry.md label IBUS for details. |
|  ltm_update_rate  | NORMAL | Defines the LTM update rate (use of bandwidth [NORMAL/MEDIUM/SLOW]). See Telemetry.md, LTM section for details. |
|  battery_capacity  | 0 | Battery capacity in mAH. This value is used in conjunction with the current meter to determine remaining battery capacity. |
|  vbat_scale  | 1100 | Battery voltage calibration value. 1100 = 11:1 voltage divider (10k:1k) x 100. Adjust this slightly if reported pack voltage is different from multimeter reading. You can get current voltage by typing "status" in cli. |
|  bat_voltage_src  | RAW | Chose between raw and sag compensated battery voltage to use for battery alarms and telemetry. Possible values are `RAW` and `SAG_COMP` |
|  bat_cells  | 0 | Number of cells of the battery (0 = autodetect), see battery documentation |
|  vbat_cell_detect_voltage  | 430 | Maximum voltage per cell, used for auto-detecting the number of cells of the battery in 0.01V units, default is 4.30V |
|  vbat_max_cell_voltage  | 420 | Maximum voltage per cell in 0.01V units, default is 4.20V |
|  vbat_min_cell_voltage  | 330 | Minimum voltage per cell, this triggers battery out alarms, in 0.01V units, default is 330 (3.3V) |
|  vbat_warning_cell_voltage  | 350 | Warning voltage per cell, this triggers battery-warning alarms, in 0.01V units, default is 350 (3.5V) |
|  current_meter_scale  | 400 | This sets the output voltage to current scaling for the current sensor in 0.1 mV/A steps. 400 is 40mV/A such as the ACS756 sensor outputs. 183 is the setting for the uberdistro with a 0.25mOhm shunt. |
|  current_meter_offset  | 0 | This sets the output offset voltage of the current sensor in millivolts. |
|  battery_capacity | 0 | Set the battery capacity in mAh or mWh (see `battery_capacity_unit`). Used to calculate the remaining battery capacity. |
|  battery_capacity_warning | 0 | If the remaining battery capacity goes below this threshold the beeper will emit short beeps and the relevant OSD items will blink. |
|  battery_capacity_critical | 0 | If the remaining battery capacity goes below this threshold the battery is considered empty and the beeper will emit long beeps. |
|  battery_capacity_unit | MAH | Unit used for `battery_capacity`, `battery_capacity_warning` and `battery_capacity_critical` [MAH/MWH] (milliAmpere hour / milliWatt hour). |
|  cruise_power  | 0 | Power draw at cruise throttle used for remaining flight time/distance estimation in 0.01W unit |
|  idle_power  | 0 | Power draw at zero throttle used for remaining flight time/distance estimation in 0.01W unit |
|  rth_energy_margin | 5 | Energy margin wanted after getting home (percent of battery energy capacity). Use for the remaining flight time/distance calculation |
|  multiwii_current_meter_output  | OFF | Default current output via MSP is in 0.01A steps. Setting this to 1 causes output in default multiwii scaling (1mA steps) |
|  current_meter_type  | ADC | ADC , VIRTUAL, NONE. The virtual current sensor, once calibrated, estimates the current value from throttle position. |
|  align_gyro  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_acc  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_mag  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_board_roll  | 0 | Arbitrary board rotation in deci-degrees (0.1 degree), to allow mounting it sideways / upside down / rotated etc |
|  align_board_pitch  | 0 | Arbitrary board rotation in deci-degrees (0.1 degree), to allow mounting it sideways / upside down / rotated etc |
|  align_board_yaw  | 0 | Arbitrary board rotation in deci-degrees (0.1 degree), to allow mounting it sideways / upside down / rotated etc |
|  align_mag_roll  | 0 | Set the external mag alignment on the roll axis (in 0.1 degree steps). If this value is non-zero, the compass is assumed to be externally mounted and both the board and on-board compass alignent (align_mag) are ignored. See also align_mag_pitch and align_mag_yaw. |
|  align_mag_pitch  | 0 | Same as align_mag_roll, but for the pitch axis. |
|  align_mag_yaw  | 0 | Same as align_mag_roll, but for the yaw axis. |
|  align_opflow | 5 | Optical flow module alignment (default CW0_DEG_FLIP) |
|  gyro_hardware_lpf  | 42HZ | Hardware lowpass filter for gyro. Allowed values depend on the driver - For example MPU6050 allows 10HZ,20HZ,42HZ,98HZ,188HZ,256Hz (8khz mode). If you have to set gyro lpf below 42Hz generally means the frame is vibrating too much, and that should be fixed first. |
|  moron_threshold  | 32 | When powering up, gyro bias is calculated. If the model is shaking/moving during this initial calibration, offsets are calculated incorrectly, and could lead to poor flying performance. This threshold means how much average gyro reading could differ before re-calibration is triggered. |
|  imu_dcm_kp  | 2500 | Inertial Measurement Unit KP Gain for accelerometer measurements |
|  imu_dcm_ki  | 50 | Inertial Measurement Unit KI Gain for accelerometer measurements |
|  imu_dcm_kp_mag  | 10000 | Inertial Measurement Unit KP Gain for compass measurements |
|  imu_dcm_ki_mag  | 0 | Inertial Measurement Unit KI Gain for compass measurements |
|  imu_acc_ignore_rate  | 0 | Total gyro rotation rate threshold [deg/s] to consider accelerometer trustworthy on airplanes |
|  imu_acc_ignore_slope | 0 | Half-width of the interval to gradually reduce accelerometer weight. Centered at `imu_acc_ignore_rate` (exactly 50% weight) |
|  pos_hold_deadband  | 20 | Stick deadband in [r/c points], applied after r/c deadband and expo |
|  alt_hold_deadband  | 50 | Defines the deadband of throttle during alt_hold [r/c points] |
|  yaw_motor_direction  | 1 | Use if you need to inverse yaw motor direction. |
|  tri_unarmed_servo  | ON | On tricopter mix only, if this is set to ON, servo will always be correcting regardless of armed state. to disable this, set it to OFF. |
|  servo_lpf_hz  | 20 | Selects the servo PWM output cutoff frequency. Value is in [Hz] |
|  servo_center_pulse  | 1500 | Servo midpoint |
|  servo_pwm_rate  | 50 | Output frequency (in Hz) servo pins. When using tricopters or gimbal with digital servo, this rate can be increased. Max of 498Hz (for 500Hz pwm period), and min of 50Hz. Most digital servos will support for example 330Hz. |
|  failsafe_delay  | 5 | Time in deciseconds to wait before activating failsafe when signal is lost. See [Failsafe documentation](Failsafe.md#failsafe_delay). |
|  failsafe_recovery_delay  | 5 | Time in deciseconds to wait before aborting failsafe when signal is recovered. See [Failsafe documentation](Failsafe.md#failsafe_recovery_delay). |
|  failsafe_off_delay  | 200 | Time in deciseconds to wait before turning off motors when failsafe is activated. 0 = No timeout. See [Failsafe documentation](Failsafe.md#failsafe_off_delay). |
|  failsafe_throttle  | 1000 | Throttle level used for landing when failsafe is enabled. See [Failsafe documentation](Failsafe.md#failsafe_throttle). |
|  failsafe_throttle_low_delay  | 100 | If failsafe activated when throttle is low for this much time - bypass failsafe and disarm, in 10th of seconds. 0 = No timeout |
|  failsafe_procedure  | SET-THR | What failsafe procedure to initiate in Stage 2. See [Failsafe documentation](Failsafe.md#failsafe_throttle). |
|  failsafe_stick_threshold  | 50 | Threshold for stick motion to consider failsafe condition resolved. If non-zero failsafe won't clear even if RC link is restored - you have to move sticks to exit failsafe. |
|  failsafe_fw_roll_angle  | -200 | Amount of banking when `SET-THR` failsafe is active on a fixed-wing machine. In 1/10 deg (deci-degrees). Negative values = left roll |
|  failsafe_fw_pitch_angle  | 100 | Amount of dive/climb when `SET-THR` failsafe is active on a fixed-wing machine. In 1/10 deg (deci-degrees). Negative values = climb |
|  failsafe_fw_yaw_rate  | -45 | Requested yaw rate to execute when `SET-THR` failsafe is active on a fixed-wing machine. In deg/s. Negative values = left turn |
|  failsafe_min_distance  | 0 | If failsafe happens when craft is closer than this distance in centimeters from home, failsafe will not execute regular failsafe_procedure, but will execute procedure specified in failsafe_min_distance_procedure instead. 0 = Normal failsafe_procedure always taken. |
|  failsafe_min_distance_procedure  | DROP | What failsafe procedure to initiate in Stage 2 when craft is closer to home than failsafe_min_distance. See [Failsafe documentation](Failsafe.md#failsafe_throttle). |
|  failsafe_lights | ON | Enable or disable the lights when the `FAILSAFE` flight mode is enabled. The target needs to be compiled with `USE_LIGHTS` [ON/OFF]. |
|  failsafe_lights_flash_period | 1000 | Time in milliseconds between two flashes when `failsafe_lights` is ON and `FAILSAFE` flight mode is enabled [40-65535]. |
|  failsafe_lights_flash_on_time | 100 | Flash lights ON time in milliseconds when `failsafe_lights` is ON and `FAILSAFE` flight mode is enabled. [20-65535]. |
|  failsafe_mission | ON | If set to `OFF` the failsafe procedure won't be triggered and the mission will continue if the FC is in WP (automatic mission) mode |
|  rx_min_usec  | 885 | Defines the shortest pulse width value used when ensuring the channel value is valid. If the receiver gives a pulse value lower than this value then the channel will be marked as bad and will default to the value of mid_rc. |
|  rx_max_usec  | 2115 | Defines the longest pulse width value used when ensuring the channel value is valid. If the receiver gives a pulse value higher than this value then the channel will be marked as bad and will default to the value of mid_rc. |
|  rx_nosignal_throttle  | HOLD | Defines behavior of throttle channel after signal loss is detected and until `failsafe_procedure` kicks in. Possible values - `HOLD` and `DROP`. |
|  acc_hardware  | AUTO | Selection of acc hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  baro_median_filter  | ON | 3-point median filtering for barometer readouts. No reason to change this setting |
|  baro_hardware  | AUTO | Selection of baro hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  mag_hardware  | AUTO | Selection of mag hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  mag_to_use | | Allow to chose between built-in and external compass sensor if they are connected to separate buses. Currently only for REVO target |
|  rangefinder_median_filter | OFF | 3-point median filtering for rangefinder readouts |
|  blackbox_rate_num  | 1 | Blackbox logging rate numerator. Use num/denom settings to decide if a frame should be logged, allowing control of the portion of logged loop iterations |
|  blackbox_rate_denom  | 1 | Blackbox logging rate denominator. See blackbox_rate_num. |
|  blackbox_device  | SPIFLASH | Selection of where to write blackbox data |
|  sdcard_detect_inverted  | `TARGET dependent` | This setting drives the way SD card is detected in card slot. On some targets (AnyFC F7 clone) different card slot was used and depending of hardware revision ON or OFF setting might be required. If card is not detected, change this value. |
|  ledstrip_visual_beeper  | OFF |  |
|  osd_video_system     | AUTO   | Video system used. Possible values are `AUTO`, `PAL` and `NTSC` |
|  osd_row_shiftdown    | 0     | Number of rows to shift the OSD display (increase if top rows are cut off) |
|  osd_units            | METRIC| IMPERIAL, METRIC, UK |
|  osd_stats_energy_unit | MAH | Unit used for the drawn energy in the OSD stats [MAH/WH] (milliAmpere hour/ Watt hour) |
|  osd_main_voltage_decimals | 1 | Number of decimals for the battery voltages displayed in the OSD [1-2]. |
|  osd_rssi_alarm       | 20    | Value bellow which to make the OSD RSSI indicator blink |
|  osd_time_alarm       | 10    | Value above which to make the OSD flight time indicator blink (minutes) |
|  osd_dist_alarm       | 1000  | Value above which to make the OSD distance from home indicator blink (meters) |
|  osd_alt_alarm        | 100   | Value above which to make the OSD relative altitude indicator blink (meters) |
|  osd_neg_alt_alarm    | 5    | Value bellow which (negative altitude) to make the OSD relative altitude indicator blink (meters) |
|  osd_gforce_alarm     | 5    | Value above which the OSD g force indicator will blink (g) |
|  osd_gforce_axis_alarm_min | -5 | Value under which the OSD axis g force indicators will blink (g) |
|  osd_gforce_axis_alarm_max | 5  | Value above which the OSD axis g force indicators will blink (g) |
|  osd_imu_temp_alarm_min | -200 | Temperature under which the IMU temperature OSD element will start blinking (decidegrees centigrade) |
|  osd_imu_temp_alarm_max | 600 | Temperature above which the IMU temperature OSD element will start blinking (decidegrees centigrade) |
|  osd_baro_temp_alarm_min | -200 | Temperature under which the baro temperature OSD element will start blinking (decidegrees centigrade) |
|  osd_baro_temp_alarm_max | 600 | Temperature above which the baro temperature OSD element will start blinking (decidegrees centigrade) |
|  osd_current_alarm | 0 | Value above which the OSD current consumption element will start blinking. Measured in full Amperes. |
|  osd_estimations_wind_compensation  | ON | Use wind estimation for remaining flight time/distance estimation |
|  osd_failsafe_switch_layout  | OFF | If enabled the OSD automatically switches to the first layout during failsafe |
|  osd_temp_label_align | LEFT | Allows to chose between left and right alignment for the OSD temperature sensor labels. Valid values are `LEFT` and `RIGHT` |
|  display_force_sw_blink  | OFF | OFF = OSD hardware blink / ON = OSD software blink. If OSD warning text/values are invisible, try setting this to ON |
|  magzero_x  | 0 | Magnetometer calibration X offset. If its 0 none offset has been applied and calibration is failed. |
|  magzero_y  | 0 | Magnetometer calibration Y offset. If its 0 none offset has been applied and calibration is failed. |
|  magzero_z  | 0 | Magnetometer calibration Z offset. If its 0 none offset has been applied and calibration is failed. |
|  acczero_x  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  acczero_y  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  acczero_z  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  accgain_x  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  accgain_y  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  accgain_z  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  nav_mc_pos_z_p  | 50 | P gain of altitude PID controller (Multirotor) |
|  nav_fw_pos_z_p  | 50 | P gain of altitude PID controller (Fixedwing) |
|  nav_fw_pos_z_i  | 0 | I gain of altitude PID controller (Fixedwing) |
|  nav_fw_pos_z_d  | 0 | D gain of altitude PID controller (Fixedwing) |
|  nav_mc_vel_z_p  | 100 | P gain of velocity PID controller |
|  nav_mc_vel_z_i  | 50 | I gain of velocity PID controller |
|  nav_mc_vel_z_d  | 10 | D gain of velocity PID controller |
|  nav_mc_pos_xy_p  | 65 | Controls how fast the drone will fly towards the target position. This is a multiplier to convert displacement to target velocity |
|  nav_mc_vel_xy_p  | 40 | P gain of Position-Rate (Velocity to Acceleration) PID controller. Higher P means stronger response when position error occurs. Too much P might cause "nervous" behavior and oscillations |
|  nav_mc_vel_xy_i  | 15 | I gain of Position-Rate (Velocity to Acceleration) PID controller. Used for drift compensation (caused by wind for example). Higher I means stronger response to drift. Too much I gain might cause target overshot |
|  nav_mc_vel_xy_d  | 100 | D gain of Position-Rate (Velocity to Acceleration) PID controller. It can damp P and I. Increasing D might help when drone overshoots target. |
|  nav_fw_pos_xy_p  | 75 | P gain of 2D trajectory PID controller. Play with this to get a straight line between waypoints or a straight RTH |
|  nav_fw_pos_xy_i  | 5 | I gain of 2D trajectory PID controller. Too high and there will be overshoot in trajectory. Better start tuning with zero |
|  nav_fw_pos_xy_d  | 8 | D gain of 2D trajectory PID controller. Too high and there will be overshoot in trajectory. Better start tuning with zero |
|  nav_mc_heading_p  | 60 | P gain of Heading Hold controller (Multirotor) |
|  nav_fw_heading_p  | 60 | P gain of Heading Hold controller (Fixedwing) |
|  deadband  | 5 | These are values (in us) by how much RC input can be different before it's considered valid. For transmitters with jitter on outputs, this value can be increased. Defaults are zero, but can be increased up to 10 or so if rc inputs twitch while idle. |
|  yaw_deadband  | 5 | These are values (in us) by how much RC input can be different before it's considered valid. For transmitters with jitter on outputs, this value can be increased. Defaults are zero, but can be increased up to 10 or so if rc inputs twitch while idle. |
|  throttle_tilt_comp_str  | 0 | Can be used in ANGLE and HORIZON mode and will automatically boost throttle when banking. Setting is in percentage, 0=disabled. |
|  flaperon_throw_offset  | 200 | Defines throw range in us for both ailerons that will be passed to servo mixer via input source 14 (`FEATURE FLAPS`) when FLAPERON mode is activated. |
|  fw_iterm_throw_limit  | 165 | Limits max/min I-term value in stabilization PID controller in case of Fixed Wing. It solves the problem of servo saturation before take-off/throwing the airplane into the air. By default, error accumulated in I-term can not exceed 1/3 of servo throw (around 165us). Set 0 to disable completely. |
|  fw_reference_airspeed  | 1000 | Reference airspeed. Set this to airspeed at which PIDs were tuned. Usually should be set to cruise airspeed. Also used for coordinated turn calculation if airspeed sensor is not present. |
|  fw_turn_assist_yaw_gain  | 1 | Gain required to keep the yaw rate consistent with the turn rate for a coordinated turn (in TURN_ASSIST mode). Value significantly different from 1.0 indicates a problem with the airspeed calibration (if present) or value of `fw_reference_airspeed` parameter |
|  fw_loiter_direction  | RIGHT | Direction of loitering: center point on right wing (clockwise - default), or center point on left wing (counterclockwise). If equal YAW then can be changed in flight using a yaw stick. |
|  mode_range_logic_operator  | OR | Control how Mode selection works in flight modes. If you example have Angle mode configured on two different Aux channels, this controls if you need both activated ( AND ) or if you only need one activated ( OR ) to active angle mode. |
|  default_rate_profile  | 0 | Default = profile number |
|  mag_declination  | 0 | Current location magnetic declination in format. For example, -6deg 37min = -637 for Japan. Leading zero in ddd not required. Get your local magnetic declination here: http://magnetic-declination.com/ . Not in use if inav_auto_mag_decl  is turned on and you acquire valid GPS fix. |
|  heading_hold_rate_limit  | 90 | This setting limits yaw rotation rate that HEADING_HOLD controller can request from PID inner loop controller. It is independent from manual yaw rate and used only when HEADING_HOLD flight mode is enabled by pilot, RTH or WAYPOINT modes. |
| mag_calibration_time | 30 | Adjust how long time the Calibration of mag will last. |
| mc_p_pitch | 40 | Multicopter rate stabilisation P-gain for PITCH               |
| mc_i_pitch | 30 | Multicopter rate stabilisation I-gain for PITCH               |
| mc_d_pitch | 23 | Multicopter rate stabilisation D-gain for PITCH               |
| mc_p_roll  | 40 | Multicopter rate stabilisation P-gain for ROLL                |
| mc_i_roll  | 30 | Multicopter rate stabilisation I-gain for ROLL                |
| mc_d_roll  | 23 | Multicopter rate stabilisation D-gain for ROLL                |
| mc_p_yaw   | 85 | Multicopter rate stabilisation P-gain for YAW                 |
| mc_i_yaw   | 45 | Multicopter rate stabilisation I-gain for YAW                 |
| mc_d_yaw   | 0  | Multicopter rate stabilisation D-gain for YAW                 |
| mc_p_level | 20 | Multicopter attitude stabilisation P-gain                     |
| mc_i_level | 15 | Multicopter attitude stabilisation low-pass filter cutoff     |
| mc_d_level | 75 | Multicopter attitude stabilisation HORIZON transition point   |
| fw_p_pitch | 5 | Fixed-wing rate stabilisation P-gain for PITCH                |
| fw_i_pitch | 7 | Fixed-wing rate stabilisation I-gain for PITCH                |
| fw_ff_pitch| 50 | Fixed-wing rate stabilisation FF-gain for PITCH               |
| fw_p_roll  | 5 | Fixed-wing rate stabilisation P-gain for ROLL                 |
| fw_i_roll  | 7 | Fixed-wing rate stabilisation I-gain for ROLL                 |
| fw_ff_roll | 50 | Fixed-wing rate stabilisation FF-gain for ROLL                |
| fw_p_yaw   | 6 | Fixed-wing rate stabilisation P-gain for YAW                  |
| fw_i_yaw   | 10 | Fixed-wing rate stabilisation I-gain for YAW                  |
| fw_ff_yaw  | 60  | Fixed-wing rate stabilisation FF-gain for YAW                 |
| fw_p_level | 20 | Fixed-wing attitude stabilisation P-gain                      |
| fw_i_level | 5 | Fixed-wing attitude stabilisation low-pass filter cutoff      |
| fw_d_level | 75 | Fixed-wing attitude stabilisation HORIZON transition point    |
|  max_angle_inclination_rll  | 300 | Maximum inclination in level (angle) mode (ROLL axis). 100=10° |
|  max_angle_inclination_pit  | 300 | Maximum inclination in level (angle) mode (PITCH axis). 100=10° |
|  fw_iterm_limit_stick_position  | 0.5 | Iterm is not allowed to grow when stick position is above threshold. This solves the problem of bounceback or followthrough when full stick deflection is applied on poorely tuned fixed wings. In other words, stabilization is partialy disabled when pilot is actively controlling the aircraft and active when sticks are not touched. `0` mean stick is in center position, `1` means it is fully deflected to either side |
|  fw_min_throttle_down_pitch  | 0 | Automatic pitch down angle when throttle is at 0 in angle mode. Progressively applied between cruise throttle and zero throttle (decidegrees) |
|  gyro_lpf_hz  | 60 | Software-based filter to remove mechanical vibrations from the gyro signal. Value is cutoff frequency (Hz). For larger frames with bigger props set to lower value. |
|  gyro_lpf_type  | BIQUAD | Specifies the type of the software LPF of the gyro signals. BIQUAD gives better filtering and more delay, PT1 less filtering and less delay, so use only on clean builds. |
|  acc_lpf_hz  | 15 | Software-based filter to remove mechanical vibrations from the accelerometer measurements. Value is cutoff frequency (Hz). For larger frames with bigger props set to lower value. |
|  acc_lpf_type  | BIQUAD | Specifies the type of the software LPF of the acc signals. BIQUAD gives better filtering and more delay, PT1 less filtering and less delay, so use only on clean builds. |
|  dterm_lpf_hz  | 40 | Dterm low pass filter cutoff frequency. Default setting is very conservative and small multirotors should use higher value between 80 and 100Hz. 80 seems like a gold spot for 7-inch builds while 100 should work best with 5-inch machines. If motors are getting too hot, lower the value |
| dterm_lpf_type  | `BIQUAD`  | Defines the type of stage 1 D-term LPF filter. Possible values: `PT1`, `BIQUAD`. `PT1` offers faster filter response while `BIQUAD` better attenuation. |
| dterm_lpf2_hz | 0   | Cutoff frequency for stage 2 D-term low pass filter |
| dterm_lpf2_type | `BIQUAD` | Defines the type of stage 1 D-term LPF filter. Possible values: `PT1`, `BIQUAD`. `PT1` offers faster filter response while `BIQUAD` better attenuation.  |
|  yaw_lpf_hz  | 30 | Yaw low pass filter cutoff frequency. Should be disabled (set to `0`) on small multirotors (7 inches and below) |
| dynamic_gyro_notch_enabled | `OFF`    | Enable/disable dynamic gyro notch also known as Matrix Filter |
| dynamic_gyro_notch_range | `MEDIUM`   | Range for dynamic gyro notches. `MEDIUM` for 5", `HIGH` for 3" and `MEDIUM`/`LOW` for 7" and bigger propellers |
| dynamic_gyro_notch_q | 120       | Q factor for dynamic notches |
| dynamic_gyro_notch_min_hz  | 150       | Minimum frequency for dynamic notches. Default value of `150` works best with 5" multirors. Should be lowered with increased size of propellers. Values around `100` work fine on 7" drones. 10" can go down to `60` - `70` |
|  gyro_stage2_lowpass_hz  | 0 | Software based second stage lowpass filter for gyro. Value is cutoff frequency (Hz) |
|  gyro_stage2_lowpass_type  | `BIQUAD` | Defines the type of stage 2 gyro LPF filter. Possible values: `PT1`, `BIQUAD`. `PT1` offers faster filter response while `BIQUAD` better attenuation. |
| rpm_gyro_filter_enabled | `OFF`   | Enables gyro RPM filtere. Set to `ON` only when ESC telemetry is working and rotation speed of the motors is correctly reported to INAV |
| rpm_gyro_harmonics | 1 | Number of harmonic frequences to be covered by gyro RPM filter. Default value of `1` usually works just fine  |
| rpm_gyro_min_hz | 150 | The lowest frequency for gyro RPM filtere. Default `150` is fine for 5" mini-quads. On 7-inch drones you can lower even down to `60`-`70` |
| rpm_gyro_q | 500  | Q factor for gyro RPM filter. Lower values give softer, wider attenuation. Usually there is no need to change this setting |
|  pidsum_limit  | 500 | A limitation to overall amount of correction Flight PID can request on each axis (Roll/Pitch). If when doing a hard maneuver on one axis machine looses orientation on other axis - reducing this parameter may help |
|  pidsum_limit_yaw  | 400 | A limitation to overall amount of correction Flight PID can request on each axis (Yaw). If when doing a hard maneuver on one axis machine looses orientation on other axis - reducing this parameter may help |
| `pid_type`    | Allows to set type of PID controller used in control loop. Possible values: `NONE`, `PID`, `PIFF`, `AUTO`. Change only in case of experimental platforms like VTOL, tailsitters, rovers, boats, etc. Airplanes should always use `PIFF` and multirotors `PID` |
|  iterm_windup  | 50 | Used to prevent Iterm accumulation on during maneuvers. Iterm will be dampened when motors are reaching it's limit (when requested motor correction range is above percentage specified by this parameter) |
|  rate_accel_limit_roll_pitch  | 0 | Limits acceleration of ROLL/PITCH rotation speed that can be requested by stick input. In degrees-per-second-squared. Small and powerful UAV flies great with high acceleration limit ( > 5000 dps^2 and even > 10000 dps^2). Big and heavy multirotors will benefit from low acceleration limit (~ 360 dps^2). When set correctly, it greatly improves stopping performance. Value of 0 disables limiting.  |
|  rate_accel_limit_yaw  | 10000 | Limits acceleration of YAW rotation speed that can be requested by stick input. In degrees-per-second-squared. Small and powerful UAV flies great with high acceleration limit ( > 10000 dps^2). Big and heavy multirotors will benefit from low acceleration limit (~ 180 dps^2). When set correctly, it greatly improves stopping performance and general stability during yaw turns. Value of 0 disables limiting. |
|  rc_expo  | 70 | Exposition value used for the PITCH/ROLL axes by all the stabilized flights modes (all but `MANUAL`) |
|  rc_yaw_expo  | 20 | Exposition value used for the YAW axis by all the stabilized flights modes (all but `MANUAL`)  |
|  manual_rc_expo  | 70 | Exposition value used for the PITCH/ROLL axes by the `MANUAL` flight mode [0-100] |
|  manual rc_yaw_expo  | 20 | Exposition value used for the YAW axis by the `MANUAL` flight mode [0-100] |
|  thr_mid  | 50 | Throttle value when the stick is set to mid-position. Used in the throttle curve calculation. |
|  thr_expo  | 0 | Throttle exposition value |
|  roll_rate  | 20 | Defines rotation rate on ROLL axis that UAV will try to archive on max. stick deflection. Rates are defined in tens of degrees  (deca-degrees) per second [rate = dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  pitch_rate  | 20 | Defines rotation rate on PITCH axis that UAV will try to archive on max. stick deflection. Rates are defined in tens of degrees (deca-degrees) per second [rate = dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  yaw_rate  | 20 | Defines rotation rate on YAW axis that UAV will try to archive on max. stick deflection. Rates are defined in tens of degrees  (deca-degrees) per second [rate = dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  manual_pitch_rate  | 100 | Servo travel multiplier for the PITCH axis in `MANUAL` flight mode [0-100]% |
|  manual_roll_rate  | 100 | Servo travel multiplier for the ROLL axis in `MANUAL` flight mode [0-100]% |
|  manual_yaw_rate  | 100 | Servo travel multiplier for the YAW axis in `MANUAL` flight mode [0-100]% |
|  tpa_rate  | 0 | Throttle PID attenuation reduces influence of P on ROLL and PITCH as throttle increases. For every 1% throttle after the TPA breakpoint, P is reduced by the TPA rate. |
|  tpa_breakpoint  | 1500 | See tpa_rate. |
|  fw_tpa_time_constant  | 0 | TPA smoothing and delay time constant to reflect non-instant speed/throttle response of the plane. Planes with low thrust/weight ratio generally need higher time constant. Default is zero for compatibility with old setups |
|  fw_autotune_overshoot_time  | 100 | Time [ms] to detect sustained overshoot |
|  fw_autotune_undershoot_time | 200 | Time [ms] to detect sustained undershoot |
|  fw_autotune_threshold       | 50  | Threshold [%] of max rate to consider overshoot/undershoot detection |
|  fw_autotune_ff_to_p_gain    | 10  | FF to P gain (strength relationship) [%] |
|  fw_autotune_ff_to_i_tc      | 600 | FF to I time (defines time for I to reach the same level of response as FF) [ms]  |
|  stats                       | OFF | General switch of the statistics recording feature (a.k.a. odometer) |
|  stats_total_time            |  0  | Total flight time [in seconds]. The value is updated on every disarm when "stats" are enabled. |
|  stats_total_dist            |  0  | Total flight distance [in meters]. The value is updated on every disarm when "stats" are enabled. |
|  vbat_adc_channel            |  -  | ADC channel to use for battery voltage sensor. Defaults to board VBAT input (if available). 0 = disabled |
|  rssi_adc_channel            |  -  | ADC channel to use for analog RSSI input. Defaults to board RSSI input (if available). 0 = disabled |
|  current_adc_channel         |  -  | ADC channel to use for analog current sensor input. Defaults to board CURRENT sensor input (if available). 0 = disabled |
|  airspeed_adc_channel        |  0  | ADC channel to use for analog pitot tube (airspeed) sensor. If board doesn't have a dedicated connector for analog airspeed sensor will default to 0 |
|  platform_type        |  "MULTIROTOR"  | Defines UAV platform type. Allowed values: "MULTIROTOR", "AIRPLANE", "HELICOPTER", "TRICOPTER", "ROVER", "BOAT". Currently only MULTIROTOR, AIRPLANE and TRICOPTER types are implemented |
|  has_flaps        |  OFF  | Defines is UAV is capable of having flaps. If ON and AIRPLANE `platform_type` is used, **FLAPERON** flight mode will be available for the pilot  |
|  model_preview_type        |  -1  | ID of mixer preset applied in a Configurator. **Do not modify manually**. Used only for backup/restore reasons.  |
|  tz_offset  | 0 | Time zone offset from UTC, in minutes. This is applied to the GPS time for logging and time-stamping of Blackbox logs |
|  tz_automatic_dst  | OFF | Automatically add Daylight Saving Time to the GPS time when needed or simply ignore it. Includes presets for EU and the USA - if you live outside these areas it is suggested to manage DST manually via `tz_offset`.  |
|  vtx_band  | 4 | Configure the VTX band. Set to zero to use `vtx_freq`. Bands: 1: A, 2: B, 3: E, 4: F, 5: Race. |
|  vtx_channel  | 1 | Channel to use within the configured `vtx_band`. Valid values are [1, 8]. |
|  vtx_freq  | 5740 | Set the VTX frequency using raw MHz. This parameter is ignored unless `vtx_band` is 0. |
|  vtx_halfduplex  | ON | Use half duplex UART to communicate with the VTX, using only a TX pin in the FC. |
|  vtx_low_power_disarm  | OFF | When the craft is disarmed, set the VTX to its lowest power. `ON` will set the power to its minimum value on startup, increase it to `vtx_power` when arming and change it back to its lowest setting after disarming. `UNTIL_FIRST_ARM` will start with minimum power, but once the craft is armed it will increase to `vtx_power` and it will never decrease until the craft is power cycled. |
|  vtx_pit_mode_freq  | Frequency to use (in MHz) when the VTX is in pit mode. |
|  vtx_power  | 1 | VTX RF power level to use. The exact number of mw depends on the VTX hardware. |
| motor_accel_time | 0 | Minimum time for the motor(s) to accelerate from 0 to 100% throttle (ms) [0-1000] |
| motor_decel_time | 0 | Minimum time for the motor(s) to deccelerate from 100 to 0% throttle (ms) [0-1000] |
| thr_comp_weight | 0.692 | Weight used for the throttle compensation based on battery voltage. See the [battery documentation](Battery.md#automatic-throttle-compensation-based-on-battery-voltage) |
| nav_mc_braking_speed_threshold | 100 | min speed in cm/s above which braking can happen |
| nav_mc_braking_disengage_speed | 75 | braking is disabled when speed goes below this value |
| nav_mc_braking_timeout | 2000 | timeout in ms for braking |
| nav_mc_braking_boost_factor | 100 | acceleration factor for BOOST phase |
| nav_mc_braking_boost_timeout | 750 | how long in ms BOOST phase can happen |
| nav_mc_braking_boost_speed_threshold | 150 | BOOST can be enabled when speed is above this value |
| nav_mc_braking_boost_disengage_speed | 100 | BOOST will be disabled when speed goes below this value |
| nav_mc_braking_bank_angle | 40 | max angle that MR is allowed to bank in BOOST mode |
| nav_mc_pos_deceleration_time | 120 | Used for stoping distance calculation. Stop position is computed as _speed_ * _nav_mc_pos_deceleration_time_ from the place where sticks are released. Braking mode overrides this setting |
| nav_mc_pos_expo | 10 | Expo for PosHold control |
| osd_artificial_horizon_max_pitch | 20 | Max pitch, in degrees, for OSD artificial horizon |
| baro_cal_tolerance | 150 | Baro calibration tolerance in cm. The default  should allow the noisiest baro to complete calibration [cm]. |
| mc_airmode_type | STICK_CENTER | Defines the Airmode state handling type for Multirotors. Default **STICK_CENTER** is the classical approach in which Airmode is always active if enabled, but when the throttle is low and ROLL/PITCH/YAW sticks are centered, Iterms is not allowed to grow (ANTI_WINDUP). **THROTTLE_THRESHOLD** is the Airmode behavior known from Betaflight. In this mode, Airmode is active as soon THROTTLE position is above `mc_airmode_threshold` and stays active until disarm. ANTI_WINDUP is never triggered. For small Multirotors (up to 7-inch propellers) it is suggested to switch to **THROTTLE_THRESHOLD** since it keeps full stabilization no matter what pilot does with the sticks. Fixed Wings always use **STICK_CENTER** mode. |
| mc_airmode_threshold | 1300 | Defines airmode THROTTLE activation threshold when `mc_airmode_type` **THROTTLE_THRESHOLD** is used |
| use_dterm_fir_filter | ON | Setting to **OFF** disabled extra filter on Dterm. **OFF** offers faster Dterm and better inflight performance with a cost of being more sensitive to gyro noise. Small and relatively clean multirotors (7 inches and below) are suggested to use **OFF** setting. If motors are getting too hot, switch back to **ON** |
| sim_ground_station_number | Empty string | Number of phone that is used to communicate with SIM module. Messages / calls from other numbers are ignored. If undefined, can be set by calling or sending a message to the module. |
| sim_pin | Empty string | PIN code for the SIM module |
| sim_transmit_interval | 60 | Text message transmission interval in seconds for SIM module. Minimum value: 10 |
| sim_transmit_flags | F | String specifying text message transmit condition flags for the SIM module. Flags can be given in any order. Empty string means the module only sends response messages. `A`: acceleration events, `T`: continuous transmission, `F`: continuous transmission in failsafe mode, `L`: continuous transmission when altitude is below `sim_low_altitude`, `G`: continuous transmission when GPS signal quality is low |
| acc_event_threshold_high | 0 | Acceleration threshold [cm/s/s] for impact / high g event text messages sent by SIM module. Acceleration values greater than 4 g can occur in fixed wing flight without an impact, so a setting of 4000 or greater is suggested. 0 = detection off. |
| acc_event_threshold_low | 0 | Acceleration threshold [cm/s/s] for low-g / freefall detection text messages sent by SIM module. A setting of less than 100 is suggested. Valid values: [0-900], 0 = detection off. |
| acc_event_threshold_neg_x | 0 | Acceleration threshold [cm/s/s] for backwards acceleration / fixed wing landing detection text messages sent by SIM module. Suggested value for fixed wing: 1100. 0 = detection off. |
| sim_low_altitude | 0 | Threshold for low altitude warning messages sent by SIM module when the 'L' transmit flag is set in `sim_transmit_flags`.|
| rssi_source       | `AUTO`    | Source of RSSI input. Possible values: `NONE`, `AUTO`, `ADC`, `CHANNEL`, `PROTOCOL`, `MSP` |
| throttle_scale    | 1.000     | Throttle scaling factor. `1` means no throttle scaling. `0.5` means throttle scaled down by 50% |
| vbat_meter_type   | `ADC`     | Vbat voltage source. Possible values: `NONE`, `ADC`, `ESC`. `ESC` required ESC telemetry enebled and running |
| antigravity_gain  | 1         | Max Antigravity gain. `1` means Antigravity is disabled, `2` means Iterm is allowed to double during rapid throttle movements |
| antigravity_accelerator | 1   | |
| antigravity_cutoff_lpf_hz | 15    | Antigravity cutoff frequenct for Throtte filter. Antigravity is based on the difference between actual and filtered throttle input. The bigger is the difference, the bigger Antigravity gain |
| sim_pin   |   | PIN for GSM card module |
