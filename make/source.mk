COMMON_SRC = \
            $(TARGET_DIR_SRC) \
            main.c \
            target/common_hardware.c \
            build/assert.c \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            common/bitarray.c \
            common/calibration.c \
            common/colorconversion.c \
            common/crc.c \
            common/encoding.c \
            common/filter.c \
            common/gps_conversion.c \
            common/log.c \
            common/logic_condition.c \
            common/maths.c \
            common/memory.c \
            common/olc.c \
            common/printf.c \
            common/streambuf.c \
            common/time.c \
            common/typeconversion.c \
            common/string_light.c \
            config/config_eeprom.c \
            config/config_streamer.c \
            config/feature.c \
            config/parameter_group.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_busdev_i2c.c \
            drivers/bus_busdev_spi.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/display.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/io_pca9685.c \
            drivers/light_led.c \
            drivers/resource.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pitotmeter_adc.c \
            drivers/pitotmeter_virtual.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_mapping.c \
            drivers/pwm_output.c \
            drivers/pinio.c \
            drivers/rcc.c \
            drivers/rx_pwm.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            drivers/lights_io.c \
            drivers/1-wire.c \
            drivers/1-wire/ds_crc.c \
            drivers/1-wire/ds2482.c \
            drivers/temperature/ds18b20.c \
            drivers/temperature/lm75.c \
            drivers/pitotmeter_ms4525.c \
            fc/cli.c \
            fc/config.c \
            fc/controlrate_profile.c \
            fc/fc_core.c \
            fc/fc_init.c \
            fc/fc_tasks.c \
            fc/fc_hardfaults.c \
            fc/fc_msp.c \
            fc/fc_msp_box.c \
            fc/rc_smoothing.c \
            fc/rc_adjustments.c \
            fc/rc_command.c \
            fc/rc_control.c \
            fc/rc_controls.c \
            fc/rc_curves.c \
            fc/rc_modes.c \
            fc/runtime_config.c \
            fc/settings.c \
            fc/stats.c \
            flight/failsafe.c \
            flight/hil.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/pid_autotune.c \
            flight/rth_estimator.c \
            flight/servos.c \
            flight/wind_estimator.c \
            io/beeper.c \
            io/lights.c \
            io/pwmdriver_i2c.c \
            io/esc_serialshot.c \
            io/piniobox.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/statusindicator.c \
            io/rcdevice.c \
            io/rcdevice_cam.c \
            msp/msp_serial.c \
            rx/crsf.c \
            rx/eleres.c \
            rx/fport.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/msp_override.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c \
            rx/pwm.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            rx/spektrum.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/uib_rx.c \
            rx/xbus.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/battery.c \
            sensors/temperature.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/diagnostics.c \
            sensors/gyro.c \
            sensors/initialisation.c \
            uav_interconnect/uav_interconnect_bus.c \
            uav_interconnect/uav_interconnect_rangefinder.c \
            blackbox/blackbox.c \
            blackbox/blackbox_encoding.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_battery.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_mixer_servo.c \
            cms/cms_menu_navigation.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            cms/cms_menu_vtx_ffpv.c \
            drivers/display_ug2864hsweg01.c \
            drivers/rangefinder/rangefinder_hcsr04.c \
            drivers/rangefinder/rangefinder_hcsr04_i2c.c \
            drivers/rangefinder/rangefinder_srf10.c \
            drivers/rangefinder/rangefinder_vl53l0x.c \
            drivers/rangefinder/rangefinder_virtual.c \
            drivers/opflow/opflow_fake.c \
            drivers/opflow/opflow_virtual.c \
            drivers/vtx_common.c \
            io/rangefinder_msp.c \
            io/rangefinder_benewake.c \
            io/opflow_cxof.c \
            io/opflow_msp.c \
            io/dashboard.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/displayport_hott.c \
            io/gps.c \
            io/gps_ublox.c \
            io/gps_nmea.c \
            io/gps_naza.c \
            io/ledstrip.c \
            io/osd_hud.c \
            io/osd.c \
            navigation/navigation.c \
            navigation/navigation_fixedwing.c \
            navigation/navigation_fw_launch.c \
            navigation/navigation_geo.c \
            navigation/navigation_multicopter.c \
            navigation/navigation_pos_estimator.c \
            navigation/navigation_pos_estimator_agl.c \
            navigation/navigation_pos_estimator_flow.c \
            sensors/barometer.c \
            sensors/pitotmeter.c \
            sensors/rangefinder.c \
            sensors/opflow.c \
            telemetry/crsf.c \
            telemetry/frsky.c \
            telemetry/frsky_d.c \
            telemetry/hott.c \
            telemetry/ibus_shared.c \
            telemetry/ibus.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/msp_shared.c \
            telemetry/smartport.c \
            telemetry/sim.c \
            telemetry/telemetry.c \
            io/vtx.c \
            io/vtx_string.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_ffpv24g.c \
            io/vtx_control.c

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

TARGET_SRC := $(STARTUP_SRC) $(COMMON_DEVICE_SRC) $(COMMON_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC)

#excludes
TARGET_SRC   := $(filter-out $(MCU_EXCLUDES), $(TARGET_SRC))

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
TARGET_SRC += \
            drivers/flash_m25p16.c \
            io/flashfs.c
endif

ifneq ($(filter SDCARD,$(FEATURES)),)
TARGET_SRC += \
            drivers/sdcard.c \
            drivers/sdcard_spi.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c
endif

ifneq ($(filter VCP,$(FEATURES)),)
TARGET_SRC += $(VCP_SRC)
endif

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
