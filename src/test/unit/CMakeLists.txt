# XXX: This should come from main project once everything
# uses cmake
set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/main")

# Keep these alphabetically sorted by test name

set_property(SOURCE alignsensor_unittest.cc PROPERTY depends
    "common/maths.c" "sensors/boardalignment.c")

set_property(SOURCE bitarray_unittest.cc PROPERTY depends "common/bitarray.c")

set_property(SOURCE maths_unittest.cc PROPERTY depends "common/maths.c")

set_property(SOURCE olc_unittest.cc PROPERTY depends "common/olc.c")

set_property(SOURCE rcdevice_unittest.cc PROPERTY definitions USE_RCDEVICE)
set_property(SOURCE rcdevice_unittest.cc PROPERTY depends
    "common/bitarray.c" "common/crc.c" "io/rcdevice.c" "io/rcdevice_cam.c"
    "fc/rc_modes.c" "common/maths.c")

set_property(SOURCE sensor_gyro_unittest.cc PROPERTY depends
    "build/debug.c" "common/maths.c" "common/calibration.c" "common/filter.c"
    "drivers/accgyro/accgyro_fake.c" "sensors/gyro.c" "sensors/boardalignment.c")

set_property(SOURCE telemetry_hott_unittest.cc PROPERTY depends
    "telemetry/hott.c" "common/gps_conversion.c" "common/string_light.c")

set_property(SOURCE time_unittest.cc PROPERTY depends "drivers/time.c")

set_property(SOURCE circular_queue_unittest.cc PROPERTY depends "common/circular_queue.c")

function(unit_test src)
    get_filename_component(basename ${src} NAME)
    string(REPLACE ".cc" "" name ${basename} )
    get_property(deps SOURCE ${src} PROPERTY depends)
    set(headers "${deps}")
    list(TRANSFORM headers REPLACE "\.c$" ".h")
    list(APPEND deps ${headers})
    get_property(defs SOURCE ${src} PROPERTY definitions)
    set(test_definitions "UNIT_TEST")
    if (defs)
        list(APPEND test_definitions ${defs})
    endif()
    list(TRANSFORM deps PREPEND "${MAIN_DIR}/")
    add_executable(${name} ${src} ${deps})
    set(gen_name ${name}_gen)
    get_generated_files_dir(gen ${gen_name})
    target_include_directories(${name} PRIVATE . ${MAIN_DIR} ${gen})
    target_compile_definitions(${name} PRIVATE ${test_definitions})
    target_compile_options(${name} PRIVATE -pthread -Wall -Wextra -Wno-extern-c-compat -ggdb3 -O0)
    enable_settings(${name} ${gen_name} OUTPUTS setting_files SETTINGS_CXX g++)
    target_sources(${name} PRIVATE ${setting_files})
    target_link_libraries(${name} gtest_main)
    gtest_discover_tests(${name})
    add_custom_target("run-${name}" "${name}" DEPENDS ${name})
    set(test_targets ${test_targets} "${name}" PARENT_SCOPE)
endfunction()

file(GLOB TEST_PROGRAMS *_unittest.cc)
foreach(source ${TEST_PROGRAMS})
    unit_test(${source})
endforeach()

add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS ${test_targets})
