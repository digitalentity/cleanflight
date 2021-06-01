include(stm32-bootloader)
include(stm32f3)
include(stm32f4)
include(stm32f7)
include(stm32h7)

include(CMakeParseArguments)

option(DEBUG_HARDFAULTS "Enable debugging of hard faults via custom handler")
option(SEMIHOSTING "Enable semihosting")

message("-- DEBUG_HARDFAULTS: ${DEBUG_HARDFAULTS}, SEMIHOSTING: ${SEMIHOSTING}")

set(CMSIS_DIR "${MAIN_LIB_DIR}/main/CMSIS")
set(CMSIS_INCLUDE_DIR "${CMSIS_DIR}/Core/Include")
set(CMSIS_DSP_DIR "${MAIN_LIB_DIR}/main/CMSIS/DSP")
set(CMSIS_DSP_INCLUDE_DIR "${CMSIS_DSP_DIR}/Include")

set(CMSIS_DSP_SRC
    BasicMathFunctions/arm_mult_f32.c
    TransformFunctions/arm_rfft_fast_f32.c
    TransformFunctions/arm_cfft_f32.c
    TransformFunctions/arm_rfft_fast_init_f32.c
    TransformFunctions/arm_cfft_radix8_f32.c
    TransformFunctions/arm_bitreversal2.S
    CommonTables/arm_common_tables.c
    ComplexMathFunctions/arm_cmplx_mag_f32.c
    StatisticsFunctions/arm_max_f32.c
)
list(TRANSFORM CMSIS_DSP_SRC PREPEND "${CMSIS_DSP_DIR}/Source/")

set(STM32_STARTUP_DIR "${MAIN_SRC_DIR}/startup")

main_sources(STM32_VCP_SRC
    drivers/serial_usb_vcp.c
    drivers/usb_io.c
)

main_sources(STM32_SDCARD_SRC
    drivers/sdcard/sdcard.c
    drivers/sdcard/sdcard_spi.c
    drivers/sdcard/sdcard_sdio.c
    drivers/sdcard/sdcard_standard.c
)

# XXX: This code is not STM32 specific
main_sources(STM32_ASYNCFATFS_SRC
    io/asyncfatfs/asyncfatfs.c
    io/asyncfatfs/fat_standard.c
)

main_sources(STM32_MSC_SRC
    msc/usbd_storage.c
)

main_sources(STM32_MSC_FLASH_SRC
    msc/usbd_storage_emfat.c
    msc/emfat.c
    msc/emfat_file.c
)

main_sources(STM32_MSC_SDCARD_SRC
    msc/usbd_storage_sd_spi.c
)

set(STM32_INCLUDE_DIRS
    "${CMSIS_INCLUDE_DIR}"
    "${CMSIS_DSP_INCLUDE_DIR}"
    "${MAIN_SRC_DIR}/target"
)

set(STM32_DEFINITIONS
)
set(STM32_DEFAULT_HSE_MHZ 8)
set(STM32_LINKER_DIR "${MAIN_SRC_DIR}/target/link")
set(MICROROS_LINKER_DIR "${MAIN_LIB_DIR}/main/micro_ros_arduino/src/cortex-m7/fpv5-sp-d16-hardfp")

set(STM32_COMPILE_OPTIONS
    -ffunction-sections
    -fdata-sections
    -fno-common
)

set(STM32_LINK_LIBRARIES
    -lm
    -lc
    -lmicroros
)

if(SEMIHOSTING)
    list(APPEND STM32_LINK_LIBRARIES --specs=rdimon.specs -lrdimon)
    list(APPEND STM32_DEFINITIONS SEMIHOSTING)
else()
    list(APPEND STM32_LINK_LIBRARIES -lnosys)
endif()

set(STM32_LINK_OPTIONS
    -nostartfiles
    --specs=nano.specs
    -static
    -Wl,-gc-sections
    -Wl,-L${STM32_LINKER_DIR}
    -Wl,-L${MICROROS_LINKER_DIR}
    -Wl,--cref
    -Wl,--no-wchar-size-warning
    -Wl,--print-memory-usage
)

macro(get_stm32_target_features output_var dir target_name)
    execute_process(COMMAND "${CMAKE_C_COMPILER}" -E -dD -D${ARGV2} "${ARGV1}/target.h"
        ERROR_VARIABLE _errors
        RESULT_VARIABLE _result
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE _contents)

    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "error extracting features for stm32 target ${ARGV2}: ${_errors}")
    endif()

    string(REGEX MATCH "#define[\t ]+USE_VCP" HAS_VCP ${_contents})
    if(HAS_VCP)
        list(APPEND ${ARGV0} VCP)
    endif()
    string(REGEX MATCH "define[\t ]+USE_FLASHFS" HAS_FLASHFS ${_contents})
    if(HAS_FLASHFS)
        list(APPEND ${ARGV0} FLASHFS)
    endif()
    string(REGEX MATCH "define[\t ]+USE_SDCARD" HAS_SDCARD ${_contents})
    if (HAS_SDCARD)
        list(APPEND ${ARGV0} SDCARD)
        string(REGEX MATCH "define[\t ]+USE_SDCARD_SDIO" HAS_SDIO ${_contents})
        if (HAS_SDIO)
            list(APPEND ${ARGV0} SDIO)
        endif()
    endif()
    if(HAS_FLASHFS OR HAS_SDCARD)
        list(APPEND ${ARGV0} MSC)
    endif()
endmacro()

function(get_stm32_flash_size out size)
    # 4: 16, 6: 32, 8: 64, B: 128, C: 256, D: 384, E: 512, F: 768, G: 1024, H: 1536, I: 2048 KiB
    string(TOUPPER ${size} s)
    if(${s} STREQUAL "4")
        set(${out} 16 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "6")
        set(${out} 32 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "8")
        set(${out} 64 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "8")
        set(${out} 64 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "B")
        set(${out} 128 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "C")
        set(${out} 256 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "D")
        set(${out} 384 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "E")
        set(${out} 512 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "F")
        set(${out} 768 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "G")
        set(${out} 1024 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "H")
        set(${out} 1536 PARENT_SCOPE)
        return()
    endif()
    if(${s} STREQUAL "I")
        set(${out} 2048 PARENT_SCOPE)
        return()
    endif()
endfunction()

function(add_hex_target name exe hex)
    add_custom_target(${name} ALL
        cmake -E env PATH=$ENV{PATH}
        # TODO: Overriding the start address with --set-start 0x08000000
        # seems to be required due to some incorrect assumptions about .hex
        # files in the configurator. Verify wether that's the case and fix
        # the bug in configurator or delete this comment.
        ${CMAKE_OBJCOPY} -Oihex --set-start 0x08000000 $<TARGET_FILE:${exe}> ${hex}
        BYPRODUCTS ${hex}
    )
endfunction()

function(add_bin_target name exe bin)
    add_custom_target(${name}
        cmake -E env PATH=$ENV{PATH}
        ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${exe}> ${bin}
        BYPRODUCTS ${bin}
    )
endfunction()

function(generate_map_file target)
    if(CMAKE_VERSION VERSION_LESS 3.15)
        set(map "$<TARGET_FILE:${target}>.map")
    else()
        set(map "$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map")
    endif()
    target_link_options(${target} PRIVATE "-Wl,-Map,${map}")
endfunction()

function(set_linker_script target script)
    set(script_path ${STM32_LINKER_DIR}/${args_LINKER_SCRIPT}.ld)
    if(NOT EXISTS ${script_path})
        message(FATAL_ERROR "linker script ${script_path} doesn't exist")
    endif()
    set_target_properties(${target} PROPERTIES LINK_DEPENDS ${script_path})
    target_link_options(${elf_target} PRIVATE -T${script_path})
endfunction()

function(add_stm32_executable)
    cmake_parse_arguments(
        args
        # Boolean arguments
        ""
        # Single value arguments
        "FILENAME;NAME;OPTIMIZATION;OUTPUT_BIN_FILENAME;OUTPUT_HEX_FILENAME;OUTPUT_TARGET_NAME"
        # Multi-value arguments
        "COMPILE_DEFINITIONS;COMPILE_OPTIONS;INCLUDE_DIRECTORIES;LINK_OPTIONS;LINKER_SCRIPT;SOURCES"
        # Start parsing after the known arguments
        ${ARGN}
    )
    set(elf_target ${args_NAME}.elf)
    add_executable(${elf_target})
    target_sources(${elf_target} PRIVATE ${args_SOURCES})
    target_include_directories(${elf_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${args_INCLUDE_DIRECTORIES} ${STM32_INCLUDE_DIRS})
    target_compile_definitions(${elf_target} PRIVATE ${args_COMPILE_DEFINITIONS})
    target_compile_options(${elf_target} PRIVATE ${STM32_COMPILE_OPTIONS} ${args_COMPILE_OPTIONS})
    if(WARNINGS_AS_ERRORS)
        target_compile_options(${elf_target} PRIVATE -Werror)
    endif()
    if (IS_RELEASE_BUILD)
        target_compile_options(${elf_target} PRIVATE ${args_OPTIMIZATION})
        target_link_options(${elf_target} PRIVATE ${args_OPTIMIZATION})
    endif()
    target_link_libraries(${elf_target} PRIVATE ${STM32_LINK_LIBRARIES})
    target_link_options(${elf_target} PRIVATE ${STM32_LINK_OPTIONS} ${args_LINK_OPTIONS})
    generate_map_file(${elf_target})
    set_linker_script(${elf_target} ${args_LINKER_SCRIPT})
    if(args_FILENAME)
        set(basename ${CMAKE_BINARY_DIR}/${args_FILENAME})
        set(hex_filename ${basename}.hex)
        add_hex_target(${args_NAME} ${elf_target} ${hex_filename})
        set(bin_filename ${basename}.bin)
        add_bin_target(${args_NAME}.bin ${elf_target} ${bin_filename})
    endif()
    if(args_OUTPUT_BIN_FILENAME)
        set(${args_OUTPUT_BIN_FILENAME} ${bin_filename} PARENT_SCOPE)
    endif()
    if(args_OUTPUT_TARGET_NAME)
        set(${args_OUTPUT_TARGET_NAME} ${elf_target} PARENT_SCOPE)
    endif()
    if(args_OUTPUT_HEX_FILENAME)
        set(${args_OUTPUT_HEX_FILENAME} ${hex_filename} PARENT_SCOPE)
    endif()
endfunction()

function(target_stm32)
    if(NOT arm-none-eabi STREQUAL TOOLCHAIN)
        return()
    endif()
    # Parse keyword arguments
    cmake_parse_arguments(
        args
        # Boolean arguments
        "DISABLE_MSC;BOOTLOADER"
        # Single value arguments
        "HSE_MHZ;LINKER_SCRIPT;NAME;OPENOCD_TARGET;OPTIMIZATION;STARTUP;SVD"
        # Multi-value arguments
        "COMPILE_DEFINITIONS;COMPILE_OPTIONS;INCLUDE_DIRECTORIES;LINK_OPTIONS;SOURCES;MSC_SOURCES;MSC_INCLUDE_DIRECTORIES;VCP_SOURCES;VCP_INCLUDE_DIRECTORIES"
        # Start parsing after the known arguments
        ${ARGN}
    )
    set(name ${args_NAME})

    if (args_HSE_MHZ)
        set(hse_mhz ${args_HSE_MHZ})
    else()
        set(hse_mhz ${STM32_DEFAULT_HSE_MHZ})
    endif()

    set(target_sources ${STM32_STARTUP_DIR}/${args_STARTUP})
    list(APPEND target_sources ${args_SOURCES})
    file(GLOB target_c_sources "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
    file(GLOB target_h_sources "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    list(APPEND target_sources ${target_c_sources} ${target_h_sources})

    set(target_include_directories ${args_INCLUDE_DIRECTORIES})

    set(target_definitions ${STM32_DEFINITIONS} ${COMMON_COMPILE_DEFINITIONS})

    get_stm32_target_features(features "${CMAKE_CURRENT_SOURCE_DIR}" ${name})
    set_property(TARGET ${elf_target} PROPERTY FEATURES ${features})

    if(VCP IN_LIST features)
        list(APPEND target_sources ${STM32_VCP_SRC} ${args_VCP_SOURCES})
        list(APPEND target_include_directories ${args_VCP_INCLUDE_DIRECTORIES})
    endif()
    if(SDCARD IN_LIST features)
        list(APPEND target_sources ${STM32_SDCARD_SRC} ${STM32_ASYNCFATFS_SRC})
    endif()

    set(msc_sources)
    if(NOT args_DISABLE_MSC AND MSC IN_LIST features)
        list(APPEND target_include_directories ${args_MSC_INCLUDE_DIRECTORIES})
        list(APPEND msc_sources ${STM32_MSC_SRC} ${args_MSC_SOURCES})
        list(APPEND target_definitions USE_USB_MSC)
        if(FLASHFS IN_LIST features)
            list(APPEND msc_sources ${STM32_MSC_FLASH_SRC})
        endif()
        if (SDCARD IN_LIST features)
            list(APPEND msc_sources ${STM32_MSC_SDCARD_SRC})
        endif()
    endif()

    math(EXPR hse_value "${hse_mhz} * 1000000")
    list(APPEND target_definitions "HSE_VALUE=${hse_value}")
    if(args_COMPILE_DEFINITIONS)
        list(APPEND target_definitions ${args_COMPILE_DEFINITIONS})
    endif()
    if(DEBUG_HARDFAULTS)
        list(APPEND target_definitions DEBUG_HARDFAULTS)
    endif()

    string(TOLOWER ${PROJECT_NAME} lowercase_project_name)
    set(binary_name ${lowercase_project_name}_${FIRMWARE_VERSION}_${name})
    if(DEFINED BUILD_SUFFIX AND NOT "" STREQUAL "${BUILD_SUFFIX}")
        set(binary_name "${binary_name}_${BUILD_SUFFIX}")
    endif()

    # Main firmware
    add_stm32_executable(
        NAME ${name}
        FILENAME ${binary_name}
        SOURCES ${target_sources} ${msc_sources} ${CMSIS_DSP_SRC} ${COMMON_SRC}
        COMPILE_DEFINITIONS ${target_definitions}
        COMPILE_OPTIONS ${args_COMPILE_OPTIONS}
        INCLUDE_DIRECTORIES ${target_include_directories}
        LINK_OPTIONS ${args_LINK_OPTIONS}
        LINKER_SCRIPT ${args_LINKER_SCRIPT}
        OPTIMIZATION ${args_OPTIMIZATION}

        OUTPUT_HEX_FILENAME main_hex_filename
        OUTPUT_TARGET_NAME main_target_name
    )

    set_property(TARGET ${main_target_name} PROPERTY OPENOCD_TARGET ${args_OPENOCD_TARGET})
    set_property(TARGET ${main_target_name} PROPERTY OPENOCD_DEFAULT_INTERFACE stlink)
    set_property(TARGET ${main_target_name} PROPERTY SVD ${args_SVD})

    setup_firmware_target(${main_target_name} ${name} ${ARGN})

    if(args_BOOTLOADER)
        # Bootloader for the target
        set(bl_suffix _bl)
        add_stm32_executable(
            NAME ${name}${bl_suffix}
            FILENAME ${binary_name}${bl_suffix}
            SOURCES ${target_sources} ${BOOTLOADER_SOURCES}
            COMPILE_DEFINITIONS ${target_definitions} BOOTLOADER MSP_FIRMWARE_UPDATE
            COMPILE_OPTIONS ${args_COMPILE_OPTIONS}
            INCLUDE_DIRECTORIES ${target_include_directories}
            LINK_OPTIONS ${args_LINK_OPTIONS}
            LINKER_SCRIPT ${args_LINKER_SCRIPT}${bl_suffix}
            OPTIMIZATION ${args_OPTIMIZATION}

            OUTPUT_BIN_FILENAME bl_bin_filename
            OUTPUT_HEX_FILENAME bl_hex_filename
            OUTPUT_TARGET_NAME bl_target_name
        )
        setup_executable(${bl_target_name} ${name})

        # Main firmware, but for running with the bootloader
        set(for_bl_suffix _for_bl)
        add_stm32_executable(
            NAME ${name}${for_bl_suffix}
            FILENAME ${binary_name}${for_bl_suffix}
            SOURCES ${target_sources} ${msc_sources} ${CMSIS_DSP_SRC} ${COMMON_SRC}
            COMPILE_DEFINITIONS ${target_definitions} MSP_FIRMWARE_UPDATE
            COMPILE_OPTIONS ${args_COMPILE_OPTIONS}
            INCLUDE_DIRECTORIES ${target_include_directories}
            LINK_OPTIONS ${args_LINK_OPTIONS}
            LINKER_SCRIPT ${args_LINKER_SCRIPT}${for_bl_suffix}
            OPTIMIZATION ${args_OPTIMIZATION}

            OUTPUT_BIN_FILENAME for_bl_bin_filename
            OUTPUT_HEX_FILENAME for_bl_hex_filename
            OUTPUT_TARGET_NAME for_bl_target_name
        )
        setup_executable(${for_bl_target_name} ${name})

        # Combined with bootloader and main firmware
        set(with_bl_suffix _with_bl)
        set(combined_hex ${CMAKE_BINARY_DIR}/${binary_name}${with_bl_suffix}.hex)
        set(with_bl_target ${name}${with_bl_suffix})
        add_custom_target(${with_bl_target}
            ${CMAKE_SOURCE_DIR}/src/utils/combine_tool ${bl_bin_filename} ${for_bl_bin_filename} ${combined_hex}
            BYPRODUCTS ${combined_hex}
        )
        add_dependencies(${with_bl_target} ${bl_target_name} ${for_bl_target_name})
    endif()

    # clean_<target>
    set(generator_cmd "")
    if (CMAKE_GENERATOR STREQUAL "Unix Makefiles")
        set(generator_cmd "make")
    elseif(CMAKE_GENERATOR STREQUAL "Ninja")
        set(generator_cmd "ninja")
    endif()
    if (NOT generator_cmd STREQUAL "")
        set(clean_target "clean_${name}")
        add_custom_target(${clean_target}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMAND ${generator_cmd} clean
            COMMENT "Removing intermediate files for ${name}")
        set_property(TARGET ${clean_target} PROPERTY
            EXCLUDE_FROM_ALL 1
            EXCLUDE_FROM_DEFAULT_BUILD 1)
    endif()
endfunction()
