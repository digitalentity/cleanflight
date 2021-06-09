include(cortex-m4f)
include(stm32-stdperiph)
include(stm32f4-usb)

set(STM32F4_STDPERIPH_DIR "${MAIN_LIB_DIR}/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver")
set(STM32F4_CMSIS_DEVICE_DIR "${MAIN_LIB_DIR}/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx")
set(STM32F4_CMSIS_DRIVERS_DIR "${MAIN_LIB_DIR}/main/STM32F4/Drivers/CMSIS")
set(STM32F4_VCP_DIR "${MAIN_SRC_DIR}/vcpf4")

set(STM32F4_STDPERIPH_SRC_EXCLUDES
    stm32f4xx_can.c
    stm32f4xx_cec.c
    stm32f4xx_crc.c
    stm32f4xx_cryp.c
    stm32f4xx_cryp_aes.c
    stm32f4xx_cryp_des.c
    stm32f4xx_cryp_tdes.c
    stm32f4xx_dbgmcu.c
    stm32f4xx_dsi.c
    stm32f4xx_flash_ramfunc.c
    stm32f4xx_fmpi2c.c
    stm32f4xx_fmc.c
    stm32f4xx_hash.c
    stm32f4xx_hash_md5.c
    stm32f4xx_hash_sha1.c
    stm32f4xx_lptim.c
    stm32f4xx_qspi.c
    stm32f4xx_sai.c
    stm32f4xx_spdifrx.c
)

set(STM32F4_STDPERIPH_SRC_DIR "${STM32F4_STDPERIPH_DIR}/src")
glob_except(STM32F4_STDPERIPH_SRC "${STM32F4_STDPERIPH_SRC_DIR}/*.c" "${STM32F4_STDPERIPH_SRC_EXCLUDES}")

main_sources(STM32F4_SRC
    target/system_stm32f4xx.c

    config/config_streamer_stm32f4.c

    drivers/adc_stm32f4xx.c
    drivers/adc_stm32f4xx.c
    drivers/bus_i2c_stm32f40x.c
    drivers/serial_uart_stm32f4xx.c
    drivers/system_stm32f4xx.c
    drivers/timer.c
    drivers/timer_impl_stdperiph.c
    drivers/timer_stm32f4xx.c
    drivers/uart_inverter.c
    drivers/dma_stm32f4xx.c
    drivers/sdcard/sdmmc_sdio_f4xx.c
)

set(STM32F4_VCP_SRC
    stm32f4xx_it.c
    usb_bsp.c
    usbd_desc.c
    usbd_usr.c
    usbd_cdc_vcp.c
)
list(TRANSFORM STM32F4_VCP_SRC PREPEND "${STM32F4_VCP_DIR}/")

main_sources(STM32F4_MSC_SRC
    drivers/usb_msc_f4xx.c
)

set(STM32F4_INCLUDE_DIRS
    "${CMSIS_INCLUDE_DIR}"
    "${CMSIS_DSP_INCLUDE_DIR}"
    "${STM32F4_STDPERIPH_DIR}/inc"
    "${STM32F4_CMSIS_DEVICE_DIR}"
    "${STM32F4_CMSIS_DRIVERS_DIR}"
    "${STM32F4_VCP_DIR}"
)

set(STM32F4_DEFINITIONS
    ${CORTEX_M4F_DEFINITIONS}
    STM32F4
    USE_STDPERIPH_DRIVER
)

function(target_stm32f4xx)
    target_stm32(
        SOURCES ${STM32_STDPERIPH_SRC} ${STM32F4_SRC}
        COMPILE_DEFINITIONS ${STM32F4_DEFINITIONS}
        COMPILE_OPTIONS ${CORTEX_M4F_COMMON_OPTIONS} ${CORTEX_M4F_COMPILE_OPTIONS}
        INCLUDE_DIRECTORIES ${STM32F4_INCLUDE_DIRS}
        LINK_OPTIONS ${CORTEX_M4F_COMMON_OPTIONS} ${CORTEX_M4F_LINK_OPTIONS}

        MSC_SOURCES ${STM32F4_USBMSC_SRC} ${STM32F4_MSC_SRC}
        VCP_SOURCES ${STM32F4_USB_SRC} ${STM32F4_VCP_SRC}
        VCP_INCLUDE_DIRECTORIES ${STM32F4_USB_INCLUDE_DIRS}

        OPTIMIZATION -O2

        OPENOCD_TARGET stm32f4x

        ${ARGN}
    )
endfunction()

set(STM32F405_COMPILE_DEFINITIONS
    STM32F40_41xxx
    STM32F405xx
    MCU_FLASH_SIZE=1024
)

function(target_stm32f405xg name)
    target_stm32f4xx(
        NAME ${name}
        STARTUP startup_stm32f40xx.s
        SOURCES ${STM32F4_STDPERIPH_SRC}
        COMPILE_DEFINITIONS ${STM32F405_COMPILE_DEFINITIONS}
        LINKER_SCRIPT stm32_flash_f405xg
        SVD STM32F405
        BOOTLOADER
        ${ARGN}
    )
endfunction()

set(STM32F411_OR_F427_STDPERIPH_SRC ${STM32F4_STDPERIPH_SRC})
set(STM32F411_OR_F427_STDPERIPH_SRC_EXCLUDES "stm32f4xx_fsmc.c")
exclude_basenames(STM32F411_OR_F427_STDPERIPH_SRC ${STM32F411_OR_F427_STDPERIPH_SRC_EXCLUDES})

set(STM32F411_COMPILE_DEFINITIONS
    STM32F411xE
    MCU_FLASH_SIZE=512
    OPTIMIZATION -Os
)

function(target_stm32f411xe name)
    target_stm32f4xx(
        NAME ${name}
        STARTUP startup_stm32f411xe.s
        SOURCES ${STM32F411_OR_F427_STDPERIPH_SRC}
        COMPILE_DEFINITIONS ${STM32F411_COMPILE_DEFINITIONS}
	LINKER_SCRIPT stm32_flash_f411xe
        SVD STM32F411
	${ARGN}
    )
endfunction()

set(STM32F427_COMPILE_DEFINITIONS
    STM32F427_437xx
    MCU_FLASH_SIZE=1024
)
function(target_stm32f427xg name)
    target_stm32f4xx(
        NAME ${name}
        STARTUP startup_stm32f427xx.s
        SOURCES ${STM32F411_OR_F427_STDPERIPH_SRC}
        COMPILE_DEFINITIONS ${STM32F427_COMPILE_DEFINITIONS}
        LINKER_SCRIPT stm32_flash_f427xg
        SVD STM32F411
        ${ARGN}
    )
endfunction()
