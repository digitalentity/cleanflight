include(cortex-m7)
include(stm32h7-usb)

set(STM32H7_CMSIS_DEVICE_DIR "${MAIN_LIB_DIR}/main/STM32H7/Drivers/CMSIS/Device/ST/STM32H7xx")
set(STM32H7_HAL_DIR "${MAIN_LIB_DIR}/main/STM32H7/Drivers/STM32H7xx_HAL_Driver")

set(STM32H7_HAL_SRC
    stm32h7xx_hal.c
    stm32h7xx_hal_adc.c
    stm32h7xx_hal_adc_ex.c
#    stm32h7xx_hal_cec.c
#    stm32h7xx_hal_comp.c
    stm32h7xx_hal_cortex.c
#    stm32h7xx_hal_crc.c
#    stm32h7xx_hal_crc_ex.c
#    stm32h7xx_hal_cryp.c
#    stm32h7xx_hal_cryp_ex.c
    stm32h7xx_hal_dac.c
    stm32h7xx_hal_dac_ex.c
#    stm32h7xx_hal_dcmi.c
#    stm32h7xx_hal_dfsdm.c
#    stm32h7xx_hal_dfsdm_ex.c
    stm32h7xx_hal_dma.c
#    stm32h7xx_hal_dma2d.c
    stm32h7xx_hal_dma_ex.c
#    stm32h7xx_hal_dsi.c
    stm32h7xx_hal_dts.c
#    stm32h7xx_hal_eth.c
#    stm32h7xx_hal_eth_ex.c
    stm32h7xx_hal_exti.c
#    stm32h7xx_hal_fdcan.c
    stm32h7xx_hal_flash.c
    stm32h7xx_hal_flash_ex.c
    stm32h7xx_hal_gfxmmu.c
    stm32h7xx_hal_gpio.c
#    stm32h7xx_hal_hash.c
#    stm32h7xx_hal_hash_ex.c
#    stm32h7xx_hal_hcd.c
#    stm32h7xx_hal_hrtim.c
#    stm32h7xx_hal_hsem.c
    stm32h7xx_hal_i2c.c
    stm32h7xx_hal_i2c_ex.c
#    stm32h7xx_hal_i2s.c
#    stm32h7xx_hal_i2s_ex.c
#    stm32h7xx_hal_irda.c
#    stm32h7xx_hal_iwdg.c
#    stm32h7xx_hal_jpeg.c
#    stm32h7xx_hal_lptim.c
#    stm32h7xx_hal_ltdc.c
#    stm32h7xx_hal_ltdc_ex.c
#    stm32h7xx_hal_mdios.c
#    stm32h7xx_hal_mdma.c
#    stm32h7xx_hal_mmc.c
#    stm32h7xx_hal_mmc_ex.c
#    stm32h7xx_hal_nand.c
#    stm32h7xx_hal_nor.c
#    stm32h7xx_hal_opamp.c
#    stm32h7xx_hal_opamp_ex.c
    stm32h7xx_hal_ospi.c
    stm32h7xx_hal_otfdec.c
    stm32h7xx_hal_pcd.c
    stm32h7xx_hal_pcd_ex.c
    stm32h7xx_hal_pssi.c
    stm32h7xx_hal_pwr.c
    stm32h7xx_hal_pwr_ex.c
    stm32h7xx_hal_qspi.c
#    stm32h7xx_hal_ramecc.c
    stm32h7xx_hal_rcc.c
    stm32h7xx_hal_rcc_ex.c
#    stm32h7xx_hal_rng.c
#    stm32h7xx_hal_rng_ex.c
    stm32h7xx_hal_rtc.c
    stm32h7xx_hal_rtc_ex.c
#    stm32h7xx_hal_sai.c
#    stm32h7xx_hal_sai_ex.c
    stm32h7xx_hal_sd.c
    stm32h7xx_hal_sd_ex.c
#    stm32h7xx_hal_sdram.c
#    stm32h7xx_hal_smartcard.c
#    stm32h7xx_hal_smartcard_ex.c
#    stm32h7xx_hal_smbus.c
#    stm32h7xx_hal_spdifrx.c
    stm32h7xx_hal_spi.c
    stm32h7xx_hal_spi_ex.c
#    stm32h7xx_hal_sram.c
#    stm32h7xx_hal_swpmi.c
    stm32h7xx_hal_tim.c
    stm32h7xx_hal_tim_ex.c
    stm32h7xx_hal_uart.c
    stm32h7xx_hal_uart_ex.c
#    stm32h7xx_hal_usart.c
#    stm32h7xx_hal_usart_ex.c
#    stm32h7xx_hal_wwdg.c
#    stm32h7xx_ll_adc.c
#    stm32h7xx_ll_bdma.c
#    stm32h7xx_ll_comp.c
#    stm32h7xx_ll_crc.c
    stm32h7xx_ll_crs.c
#    stm32h7xx_ll_dac.c
#    stm32h7xx_ll_delayblock.c
    stm32h7xx_ll_dma.c
#    stm32h7xx_ll_dma2d.c
    stm32h7xx_ll_exti.c
#    stm32h7xx_ll_fmc.c
#    stm32h7xx_ll_gpio.c
#    stm32h7xx_ll_hrtim.c
    stm32h7xx_ll_i2c.c
#    stm32h7xx_ll_lptim.c
#    stm32h7xx_ll_lpuart.c
#    stm32h7xx_ll_mdma.c
#    stm32h7xx_ll_opamp.c
#    stm32h7xx_ll_pwr.c
#    stm32h7xx_ll_rcc.c
#    stm32h7xx_ll_rng.c
#    stm32h7xx_ll_rtc.c
    stm32h7xx_ll_sdmmc.c
#    stm32h7xx_ll_spi.c
#    stm32h7xx_ll_swpmi.c
    stm32h7xx_ll_tim.c
#    stm32h7xx_ll_usart.c
    stm32h7xx_ll_usb.c
#    stm32h7xx_ll_utils.c
)

list(TRANSFORM STM32H7_HAL_SRC PREPEND "${STM32H7_HAL_DIR}/Src/")

set(STM32H7_VCP_DIR "${MAIN_SRC_DIR}/vcp_hal")

set(STM32H7_VCP_SRC
    usbd_desc.c
    usbd_conf_stm32h7xx.c
    usbd_cdc_interface.c
)
list(TRANSFORM STM32H7_VCP_SRC PREPEND "${STM32H7_VCP_DIR}/")

set(STM32H7_INCLUDE_DIRS
    ${STM32H7_HAL_DIR}/Inc
    ${STM32H7_CMSIS_DEVICE_DIR}/Include
)

main_sources(STM32H7_SRC
    target/system_stm32h7xx.c

    config/config_streamer_stm32h7.c

#    drivers/adc_stm32h7xx.c
    drivers/bus_i2c_hal.c
    drivers/dma_stm32h7xx.c
    drivers/bus_spi_hal.c
    drivers/memprot.h
    drivers/memprot_hal.c
    drivers/memprot_stm32h7xx.c
    drivers/timer.c
    drivers/timer_impl_hal.c
    drivers/timer_stm32h7xx.c
    drivers/system_stm32h7xx.c
    drivers/serial_uart_stm32h7xx.c
    drivers/serial_uart_hal.c
#    drivers/sdcard/sdmmc_sdio_h7xx.c
)

main_sources(STM32H7_MSC_SRC
    drivers/usb_msc_h7xx.c
)

set(STM32H7_DEFINITIONS
    ${CORTEX_M7_DEFINITIONS}
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    MAX_MPU_REGIONS=16
)

function(target_stm32h7xx)
    target_stm32(
        SOURCES ${STM32H7_HAL_SRC} ${STM32H7_SRC}
        COMPILE_DEFINITIONS ${STM32H7_DEFINITIONS}
        COMPILE_OPTIONS ${CORTEX_M7_COMMON_OPTIONS} ${CORTEX_M7_COMPILE_OPTIONS}
        INCLUDE_DIRECTORIES ${STM32H7_INCLUDE_DIRS}
        LINK_OPTIONS ${CORTEX_M7_COMMON_OPTIONS} ${CORTEX_M7_LINK_OPTIONS}

        MSC_SOURCES ${STM32H7_USBMSC_SRC} ${STM32H7_MSC_SRC}
        VCP_SOURCES ${STM32H7_USB_SRC} ${STM32H7_VCP_SRC}
        VCP_INCLUDE_DIRECTORIES ${STM32H7_USB_INCLUDE_DIRS} ${STM32H7_VCP_DIR}

        OPTIMIZATION -O2

        OPENOCD_TARGET stm32h7x

        DISABLE_MSC # This should be temporary

#        BOOTLOADER

        ${ARGN}
    )
endfunction()

macro(define_target_stm32h7 subfamily size)
    function(target_stm32h7${subfamily}x${size} name)
        set(func_ARGV ARGV)
        string(TOUPPER ${size} upper_size)
        get_stm32_flash_size(flash_size ${size})
        set(definitions
            STM32H7
            STM32H7${subfamily}xx
            STM32H7${subfamily}x${upper_size}
            # stm32h743xx.h defined FLASH_SIZE, used by HAL, but in bytes
            # use MCU_FLASH_SIZE since we use KiB in our code
            MCU_FLASH_SIZE=${flash_size}
        )
        target_stm32h7xx(
            NAME ${name}
            STARTUP startup_stm32h7${subfamily}xx.s
            COMPILE_DEFINITIONS ${definitions}
            LINKER_SCRIPT stm32_flash_h7${subfamily}x${size}
            ${${func_ARGV}}
        )
    endfunction()
endmacro()

define_target_stm32h7(43 i)