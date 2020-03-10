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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/log.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "scheduler/scheduler.h"

const gyroFilterAndRateConfig_t * chooseGyroConfig(uint8_t desiredLpf, uint16_t desiredRateHz, const gyroFilterAndRateConfig_t * configs, int count)
{
    int i;
    int8_t selectedLpf = configs[0].gyroLpf;
    const gyroFilterAndRateConfig_t * candidate = &configs[0];

    // Choose closest supported LPF value
    for (i = 1; i < count; i++) {
        if (ABS(desiredLpf - configs[i].gyroLpf) < ABS(desiredLpf - selectedLpf)) {
            selectedLpf = configs[i].gyroLpf;
            candidate = &configs[i];
        }
    }

    // Now find the closest update rate
    for (i = 0; i < count; i++) {
        if ((configs[i].gyroLpf == selectedLpf) && (ABS(desiredRateHz - candidate->gyroRateHz) > ABS(desiredRateHz - configs[i].gyroRateHz))) {
            candidate = &configs[i];
        }
    }

    LOG_V(GYRO, "GYRO CONFIG { %d, %d } -> { %d, %d}; regs 0x%02X, 0x%02X",
                desiredLpf, desiredRateHz,
                candidate->gyroLpf, candidate->gyroRateHz,
                candidate->gyroConfigValues[0], candidate->gyroConfigValues[1]);

    return candidate;
}

/*
 * Gyro interrupt service routine
 */
#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)
static void gyroIntExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    taskSemaphoreSignal(&gyro->dataReadySemaphore);
    if (gyro->updateFn) {
        gyro->updateFn(gyro);
    }
}
#endif

void gyroIntExtiInit(gyroDev_t *gyro)
{
    if (!gyro->busDev->irqPin) {
        return;
    }

#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)
#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(gyro->busDev->irqPin);
    if (status) {
        return;
    }
#endif

#if defined (STM32F7)
    IOInit(gyro->busDev->irqPin, OWNER_MPU, RESOURCE_EXTI, 0);

    EXTIHandlerInit(&gyro->exti, gyroIntExtiHandler);
    EXTIConfig(gyro->busDev->irqPin, &gyro->exti, NVIC_PRIO_GYRO_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));   // TODO - maybe pullup / pulldown ?
#else
    IOInit(gyro->busDev->irqPin, OWNER_MPU, RESOURCE_EXTI, 0);
    IOConfigGPIO(gyro->busDev->irqPin, IOCFG_IN_FLOATING);

    EXTIHandlerInit(&gyro->exti, gyroIntExtiHandler);
    EXTIConfig(gyro->busDev->irqPin, &gyro->exti, NVIC_PRIO_GYRO_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(gyro->busDev->irqPin, true);
#endif
#endif
}
