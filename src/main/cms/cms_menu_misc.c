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

#include "platform.h"

#ifdef USE_CMS

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_misc.h"

#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/settings.h"

#include "rx/rx.h"

#include "sensors/battery.h"

static const OSD_Entry menuMiscEntries[]=
{
    OSD_LABEL_ENTRY("-- MISC --"),

    OSD_SETTING_ENTRY("THR IDLE", SETTING_THROTTLE_IDLE),
#ifdef USE_OSD
#ifdef USE_ADC
    OSD_SETTING_ENTRY("OSD VOLT DECIMALS", SETTING_OSD_MAIN_VOLTAGE_DECIMALS),
    OSD_SETTING_ENTRY("STATS ENERGY UNIT", SETTING_OSD_STATS_ENERGY_UNIT),
#endif // ADC
    OSD_SETTING_ENTRY("STATS PAGE SWAP TIME", SETTING_OSD_STATS_PAGE_AUTO_SWAP_TIME),
#endif // OSD
    OSD_SETTING_ENTRY("FS PROCEDURE", SETTING_FAILSAFE_PROCEDURE),


    OSD_BACK_AND_END_ENTRY,
};

const CMS_Menu cmsx_menuMisc = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XMISC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuMiscEntries
};

#endif // CMS
