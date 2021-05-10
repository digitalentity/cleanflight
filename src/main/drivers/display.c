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
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "config/parameter_group_ids.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"
#include "drivers/display_font_metadata.h"
#include "drivers/time.h"

#include "fc/settings.h"

#define SW_BLINK_CYCLE_MS 200 // 200ms on / 200ms off

// XXX: This is the number of characters in a MAX7456 line.
// Increment this number appropiately or enable support for
// multiple iterations in displayWriteWithAttr() if bigger
// displays are supported (implementation can be found in commit
// 22a48278 before it was deleted).
#define DISPLAY_MAX_STRING_SIZE 30

PG_REGISTER_WITH_RESET_TEMPLATE(displayConfig_t, displayConfig, PG_DISPLAY_CONFIG, 0);

PG_RESET_TEMPLATE(displayConfig_t, displayConfig,
    .force_sw_blink = SETTING_DISPLAY_FORCE_SW_BLINK_DEFAULT
);

static bool displayAttributesRequireEmulation(displayPort_t *instance, textAttributes_t attr)
{
    if (attr & ~instance->cachedSupportedTextAttributes) {
        // We only emulate blink for now
        return TEXT_ATTRIBUTES_HAVE_BLINK(attr);
    }
    return false;
}

static bool displayEmulateTextAttributes(displayPort_t *instance,
                                        char *buf, size_t length,
                                        textAttributes_t *attr)
{
    UNUSED(instance);

    // We only emulate blink for now, so there's no need to test
    // for it again.
    TEXT_ATTRIBUTES_REMOVE_BLINK(*attr);
    if ((millis() / SW_BLINK_CYCLE_MS) % 2) {
        memset(buf, ' ', length);
        buf[length] = '\0';
        // Tell the caller to use buf
        return true;
    }
    // Tell the caller to use s but with the updated attributes
    return false;
}

static void displayUpdateMaxChar(displayPort_t *instance)
{
    if (displayIsReady(instance)) {
        displayFontMetadata_t metadata;
        if (displayGetFontMetadata(&metadata, instance)) {
            instance->maxChar = metadata.charCount - 1;
        } else {
            // Assume 8-bit character implementation
            instance->maxChar = 255;
        }
    }
}

void displayClearScreen(displayPort_t *instance)
{
    instance->vTable->clearScreen(instance);
    instance->cleared = true;
    instance->cursorRow = -1;
}

void displayDrawScreen(displayPort_t *instance)
{
    if (instance->rows == 0 || instance->cols == 0) {
        // Display not fully initialized yet
        displayResync(instance);
    }
    instance->vTable->drawScreen(instance);
}

int displayScreenSize(const displayPort_t *instance)
{
    return instance->vTable->screenSize(instance);
}

void displayGrab(displayPort_t *instance)
{
    instance->vTable->grab(instance);
    instance->vTable->clearScreen(instance);
    ++instance->grabCount;
}

void displayRelease(displayPort_t *instance)
{
    instance->vTable->release(instance);
    // displayPort_t is changing owner. Clear it, since
    // the new owner might expect a clear canvas.
    instance->vTable->clearScreen(instance);
    --instance->grabCount;
}

void displayReleaseAll(displayPort_t *instance)
{
    instance->vTable->release(instance);
    instance->grabCount = 0;
}

bool displayIsGrabbed(const displayPort_t *instance)
{
    // can be called before initialised
    return (instance && instance->grabCount > 0);
}

void displaySetXY(displayPort_t *instance, uint8_t x, uint8_t y)
{
    instance->posX = x;
    instance->posY = y;
}

int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, const char *s)
{
    instance->posX = x + strlen(s);
    instance->posY = y;
    return instance->vTable->writeString(instance, x, y, s, TEXT_ATTRIBUTES_NONE);
}

int displayWriteWithAttr(displayPort_t *instance, uint8_t x, uint8_t y, const char *s, textAttributes_t attr)
{
    size_t length = strlen(s);
    char buf[DISPLAY_MAX_STRING_SIZE + 1];

    instance->posX = x + length;
    instance->posY = y;

    if (displayAttributesRequireEmulation(instance, attr)) {
        // We can't overwrite s, so we use an intermediate buffer if we need
        // text attribute emulation.
        size_t blockSize = length > sizeof(buf) ? sizeof(buf) : length;
        if (displayEmulateTextAttributes(instance, buf, blockSize, &attr)) {
            // Emulation required rewriting the string, use buf.
            s = buf;
        }
    }

    return instance->vTable->writeString(instance, x, y, s, attr);
}

int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint16_t c)
{
    if (instance->maxChar == 0) {
        displayUpdateMaxChar(instance);
    }
    if (c > instance->maxChar) {
        return -1;
    }
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeChar(instance, x, y, c, TEXT_ATTRIBUTES_NONE);
}

int displayWriteCharWithAttr(displayPort_t *instance, uint8_t x, uint8_t y, uint16_t c, textAttributes_t attr)
{
    if (instance->maxChar == 0) {
        displayUpdateMaxChar(instance);
    }
    if (c > instance->maxChar) {
        return -1;
    }
    if (displayAttributesRequireEmulation(instance, attr)) {
        char ec[2];
        if (displayEmulateTextAttributes(instance, ec, 1, &attr)) {
            c = ec[0];
        }
    }
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeChar(instance, x, y, c, attr);
}

bool displayReadCharWithAttr(displayPort_t *instance, uint8_t x, uint8_t y, uint16_t *c, textAttributes_t *attr)
{
    uint16_t dc;
    textAttributes_t dattr;

    if (!instance->vTable->readChar) {
        return false;
    }

    if (!c) {
        c = &dc;
    }

    if (!attr) {
        attr = &dattr;
    }

    return instance->vTable->readChar(instance, x, y, c, attr);
}

bool displayIsTransferInProgress(const displayPort_t *instance)
{
    return instance->vTable->isTransferInProgress(instance);
}

void displayHeartbeat(displayPort_t *instance)
{
    instance->vTable->heartbeat(instance);
}

void displayResync(displayPort_t *instance)
{
    instance->vTable->resync(instance);
}

uint16_t displayTxBytesFree(const displayPort_t *instance)
{
    return instance->vTable->txBytesFree(instance);
}

bool displayGetFontMetadata(displayFontMetadata_t *metadata, const displayPort_t *instance)
{
    if (instance->vTable->getFontMetadata) {
        return instance->vTable->getFontMetadata(metadata, instance);
    }
    return false;
}

int displayWriteFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    if (instance->vTable->writeFontCharacter) {
        return instance->vTable->writeFontCharacter(instance, addr, chr);
    }
    return -1;
}

bool displayIsReady(displayPort_t *instance)
{
    if (instance->vTable->isReady) {
        return instance->vTable->isReady(instance);
    }
    // Drivers that don't provide an isReady method are
    // assumed to be immediately ready (either by actually
    // begin ready very quickly or by blocking)
    return true;
}

void displayBeginTransaction(displayPort_t *instance, displayTransactionOption_e opts)
{
    if (instance->vTable->beginTransaction) {
        instance->vTable->beginTransaction(instance, opts);
    }
}

void displayCommitTransaction(displayPort_t *instance)
{
    if (instance->vTable->commitTransaction) {
        instance->vTable->commitTransaction(instance);
    }
}

bool displayGetCanvas(displayCanvas_t *canvas, const displayPort_t *instance)
{
#if defined(USE_CANVAS)
    if (canvas && instance->vTable->getCanvas && instance->vTable->getCanvas(canvas, instance)) {
        canvas->gridElementWidth = canvas->width / instance->cols;
        canvas->gridElementHeight = canvas->height / instance->rows;
        return true;
    }
#else
    UNUSED(canvas);
    UNUSED(instance);
#endif
    return false;
}

void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable)
{
    instance->vTable = vTable;
    instance->vTable->clearScreen(instance);
    instance->useFullscreen = false;
    instance->cleared = true;
    instance->grabCount = 0;
    instance->cursorRow = -1;
    instance->cachedSupportedTextAttributes = TEXT_ATTRIBUTES_NONE;
    if (vTable->supportedTextAttributes) {
        instance->cachedSupportedTextAttributes = vTable->supportedTextAttributes(instance);
    }
    if (displayConfig()->force_sw_blink) {
        TEXT_ATTRIBUTES_REMOVE_BLINK(instance->cachedSupportedTextAttributes);
    }

    instance->maxChar = 0;
    displayUpdateMaxChar(instance);
}
