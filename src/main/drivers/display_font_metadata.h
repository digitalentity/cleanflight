#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct osdCharacter_s osdCharacter_t;

typedef struct displayFontMetadata_s {
    uint8_t version;
    uint16_t charCount;
} displayFontMetadata_t;

// 'I', 'N', 'A', 'V'
#define FONT_CHR_IS_METADATA(chr) ((chr)->data[0] == 'I' && \
    (chr)->data[1] == 'N' && \
    (chr)->data[2] == 'A' && \
    (chr)->data[3] == 'V')

#define FONT_METADATA_CHR_INDEX 255
// Used for runtime detection of display drivers that might
// support 256 or 512 characters.
#define FONT_METADATA_CHR_INDEX_2ND_PAGE 256

bool displayFontMetadataUpdateFromCharacter(displayFontMetadata_t *metadata, const osdCharacter_t *chr);
