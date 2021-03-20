#ifndef PIFROM_H_
#define PIFROM_H_

#include <stdint.h>
#include <stdbool.h>

enum PifRomType {
    PR_INVALID,
    PR_NTSC,
    PR_PAL
};

struct PifRom_t
{
    enum PifRomType Type;
    const uint8_t * Data;
    uint16_t Size;
};

bool PIFROM_GetRom(enum PifRomType type, struct PifRom_t * dst);


#endif /* PIFROM_H_ */
