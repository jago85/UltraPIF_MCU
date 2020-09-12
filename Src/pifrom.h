#ifndef PIFROM_H_
#define PIFROM_H_

#include <stdint.h>

enum PifRomType {
    PR_NTSC,
    PR_PAL
};

const uint8_t * PIFROM_GetRom(enum PifRomType type);

#endif /* PIFROM_H_ */
