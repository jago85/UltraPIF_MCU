#include "pifrom.h"
#include <stdlib.h>
#include "pifrom_data.h"

const uint8_t * PIFROM_GetRom(enum PifRomType type)
{
    const uint8_t * pifrom;

    switch (type)
    {
    case PR_PAL:
        pifrom = PifRom_Pal;
        break;

    case PR_NTSC:
        pifrom = PifRom_Ntsc;
        break;

    default:
        pifrom = NULL;
    }

    return pifrom;
}
