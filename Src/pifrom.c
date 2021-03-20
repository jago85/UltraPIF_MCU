#include "pifrom.h"
#include <stdlib.h>
#include <assert.h>
#include "pifrom_data.h"

bool PIFROM_GetRom(enum PifRomType type, struct PifRom_t * dst)
{
    assert(dst != NULL);

    switch (type)
    {
    case PR_PAL:
        dst->Type = PR_PAL;
        dst->Data = PifRom_Pal;
        dst->Size = sizeof(PifRom_Pal);
        break;

    case PR_NTSC:
        dst->Type = PR_PAL;
        dst->Data = PifRom_Ntsc;
        dst->Size = sizeof(PifRom_Ntsc);
        break;

    default:
        dst->Type = PR_INVALID;
        dst->Data = NULL;
        dst->Size = 0;
    }

    return (dst->Type == PR_INVALID) ? false : true;
}
