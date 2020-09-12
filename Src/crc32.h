#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

void CRC_BuildTable(void);
uint32_t CRC_Calculate(void *buffer, uint32_t count);
void CRC_Initialize(uint32_t *crc);
void CRC_Finalize(uint32_t *crc);
void CRC_CalculateCont(uint32_t *crc, void *buffer, uint32_t count);

#endif /* CRC32_H_ */
