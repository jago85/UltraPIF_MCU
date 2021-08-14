/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "n64_cic_nus_6105.h"

#include "si5351-samd21-minimal.h"
#include "pifrom.h"

#include "spiffs.h"

#include "crc32.h"

#define MCU_VERSION 0x02000B00

// FPGA SPI Registers
#define SPI_REG_INT 0x00
#define SPI_REG_PIFCMD 0x01
#define SPI_REG_GPIO 0x02
#define SPI_REG_RED 0x03
#define SPI_REG_GREEN 0x04
#define SPI_REG_BLUE 0x05
#define SPI_REG_I2C_SLAVEADDR 0x06
#define SPI_REG_SI_CHANNEL 0x07
#define SPI_REG_DMA_MEMADDR 0x08
#define SPI_REG_DMA_CNT 0x09
#define SPI_REG_DMA_CMD 0x0A
#define SPI_REG_DMA_STAT 0x0B
#define SPI_REG_PIFRAMBANK 0x0C
#define SPI_REG_SI_RXCNT 0x0D
#define SPI_REG_FPGA_VER0 0x0E
#define SPI_REG_FPGA_VER1 0x0F

// FPGA DMA Commands
#define SPI_DMA_CMD_I2C_WRITE 0x01
#define SPI_DMA_CMD_I2C_READ 0x02
#define SPI_DMA_CMD_SI_TRANSCEIVE 0x04
#define SPI_DMA_CMD_CIC_TRANSCEIVE 0x8
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// TODO: move typedefs here!
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Only Power of 2 is allowed
#define FILE_BUFFER_SIZE (256)

#if (FILE_BUFFER_SIZE & (FILE_BUFFER_SIZE-1))
#error "FILE_BUFFER_SIZE must be power of 2"
#endif

#define PIFRAM_SIZE (64)
#define APP_BUFFER_SIZE (32)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint8_t PifUuid[16] = {
        0xfe, 0xa5, 0x14, 0x72, 0xb8, 0x17, 0x49, 0xf2,
        0x82, 0xde, 0x82, 0xee, 0xc3, 0xb4, 0x56, 0xaa
};

SPI_HandleTypeDef *hspi_fpga;
uint8_t TestMem[PIFRAM_SIZE];

static spiffs fs;
#define LOG_PAGE_SIZE       256

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[32*4];

static const char * _PifRomFilename = "pifrom_ultrapif.bin";

static const char * _RomFiles[] = {
        "bootcode_ultrapif.bin",
        "menu_ultrapif.bin",
        "bootcodepatch.bin"
//        "bootcode.ips"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t ReadRegister(uint8_t addr);
void WriteRegister(uint8_t addr, uint16_t val);
void WriteMultipleRegisters(uint8_t baseAddr, uint16_t *mem, uint8_t cnt);
void ReadExtMem(uint16_t addr, void *data, uint16_t len);
void WriteExtMem(uint16_t addr, void const *data, uint16_t len);
uint8_t EepromTransmitReceive(const uint8_t *txBuf, uint8_t txLen, uint8_t *rxBuf, uint8_t rxLen);
void WriteGpio(uint16_t val);

void FPGA_Refresh(void);
void FPGA_UpdateFromFile(spiffs_file updateFile);

void ErrorBlink(int count);

void WriteU16ToBufferBigEndian(uint8_t *buffer, uint16_t val);
uint16_t ReadU16FromBufferBigEndian(uint8_t *buffer);
void WriteU32ToBufferBigEndian(uint8_t *buffer, uint32_t val);
uint32_t ReadU32FromBufferBigEndian(uint8_t *buffer);
void WriteU64ToBufferBigEndian(uint8_t *buffer, uint64_t val);
uint64_t ReadU64FromBufferBigEndian(uint8_t *buffer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum PifState {
    PIF_S_POR = 0,
    PIF_S_LED,
    PIF_S_POR_DELAY,
    PIF_S_READ_CIC,
    PIF_S_BOOTING,
    PIF_S_RUNNING,
    PIF_S_WAIT_WRITE
};

enum PifRcpCommand {
    PIF_RCP_CMD_WRITE_64 = 0,
    PIF_RCP_CMD_READ_64 = 1,
    PIF_RCP_CMD_WRITE_4 = 2,
    PIF_RCP_CMD_READ_4 = 3
};

enum UltraPifBootCommand {
    ULTRAPIF_BCMD_ENABLE = 0,
    ULTRAPIF_BCMD_OPEN_ROM = 1,
    ULTRAPIF_BCMD_LOAD_ROM_PAGE = 2,
    ULTRAPIF_BCMD_SET_RGBLED = 3,
    ULTRAPIF_BCMD_PUTCHAR = 4,
    ULTRAPIF_BCMD_DEBUG = 0xffff
};

enum UltraPifCommand {
    ULTRAPIF_CMD_PING,
    ULTRAPIF_CMD_GET_VERSION,

    ULTRAPIF_CMD_FILE_OPEN,
    ULTRAPIF_CMD_FILE_CLOSE,
    ULTRAPIF_CMD_FILE_REMOVE,
    ULTRAPIF_CMD_FILE_READ,
    ULTRAPIF_CMD_FILE_WRITE,
    ULTRAPIF_CMD_FILE_SEEK,
    ULTRAPIF_CMD_FILE_TELL,
    ULTRAPIF_CMD_DIR_OPEN,
    ULTRAPIF_CMD_DIR_READ,
    ULTRAPIF_CMD_DIR_CLOSE,

    ULTRAPIF_CMD_READ_CIC,          // unused
    ULTRAPIF_CMD_COLD_RESET,
    ULTRAPIF_CMD_GET_CLOCK_RATES,
    ULTRAPIF_CMD_SET_RGBLED,

    ULTRAPIF_CMD_SET_APPBUFFER,
    ULTRAPIF_CMD_GET_APPBUFFER,

    ULTRAPIF_CMD_GET_RESET_SOURCE,

    ULTRAPIF_CMD_SET_EEPROM_EMULATION,
    ULTRAPIF_CMD_SET_CONTROLLERPAK_EMULATION,
    ULTRAPIF_CMD_SET_RTC_EMULATION,

    ULTRAPIF_CMD_UPDATE_FPGA,
    ULTRAPIF_CMD_WRITE_FIRMWAREUPDATE,
    ULTRAPIF_CMD_START_FIRMWAREUPDATE,

    ULTRAPIF_CMD_GET_FS_INFO,

    ULTRAPIF_CMD_ENABLE_CLOCK,      // can ONLY be used once after first boot (need to use COLD_RESET then)

    ULTRAPIF_CMD_DEBUG = 0xffff
};

enum ResetSource {
    RESET_SOURCE_POR,
    RESET_SOURCE_BUTTON,
    RESET_SOURCE_BUTTON_LONG,
    RESET_SOURCE_CONTROLLER,
    RESET_SOURCE_CMD
};

typedef struct CmdAddr_st {
    unsigned Addr:9;
    enum PifRcpCommand Cmd:2;
    unsigned __reserved:5;
} CmdAddr_t;

typedef struct N64_Inputs_st
{
    unsigned Dright:1;
    unsigned Dleft:1;
    unsigned Ddown:1;
    unsigned Dup:1;
    unsigned Start:1;
    unsigned Z:1;
    unsigned B:1;
    unsigned A:1;
    unsigned Cright:1;
    unsigned Cleft:1;
    unsigned Cdown:1;
    unsigned Cup:1;
    unsigned R:1;
    unsigned L:1;
    unsigned __reserved1:1;
    unsigned __reserved2:1;
    unsigned X:8;
    unsigned Y:8;
} N64_Inputs_t;

// Outputs
#define PIF_GPIO_COLDRESET      (0x0001)
#define PIF_GPIO_NMI            (0x0002)
#define PIF_GPIO_PRENMI         (0x0004)
#define PIF_GPIO_CIC_DCLK       (0x0008)
#define PIF_GPIO_CIC_DO         (0x0010)
#define PIF_GPIO_LED            (0x0080)

// Inputs
#define PIF_GPIO_RESET_BUTTON   (0x0100)
#define PIF_GPIO_RC_POR         (0x0200)
#define PIF_GPIO_CIC_DI         (0x0400)
#define PIF_GPIO_EN             (0x0800)

// JP1 is clock mode
// Open jumper (pin set) = Normal (Crystal) Mode (drive the PLL)
// Closed jumper (pin reset) = Direct (VCLK) mode (drive VCLK)
#define PIF_GPIO_JP1            (0x1000)

// Unused
#define PIF_GPIO_JP2            (0x2000)
#define PIF_GPIO_JP3            (0x4000)
#define PIF_GPIO_JP4            (0x8000)

enum PakEmulationState {
  PAK_EMUL_REMOVED,
  PAK_EMUL_INSERTING,
  PAK_EMUL_INSERTED,
  PAK_EMUL_REMOVING,
};

typedef struct PakEmulation_st {
    bool IsEmulationEnabled;
    enum PakEmulationState State;
    uint32_t Tick;
    bool IsPakEnabled;
    bool HasPakChanged;
    spiffs_file PakFile;
    uint8_t Block0[32];             // Block0 is used for Pak tests, dont waste the Flash for this tests
} PakEmulation_t;

typedef struct Pif_st {
    uint8_t PifRamRead[PIFRAM_SIZE];     // buffer read from N64 (does not get changed)
    uint8_t PifRamWrite[PIFRAM_SIZE];    // working buffer (written to N64 after modification)
    uint8_t FileBuffer[FILE_BUFFER_SIZE];
    uint8_t AppBuffer[APP_BUFFER_SIZE];

    enum ResetSource ResetSource;
    enum PifState State;
    uint16_t SavedLedColor[3];
    uint32_t CurrentTick;
    bool IsBooting;
    uint32_t BootTick;
    uint32_t ResetBtnTick;
    uint32_t ResetBtnHoldTick;
    uint32_t NmiTick;
    uint16_t GpioIn;
    uint8_t GpioOut;

    bool UseDirectVclkMode;
    uint8_t CicRegion;
    uint8_t CicSeed[2];
    bool IsChechsumValid;
    uint8_t CicChecksum[6];

    enum PifRomType RomType;
    CmdAddr_t CmdAddr;

    bool IsEepromEmulationEnabled;
    spiffs_file EepromFile;
    uint8_t EepromType[3];
    spiffs_file RomFile;
    uint32_t RomSize;
    uint32_t LastFileOffset;

    spiffs_DIR Directory;

    PakEmulation_t PakEmulationInfo[4];

    bool IsRtcEmulationEnabled;
    uint16_t RtcControl;

    bool IsUltraPifEnabled;

    uint32_t Crystal;
    bool Fsel;
    uint32_t Fso_5;
    uint32_t Fsc;

    uint32_t RemoteResetTick;
    bool RemoteResetPending;
    uint32_t Debug;
    uint64_t Debug64;
} Pif_t;

uint8_t _SpiTxBuf[256];
uint8_t _SpiRxBuf[256];
uint8_t _UartTxBuf[54];

Pif_t _Pif;

const uint8_t EepromType16k[3] = {0x00, 0xC0, 0x00};
const uint8_t EepromType4k[3] = {0x00, 0x80, 0x00};

// from MAME
uint8_t calc_mempak_crc(uint8_t *buffer, int length)
{
    uint32_t crc = 0;
    uint32_t temp2 = 0;

    for(int i = 0; i <= length; i++)
    {
        for(int j = 7; j >= 0; j--)
        {
            if ((crc & 0x80) != 0)
            {
                temp2 = 0x85;
            }
            else
            {
                temp2 = 0;
            }

            crc <<= 1;

            if (i == length)
            {
                crc &= 0xff;
            }
            else
            {
                if ((buffer[i] & (1 << j)) != 0)
                {
                    crc |= 0x1;
                }
            }

            crc ^= temp2;
        }
    }

    return crc;
}

uint16_t N64CRC_GetBusAddr(uint16_t addr)
{
    uint8_t crc;
    uint8_t t, t2;
    uint32_t i;
    uint16_t addrTmp;

    addrTmp = addr;
    t = 0;
    for (i = 0; i < 16; i++)
    {
        if ((t & 0x10) != 0)
            t2 = 0x15;
        else
            t2 = 0x00;
        t <<= 1;
        if ((addrTmp & 0x400) != 0)
            t |= 0x01;
        addrTmp <<= 1;
        t = t ^ t2;
    }
    crc = t & 0x1f;
    return (addr << 5) | crc;
}

// return true: valid, false: invalid
bool GetPakAddr(uint8_t *buffer, uint16_t *pakAddress)
{
    uint16_t addr;

    if ((buffer == NULL) || (pakAddress == NULL))
        return 0;

    addr = (buffer[0] << 8) | buffer[1];

    *pakAddress = addr & ~0x1f;

    if (N64CRC_GetBusAddr((addr & ~0x1f) >> 5) == addr)
    {
        return true;
    }
    return false;
}

void WaitDmaBusy(void)
{
    uint8_t dmaStat;
    uint32_t cnt = 0;

    do
    {
        cnt++;
        dmaStat = ReadRegister(SPI_REG_DMA_STAT);
    } while (dmaStat & 1);
}

uint8_t ControllerSendReceive(uint8_t port, const uint8_t *txBuf, uint8_t txLen, uint8_t *rxBuf, uint8_t rxLen)
{
    uint8_t res;

    uint16_t siRxCnt;
    uint16_t regs[4];

    WriteExtMem(0, txBuf, txLen);
    regs[0] = 1 + port;                     // SPI_REG_SI_CHANNEL
    regs[1] = 0;                            // SPI_REG_DMA_MEMADDR
    regs[2] = txLen;                        // SPI_REG_DMA_CNT
    regs[3] = SPI_DMA_CMD_SI_TRANSCEIVE;    // SPI_REG_DMA_CMD
    WriteMultipleRegisters(SPI_REG_SI_CHANNEL, regs, 4);

    WaitDmaBusy();

    siRxCnt = ReadRegister(SPI_REG_SI_RXCNT);
    res = rxLen;
    if (siRxCnt < rxLen)
    {
        res |= 0x80;
        rxLen = siRxCnt;
    }
    else if (siRxCnt != rxLen)
    {
        res |= 0x40;
    }
    ReadExtMem(txLen, rxBuf, rxLen);

    return res;
}

uint8_t byte_to_bcd(uint8_t b)
{
    uint8_t res;

    if (b > 99)
        return 0x99;

    res = b % 10;
    res |= (b / 10) << 4;

    return res;
}

void EepromEmulate(Pif_t *pif, uint8_t *txBuf, uint8_t *rxBuf)
{
    uint16_t eepAddr;
    int32_t res;

    if ((txBuf[0] == 4) || (txBuf[0] == 5))
    {
        eepAddr = txBuf[1] << 3;
        res = SPIFFS_lseek(&fs, pif->EepromFile, eepAddr, SPIFFS_SEEK_SET);
        if (res < 0)
        {
            txBuf[-1] |= 80;
            return;
        }
    }

    switch (txBuf[0])
    {
    case 0x00: // Status
        memcpy(rxBuf, pif->EepromType, 3);
        break;

    case 0x04: // Read
        res = SPIFFS_read(&fs, pif->EepromFile, rxBuf, 8);
        break;

    case 0x05: // Write
        res = SPIFFS_write(&fs, pif->EepromFile, &txBuf[2], 8);
        rxBuf[0] = 0;
        break;
    }
}

void RtcEmulate(Pif_t *pif, uint8_t *txBuf, uint8_t *rxBuf)
{
    // TODO: Remove
//    uint8_t msgLen;
//
//    while (huart1.gState != HAL_UART_STATE_READY) ;
//    memset(TestMem, 0, sizeof(TestMem));
//    if ((txBuf[0] == 7) && (txBuf[1] == 0))
//    {
//        msgLen = sprintf(TestMem, "\e[34;40mPIF  %2d:", txBuf[-2]);
//    }
//    else
//    {
//        msgLen = sprintf(TestMem, "\e[31;40mPIF  %2d:", txBuf[-2]);
//    }
//    for (int i = 0; i < txBuf[-2]; i++)
//    {
//        msgLen += sprintf(&TestMem[msgLen], " %02X", txBuf[i]);
//    }
//    msgLen += sprintf(&TestMem[msgLen], "\r\n");
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TestMem, msgLen);

    switch (txBuf[0])
    {
    case 0x06:
        rxBuf[0] = 0;
        rxBuf[1] = 0x10;
        rxBuf[2] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
        break;

    case 0x07:
        switch (txBuf[1])
        {
        case 0:
            WriteU16ToBufferBigEndian(rxBuf, pif->RtcControl);
            memset(&rxBuf[2], 0, 6);
            rxBuf[8] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        case 1:
            // Read of block 1 is not implemented (returns all 0's)
            memset(rxBuf, 0, 8);
            rxBuf[8] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        case 2:
        {
            RTC_TimeTypeDef rtcTime;
            RTC_DateTypeDef rtcDate;

            HAL_RTC_WaitForSynchro(&hrtc);
            HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BCD);
            HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);

            rxBuf[0] = rtcTime.Seconds;
            rxBuf[1] = rtcTime.Minutes;
            rxBuf[2] = 0x80 | rtcTime.Hours;
            rxBuf[3] = rtcDate.Date;
            rxBuf[4] = (rtcDate.WeekDay == RTC_WEEKDAY_SUNDAY) ? 0 : rtcDate.WeekDay;
            rxBuf[5] = rtcDate.Month;
            rxBuf[6] = rtcDate.Year;
            rxBuf[7] = 0x01;
            rxBuf[8] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        }
        }
        break;

   case 0x08:
        switch (txBuf[1])
        {
        case 0:
        {
            pif->RtcControl = ReadU16FromBufferBigEndian(&txBuf[2]);
            rxBuf[0] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        }
        case 1:
            // Write of block 1 is not implemented
            rxBuf[0] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        case 2:
        {
            // Check write protect bit for block 2
            if ((pif->RtcControl & 0x0200) == 0)
            {
                RTC_TimeTypeDef rtcTime = {0};
                RTC_DateTypeDef rtcDate = {0};

                rtcTime.Seconds = txBuf[2];
                rtcTime.Minutes = txBuf[3];
                rtcTime.Hours = txBuf[4];
                rtcDate.Date = txBuf[5];
                rtcDate.WeekDay = (txBuf[6] == 0) ? RTC_WEEKDAY_SUNDAY : txBuf[6];
                rtcDate.Month = txBuf[7];

                // only supporting years from 2000
                rtcDate.Year = (txBuf[9] == 1) ? txBuf[8] : 0;

                HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);
                HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BCD);
            }
            rxBuf[0] = (pif->RtcControl & 0x0004) ? 0x80 : 0x00;
            break;
        }
        }
        break;
    }

    // TODO: Remove
//    while (huart1.gState != HAL_UART_STATE_READY) ;
//    memset(TestMem, 0, sizeof(TestMem));
//    msgLen = sprintf(TestMem, "\e[33;40mCART %2d:", txBuf[-1]);
//    for (int i = 0; i < txBuf[-1]; i++)
//    {
//        msgLen += sprintf(&TestMem[msgLen], " %02X", rxBuf[i]);
//    }
//    msgLen += sprintf(&TestMem[msgLen], "\r\n");
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TestMem, msgLen);
}

// Returns true, if the Request has been handled, false if the request must be sent to the controller
bool ControllerPakEmulate(Pif_t *pif, uint8_t channel, uint8_t *txBuf, uint8_t txLen, uint8_t *rxBuf, uint8_t rxLen)
{
    bool res = false;
    uint16_t pakAddr;
    bool crcValid;
    uint8_t requestResult;

    switch (pif->PakEmulationInfo[channel].State)
    {
    case PAK_EMUL_REMOVED:
        if (pif->PakEmulationInfo[channel].IsPakEnabled == true)
        {
            pif->PakEmulationInfo[channel].Tick = pif->CurrentTick;
            pif->PakEmulationInfo[channel].State = PAK_EMUL_INSERTING;
        }
        break;

    case PAK_EMUL_INSERTING:
        if (pif->PakEmulationInfo[channel].IsPakEnabled == false)
        {
            pif->PakEmulationInfo[channel].State = PAK_EMUL_REMOVED;
        }
        else if (pif->CurrentTick - pif->PakEmulationInfo[channel].Tick > 500)
        {
            pif->PakEmulationInfo[channel].State = PAK_EMUL_INSERTED;
            pif->PakEmulationInfo[channel].HasPakChanged = true;
        }
        break;

    case PAK_EMUL_INSERTED:
        if (pif->PakEmulationInfo[channel].IsPakEnabled == false)
        {
            pif->PakEmulationInfo[channel].Tick = pif->CurrentTick;
            pif->PakEmulationInfo[channel].State = PAK_EMUL_REMOVING;
        }
        break;

    case PAK_EMUL_REMOVING:
        if (pif->CurrentTick - pif->PakEmulationInfo[channel].Tick > 500)
        {
            pif->PakEmulationInfo[channel].State = PAK_EMUL_REMOVED;
            pif->PakEmulationInfo[channel].HasPakChanged = true;
        }
        break;
    }

    switch (txBuf[0])
    {
    case 0:
    case 0xFF:
        pif->GpioOut |= PIF_GPIO_LED;
        WriteGpio(pif->GpioOut);

        // send the request to the real controller
        requestResult = ControllerSendReceive(channel, txBuf, txLen, rxBuf, rxLen);
        txBuf[-1] = requestResult;

        // if the result is ok and it is a valid controller
        if ((requestResult == rxLen) && (rxBuf[0] == 5))
        {
            switch (pif->PakEmulationInfo[channel].State)
            {
            case PAK_EMUL_REMOVED:
                if ((rxBuf[2] & 1) && pif->PakEmulationInfo[channel].HasPakChanged)
                {
                    rxBuf[2] = 3;
                    pif->PakEmulationInfo[channel].HasPakChanged = false;
                }
                break;

            case PAK_EMUL_INSERTED:
                // emulate the virtual Pak state
                if (pif->PakEmulationInfo[channel].HasPakChanged)
                {
                    rxBuf[2] = 3;
                    pif->PakEmulationInfo[channel].HasPakChanged = false;
                }
                else
                {
                    rxBuf[2] = 1;
                }
                break;

            default:
                // no pak
                rxBuf[2] = 2;
            }
        }

        // request has been handled
        res = true;
        break;

    case 2:
        if (pif->PakEmulationInfo[channel].State != PAK_EMUL_REMOVED)
        {
            crcValid = GetPakAddr(&txBuf[1], &pakAddr);

            // check if the address is in the ControllerPak's range
            if ((pif->PakEmulationInfo[channel].State == PAK_EMUL_INSERTED) && (pakAddr < 0x8000))
            {
                if (pakAddr != 0)
                {
                    SPIFFS_lseek(&fs, pif->PakEmulationInfo[channel].PakFile, pakAddr, SEEK_SET);
                    SPIFFS_read(&fs, pif->PakEmulationInfo[channel].PakFile, rxBuf, 32);
                }
                else
                {
                    // Block0 is used for Pak tests, dont waste the Flash for this tests
                    memcpy(rxBuf, pif->PakEmulationInfo[channel].Block0, 32);
                }
            }
            else
            {
                // return nulls that the Pak can be identified
                memset(rxBuf, 0, 32);
            }
            rxBuf[32] = calc_mempak_crc(rxBuf, 32);

            // invert the crc if the address crc is invalid or the Pak has not been initialized
            if (!crcValid || pif->PakEmulationInfo[channel].HasPakChanged || (pif->PakEmulationInfo[channel].State != PAK_EMUL_INSERTED))
            {
                rxBuf[32] ^= 0xFF;
            }

            // request has been handled
            res = true;
        }
        break;

    case 3:
        if (pif->PakEmulationInfo[channel].State != PAK_EMUL_REMOVED)
        {
            crcValid = GetPakAddr(&txBuf[1], &pakAddr);
            rxBuf[0] = calc_mempak_crc(&txBuf[3], 32);

            // write the data only if the address crc is valid and the Pak has been initialized
            if ((crcValid == true) && (pif->PakEmulationInfo[channel].HasPakChanged == false) && (pif->PakEmulationInfo[channel].State == PAK_EMUL_INSERTED))
            {
                // check if the address is in the ControllerPak's range
                if (pakAddr < 0x8000)
                {
                    if (pakAddr != 0)
                    {
                        // first read the block and check if it really has to be written
                        uint8_t pakBlock[32];
                        SPIFFS_lseek(&fs, pif->PakEmulationInfo[channel].PakFile, pakAddr, SEEK_SET);
                        SPIFFS_read(&fs, pif->PakEmulationInfo[channel].PakFile, pakBlock, 32);

                        // if the data is different, write the block
                        if (memcmp(&txBuf[3], pakBlock, 32))
                        {
                            SPIFFS_lseek(&fs, pif->PakEmulationInfo[channel].PakFile, pakAddr, SEEK_SET);
                            SPIFFS_write(&fs, pif->PakEmulationInfo[channel].PakFile, &txBuf[3], 32);
                        }
                    }
                    else
                    {
                        // Block0 is used for Pak tests, dont waste the Flash for this tests
                        memcpy(pif->PakEmulationInfo[channel].Block0, &txBuf[3], 32);
                    }
                }
            }
            else
            {
                // invert the data crc on errors
                rxBuf[0] ^= 0xFF;
            }

            // request has been handled
            res = true;
        }
        break;
    }

    return res;
}

void ExecutePifCommand(Pif_t *pif, uint8_t channel, uint8_t *txBuf, uint8_t txLen, uint8_t *rxBuf, uint8_t rxLen)
{
    if (channel < 4)
    {
        bool requestHandled = false;
        if (pif->PakEmulationInfo[channel].IsEmulationEnabled)
        {
            requestHandled = ControllerPakEmulate(pif, channel, txBuf, txLen, rxBuf, rxLen);
        }

        if (!requestHandled)
        {
            if (pif->GpioOut & PIF_GPIO_LED)
            {
                pif->GpioOut &= ~PIF_GPIO_LED;
                WriteGpio(pif->GpioOut);
            }
            txBuf[-1] = ControllerSendReceive(channel, txBuf, txLen, rxBuf, rxLen);
        }

        if ((txBuf[-1] == rxLen) && (txBuf[0] == 0x01))
        {
            N64_Inputs_t * inp = (N64_Inputs_t *)rxBuf;

            // Capture Inputs of Controller Port 1
            // Initiate a remote reset if A, B, L, R and Z are pressed
            if (channel == 0)
            {
                if (inp->A && inp->B && inp->Z && inp->R && inp->L)
                {
                    if (pif->RemoteResetTick == 0)
                    {
                        pif->RemoteResetTick = pif->CurrentTick;
                    }
                }
                else
                {
                    pif->RemoteResetTick = 0;
                }
            }

            if (pif->PakEmulationInfo[channel].IsEmulationEnabled && inp->L && inp->Z && inp->R)
            {
                if ((inp->Dup == true) && (pif->PakEmulationInfo[channel].IsPakEnabled == false))
                {
                    pif->PakEmulationInfo[channel].IsPakEnabled = true;
                }

                if ((inp->Ddown == true) && (pif->PakEmulationInfo[channel].IsPakEnabled == true))
                {
                    pif->PakEmulationInfo[channel].IsPakEnabled = false;
                }
            }

        }
    }
    else if (channel == 4)
    {
        if (pif->IsEepromEmulationEnabled
                && ((txBuf[0] == 0) || (txBuf[0] == 4) || (txBuf[0] == 5)))
        {
            EepromEmulate(pif, txBuf, rxBuf);
        }
        else if (pif->IsRtcEmulationEnabled
                && ((txBuf[0] == 6) || (txBuf[0] == 7) || (txBuf[0] == 8)))
        {
            RtcEmulate(pif, txBuf, rxBuf);
        }
        else
        {
            txBuf[-1] = EepromTransmitReceive(txBuf, txLen, rxBuf, rxLen);
        }
    }
}

void Execute6105(Pif_t *pif)
{
    char challenge[CHL_LEN];
    char response[CHL_LEN];
    uint32_t i;
    // format the 'challenge' message into 30 nibbles for X-Scale's CIC code
    for (i = 0; i < 15; i++)
    {
        challenge[i*2] =   (pif->PifRamWrite[48+i] >> 4) & 0x0f;
        challenge[i*2+1] =  pif->PifRamWrite[48+i]       & 0x0f;
    }
    // calculate the proper response for the given challenge (X-Scale's algorithm)
    n64_cic_nus_6105(challenge, response, CHL_LEN - 2);
    pif->PifRamWrite[46] = 0;
    pif->PifRamWrite[47] = 0;
    // re-format the 'response' into a byte stream
    for (i = 0; i < 15; i++)
    {
        pif->PifRamWrite[48+i] = (response[i*2] << 4) + response[i*2+1];
    }

}

void PIF_ParseRam(Pif_t *pif)
{
    uint8_t i;
    uint8_t channel = 0;
    uint8_t txCnt = 0;
    uint8_t rxCnt = 0;
    uint8_t *txBuf;
    uint8_t *rxBuf;

    if (pif->PifRamWrite[63] & 0x02)
    {
        switch (pif->PifRamWrite[63])
        {
        case 2:
            pif->GpioOut |= PIF_GPIO_LED;
            WriteGpio(pif->GpioOut);
            Execute6105(pif);
            pif->GpioOut &= ~PIF_GPIO_LED;
            WriteGpio(pif->GpioOut);
            pif->PifRamWrite[63] = 0;
            break;
        case 8:
            pif->PifRamWrite[63] = 0;
            break;
        default:
            pif->PifRamWrite[63] = 0;
        }
        return;
    }

    for (i = 0; i < PIFRAM_SIZE; i++)
    {
        switch (pif->PifRamWrite[i])
        {
        case 0x00:
            channel++;
            break;
        case 0xFE:
            i = PIFRAM_SIZE;
            break;
        case 0xFF:
            break;
        default:
            txCnt = pif->PifRamWrite[i];
            rxCnt = pif->PifRamWrite[i + 1];
            if (rxCnt == 0xfe)
            {
                i = PIFRAM_SIZE;
                break;
            }
            txBuf = &pif->PifRamWrite[i + 2];
            rxBuf = &pif->PifRamWrite[i + 2 + txCnt];
            ExecutePifCommand(pif, channel, txBuf, txCnt, rxBuf, rxCnt);
            i += txCnt + rxCnt + 1;
            channel++;
        }
        if (channel > 4)
            break;

    }
    pif->PifRamWrite[63] = 0;
}

bool PIF_IsCmd(uint8_t *buf)
{
    if (memcmp(buf, PifUuid, 8) == 0)
    {
        return true;
    }
    return false;
}

void ReadPifRam(Pif_t *pif)
{
    memset(&_SpiTxBuf[1], 0, PIFRAM_SIZE + 1);
    _SpiTxBuf[0] = 0xC0;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_fpga, _SpiTxBuf, _SpiRxBuf, PIFRAM_SIZE + 2);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
    memcpy(pif->PifRamRead, &_SpiRxBuf[2], PIFRAM_SIZE);
}

void WritePifRam(Pif_t *pif)
{
    memcpy(&_SpiTxBuf[1], pif->PifRamWrite, PIFRAM_SIZE);
    _SpiTxBuf[0] = 0x80;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, PIFRAM_SIZE + 1);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

void WriteRegister(uint8_t addr, uint16_t val)
{
    if (addr >= 0x1F)
        return;

    _SpiTxBuf[0] = 0x40 | addr;
    _SpiTxBuf[1] = (uint8_t)val;
    _SpiTxBuf[2] = (uint8_t)(val >> 8);
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, 3);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

void WriteMultipleRegisters(uint8_t baseAddr, uint16_t *mem, uint8_t cnt)
{
    if (mem == NULL)
        return;
    if (cnt == 0)
        return;

    baseAddr &= 0x1f;
    _SpiTxBuf[0] = 0x40 | baseAddr;
    memcpy(&_SpiTxBuf[1], mem, 2 * cnt);
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_fpga, _SpiTxBuf, _SpiRxBuf, 1 + cnt * 2);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

uint16_t ReadRegister(uint8_t addr)
{
    if (addr >= 0x1F)
        return 0;

    memset(&_SpiTxBuf[1], 0, 3);
    _SpiTxBuf[0] = 0x60 | addr;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_fpga, _SpiTxBuf, _SpiRxBuf, 4);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);

    uint16_t res = 0;
    memcpy(&res, &_SpiRxBuf[2], sizeof(uint16_t));
    return res;
}

void ReadMultipleRegisters(uint8_t baseAddr, uint16_t *mem, uint8_t cnt)
{
    if (mem == NULL)
        return;
    if (cnt == 0)
        return;

    baseAddr &= 0x1f;
    _SpiTxBuf[0] = 0x60 | baseAddr;
    _SpiTxBuf[1] = 0;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_fpga, _SpiTxBuf, _SpiRxBuf, 3 + cnt * 2);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);

    memcpy(mem, &_SpiRxBuf[2], cnt * 2);
}

void WriteGpio(uint16_t val)
{
    WriteRegister(SPI_REG_GPIO, val);
}

uint16_t ReadGpio(void)
{
    return ReadRegister(SPI_REG_GPIO);
}

uint32_t ReadFpgaVersion(void)
{
    uint32_t res;

    ReadMultipleRegisters(SPI_REG_FPGA_VER0, (uint16_t *)&res, 2);

    return res;
}

void ReadRgbLed(uint16_t * red, uint16_t * green, uint16_t * blue)
{
    uint16_t ledTmp[3];

    ReadMultipleRegisters(SPI_REG_RED, ledTmp, 3);

    if (red)
        *red = ledTmp[0];
    if (green)
        *green = ledTmp[1];
    if (blue)
        *blue = ledTmp[2];
}

void WriteRgbLed(uint16_t red, uint16_t green, uint16_t blue)
{
    uint16_t regs[3];
    regs[0] = red;          // SPI_REG_RED
    regs[1] = green;        // SPI_REG_GREEN
    regs[2] = blue;         // SPI_REG_BLUE
    WriteMultipleRegisters(SPI_REG_RED, regs, 3);
}

void WriteExtMem(uint16_t addr, void const *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
        return;
    if (3 + len > sizeof(_SpiRxBuf))
        return;
    addr &= 0x0fff;
    _SpiTxBuf[0] = 0x20 | (addr >> 8);
    _SpiTxBuf[1] = addr;
    memcpy(&_SpiTxBuf[2], data, len);
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, 2 + len);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

void ReadExtMem(uint16_t addr, void *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
        return;
    if (3 + len > sizeof(_SpiRxBuf))
        return;
    addr &= 0x0fff;
    _SpiTxBuf[0] = 0x30 | (addr >> 8);
    _SpiTxBuf[1] = addr;
    memset(&_SpiTxBuf[2], 0, 1 + len);
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_fpga, _SpiTxBuf, _SpiRxBuf, 3 + len);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
    memcpy(data, &_SpiRxBuf[3], len);
}

void FillExtMem(uint16_t addr, uint8_t value, uint16_t len)
{
    if (len == 0)
        return;
    if (2 + len > sizeof(_SpiRxBuf))
        return;
    addr &= 0x0fff;
    _SpiTxBuf[0] = 0x20 | (addr >> 8);
    _SpiTxBuf[1] = addr;
    memset(&_SpiTxBuf[2], value, len);
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, 2 + len);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

void I2cTransmit(uint8_t devAddr, uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0))
        return;

    WriteExtMem(0, data, length);
    WriteRegister(SPI_REG_I2C_SLAVEADDR, devAddr >> 1);
    WriteRegister(SPI_REG_DMA_MEMADDR, 0);
    WriteRegister(SPI_REG_DMA_CNT, length - 1);
    WriteRegister(SPI_REG_DMA_CMD, SPI_DMA_CMD_I2C_WRITE);

    WaitDmaBusy();
}

void I2cReceive(uint8_t devAddr, uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0))
        return;

    WriteRegister(SPI_REG_I2C_SLAVEADDR, devAddr >> 1);
    WriteRegister(SPI_REG_DMA_MEMADDR, 0);
    WriteRegister(SPI_REG_DMA_CNT, length - 1);
    WriteRegister(SPI_REG_DMA_CMD, SPI_DMA_CMD_I2C_READ);

    WaitDmaBusy();

    ReadExtMem(0, data, length);
}

void I2cWriteTest(void)
{
    TestMem[0] = 3;
    I2cTransmit(0xC0, TestMem, 1);
    memset(TestMem, 0xCC, sizeof(TestMem));
    WriteExtMem(0, TestMem, sizeof(TestMem));
    I2cReceive(0xC0, TestMem, 1);
}

void SetClockGen(Pif_t *pif)
{
    si5351_output_enable(SI5351_CLK1, 0);
    si5351_output_enable(SI5351_CLK2, 0);

    //si5351_set_freq(25000000, SI5351_CLK0);     // CLK_INT
    si5351_set_freq(pif->Fso_5, SI5351_CLK1);   // FSO/5
    si5351_set_freq(pif->Fsc, SI5351_CLK2);     // FSC

    //si5351_output_enable(SI5351_CLK0, 1);
    si5351_output_enable(SI5351_CLK1, 1);
    si5351_output_enable(SI5351_CLK2, 1);
}

// FSO/5 ist OSC_IN, FSC ist FSEL
void SetClockGen2(uint64_t osc_freq, bool fsel)
{
    si5351_output_enable(SI5351_CLK1, 0);

    if (fsel == false)
    {
        si5351_set_clock_disable(SI5351_CLK2, SI5351_CLK_DISABLE_LOW);  // FSEL low
    }
    else
    {
        si5351_set_clock_disable(SI5351_CLK2, SI5351_CLK_DISABLE_HIGH); // FSEL high
    }

    if (osc_freq != 0)
    {
        si5351_set_freq(osc_freq, SI5351_CLK1);   // OSC_IN
        si5351_output_enable(SI5351_CLK1, 1);
    }
}

void UpdateDerivedClocks(Pif_t * pif)
{
    if (pif->Fsel == true)
    {
        pif->Fso_5 = pif->Crystal * 17 / 5;
    }
    else
    {
        pif->Fso_5 = pif->Crystal * 14 / 5;
    }
    pif->Fsc = pif->Crystal / 4;
}

uint8_t TestController(void)
{
    static const uint8_t ReadControllerCommand[1] = { 0x01 };
    uint16_t siRxCnt;

    WriteExtMem(0, ReadControllerCommand, sizeof(ReadControllerCommand));
    WriteRegister(SPI_REG_SI_CHANNEL, 1);
    WriteRegister(SPI_REG_DMA_MEMADDR, 0);
    WriteRegister(SPI_REG_DMA_CNT, 1);
    WriteRegister(SPI_REG_DMA_CMD, SPI_DMA_CMD_SI_TRANSCEIVE);

    WaitDmaBusy();

    siRxCnt = ReadRegister(SPI_REG_SI_RXCNT);
    if (siRxCnt > sizeof(TestMem))
        siRxCnt = sizeof(TestMem);
    memset(TestMem, 0, sizeof(TestMem));
    ReadExtMem(sizeof(ReadControllerCommand), TestMem, siRxCnt);
    return (uint8_t)siRxCnt;
}

void TestEeprom(void)
{
    static const uint8_t EepromStatusCommand[1] = { 0x00 };
    uint16_t siRxCnt;

    WriteExtMem(0, EepromStatusCommand, sizeof(EepromStatusCommand));
    WriteRegister(SPI_REG_SI_CHANNEL, 5);
    WriteRegister(SPI_REG_DMA_MEMADDR, 0);
    WriteRegister(SPI_REG_DMA_CNT, 1);
    WriteRegister(SPI_REG_DMA_CMD, SPI_DMA_CMD_SI_TRANSCEIVE);

    WaitDmaBusy();

    siRxCnt = ReadRegister(SPI_REG_SI_RXCNT);
    if (siRxCnt > sizeof(TestMem))
        siRxCnt = sizeof(TestMem);
    memset(TestMem, 0, sizeof(TestMem));
    ReadExtMem(sizeof(EepromStatusCommand), TestMem, siRxCnt);
}

void TestEeprom2(void)
{
    uint8_t txBuf[10];
    uint8_t rxBuf[8];
    uint32_t index;

    memset(txBuf, 0, sizeof(txBuf));
    memset(rxBuf, 0, sizeof(rxBuf));

    txBuf[0] = 0;
    EepromTransmitReceive(txBuf, 1, rxBuf, 3);

    for (index = 0; index < 256; index++)
    {
        txBuf[0] = 5;
        txBuf[1] = (uint8_t)index;

        txBuf[2] = (uint8_t)(index + 1);
        txBuf[3] = (uint8_t)(index + 2);
        txBuf[4] = (uint8_t)(index + 3);
        txBuf[5] = (uint8_t)(index + 4);
        txBuf[6] = (uint8_t)(index + 5);
        txBuf[7] = (uint8_t)(index + 6);
        txBuf[8] = (uint8_t)(index + 7);
        txBuf[9] = (uint8_t)(index + 8);
        EepromTransmitReceive(txBuf, 10, rxBuf, 1);

        txBuf[0] = 4;
        EepromTransmitReceive(txBuf, 2, rxBuf, 8);

        while (memcmp(rxBuf, txBuf + 2, 8) != 0)
        {
            txBuf[0]++;
        }

    }

    txBuf[0] = 0;
    EepromTransmitReceive(txBuf, 1, rxBuf, 3);
}

//uint8_t EepromBuffer[2048];
//void CopyEeprom(void)
//{
//    uint8_t txBuf[10];
//    uint8_t rxBuf[8];
//    uint32_t index;
//
//    memset(txBuf, 0, sizeof(txBuf));
//    memset(rxBuf, 0, sizeof(rxBuf));
//
//    txBuf[0] = 0;
//    EepromTransmitReceive(txBuf, 1, rxBuf, 3);
//
//    for (index = 0; index < 256; index++)
//    {
//        txBuf[0] = 4;
//        txBuf[1] = (uint8_t)index;
//
//        EepromTransmitReceive(txBuf, 2, rxBuf, 8);
//        memcpy(EepromBuffer + (index << 3), rxBuf, 8);
//    }
//    txBuf[0] = 0;
//    EepromTransmitReceive(txBuf, 1, rxBuf, 3);
//
//    for (index = 0; index < 256; index++)
//    {
//        txBuf[0] = 5;
//        txBuf[1] = (uint8_t)index;
//        memcpy(txBuf + 2, EepromBuffer + (index << 3), 8);
//        EepromTransmitReceive(txBuf, 10, rxBuf, 1);
//    }
//}

uint8_t EepromTransmitReceive(const uint8_t *txBuf, uint8_t txLen, uint8_t *rxBuf, uint8_t rxLen)
{
    uint8_t res;
    uint16_t siRxCnt;
    uint16_t regs[4];

    // TODO: Remove
//    uint8_t msgLen;
//
//    while (huart1.gState != HAL_UART_STATE_READY) ;
//    memset(TestMem, 0, sizeof(TestMem));
//    if ((txBuf[0] == 7) && (txBuf[1] == 0))
//    {
//        msgLen = sprintf(TestMem, "\e[34;40mPIF  %2d:", txLen);
//    }
//    else
//    {
//        msgLen = sprintf(TestMem, "\e[31;40mPIF  %2d:", txLen);
//    }
//    for (int i = 0; i < txLen; i++)
//    {
//        msgLen += sprintf(&TestMem[msgLen], " %02X", txBuf[i]);
//    }
//    msgLen += sprintf(&TestMem[msgLen], "\r\n");
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TestMem, msgLen);

    WriteExtMem(0, txBuf, txLen);
    regs[0] = 5;                            // SPI_REG_SI_CHANNEL
    regs[1] = 0;                            // SPI_REG_DMA_MEMADDR
    regs[2] = txLen;                        // SPI_REG_DMA_CNT
    regs[3] = SPI_DMA_CMD_SI_TRANSCEIVE;    // SPI_REG_DMA_CMD
    WriteMultipleRegisters(SPI_REG_SI_CHANNEL, regs, 4);

    WaitDmaBusy();

    siRxCnt = ReadRegister(SPI_REG_SI_RXCNT);
    res = rxLen;
    if (siRxCnt < rxLen)
    {
        res |= 0x80;
        rxLen = siRxCnt;
    }
    else if (siRxCnt != rxLen)
    {
        res |= 0x40;
    }
    ReadExtMem(txLen, rxBuf, rxLen);

    // TODO: Remove
//    while (huart1.gState != HAL_UART_STATE_READY) ;
//    memset(TestMem, 0, sizeof(TestMem));
//    msgLen = sprintf(TestMem, "\e[33;40mCART %2d:", siRxCnt);
//    for (int i = 0; i < siRxCnt; i++)
//    {
//        msgLen += sprintf(&TestMem[msgLen], " %02X", rxBuf[i]);
//    }
//    msgLen += sprintf(&TestMem[msgLen], "\r\n");
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TestMem, msgLen);

    return res;
}

void SetPifRamBank(uint8_t bank)
{
    bank &= 0x1f;
    WriteRegister(SPI_REG_PIFRAMBANK, bank);
}

uint8_t GetPifRamBank(void)
{
    return ReadRegister(SPI_REG_PIFRAMBANK);
}

void WritePifRamBank(uint8_t bank, uint8_t const *data)
{
    if (data == NULL)
        return;

    SetPifRamBank(bank);
    memcpy(&_SpiTxBuf[1], data, PIFRAM_SIZE);
    _SpiTxBuf[0] = 0x80;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, 65);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}

void WritePifRamComplete(const struct PifRom_t * pifrom)
{
    uint8_t bank;
    uint8_t buf[PIFRAM_SIZE];
    uint16_t bytesToRead;
    uint16_t pos = 0;

    assert(pifrom != NULL);
    assert(pifrom->Type != PR_INVALID);
    assert(pifrom->Data != NULL);

    for (bank = 0; bank < 32; bank++)
    {
        memset(buf, 0, sizeof(buf));
        bytesToRead = 0;

        if (pos < pifrom->Size)
        {
            bytesToRead = pifrom->Size - pos;
            if (bytesToRead > PIFRAM_SIZE)
                bytesToRead = PIFRAM_SIZE;
        }
        memcpy(buf, &pifrom->Data[pos], bytesToRead);

        WritePifRamBank(bank, buf);

        pos += PIFRAM_SIZE;
    }
}

// Copy file to PIF ROM + RAM
// if the file is smaller than 2048 bytes, the rest will be set to 0
// if the file does not exist, all bytes are set to 0
// return true if the file exists
bool WritePifRamCompleteFromFile(char const * fileName)
{
    bool res;
    uint8_t bank;
    spiffs_file fil;
    uint8_t buf[PIFRAM_SIZE];

    fil = SPIFFS_open(&fs, fileName, SPIFFS_O_RDONLY, 0);
    res = (fil > 0) ? true : false;
    for (bank = 0; bank < 32; bank++)
    {
        memset(buf, 0, sizeof(buf));
        if (fil > 0)
            SPIFFS_read(&fs, fil, buf, sizeof(buf));
        WritePifRamBank(bank, buf);
    }
    if (fil > 0)
        SPIFFS_close(&fs, fil);

    return res;
}

void SetCicType(Pif_t *pif, uint8_t type)
{
    memset(pif->PifRamWrite, 0, PIFRAM_SIZE);
    pif->PifRamWrite[0x26] = type;
    pif->PifRamWrite[0x27] = type;
    WritePifRamBank(31, pif->PifRamWrite);
}

void ReadCicNibbles(uint8_t *data, uint8_t count)
{
    uint32_t cnt = 0;
    uint8_t nibbleBuf[4];

    if (data == NULL)
        return;
    if (count == 0)
        return;

    FillExtMem(0, 0xff, 4 * count);
    WriteRegister(SPI_REG_DMA_MEMADDR, 0);
    WriteRegister(SPI_REG_DMA_CNT, 4 * count);
    WriteRegister(SPI_REG_DMA_CMD, SPI_DMA_CMD_CIC_TRANSCEIVE);

    WaitDmaBusy();

    for (cnt = 0; cnt < count; cnt++)
    {
        ReadExtMem(1 + 4 * cnt, nibbleBuf, 4);
        data[cnt] = (nibbleBuf[0] << 3) + (nibbleBuf[1] << 2) + (nibbleBuf[2] << 1) + nibbleBuf[3];
    }
}

void inverse_22b(uint8_t *mem, int start)
{
    int i;
    uint8_t A, nextA;
    A = mem[start];
    nextA = A;
    for (i = start + 1; i < 16; ++i)
    {
        nextA = mem[i];
        mem[i] -= (A + 1);
        if (mem[i] > 16)
            mem[i] += 16;
        A = nextA;
    }
}

void PackNibbles(uint8_t *dest, uint8_t *nibbleSrc, uint8_t numNibbles)
{
    uint8_t cnt;

    cnt = numNibbles >> 1;

    while (cnt--)
    {
        *dest = (*nibbleSrc << 4) | *(nibbleSrc + 1);
        dest++;
        nibbleSrc += 2;
    }
    if (numNibbles & 1)
    {
        *dest = (*nibbleSrc << 4);
    }
}

uint8_t ReadCicRegionAndSeed(uint8_t *seed)
{
    uint8_t cicHello;
    uint8_t seedBuf[16];
    uint16_t delay;

    ReadCicNibbles(&cicHello, 1);

    // Delay a bit
    delay = 5000;
    while (--delay);

    memset(seedBuf, 0, sizeof(seedBuf));
    ReadCicNibbles(&seedBuf[10], 6);

    inverse_22b(seedBuf, 10);
    inverse_22b(seedBuf, 10);

    if (seed != NULL)
    {
        PackNibbles(seed, &seedBuf[12], 4);
    }

    if (cicHello == 1)
        return 0;
    if (cicHello == 5)
        return 1;
    return 0xff;
}

void ReadCicChecksum(Pif_t *pif)
{
    uint8_t checksumBuf[16];
    uint8_t cnt = 0;

    memset(checksumBuf, 0, sizeof(checksumBuf));

    pif->GpioOut &= ~PIF_GPIO_CIC_DCLK;
    WriteGpio(pif->GpioOut);
    while (ReadGpio() & PIF_GPIO_CIC_DI)
    {
        cnt++;
    }
    pif->GpioOut |= PIF_GPIO_CIC_DCLK;
    WriteGpio(pif->GpioOut);

    ReadCicNibbles(checksumBuf, 16);
    inverse_22b(checksumBuf, 0);
    inverse_22b(checksumBuf, 0);
    inverse_22b(checksumBuf, 0);
    inverse_22b(checksumBuf, 0);

    PackNibbles(pif->CicChecksum, &checksumBuf[4], 12);
}

void ReadJedecId(void)
{
    uint8_t cmd[1];
    uint8_t res[3];

    cmd[0] = 0x9f;

    memset(res, 0, sizeof(res));

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 1, 1000);
    HAL_SPI_Receive(&hspi1, res, 3, 1000);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

enum FLASH_Erase_e {
    ERASE_4K,
    ERASE_32K,
    ERASE_64K,
    ERASE_CHIP
};

#define CS_ENABLED GPIO_PIN_RESET
#define CS_DISABLED GPIO_PIN_SET

void FLASH_SetCs(GPIO_PinState pinState)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, pinState);
}

void FLASH_WriteEnable(void)
{
    uint8_t cmd[1];

    cmd[0] = 0x06;

    FLASH_SetCs(CS_ENABLED);

    HAL_SPI_Transmit_DMA(hspi_fpga, cmd, sizeof(cmd));
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    FLASH_SetCs(CS_DISABLED);
}

void FLASH_WaitBusy(void)
{
    uint8_t cmd[1];
    uint8_t res[1];

    cmd[0] = 0x05;

    do
    {
        FLASH_SetCs(CS_ENABLED);

        HAL_SPI_Transmit_DMA(hspi_fpga, cmd, sizeof(cmd));
        while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

        res[0] = 0xff;
        HAL_SPI_Receive_DMA(hspi_fpga, res, sizeof(res));
        while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

        FLASH_SetCs(CS_DISABLED);
    } while (res[0] & 0x01); // while BUSY = '1'
}

void FLASH_Read(uint32_t addr, uint16_t size, uint8_t *dst)
{
    uint8_t cmd[4];

    cmd[0] = 0x03;
    cmd[1] = (addr >> 16);
    cmd[2] = (addr >> 8);
    cmd[3] = (addr >> 0);

    FLASH_SetCs(CS_ENABLED);

    HAL_SPI_Transmit_DMA(hspi_fpga, cmd, sizeof(cmd));
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    HAL_SPI_Receive_DMA(hspi_fpga, dst, size);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    FLASH_SetCs(CS_DISABLED);
}

void FLASH_Write(uint32_t addr, uint16_t size, uint8_t *src)
{
    uint8_t cmd[4];

    cmd[0] = 0x02;
    cmd[1] = (addr >> 16);
    cmd[2] = (addr >> 8);
    cmd[3] = (addr >> 0);

    FLASH_SetCs(CS_ENABLED);

    HAL_SPI_Transmit_DMA(hspi_fpga, cmd, sizeof(cmd));
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    HAL_SPI_Transmit_DMA(hspi_fpga, src, (uint16_t)size);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    FLASH_SetCs(CS_DISABLED);
}

void FLASH_Erase(uint32_t addr, enum FLASH_Erase_e eraseMode)
{
    uint8_t cmd[4];
    uint8_t cmdLen;

    // default lenght is 4
    cmdLen = 4;

    // select erase command
    switch (eraseMode)
    {
    case ERASE_4K:
        cmd[0] = 0x20;
        break;
    case ERASE_32K:
        cmd[0] = 0x52;
        break;
    case ERASE_64K:
        cmd[0] = 0xd8;
        break;
    case ERASE_CHIP:
        cmd[0] = 0x60;

        // send EraseChip command without any address
        cmdLen = 1;
        break;
    }

    // write address in all cases
    cmd[1] = (addr >> 16);
    cmd[2] = (addr >> 8);
    cmd[3] = (addr >> 0);

    FLASH_SetCs(CS_ENABLED);

    HAL_SPI_Transmit_DMA(hspi_fpga, cmd, cmdLen);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;

    FLASH_SetCs(CS_DISABLED);
}

static s32_t my_spiffs_read(u32_t addr, u32_t size, u8_t *dst)
{
//    _Pif.GpioOut |= PIF_GPIO_LED;
//    WriteGpio(_Pif.GpioOut);

    FLASH_Read(addr, (uint16_t)size, dst);

//    _Pif.GpioOut &= ~PIF_GPIO_LED;
//    WriteGpio(_Pif.GpioOut);

    return SPIFFS_OK;
}

static s32_t my_spiffs_write(u32_t addr, u32_t size, u8_t *src)
{
    if (size > 256)
    {
        return SPIFFS_ERR_INTERNAL;
    }

//    _Pif.GpioOut |= PIF_GPIO_LED;
//    WriteGpio(_Pif.GpioOut);

    FLASH_WriteEnable();
    FLASH_Write(addr, size, src);
    FLASH_WaitBusy();

//    _Pif.GpioOut &= ~PIF_GPIO_LED;
//    WriteGpio(_Pif.GpioOut);

    return SPIFFS_OK;
}

static s32_t my_spiffs_erase(u32_t addr, u32_t size)
{

    if (fs.mounted == 0)
    {
        if (_Pif.GpioOut & PIF_GPIO_LED)
        {
            _Pif.GpioOut &= ~PIF_GPIO_LED;
        }
        else
        {
            _Pif.GpioOut |= PIF_GPIO_LED;
        }
        WriteGpio(_Pif.GpioOut);
    }

    FLASH_WriteEnable();
    FLASH_Erase(addr, ERASE_32K);
    FLASH_WaitBusy();

    return SPIFFS_OK;
}

s32_t my_spiffs_mount()
{

    spiffs_config cfg;

    memset(&cfg, 0, sizeof(spiffs_config));

#if SPIFFS_SINGLETON == 0
    cfg.phys_size = 16*1024*1024 - 128*1024; // use all spi flash
    cfg.phys_addr = 128*1024; // start spiffs at start of spi flash
    cfg.phys_erase_block = 32*1024; // according to datasheet
    cfg.log_block_size = 64*1024; // let us not complicate things
    cfg.log_page_size = LOG_PAGE_SIZE; // as we said
#endif

    cfg.hal_read_f = my_spiffs_read;
    cfg.hal_write_f = my_spiffs_write;
    cfg.hal_erase_f = my_spiffs_erase;

    int res = SPIFFS_mount(&fs,
      &cfg,
      spiffs_work_buf,
      spiffs_fds,
      sizeof(spiffs_fds),
      NULL,
      0,
      0);

//    if (res != SPIFFS_OK)
//    {
//        res = SPIFFS_format(&fs);
//
//        if (res == SPIFFS_OK)
//        {
//            res = SPIFFS_mount(&fs,
//                  &cfg,
//                  spiffs_work_buf,
//                  spiffs_fds,
//                  sizeof(spiffs_fds),
//                  NULL,
//                  0,
//                  0);
//        }
//    }

    //res = SPIFFS_check(&fs);
    return res;
}

void CopyFile(spiffs_file fdst, const char* src)
{
    spiffs_file fsrc;
    uint8_t buf[64];
    int32_t len;

    if (fdst > 0)
    {
        fsrc = SPIFFS_open(&fs, src, SPIFFS_O_RDONLY, 0);

        if (fsrc > 0)
        {
            SPIFFS_lseek(&fs, fdst, 0, SPIFFS_SEEK_SET);
            while (!SPIFFS_eof(&fs, fsrc))
            {
                len = SPIFFS_read(&fs, fsrc, buf, 64);
                if (len > 0)
                {
                    SPIFFS_write(&fs, fdst, buf, len);
                }
            }
        }

        SPIFFS_close(&fs, fsrc);
    }
}

void OpenRomFile(Pif_t * pif, const char * path)
{
    SPIFFS_close(&fs, pif->RomFile);
    pif->LastFileOffset = 0;
    pif->RomFile = SPIFFS_open(&fs, path, SPIFFS_O_RDONLY, 0);
    if (pif->RomFile > 0)
    {
        spiffs_stat stat;
        SPIFFS_fstat(&fs, pif->RomFile, &stat);
        pif->RomSize  = stat.size;
    }
    else
    {
        pif->RomSize  = 0;
    }
}

void LoadRomData(Pif_t * pif)
{
    int fileBufferIndex = pif->LastFileOffset & (FILE_BUFFER_SIZE - 1);

    // the FileBuffer is larger than the PifRam
    // so always load new data from the file when the FileBuffer index is at 0
    if (fileBufferIndex == 0)
    {
        memset( pif->FileBuffer, 0, FILE_BUFFER_SIZE);
        SPIFFS_read(&fs, pif->RomFile, pif->FileBuffer, sizeof(pif->FileBuffer));
    }
    memcpy(pif->PifRamWrite, &pif->FileBuffer[fileBufferIndex], PIFRAM_SIZE);
    pif->LastFileOffset += PIFRAM_SIZE;
}

void WriteU16ToBufferBigEndian(uint8_t *buffer, uint16_t val)
{
    buffer[0] = val >> 8;
    buffer[1] = val >> 0;
}

uint16_t ReadU16FromBufferBigEndian(uint8_t *buffer)
{
    uint16_t val = 0;
    val |= buffer[0] << 8;
    val |= buffer[1] << 0;

    return val;
}

void WriteU32ToBufferBigEndian(uint8_t *buffer, uint32_t val)
{
    buffer[0] = val >> 24;
    buffer[1] = val >> 16;
    buffer[2] = val >> 8;
    buffer[3] = val >> 0;
}

uint32_t ReadU32FromBufferBigEndian(uint8_t *buffer)
{
    uint32_t val = 0;
    val |= buffer[0] << 24;
    val |= buffer[1] << 16;
    val |= buffer[2] << 8;
    val |= buffer[3] << 0;

    return val;
}

void WriteU64ToBufferBigEndian(uint8_t *buffer, uint64_t val)
{
    buffer[0] = val >> 56;
    buffer[1] = val >> 48;
    buffer[2] = val >> 40;
    buffer[3] = val >> 32;
    buffer[4] = val >> 24;
    buffer[5] = val >> 16;
    buffer[6] = val >> 8;
    buffer[7] = val >> 0;
}

uint64_t ReadU64FromBufferBigEndian(uint8_t *buffer)
{
    uint64_t val = 0;
    val |= (uint64_t)buffer[0] << 56;
    val |= (uint64_t)buffer[1] << 48;
    val |= (uint64_t)buffer[2] << 40;
    val |= (uint64_t)buffer[3] << 32;
    val |= (uint64_t)buffer[4] << 24;
    val |= (uint64_t)buffer[5] << 16;
    val |= (uint64_t)buffer[6] << 8;
    val |= (uint64_t)buffer[7] << 0;

    return val;
}

void PIF_ExecuteCommand(Pif_t *pif)
{
    memset(&pif->PifRamWrite[8], 0, PIFRAM_SIZE - 8);
    uint16_t upifCmd = ReadU16FromBufferBigEndian(&pif->PifRamRead[8]);
    //uint8_t handle = pif->PifRamRead[10];
    uint8_t size = pif->PifRamRead[11];
    uint8_t filenamelen = pif->PifRamRead[12];

    char filename[33];
    memset(filename, 0, sizeof(filename));
    int32_t spiffs_ret;

    switch (upifCmd)
    {
    case ULTRAPIF_CMD_PING:
        memcpy(&pif->PifRamWrite, PifUuid, 16);
        break;

    case ULTRAPIF_CMD_GET_VERSION:
    {
        uint32_t fpgaVer = ReadFpgaVersion();

        // Version of UltraPIF FPGA (ID, Major, Minor, Debug)
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[10], fpgaVer);

        // Version of UltraPIF MCU (ID, Major, Minor, Debug)
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[14], MCU_VERSION);
        break;
    }

    case ULTRAPIF_CMD_SET_RGBLED:
    {
        uint16_t red = ReadU16FromBufferBigEndian(&pif->PifRamRead[10]);
        uint16_t green = ReadU16FromBufferBigEndian(&pif->PifRamRead[12]);
        uint16_t blue = ReadU16FromBufferBigEndian(&pif->PifRamRead[14]);
        WriteRgbLed(red, green, blue);
        break;
    }

    case ULTRAPIF_CMD_FILE_OPEN:
    {
        uint8_t mode = pif->PifRamRead[11];
        if (filenamelen < 32)
        {
            if (pif->RomFile > 0)
                SPIFFS_close(&fs, pif->RomFile);
            memcpy(filename, &pif->PifRamRead[13], filenamelen);
            pif->RomFile = SPIFFS_open(&fs, filename, mode, 0);
        }
        if (pif->RomFile <= 0)
        {
            pif->PifRamWrite[10] = 0xff;
        }
        else
        {
            spiffs_stat stat;
            SPIFFS_fstat(&fs, pif->RomFile, &stat);
            WriteU32ToBufferBigEndian(&pif->PifRamWrite[11], stat.size);
        }
        break;
    }

    case ULTRAPIF_CMD_FILE_CLOSE:
        SPIFFS_close(&fs, pif->RomFile);
        pif->RomFile = -1;
        break;

    case ULTRAPIF_CMD_FILE_REMOVE:
        if (filenamelen < 32)
        {
            memcpy(filename, &pif->PifRamRead[13], filenamelen);
            spiffs_ret = SPIFFS_remove(&fs, filename);
            if (spiffs_ret != SPIFFS_OK)
                pif->PifRamWrite[10] = 0xff;
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;

    case ULTRAPIF_CMD_FILE_READ:
        spiffs_ret = SPIFFS_read(&fs, pif->RomFile, &pif->PifRamWrite[12], size);
        if (spiffs_ret >= 0)
        {
            pif->PifRamWrite[11] = spiffs_ret;
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;

    case ULTRAPIF_CMD_FILE_WRITE:
        spiffs_ret = SPIFFS_write(&fs, pif->RomFile, &pif->PifRamRead[12], size);
        if (spiffs_ret >= 0)
        {
            pif->PifRamWrite[11] = spiffs_ret;
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;

    case ULTRAPIF_CMD_FILE_SEEK:
    {
        int32_t offset = ReadU32FromBufferBigEndian(&pif->PifRamRead[11]);
        int whence = pif->PifRamRead[15];

        spiffs_ret = SPIFFS_lseek(&fs, pif->RomFile, offset, whence);

        if (spiffs_ret >= 0)
        {
            WriteU32ToBufferBigEndian(&pif->PifRamWrite[11], spiffs_ret);
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;
    }

    case ULTRAPIF_CMD_FILE_TELL:
    {
        s32_t pos = SPIFFS_tell(&fs, pif->RomFile);
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[10], (uint32_t)pos);
        break;
    }

    case ULTRAPIF_CMD_DIR_OPEN:
        if (pif->Directory.fs != NULL)
        {
            SPIFFS_closedir(&pif->Directory);
        }

        memcpy(filename, &pif->PifRamRead[13], filenamelen);
        if (NULL == SPIFFS_opendir(&fs, filename, &pif->Directory))
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;

    case ULTRAPIF_CMD_DIR_READ:
    {
        struct spiffs_dirent entry;
        if (NULL != SPIFFS_readdir(&pif->Directory, &entry))
        {
            pif->PifRamWrite[11] = entry.type;
            WriteU32ToBufferBigEndian(&pif->PifRamWrite[12], entry.size);
            memcpy(&pif->PifRamWrite[16], entry.name, 32);
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }

        break;
    }
    case ULTRAPIF_CMD_DIR_CLOSE:
        SPIFFS_closedir(&pif->Directory);
        pif->Directory.fs = NULL;
        break;

    case ULTRAPIF_CMD_GET_FS_INFO:
    {
        u32_t total;
        u32_t used;
        spiffs_ret = SPIFFS_info(&fs, &total, &used);
        if (spiffs_ret != SPIFFS_OK)
        {
            total = 0;
            used = 0;
        }
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[10], total);
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[14], used);
        break;
    }

    case ULTRAPIF_CMD_COLD_RESET:
        if (0 != ReadU32FromBufferBigEndian(&pif->PifRamRead[10]))
        {
            pif->Crystal = ReadU32FromBufferBigEndian(&pif->PifRamRead[10]);
            pif->Fsel = (pif->PifRamRead[14] != 0);
        }
        pif->State = PIF_S_POR;
        pif->ResetSource = RESET_SOURCE_CMD;
        break;

    case ULTRAPIF_CMD_ENABLE_CLOCK:
        if (pif->Crystal == 0)
        {
            pif->Crystal = ReadU32FromBufferBigEndian(&pif->PifRamRead[10]);
            pif->Fsel = (pif->PifRamRead[14] != 0);
            UpdateDerivedClocks(pif);
            if (_Pif.UseDirectVclkMode)
            {
                SetClockGen2(pif->Fso_5, pif->Fsel);
            }
            else
            {
                SetClockGen2(pif->Crystal, pif->Fsel);
            }
        }
        else
        {
            // unable to set the clocks again!
            // use a COLD_RESET!
            pif->PifRamWrite[10] = 0xff;
        }
        break;

    case ULTRAPIF_CMD_GET_CLOCK_RATES:
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[10], pif->Fso_5);
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[14], pif->Fsc);
        WriteU32ToBufferBigEndian(&pif->PifRamWrite[18], pif->Crystal);
        pif->PifRamWrite[22] = pif->Fsel;
        break;

    case ULTRAPIF_CMD_SET_APPBUFFER:
        memcpy(pif->AppBuffer, &pif->PifRamRead[10], APP_BUFFER_SIZE);
        break;

    case ULTRAPIF_CMD_GET_APPBUFFER:
        memcpy(&pif->PifRamWrite[10], pif->AppBuffer, APP_BUFFER_SIZE);
        break;

    case ULTRAPIF_CMD_GET_RESET_SOURCE:
        pif->PifRamWrite[10] = pif->ResetSource;
        break;

    case ULTRAPIF_CMD_SET_CONTROLLERPAK_EMULATION:
    {
        uint8_t controllerNo = pif->PifRamRead[10];
        if (controllerNo < 4)
        {
            pif->PakEmulationInfo[controllerNo].IsEmulationEnabled = (pif->PifRamRead[11] & 0x01);
            if (pif->PakEmulationInfo[controllerNo].IsEmulationEnabled == true)
            {
                if (filenamelen < 32)
                {
                    memcpy(filename, &pif->PifRamRead[13], filenamelen);
                    if (pif->PakEmulationInfo[controllerNo].PakFile > 0)
                    {
                        SPIFFS_close(&fs, pif->PakEmulationInfo[controllerNo].PakFile);
                    }

                    // the file must already exist
                    // TODO: we could create a new file here
                    pif->PakEmulationInfo[controllerNo].PakFile = SPIFFS_open(&fs, filename, SPIFFS_O_RDWR, 0);
                    if (pif->PakEmulationInfo[controllerNo].PakFile <= 0)
                    {
                        pif->PakEmulationInfo[controllerNo].IsEmulationEnabled = false;
                        pif->PifRamWrite[10] = 0xff;
                    }
                    else
                    {
                        // insert the PAK now?
                        if (pif->PifRamRead[11] & 0x02)
                        {
                            pif->PakEmulationInfo[controllerNo].IsPakEnabled = true;
                            pif->PakEmulationInfo[controllerNo].State = PAK_EMUL_INSERTED;
                        }
                        else
                        {
                            pif->PakEmulationInfo[controllerNo].IsPakEnabled = false;
                            pif->PakEmulationInfo[controllerNo].State = PAK_EMUL_REMOVED;
                        }
                    }
                }
            }
            else
            {
                if (pif->PakEmulationInfo[controllerNo].PakFile > 0)
                {
                    SPIFFS_close(&fs, pif->PakEmulationInfo[controllerNo].PakFile);
                    pif->PakEmulationInfo[controllerNo].PakFile = 0;
                }
            }
        }
        else
        {
            pif->PifRamWrite[10] = 0xff;
        }
        break;
    }

    case ULTRAPIF_CMD_SET_RTC_EMULATION:
        pif->IsRtcEmulationEnabled = (pif->PifRamRead[10] != 0);
        break;

    case ULTRAPIF_CMD_SET_EEPROM_EMULATION:
    {
        pif->IsEepromEmulationEnabled = (pif->PifRamRead[10] & 0x01);
        if ((pif->PifRamRead[10] & 0x02) == 0)
        {
            memcpy(pif->EepromType, EepromType4k, sizeof(pif->EepromType));
        }
        else
        {
            memcpy(pif->EepromType, EepromType16k, sizeof(pif->EepromType));
        }

        if (pif->IsEepromEmulationEnabled == true)
        {
            if (filenamelen < 32)
            {
                memcpy(filename, &pif->PifRamRead[13], filenamelen);
                if (pif->EepromFile > 0)
                {
                    SPIFFS_close(&fs, pif->EepromFile);
                }

                // the file must already exist
                // TODO: we could create a new file here
                pif->EepromFile = SPIFFS_open(&fs, filename, SPIFFS_O_RDWR, 0);

                // send back an error if file could not be opened
                if (pif->EepromFile <= 0)
                {
                    pif->IsEepromEmulationEnabled = false;
                    pif->PifRamWrite[10] = 0xff;
                }
            }
            else
            {
                // send back an error if filename is too long
                pif->PifRamWrite[10] = 0xff;
            }
        }
        else
        {
            if (pif->EepromFile > 0)
            {
                SPIFFS_close(&fs, pif->EepromFile);
                pif->EepromFile = 0;
            }
        }

        break;
    }

    case ULTRAPIF_CMD_UPDATE_FPGA:
    {
        spiffs_file updateFile;
        if (filenamelen < 32)
        {
            memcpy(filename, &pif->PifRamRead[13], filenamelen);
            updateFile = SPIFFS_open(&fs, filename, SPIFFS_O_RDONLY, 0);
            if (updateFile > 0)
            {

                FPGA_UpdateFromFile(updateFile);
                SPIFFS_close(&fs, updateFile);
            }
        }
        break;
    }

    case ULTRAPIF_CMD_WRITE_FIRMWAREUPDATE:
    {
        spiffs_file updateFile;
        if (filenamelen < 32)
        {

            memcpy(filename, &pif->PifRamRead[13], filenamelen);
            updateFile = SPIFFS_open(&fs, filename, SPIFFS_O_RDONLY, 0);
            if (updateFile <= 0)
            {
                pif->PifRamWrite[10] = 0xff;
            }
            else
            {
                // Erase first 128 KB
                FLASH_WriteEnable();
                FLASH_Erase(0, ERASE_64K);
                FLASH_WaitBusy();
                FLASH_WriteEnable();
                FLASH_Erase(64 * 1024, ERASE_64K);
                FLASH_WaitBusy();

                uint8_t fileBuf[64];
                uint32_t crc;
                CRC_Initialize(&crc);
                int addr;

                for (addr = 0; addr < 112 * 1024 - 4; addr += sizeof(fileBuf))
                {
                    spiffs_ret = SPIFFS_read(&fs, updateFile, fileBuf, sizeof(fileBuf));

                    if (spiffs_ret > 0)
                    {
                        CRC_CalculateCont(&crc, fileBuf, spiffs_ret);

                        // Write data to flash
                        FLASH_WriteEnable();
                        FLASH_Write(addr, spiffs_ret, fileBuf);
                        FLASH_WaitBusy();
                    }
                    if (spiffs_ret < sizeof(fileBuf))
                        break;
                }
                SPIFFS_close(&fs, updateFile);

                fileBuf[0] = 0xff;
                for (addr += spiffs_ret; addr < 112 * 1024 - 4; addr++)
                {
                    CRC_CalculateCont(&crc, fileBuf, 1);
                }

                CRC_Finalize(&crc);

                // Write CRC to end of 128 KB section
                FLASH_WriteEnable();
                FLASH_Write(112 * 1024 - 4, 4, (uint8_t *)&crc);
                FLASH_WaitBusy();

                // return the calculated CRC
                WriteU32ToBufferBigEndian(&pif->PifRamWrite[11], crc);
            }
        }
        break;
    }

    case ULTRAPIF_CMD_START_FIRMWAREUPDATE:
    {
        uint8_t flags = pif->PifRamRead[10];
        if ((flags & 0x03) != 0)
        {
            // Refresh FPGA
            if (flags & 0x01)
            {
                FPGA_Refresh();
            }

            // update MCU
            if (flags & 0x02)
            {
                // invalidate firmware -> set CRC to 0x00000000
                HAL_FLASH_Unlock();
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000 + 128*1024 - 4, 0);
                HAL_FLASH_Lock();
            }

            // reset the MCU
            __disable_irq();
            HAL_NVIC_SystemReset();

            // will never get here
        }
        else
        {
            // no valid flag set
            pif->PifRamWrite[10] = 0xff;
        }
        break;
    }

    case ULTRAPIF_CMD_DEBUG:
    {
        // wait for DMA to be ready
        while (huart1.gState != HAL_UART_STATE_READY) ;

        // Send debug string to UART (max. 54 characters)
        uint8_t len = strnlen((char *)&pif->PifRamRead[10], 54);
        memcpy(_UartTxBuf, &pif->PifRamRead[10], len);
        HAL_UART_Transmit_DMA(&huart1, _UartTxBuf, len);
        break;
    }
    }
    pif->PifRamWrite[63] = 0;
}

void PIF_HandleRequest(Pif_t *pif)
{
    switch (pif->CmdAddr.Cmd)
    {
    case PIF_RCP_CMD_READ_64:
        if (pif->IsBooting == false)
        {
            memcpy(pif->PifRamWrite, pif->PifRamRead, PIFRAM_SIZE);
            if (PIF_IsCmd(pif->PifRamRead))
            {
                PIF_ExecuteCommand(pif);
            }
            else
            {
                PIF_ParseRam(pif);
            }
            WritePifRam(pif);
        }
        break;

    case PIF_RCP_CMD_READ_4:
        if (pif->IsBooting == false)
        {
            pif->Debug = pif->CmdAddr.Addr;
        }
        break;

    case PIF_RCP_CMD_WRITE_4:
    case PIF_RCP_CMD_WRITE_64:
        pif->State = PIF_S_WAIT_WRITE;
        break;
    }

    WriteRegister(SPI_REG_INT, 1);
}

void PIF_Process(Pif_t *pif)
{
    pif->CurrentTick = HAL_GetTick();
    pif->GpioIn = ReadGpio();

    switch (pif->State)
    {
    case PIF_S_POR:
        pif->GpioOut &= ~PIF_GPIO_COLDRESET;
        pif->GpioOut |= PIF_GPIO_CIC_DCLK | PIF_GPIO_CIC_DO;
        WriteGpio(pif->GpioOut);
        pif->BootTick = pif->CurrentTick;

        if (pif->ResetSource == RESET_SOURCE_BUTTON_LONG)
        {
            // Backup current LED color
            ReadRgbLed(&pif->SavedLedColor[0], &pif->SavedLedColor[1], &pif->SavedLedColor[2]);
            pif->State = PIF_S_LED;
        }
        else
        {
            pif->State = PIF_S_POR_DELAY;
        }
        if (fs.mounted == 0)
        {
            WriteRgbLed(50, 0, 0);

            // mount spiffs during reset
            if (SPIFFS_OK != my_spiffs_mount())
            {
                // Error: Invalid Filesystem
                ErrorBlink(1);
            }
        }
        break;

    case PIF_S_LED:
        WriteRgbLed(100, 0, 0);
        HAL_Delay(50);
        WriteRgbLed(0, 0, 0);
        HAL_Delay(50);

        if ((pif->GpioIn & PIF_GPIO_RESET_BUTTON) != 0)
        {
            // Restore LED
            WriteRgbLed(pif->SavedLedColor[0], pif->SavedLedColor[1], pif->SavedLedColor[2]);
            pif->State = PIF_S_POR_DELAY;
        }
        break;

    case PIF_S_POR_DELAY:
        // wait for PIF_EN but at least 250 ms
        if ((pif->CurrentTick - pif->BootTick > 250)
                && (pif->GpioIn & PIF_GPIO_EN))
        {
            pif->State = PIF_S_READ_CIC;
        }
        break;

    case PIF_S_READ_CIC:

        //  0x3F ;6102/7101
        //  0x78 ;6103/7103
        //  0x85 ;6106/7106
        //  0x3F ;6101/7102
        //  0x91 ;6105/7105
//        SetCicType(0);

        // work with last 64 bytes
        SetPifRamBank(31);

        pif->GpioOut |= PIF_GPIO_COLDRESET; // CR high
        WriteGpio(pif->GpioOut);
        if (pif->Crystal == 0)
        {
            pif->CicRegion = ReadCicRegionAndSeed(pif->CicSeed);
            pif->RomType = (pif->CicRegion) ? PR_PAL : PR_NTSC;
        }

        // check if menu_ultrapif.bin exists
        spiffs_stat stat;
        if (SPIFFS_stat(&fs, _RomFiles[1], &stat) == SPIFFS_OK)
        {
            // menu_ultrapif.bin exists -> load pifrom_ultrapif.bin
            if (false == WritePifRamCompleteFromFile(_PifRomFilename))
            {
                if (false == WritePifRamCompleteFromFile((pif->RomType == PR_PAL) ? "pifrom_pal.bin" : "pifrom_ntsc.bin"))
                {
                    // Error no PIFROM
                    ErrorBlink(2);
                }
            }
        }
        else
        {
            // menu_ultrapif.bin does not exist -> load standard PIF ROM
            if (false == WritePifRamCompleteFromFile((pif->RomType == PR_PAL) ? "pifrom_pal.bin" : "pifrom_ntsc.bin"))
            {
                // Error no PIFROM
                ErrorBlink(3);
            }

            // Setup clocks
            if (pif->RomType == PR_PAL)
            {
                pif->Crystal = 17734475;
                pif->Fsel = false;
            }
            else
            {
                pif->Crystal = 14318180;
                pif->Fsel = true;
            }
        }

        // only start the clocks when the values have been set
        if (pif->Crystal != 0)
        {
            UpdateDerivedClocks(pif);
            if (_Pif.UseDirectVclkMode)
            {
                SetClockGen2(pif->Fso_5, pif->Fsel);
            }
            else
            {
                SetClockGen2(pif->Crystal, pif->Fsel);
            }
        }

        SetCicType(pif, pif->CicSeed[0]);
        pif->State = PIF_S_BOOTING;
        pif->IsBooting = false;
        break;

    case PIF_S_BOOTING:
        if (pif->IsBooting == false)
        {
            pif->RemoteResetPending = false;
            pif->RemoteResetTick = 0;
            pif->ResetBtnHoldTick = 0;
            pif->ResetBtnTick = 0;
            pif->BootTick = pif->CurrentTick;
            pif->GpioOut |= PIF_GPIO_LED;
            pif->GpioOut &= ~(PIF_GPIO_PRENMI | PIF_GPIO_NMI);
            WriteGpio(pif->GpioOut);
        }
        pif->IsBooting = true;
        if (HAL_GPIO_ReadPin(FPGA_IRQ_GPIO_Port, FPGA_IRQ_Pin) == GPIO_PIN_RESET)
        {
            pif->BootTick = pif->CurrentTick;
            ReadMultipleRegisters(SPI_REG_PIFCMD, (uint16_t *)&pif->CmdAddr, 1);
            PIF_HandleRequest(pif);
        }

        // Watchdog
        if (pif->CurrentTick - pif->BootTick > 5000)
        {
            pif->State = PIF_S_POR;
        }
        break;

    case PIF_S_RUNNING:
        if (HAL_GPIO_ReadPin(FPGA_IRQ_GPIO_Port, FPGA_IRQ_Pin) == GPIO_PIN_RESET)
        {
            ReadMultipleRegisters(SPI_REG_PIFCMD, (uint16_t *)&pif->CmdAddr, 1);
            PIF_HandleRequest(pif);
        }

        // Don't care about resets when an RCP write is pending
        if (pif->State != PIF_S_WAIT_WRITE)
        {
            // Handle reset button and remote reset
            if (((pif->GpioIn & PIF_GPIO_RESET_BUTTON) == 0)
                    || ((pif->RemoteResetPending == false)
                            && (pif->RemoteResetTick != 0)
                            && (pif->CurrentTick - pif->RemoteResetTick > 2000)))
            {
                pif->ResetSource = (pif->RemoteResetTick) ? RESET_SOURCE_CONTROLLER : RESET_SOURCE_BUTTON;
                if (pif->RemoteResetTick != 0)
                {
                    pif->RemoteResetPending = true;
                }
                if (pif->ResetBtnHoldTick == 0)
                {
                    pif->ResetBtnHoldTick = pif->CurrentTick;
                }
                else if (pif->CurrentTick - pif->ResetBtnHoldTick > 1500)
                {
                    pif->State = PIF_S_POR;
                    pif->GpioOut &= PIF_GPIO_COLDRESET;
                    WriteGpio(pif->GpioOut);
                    pif->ResetSource = RESET_SOURCE_BUTTON_LONG;

                    // stop the crystal to allow new setup
                    pif->Fsc = 0;
                    pif->Fso_5 = 0;
                    pif->Fsel = false;
                    pif->Crystal = 0;
                    SetClockGen2(0, false);
                }

                if (pif->ResetBtnTick == 0)
                {
                    pif->GpioOut |= PIF_GPIO_PRENMI;
                    WriteGpio(pif->GpioOut);
                    pif->ResetBtnTick = pif->CurrentTick;
                }
            }
            else
            {
                pif->ResetBtnHoldTick = 0;
                if (pif->ResetBtnTick)
                {
                    if ((pif->CurrentTick - pif->ResetBtnTick) > 500)
                    {
                        pif->GpioOut |= PIF_GPIO_NMI;
                        WriteGpio(pif->GpioOut);
                        SetCicType(pif, pif->CicSeed[0]);
                        pif->State = PIF_S_BOOTING;
                    }
                }
            }
        }
        break;

    case PIF_S_WAIT_WRITE:
        if (HAL_GPIO_ReadPin(FPGA_IRQ_GPIO_Port, FPGA_IRQ_Pin) == GPIO_PIN_RESET)
        {
            ReadPifRam(pif);
            memcpy(pif->PifRamWrite, pif->PifRamRead, PIFRAM_SIZE);
            if (pif->CmdAddr.Cmd == PIF_RCP_CMD_WRITE_4)
            {
                // check for page load commands while booting
                if ((pif->IsBooting) && (pif->CmdAddr.Addr == 0x1f4))
                {
                    uint16_t cmd = ((uint16_t)pif->PifRamRead[16] << 8) | pif->PifRamRead[17];
                    uint16_t subcmd = ((uint16_t)pif->PifRamRead[18] << 8) | pif->PifRamRead[19];
                    if (cmd == ULTRAPIF_BCMD_ENABLE)
                    {
                        if (PIF_IsCmd(pif->PifRamRead))
                        {
                            pif->IsUltraPifEnabled = (subcmd & 1);
                        }
                    }
                    else if (pif->IsUltraPifEnabled)
                    {
                        switch (cmd)
                        {
                        case ULTRAPIF_BCMD_OPEN_ROM:
                            pif->RomSize = 0;
                            if (subcmd < 3)
                            {
                                OpenRomFile(pif, _RomFiles[subcmd]);
                            }
                            WriteU32ToBufferBigEndian(&pif->PifRamWrite[20], pif->RomSize);
                            WritePifRam(pif);
                            break;

                        case ULTRAPIF_BCMD_LOAD_ROM_PAGE:
                            LoadRomData(pif);
                            WritePifRam(pif);
                            break;

                        case ULTRAPIF_BCMD_SET_RGBLED:
                        {
                            uint16_t red = (pif->PifRamRead[0] << 8) | pif->PifRamRead[1];
                            uint16_t green = (pif->PifRamRead[2] << 8) | pif->PifRamRead[3];
                            uint16_t blue = (pif->PifRamRead[4] << 8) | pif->PifRamRead[5];
                            WriteRgbLed(red, green, blue);
                            break;
                        }

                        case ULTRAPIF_BCMD_PUTCHAR:
                        {
                            // wait for DMA to be ready
                            while (huart1.gState != HAL_UART_STATE_READY) ;

                            // Send debug char to UART
                            _UartTxBuf[0] = (uint8_t)subcmd;
                            HAL_UART_Transmit_DMA(&huart1, _UartTxBuf, 1);
                            break;
                        }

                        case ULTRAPIF_BCMD_DEBUG:
                        {
                            uint64_t debug64 = pif->PifRamRead[7];
                            debug64 |= (uint64_t)pif->PifRamRead[6] << 8;
                            debug64 |= (uint64_t)pif->PifRamRead[5] << 16;
                            debug64 |= (uint64_t)pif->PifRamRead[4] << 24;
                            debug64 |= (uint64_t)pif->PifRamRead[3] << 32;
                            debug64 |= (uint64_t)pif->PifRamRead[2] << 40;
                            debug64 |= (uint64_t)pif->PifRamRead[1] << 48;
                            debug64 |= (uint64_t)pif->PifRamRead[0] << 56;
                            pif->Debug64 = debug64;
                            break;
                        }
                        }
                    }
                }

                if (pif->CmdAddr.Addr == 0x1ff)
                {
                    if (pif->PifRamWrite[63] == 0x30)
                    {
                        if (pif->IsChechsumValid == false)
                        {
                            ReadCicChecksum(pif);
                            pif->IsChechsumValid = true;
                        }
                        if (memcmp(pif->CicChecksum, &pif->PifRamWrite[50], 6))
                        {
                            // Checksum error
                        }
                        pif->PifRamWrite[63] |= 0x80;
                        WritePifRam(pif);
                    }
                    else if (pif->PifRamWrite[63] == 0xf0)
                    {
                        pif->PifRamWrite[63] = 0x00;
                        WritePifRam(pif);
                    }
                    else if (pif->PifRamWrite[63] == 0x08)
                    {
                        pif->PifRamWrite[63] = 0;
                        pif->IsBooting = false;
                        SPIFFS_close(&fs, pif->RomFile);
                        pif->RomFile = -1;
                        pif->GpioOut &= ~PIF_GPIO_LED;
                        WriteGpio(pif->GpioOut);
                    }
                }
            }
            WriteRegister(SPI_REG_INT, 2);
            pif->State = (pif->IsBooting) ? PIF_S_BOOTING : PIF_S_RUNNING;
        }

        break;
    }
}

void FPGA_Refresh(void)
{
    // 0x79 00 00
    _SpiTxBuf[0] = 0x79;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;

    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_fpga, _SpiTxBuf, 3);
    while (hspi_fpga->State != HAL_SPI_STATE_READY) ;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);
}

void FPGA_UpdateFromFile(spiffs_file updateFile)
{
    // Read Device ID
    // 0xE0 00 00 00
    _SpiTxBuf[0] = 0xE0;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_SPI_Receive(hspi_fpga, _SpiRxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    // Enable Configuration Interface Transparent Mode
    // 0x74 08 00 00
    _SpiTxBuf[0] = 0x74;
    _SpiTxBuf[1] = 0x08;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    // Wait Busy
    // 0xF0 00 00 00
    _SpiTxBuf[0] = 0xF0;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    do
    {
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
        HAL_SPI_Receive(hspi_fpga, _SpiRxBuf, 1, 1000);
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);
    } while (_SpiRxBuf[0]);

    // Erase Flash CF
    // 0x0E 04 00 00
    _SpiTxBuf[0] = 0x0E;
    _SpiTxBuf[1] = 0x04;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    // Read Status Register
    // 0x3C 00 00 00
    _SpiTxBuf[0] = 0x3C;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    do
    {
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
        HAL_SPI_Receive(hspi_fpga, _SpiRxBuf, 4, 1000);
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);
    } while (_SpiRxBuf[2] & (1 << 4)); // BUSY Bit 12

    // FAIL Bit 13?
    if (_SpiRxBuf[2] & (1 << 5))
    {
        while (1) ;
    }

    // Reset Configuration Flash Address
    // 0x46 00 00 00
    _SpiTxBuf[0] = 0x46;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);


    while (!SPIFFS_eof(&fs, updateFile))
    {
        int readBytes;

        // Programm 128 Bit of Data
        // 0x70 00 00 01
        _SpiTxBuf[0] = 0x70;
        _SpiTxBuf[1] = 0x00;
        _SpiTxBuf[2] = 0x00;
        _SpiTxBuf[3] = 0x01;

        readBytes = SPIFFS_read(&fs, updateFile, &_SpiTxBuf[4], 16);

        if (readBytes < 16)
        {
            while (1) ;
        }

        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 20, 1000);
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

        // Wait Busy
        // 0xF0 00 00 00
        _SpiTxBuf[0] = 0xF0;
        _SpiTxBuf[1] = 0x00;
        _SpiTxBuf[2] = 0x00;
        _SpiTxBuf[3] = 0x00;
        do
        {
            HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
            HAL_SPI_Receive(hspi_fpga, _SpiRxBuf, 1, 1000);
            HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);
        } while (_SpiRxBuf[0]);
    }

    // Program DONE
    // 0x5E 00 00 00
    _SpiTxBuf[0] = 0x5E;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(10);

    // Read Status Register
    // 0x3C 00 00 00
    _SpiTxBuf[0] = 0x3C;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    _SpiTxBuf[3] = 0x00;
    do
    {
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
        HAL_SPI_Receive(hspi_fpga, _SpiRxBuf, 4, 1000);
        HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);
    } while (_SpiRxBuf[2] & (1 << 4)); // BUSY Bit 12

    // FAIL Bit 13?
    if (_SpiRxBuf[2] & (1 << 5))
    {
        while (1) ;
    }

    // Disable Configuration Interface Transparent Mode
    // 0x26 0x00 0x00
    _SpiTxBuf[0] = 0x26;
    _SpiTxBuf[1] = 0x00;
    _SpiTxBuf[2] = 0x00;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 3, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(10);

    // Bypass
    // 0xFF 0xFF 0xFF 0xFF
    _SpiTxBuf[0] = 0xFF;
    _SpiTxBuf[1] = 0xFF;
    _SpiTxBuf[2] = 0xFF;
    _SpiTxBuf[3] = 0xFF;
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_fpga, _SpiTxBuf, 4, 1000);
    HAL_GPIO_WritePin(UFM_CS_GPIO_Port, UFM_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(10);
}

// Endless error code blinking
void ErrorBlink(int count)
{
    while (1)
    {
        for (int i = 0; i < 5; i++)
        {
            WriteRgbLed(50, 0, 0);
            HAL_Delay(50);
            WriteRgbLed(0, 0, 0);
            HAL_Delay(50);
        }
        HAL_Delay(250);
        for (int i = 0; i < count; i++)
        {
            WriteRgbLed(50, 10, 0);
            HAL_Delay(200);
            WriteRgbLed(0, 0, 0);
            HAL_Delay(300);
        }
        HAL_Delay(1000);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    __disable_irq();
    memcpy((void *)0x20000000, (void *)0x08004000, 192);
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
    __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  hspi_fpga = &hspi1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  CRC_BuildTable();

  FLASH_WaitBusy();

  // Disable all outputs
  si5351_write(SI5351_OUTPUT_ENABLE_CTRL, 0xff);

  si5351_init(SI5351_CRYSTAL_LOAD_10PF, 0);
  
  // FSEL drive strength
  si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  _Pif.GpioIn = ReadGpio();
  _Pif.UseDirectVclkMode = (_Pif.GpioIn & PIF_GPIO_JP1) ? false : true;

  // XTAL drive strength
  if (_Pif.UseDirectVclkMode)
  {
      // direct mode needs more strength
      si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  }
  else
  {
      // only feeding the MX83x0 PLL
      si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  }

  RTC_DateTypeDef rtcDate;
  HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);

  if ((rtcDate.Date == 1) && (rtcDate.Month == 1) && (rtcDate.WeekDay == 1) && (rtcDate.Year == 0))
  {
      // Reset last date
      // TODO: Backup/Restore in filesystem
      rtcDate.Date = 1;
      rtcDate.Month = 1;
      rtcDate.Year = 0x18;
      rtcDate.WeekDay = RTC_WEEKDAY_MONDAY;
      HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);
  }

//    _Pif.IsRtcEmulationEnabled = true;
//    _Pif.RtcControl = 0x0003;

//  _Pif.IsEepromEmulationEnabled = true;
//  memcpy(_Pif.EepromType, EepromType4k, 3);
//  _Pif.EepromFile = SPIFFS_open(&fs, "eep4k.bin", SPIFFS_O_RDWR, 0);
//  if (_Pif.EepromFile < 0)
//  {
//      _Pif.EepromFile = SPIFFS_open(&fs, "eep4k.bin", SPIFFS_O_CREAT | SPIFFS_O_RDWR, 0);
//      if (_Pif.EepromFile > 0)
//      {
//          // write 2048 bytes 0x00 to EEPROM file
//          for (int i = 0; i < 32; i++)
//          {
//              // use PifRam as dummy
//              SPIFFS_write(&fs, _Pif.EepromFile, _Pif.PifRamWrite, PIFRAM_SIZE);
//          }
//      }
//  }

//  _Pif.PakEmulationInfo[0].IsEmulationEnabled = true;
//  _Pif.PakEmulationInfo[0].PakFile = SPIFFS_open(&fs, "pak1.bin", SPIFFS_O_RDWR, 0);
//  if (_Pif.PakEmulationInfo[0].PakFile < 0)
//  {
//      _Pif.PakEmulationInfo[0].PakFile = SPIFFS_open(&fs, "pak1.bin", SPIFFS_O_CREAT | SPIFFS_O_RDWR, 0);
//      if (_Pif.PakEmulationInfo[0].PakFile > 0)
//      {
//          // write 32768 bytes 0x00 to Pak file
//            for (int i = 0; i < 512; i++)
//            {
//                // use PifRam as dummy
//                SPIFFS_write(&fs, _Pif.PakEmulationInfo[0].PakFile, _Pif.PifRamWrite, PIFRAM_SIZE);
//            }
//      }
//      else
//      {
//          _Pif.PakEmulationInfo[0].IsEmulationEnabled = false;
//      }
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      PIF_Process(&_Pif);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UFM_CS_Pin|FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : FPGA_DONE_Pin */
  GPIO_InitStruct.Pin = FPGA_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FPGA_DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UFM_CS_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = UFM_CS_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FPGA_CS_Pin */
  GPIO_InitStruct.Pin = FPGA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FPGA_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FPGA_IRQ_Pin */
  GPIO_InitStruct.Pin = FPGA_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FPGA_IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
