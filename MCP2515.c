/*
 * MCP2515.c
 *
 *  Created on: Nov 19, 2023
 *      Author: Mina Habibi
 */

/*
 *  Edited on: Jan 4, 2024
 *      Author: Wizapply
 */

#include "mpy_header.h"
#include "MCP2515.h"

static const uint8_t CANCTRL_REQOP = 0xE0;
//static const uint8_t CANCTRL_ABAT = 0x10;
//static const uint8_t CANCTRL_OSM = 0x08;
static const uint8_t CANCTRL_CLKEN = 0x04;
static const uint8_t CANCTRL_CLKPRE = 0x03;

typedef enum CANCTRL_REQOP_MODE
{
    CANCTRL_REQOP_NORMAL     = 0x00,
    CANCTRL_REQOP_SLEEP      = 0x20,
    CANCTRL_REQOP_LOOPBACK   = 0x40,
    CANCTRL_REQOP_LISTENONLY = 0x60,
    CANCTRL_REQOP_CONFIG     = 0x80,
    CANCTRL_REQOP_POWERUP    = 0x80
}CANCTRL_REQOP_MODE;

static const uint8_t CANSTAT_OPMOD = 0xE0;
//static const uint8_t CANSTAT_ICOD = 0x0E;

static const uint8_t CNF3_SOF = 0x80;

static const uint8_t TXB_EXIDE_MASK = 0x08;
static const uint8_t DLC_MASK       = 0x0F;
static const uint8_t RTR_MASK       = 0x40;

//static const uint8_t RXBnCTRL_RXM_STD    = 0x20;
//static const uint8_t RXBnCTRL_RXM_EXT    = 0x40;
static const uint8_t RXBnCTRL_RXM_STDEXT = 0x00;
static const uint8_t RXBnCTRL_RXM_MASK   = 0x60;
static const uint8_t RXBnCTRL_RTR        = 0x08;
static const uint8_t RXB0CTRL_BUKT       = 0x04;
static const uint8_t RXB0CTRL_FILHIT_MASK = 0x03;
static const uint8_t RXB1CTRL_FILHIT_MASK = 0x07;
static const uint8_t RXB0CTRL_FILHIT = 0x00;
static const uint8_t RXB1CTRL_FILHIT = 0x01;

static const uint8_t MCP_SIDH = 0;
static const uint8_t MCP_SIDL = 1;
static const uint8_t MCP_EID8 = 2;
static const uint8_t MCP_EID0 = 3;
static const uint8_t MCP_DLC  = 4;
static const uint8_t MCP_DATA = 5;

enum STAT
{
    STAT_RX0IF = (1<<0),
    STAT_RX1IF = (1<<1)
};

static const uint8_t STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF;

enum TXBnCTRL
{
    TXB_ABTF   = 0x40,
    TXB_MLOA   = 0x20,
    TXB_TXERR  = 0x10,
    TXB_TXREQ  = 0x08,
    TXB_TXIE   = 0x04,
    TXB_TXP    = 0x03
};

static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR
                                    | EFLG_RX0OVR
                                    | EFLG_TXBO
                                    | EFLG_TXEP
                                    | EFLG_RXEP;

enum INSTRUCTION
{
    INSTRUCTION_WRITE       = 0x02,
    INSTRUCTION_READ        = 0x03,
    INSTRUCTION_BITMOD      = 0x05,
    INSTRUCTION_LOAD_TX0    = 0x40,
    INSTRUCTION_LOAD_TX1    = 0x42,
    INSTRUCTION_LOAD_TX2    = 0x44,
    INSTRUCTION_RTS_TX0     = 0x81,
    INSTRUCTION_RTS_TX1     = 0x82,
    INSTRUCTION_RTS_TX2     = 0x84,
    INSTRUCTION_RTS_ALL     = 0x87,
    INSTRUCTION_READ_RX0    = 0x90,
    INSTRUCTION_READ_RX1    = 0x94,
    INSTRUCTION_READ_STATUS = 0xA0,
    INSTRUCTION_RX_STATUS   = 0xB0,
    INSTRUCTION_RESET       = 0xC0
};

typedef enum REGISTER
{
    MCP_RXF0SIDH = 0x00,
    MCP_RXF0SIDL = 0x01,
    MCP_RXF0EID8 = 0x02,
    MCP_RXF0EID0 = 0x03,
    MCP_RXF1SIDH = 0x04,
    MCP_RXF1SIDL = 0x05,
    MCP_RXF1EID8 = 0x06,
    MCP_RXF1EID0 = 0x07,
    MCP_RXF2SIDH = 0x08,
    MCP_RXF2SIDL = 0x09,
    MCP_RXF2EID8 = 0x0A,
    MCP_RXF2EID0 = 0x0B,
    MCP_CANSTAT  = 0x0E,
    MCP_CANCTRL  = 0x0F,
    MCP_RXF3SIDH = 0x10,
    MCP_RXF3SIDL = 0x11,
    MCP_RXF3EID8 = 0x12,
    MCP_RXF3EID0 = 0x13,
    MCP_RXF4SIDH = 0x14,
    MCP_RXF4SIDL = 0x15,
    MCP_RXF4EID8 = 0x16,
    MCP_RXF4EID0 = 0x17,
    MCP_RXF5SIDH = 0x18,
    MCP_RXF5SIDL = 0x19,
    MCP_RXF5EID8 = 0x1A,
    MCP_RXF5EID0 = 0x1B,
    MCP_TEC      = 0x1C,
    MCP_REC      = 0x1D,
    MCP_RXM0SIDH = 0x20,
    MCP_RXM0SIDL = 0x21,
    MCP_RXM0EID8 = 0x22,
    MCP_RXM0EID0 = 0x23,
    MCP_RXM1SIDH = 0x24,
    MCP_RXM1SIDL = 0x25,
    MCP_RXM1EID8 = 0x26,
    MCP_RXM1EID0 = 0x27,
    MCP_CNF3     = 0x28,
    MCP_CNF2     = 0x29,
    MCP_CNF1     = 0x2A,
    MCP_CANINTE  = 0x2B,
    MCP_CANINTF  = 0x2C,
    MCP_EFLG     = 0x2D,
    MCP_TXB0CTRL = 0x30,
    MCP_TXB0SIDH = 0x31,
    MCP_TXB0SIDL = 0x32,
    MCP_TXB0EID8 = 0x33,
    MCP_TXB0EID0 = 0x34,
    MCP_TXB0DLC  = 0x35,
    MCP_TXB0DATA = 0x36,
    MCP_TXB1CTRL = 0x40,
    MCP_TXB1SIDH = 0x41,
    MCP_TXB1SIDL = 0x42,
    MCP_TXB1EID8 = 0x43,
    MCP_TXB1EID0 = 0x44,
    MCP_TXB1DLC  = 0x45,
    MCP_TXB1DATA = 0x46,
    MCP_TXB2CTRL = 0x50,
    MCP_TXB2SIDH = 0x51,
    MCP_TXB2SIDL = 0x52,
    MCP_TXB2EID8 = 0x53,
    MCP_TXB2EID0 = 0x54,
    MCP_TXB2DLC  = 0x55,
    MCP_TXB2DATA = 0x56,
    MCP_RXB0CTRL = 0x60,
    MCP_RXB0SIDH = 0x61,
    MCP_RXB0SIDL = 0x62,
    MCP_RXB0EID8 = 0x63,
    MCP_RXB0EID0 = 0x64,
    MCP_RXB0DLC  = 0x65,
    MCP_RXB0DATA = 0x66,
    MCP_RXB1CTRL = 0x70,
    MCP_RXB1SIDH = 0x71,
    MCP_RXB1SIDL = 0x72,
    MCP_RXB1EID8 = 0x73,
    MCP_RXB1EID0 = 0x74,
    MCP_RXB1DLC  = 0x75,
    MCP_RXB1DATA = 0x76
}REGISTER;

#define N_TXBUFFERS 3
#define N_RXBUFFERS 2

static const struct TXBn_REGS
{
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
}TXB[N_TXBUFFERS] = {   {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
                        {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
                        {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}  };

static const struct RXBn_REGS
{
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    MCP2515_CANINTF  CANINTF_RXnIF;
} RXB[N_RXBUFFERS] = {  {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
                        {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}   };

//--------------------------- HAL Functions ---------------------------
spi_inst_t *spi_null = NULL;

void MCP_startSpi(void);
void MCP_endSpi(void);
void MCP_transmitSpi(spi_inst_t *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void MCP_receiveSpi(spi_inst_t *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint32_t MCP_getTick(void);
void MCP_delay(uint32_t delay);

// ---------------------------------------------------------------------

uint8_t MCP_getStatus(void)
{
    uint8_t rxData = 0;

    MCP_startSpi();
    uint8_t txData = INSTRUCTION_READ_STATUS;
    MCP_transmitSpi(spi_null, &txData, 1, MCP_SPI_TIMEOUT);
    MCP_receiveSpi(spi_null, &rxData, 1, MCP_SPI_TIMEOUT);
    MCP_endSpi();

    return rxData;
}

static void MCP_setRegister(const REGISTER reg, const uint8_t value)
{
    uint8_t txData[3] = {0};
    MCP_startSpi();
    txData[0] = INSTRUCTION_WRITE;
    txData[1] = reg;
    txData[2] = value;
    MCP_transmitSpi(spi_null, txData, 3, MCP_SPI_TIMEOUT);
    MCP_endSpi();
}

static void MCP_setRegisters(const REGISTER reg, uint8_t values[], const uint8_t n)
{
    uint8_t txData[2];
    MCP_startSpi();
    txData[0] = INSTRUCTION_WRITE;
    txData[1] = reg;
    MCP_transmitSpi(spi_null, txData, 2, MCP_SPI_TIMEOUT);
    MCP_transmitSpi(spi_null, values, n, MCP_SPI_TIMEOUT);
    MCP_endSpi();
}

static uint8_t MCP_readRegister(const REGISTER reg)
{
    uint8_t txData[2] = {0};
    uint8_t rxData = 0;
    MCP_startSpi();
    txData[0] = INSTRUCTION_READ;
    txData[1] = reg;
    MCP_transmitSpi(spi_null, txData, 2, MCP_SPI_TIMEOUT);
    MCP_receiveSpi(spi_null, &rxData, 1, MCP_SPI_TIMEOUT);
    MCP_endSpi();

    return rxData;
}

static void MCP_readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n)
{
    uint8_t txData[2] = {0};

    MCP_startSpi();
    txData[0] = INSTRUCTION_READ;
    txData[1] = reg;
    MCP_transmitSpi(spi_null, txData, 2, MCP_SPI_TIMEOUT);
    // mcp2515 has auto-increment of address-pointer
    MCP_receiveSpi(spi_null, values, n, MCP_SPI_TIMEOUT);
    MCP_endSpi();
}

static void MCP_modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
    uint8_t txData[4];

    MCP_startSpi();
    txData[0] = INSTRUCTION_BITMOD;
    txData[1] = reg;
    txData[2] = mask;
    txData[3] = data;
    MCP_transmitSpi(spi_null, txData, 4, 10);
    MCP_endSpi();
}

static MCP_ERROR MCP_setMode(const CANCTRL_REQOP_MODE mode)
{
    MCP_modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    uint32_t tickstart = MCP_getTick();
    int modeMatch = 0;
    while((MCP_getTick() - tickstart) < 10)
    {
        uint8_t newmode =  MCP_readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;
        modeMatch = (newmode == mode);
        if (modeMatch)
        {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

MCP_ERROR MCP_setConfigMode(void)
{
    return MCP_setMode(CANCTRL_REQOP_CONFIG);
}

MCP_ERROR MCP_setListenOnlyMode(void)
{
    return MCP_setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP_ERROR MCP_setSleepMode(void)
{
    return MCP_setMode(CANCTRL_REQOP_SLEEP);
}

MCP_ERROR MCP_setLoopbackMode(void)
{
    return MCP_setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP_ERROR MCP_setNormalMode(void)
{
    return MCP_setMode(CANCTRL_REQOP_NORMAL);
}

static void MCP_prepareId(uint8_t *buffer, const int ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext)
    {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else
    {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

MCP_ERROR MCP_setBitrate(const MCP_CAN_SPEED canSpeed, MCP_CAN_CLOCK canClock)
{
    MCP_ERROR error = MCP_setConfigMode();
    if (error != ERROR_OK)
    {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;

    set = 1;
    cfg1 = 0x00; cfg2 = 0x00; cfg3 = 0x00;

    if(canClock == MCP_8MHZ)
    {
        switch (canSpeed)
        {
            case (CAN_5KBPS):    cfg1 = 0x1F; cfg2 = 0xBF; cfg3 = 0x87; break;  // 5      KBPS
            case (CAN_10KBPS):   cfg1 = 0x0F; cfg2 = 0xBF; cfg3 = 0x87; break;  // 10     KBPS
            case (CAN_20KBPS):   cfg1 = 0x07; cfg2 = 0xBF; cfg3 = 0x87; break;  // 20     KBPS
            case (CAN_31K25BPS): cfg1 = 0x07; cfg2 = 0xA4; cfg3 = 0x84; break;  // 31.25  KBPS
            case (CAN_33KBPS):   cfg1 = 0x47; cfg2 = 0xE2; cfg3 = 0x85; break;  // 33.333 KBPS
            case (CAN_40KBPS):   cfg1 = 0x03; cfg2 = 0xBF; cfg3 = 0x87; break;  // 40     Kbps
            case (CAN_50KBPS):   cfg1 = 0x03; cfg2 = 0xB4; cfg3 = 0x86; break;  // 50     Kbps
            case (CAN_80KBPS):   cfg1 = 0x01; cfg2 = 0xBF; cfg3 = 0x87; break;  // 80     Kbps
            case (CAN_100KBPS):  cfg1 = 0x01; cfg2 = 0xB4; cfg3 = 0x86; break;  // 100    Kbps
            case (CAN_125KBPS):  cfg1 = 0x01; cfg2 = 0xB1; cfg3 = 0x85; break;  // 125    Kbps
            case (CAN_200KBPS):  cfg1 = 0x00; cfg2 = 0xB4; cfg3 = 0x86; break;  // 200    Kbps
            case (CAN_250KBPS):  cfg1 = 0x00; cfg2 = 0xB1; cfg3 = 0x85; break;  // 250    Kbps
            case (CAN_500KBPS):  cfg1 = 0x00; cfg2 = 0x90; cfg3 = 0x82; break;  // 500    Kbps
            case (CAN_1000KBPS): cfg1 = 0x00; cfg2 = 0x80; cfg3 = 0x80; break;  // 1      Mbps
            default: set = 0; break;
        }
     }
     else if(canClock == MCP_16MHZ)
     {
        switch (canSpeed)
        {
            case (CAN_5KBPS):    cfg1 = 0x3F; cfg2 = 0xFF; cfg3 = 0x87; break; // 5      Kbps
            case (CAN_10KBPS):   cfg1 = 0x1F; cfg2 = 0xFF; cfg3 = 0x87; break; // 10     Kbps
            case (CAN_20KBPS):   cfg1 = 0x0F; cfg2 = 0xFF; cfg3 = 0x87; break; // 20     Kbps
            case (CAN_33KBPS):   cfg1 = 0x4E; cfg2 = 0xF1; cfg3 = 0x85; break; // 33.333 Kbps
            case (CAN_40KBPS):   cfg1 = 0x07; cfg2 = 0xFF; cfg3 = 0x87; break; // 40     Kbps
            case (CAN_50KBPS):   cfg1 = 0x07; cfg2 = 0xFA; cfg3 = 0x87; break; // 50     Kbps
            case (CAN_80KBPS):   cfg1 = 0x03; cfg2 = 0xFF; cfg3 = 0x87; break; // 80     Kbps
            case (CAN_83K3BPS):  cfg1 = 0x03; cfg2 = 0xBE; cfg3 = 0x07; break; // 83.333 Kbps
            case (CAN_95KBPS):   cfg1 = 0x03; cfg2 = 0xAD; cfg3 = 0x07; break; // 95     Kbps
            case (CAN_100KBPS):  cfg1 = 0x03; cfg2 = 0xFA; cfg3 = 0x87; break; // 100    Kbps
            case (CAN_125KBPS):  cfg1 = 0x03; cfg2 = 0xF0; cfg3 = 0x86; break; // 125    Kbps
            case (CAN_200KBPS):  cfg1 = 0x01; cfg2 = 0xFA; cfg3 = 0x87; break; // 200    Kbps
            case (CAN_250KBPS):  cfg1 = 0x41; cfg2 = 0xF1; cfg3 = 0x85; break; // 250    Kbps
            case (CAN_500KBPS):  cfg1 = 0x00; cfg2 = 0xF0; cfg3 = 0x86; break; // 500    Kbps
            case (CAN_1000KBPS): cfg1 = 0x00; cfg2 = 0xD0; cfg3 = 0x82; break; // 1      Mbps
            default: set = 0; break;
        }
     }
     else if(canClock == MCP_20MHZ)
     {
        switch (canSpeed)
        {
            case (CAN_33KBPS):   cfg1 = 0x0B; cfg2 = 0xFF; cfg3 = 0x87; break; // 33.333 Kbps
            case (CAN_40KBPS):   cfg1 = 0x09; cfg2 = 0xFF; cfg3 = 0x87; break; // 40     Kbps
            case (CAN_50KBPS):   cfg1 = 0x09; cfg2 = 0xFA; cfg3 = 0x87; break; // 50     Kbps
            case (CAN_80KBPS):   cfg1 = 0x04; cfg2 = 0xFF; cfg3 = 0x87; break; // 80     Kbps
            case (CAN_83K3BPS):  cfg1 = 0x04; cfg2 = 0xFE; cfg3 = 0x87; break; // 83.333 Kbps
            case (CAN_100KBPS):  cfg1 = 0x04; cfg2 = 0xFA; cfg3 = 0x87; break; // 100    Kbps
            case (CAN_125KBPS):  cfg1 = 0x03; cfg2 = 0xFA; cfg3 = 0x87; break; // 125    Kbps
            case (CAN_200KBPS):  cfg1 = 0x01; cfg2 = 0xFF; cfg3 = 0x87; break; // 200    Kbps
            case (CAN_250KBPS):  cfg1 = 0x41; cfg2 = 0xFB; cfg3 = 0x86; break; // 250    Kbps
            case (CAN_500KBPS):  cfg1 = 0x00; cfg2 = 0xFA; cfg3 = 0x87; break; // 500    Kbps
            case (CAN_1000KBPS): cfg1 = 0x00; cfg2 = 0xD9; cfg3 = 0x82; break; // 1      Mbps
            default: set = 0; break;
        }
     }
     else {
        set = 0;
     }

    if (set)
    {
        MCP_setRegister(MCP_CNF1, cfg1);
        MCP_setRegister(MCP_CNF2, cfg2);
        MCP_setRegister(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else
    {
        return ERROR_FAIL;
    }
}

MCP_ERROR MCP_setClkOut(const MCP_CAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE)
    {
        /* Turn off CLKEN */
        MCP_modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        /* Turn on CLKOUT for SOF */
        MCP_modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    MCP_modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    MCP_modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    MCP_modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
    return ERROR_OK;
}

MCP_ERROR MCP_setFilterMask(const MCP_MASK mask, const int ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP_setConfigMode();
    if (res != ERROR_OK)
    {
        return res;
    }

    uint8_t tbufdata[4];
    MCP_prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask)
    {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    MCP_setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

MCP_ERROR MCP_setFilter(const MCP_RXF num, const int ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP_setConfigMode();
    if (res != ERROR_OK)
    {
        return res;
    }

    REGISTER reg;

    switch (num)
    {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    MCP_prepareId(tbufdata, ext, ulData);
    MCP_setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

MCP_ERROR MCP_reset(void)
{
    MCP_startSpi();
    uint8_t txData = INSTRUCTION_RESET;
    MCP_transmitSpi(spi_null, &txData, 1, MCP_SPI_TIMEOUT);
    MCP_endSpi();

    MCP_delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    MCP_setRegisters(MCP_TXB0CTRL, zeros, 14);
    MCP_setRegisters(MCP_TXB1CTRL, zeros, 14);
    MCP_setRegisters(MCP_TXB2CTRL, zeros, 14);

    MCP_setRegister(MCP_RXB0CTRL, 0);
    MCP_setRegister(MCP_RXB1CTRL, 0);

    MCP_setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    MCP_modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    MCP_modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    MCP_RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++)
    {
        int ext = (i == 1);
        MCP_ERROR result = MCP_setFilter(filters[i], ext, 0);
        if (result != ERROR_OK)
        {
            return result;
        }
    }

    MCP_MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++)
    {
        MCP_ERROR result = MCP_setFilterMask(masks[i], true, 0);
        if (result != ERROR_OK)
        {
            return result;
        }
    }

    return ERROR_OK;
}

static MCP_ERROR MCP_sendData(const MCP_TXBn txbn, const canFrame *frame)
{
    if (frame->dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    int ext = (frame->id & CAN_EFF_FLAG);
    int rtr = (frame->id & CAN_RTR_FLAG);
    uint32_t id = (frame->id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    MCP_prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->dlc | RTR_MASK) : frame->dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->dlc);

    MCP_setRegisters(txbuf->SIDH, data, 5 + frame->dlc);

    MCP_modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    uint8_t ctrl = MCP_readRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0)
    {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

MCP_ERROR MCP_sendMessage(const canFrame *frame)
{
    if (frame->dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAILTX;
    }

    MCP_TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i=0; i<N_TXBUFFERS; i++)
    {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = MCP_readRegister(txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return MCP_sendData(txBuffers[i], frame);
        }
    }
    return ERROR_ALLTXBUSY;
}


static MCP_ERROR MCP_readData(const MCP_RXBn rxbn, canFrame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5] ={0};

    MCP_readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK )
    {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAIL;
    }

    uint8_t ctrl = MCP_readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR)
    {
        id |= CAN_RTR_FLAG;
    }

    frame->id = id;
    frame->dlc = dlc;

    MCP_readRegisters(rxb->DATA, frame->data, dlc);

    MCP_modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

MCP_ERROR MCP_readMessage(canFrame *frame)
{
    MCP_ERROR rc;
    uint8_t stat = MCP_getStatus();

    if ( stat & STAT_RX0IF )
    {
        rc = MCP_readData(RXB0, frame);
    }
    else if ( stat & STAT_RX1IF )
    {
        rc = MCP_readData(RXB1, frame);
    } else
    {
        rc = ERROR_NOMSG;
    }

    return rc;
}


int MCP_checkReceive(void)
{
    uint8_t res = MCP_getStatus();

    return ((res & STAT_RXIF_MASK) ? true : false);
}

uint8_t MCP_getErrorFlags(void)
{
    return MCP_readRegister(MCP_EFLG);
}

int MCP_checkError(void)
{
    uint8_t eflg = MCP_getErrorFlags();

    return ((eflg & EFLG_ERRORMASK) ? true : false);
}

void MCP_clearRXnOVRFlags(void)
{
    MCP_modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP_getInterrupts(void)
{
    return MCP_readRegister(MCP_CANINTF);
}

void MCP_clearInterrupts(void)
{
    MCP_setRegister(MCP_CANINTF, 0);
}

uint8_t MCP_getInterruptEnableMask(void)
{
    return MCP_readRegister(MCP_CANINTE);
}

void MCP_clearTXInterrupts(void)
{
    MCP_modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP_clearRXnOVR(void)
{
    uint8_t eflg = MCP_getErrorFlags();
    if (eflg != 0)
    {
        MCP_clearRXnOVRFlags();
        MCP_clearInterrupts();
    }

}

void MCP_clearMERR(void)
{
    MCP_modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP_clearERRIF(void)
{
    MCP_modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t MCP_errorCountRX(void)
{
    return MCP_readRegister(MCP_REC);
}

uint8_t MCP_errorCountTX(void)
{
    return MCP_readRegister(MCP_TEC);
}

