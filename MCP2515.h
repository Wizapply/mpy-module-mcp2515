/*
 * MCP2515.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Mina Habibi
 */

/*
 *  Edited on: Jan 4, 2024
 *      Author: Wizapply
 */

#ifndef INC_MCP2515_H_
#define INC_MCP2515_H_

#define MCP_SPI_TIMEOUT     10

typedef enum MCP_CAN_CLOCK
{
    MCP_20MHZ = 0,
    MCP_16MHZ,
    MCP_8MHZ
} MCP_CAN_CLOCK;

typedef enum MCP_CAN_SPEED
{
    CAN_5KBPS = 5000,
    CAN_10KBPS = 10000,
    CAN_20KBPS = 20000,
    CAN_31K25BPS = 31250,
    CAN_33KBPS = 33000,
    CAN_40KBPS = 40000,
    CAN_50KBPS = 50000,
    CAN_80KBPS = 80000,
    CAN_83K3BPS = 83000,
    CAN_95KBPS = 95000,
    CAN_100KBPS = 100000,
    CAN_125KBPS = 125000,
    CAN_200KBPS = 200000,
    CAN_250KBPS = 250000,
    CAN_500KBPS = 500000,
    CAN_1000KBPS = 1000000,
} MCP_CAN_SPEED;

typedef enum MCP_CAN_CLKOUT
{
    CLKOUT_DISABLE = -1,
    CLKOUT_DIV1    = 0x0,
    CLKOUT_DIV2    = 0x1,
    CLKOUT_DIV4    = 0x2,
    CLKOUT_DIV8    = 0x3,
} MCP_CAN_CLKOUT;

typedef enum MCP_ERROR
{
    ERROR_OK        = 0,
    ERROR_FAIL      = 1,
    ERROR_ALLTXBUSY = 2,
    ERROR_FAILINIT  = 3,
    ERROR_FAILTX    = 4,
    ERROR_NOMSG     = 5
} MCP_ERROR;

typedef enum MCP_MASK
{
    MASK0,
    MASK1
} MCP_MASK;

typedef enum MCP_RXF
{
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
} MCP_RXF;

typedef enum MCP_RXBn
{
    RXB0 = 0,
    RXB1 = 1
} MCP_RXBn;

typedef enum MCP_TXBn
{
    TXB0 = 0,
    TXB1 = 1,
    TXB2 = 2
} MCP_TXBn;

typedef enum MCP2515_CANINTF
{
    CANINTF_RX0IF = 0x01,
    CANINTF_RX1IF = 0x02,
    CANINTF_TX0IF = 0x04,
    CANINTF_TX1IF = 0x08,
    CANINTF_TX2IF = 0x10,
    CANINTF_ERRIF = 0x20,
    CANINTF_WAKIF = 0x40,
    CANINTF_MERRF = 0x80
} MCP2515_CANINTF;

typedef enum MCP2515_EFLG
{
    EFLG_RX1OVR = (1<<7),
    EFLG_RX0OVR = (1<<6),
    EFLG_TXBO   = (1<<5),
    EFLG_TXEP   = (1<<4),
    EFLG_RXEP   = (1<<3),
    EFLG_TXWAR  = (1<<2),
    EFLG_RXWAR  = (1<<1),
    EFLG_EWARN  = (1<<0)
} MCP2515_EFLG;


/**
 * @brief Gets the status from the device.
 * @return uint8_t The status byte read from the device.
 */
uint8_t   MCP_getStatus(void);

/**
 * @brief Sets the device to configuration mode.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the configuration mode.
 */
MCP_ERROR MCP_setConfigMode(void);

/**
 * @brief Sets the device to listen-only mode.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the listen-only mode.
 */
MCP_ERROR MCP_setListenOnlyMode(void);

/**
 * @brief Sets the device to sleep mode.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the sleep mode.
 */
MCP_ERROR MCP_setSleepMode(void);

/**
 * @brief Sets the device to loop-back mode.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the loop-back mode.
 */
MCP_ERROR MCP_setLoopbackMode(void);

/**
 * @brief Sets the device to normal mode.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the normal mode.
 */
MCP_ERROR MCP_setNormalMode(void);

/**
 * @brief Sets the device bit rate and configuration.
 *
 * @param canSpeed The desired CAN bus speed (e.g., CAN_500KBPS).
 * @param canClock The device clock frequency (e.g., MCP_8MHZ).
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the bit rate and configuration.
 */
MCP_ERROR MCP_setBitrate(const MCP_CAN_SPEED canSpeed, MCP_CAN_CLOCK canClock);

/**
 * @brief Sets the clock output configuration for the device.
 *
 * @param divisor The clock output divisor (e.g., CLKOUT_DISABLE).
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the clock output configuration.
 */
MCP_ERROR MCP_setClkOut(const MCP_CAN_CLKOUT divisor);

/**
 * @brief Sets the filter mask for the device.
 *
 * This function sets the filter mask for the MCP device based on the
 * specified mask type, whether it is an extended frame, and the
 * provided data value.
 *
 * @param mask The filter mask to be set (e.g., MASK0).
 * @param ext A boolean indicating whether the filter mask is for an
 * extended frame (true) or a standard frame (false).
 * @param ulData The data value used in configuring the filter mask.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the filter mask.
 */
MCP_ERROR MCP_setFilterMask(const MCP_MASK mask, const int ext, const uint32_t ulData);

/**
 * @brief Sets the filter for the device.
 *
 * This function sets the filter for the MCP device based on the
 * specified filter number, whether it is an extended frame, and
 * the provided data value.
 *
 * @param num The filter number to be set (e.g., RXF0).
 * @param ext A boolean indicating whether the filter is for an
 * extended frame (true) or a standard frame (false).
 * @param ulData The data value used in configuring the filter.
 * @return MCP_ERROR An error code indicating the success or failure
 * of setting the filter.
 */
MCP_ERROR MCP_setFilter(const MCP_RXF num, const int ext, const uint32_t ulData);

/**
 * @brief Resets the device to its default state.
 *
 * This function performs a reset operation on the device by
 * sending the reset instruction over SPI. It then initializes
 * various registers and settings to their default values.
 *
 * @return MCP_ERROR An error code indicating the success or failure
 * of the reset operation.
 */
MCP_ERROR MCP_reset(void);

/**
 * @brief Sends a CAN message using the device.
 *
 * This function attempts to send a CAN message using an available
 * transmit buffer on the MCP device. It checks each transmit buffer
 * for availability and sends the message if a free buffer is found.
 *
 * @param frame Pointer to the CAN frame to be sent.
 * @return MCP_ERROR An error code indicating the success or failure
 * of sending the CAN message.
 */
MCP_ERROR MCP_sendMessage(const canFrame *frame);

/**
 * @brief Reads a CAN message from the device.
 *
 * This function reads a CAN message from the device. It checks
 * the receive buffer status and reads the message from the corresponding
 * buffer (RXB0 or RXB1) if a message is available.
 *
 * @param frame Pointer to the structure where the received CAN frame
 * will be stored.
 * @return MCP_ERROR An error code indicating the success or failure
 * of reading the CAN message.
 */
MCP_ERROR MCP_readMessage(canFrame *frame);

/**
 * @brief Checks if a CAN message is available for reception.
 *
 * This function checks the status of the device to determine
 * if a CAN message is available in the receive buffers.
 *
 * @return bool True if a CAN message is available, false otherwise.
 */
int      MCP_checkReceive(void);

/**
 * @brief Gets the error flags from the device.
 *
 * @return uint8_t The error flags from the device.
 */
uint8_t   MCP_getErrorFlags(void);

/**
 * @brief Checks if any error condition is present in the device.
 *
 * @return bool True if an error condition is present, false otherwise.
 */
int      MCP_checkError(void);

/**
 * @brief Clears the receive buffer overflow flags in the device.
 *
 * @return None
 */
void      MCP_clearRXnOVRFlags(void);

/**
 * @brief Gets the interrupt flags from the device.
 *
 * @return uint8_t The interrupt flags from the device.
 */
uint8_t   MCP_getInterrupts(void);

/**
 * @brief Clears the interrupt flags in the device.
 *
 * @return None
 */
void      MCP_clearInterrupts(void);

/**
 * @brief Gets CANINTE register from the device.
 *
 * This function reads the interrupt enable register of the device
 * to obtain information about which interrupts are enabled.
 *
 * @return uint8_t The interrupt mask from the device.
 */
uint8_t   MCP_getInterruptEnableMask(void);

/**
 * @brief Clears the transmit interrupt flags in the device.
 *
 * This function clears the interrupt flags (TX0IF, TX1IF, TX2IF).
 *
 * @return None
 */
void      MCP_clearTXInterrupts(void);

/**
 * @brief Clears the receive buffer overflow flag.
 *
 * @return None
 */
void      MCP_clearRXnOVR(void);

/**
 * @brief Clears the MERR (Message Error) interrupt flag in the device.
 *
 * This bit indicates whether any error occurred during message reception/ transmission.
 *
 * @return None
 */
void      MCP_clearMERR(void);

/**
 * @brief Clears the ERRIF (Error Interrupt Flag) in the device.
 *
 * This bit indicates whether any error occurred in the device.
 *
 * @return None
 */
void      MCP_clearERRIF(void);

/**
 * @brief Reads the error count for receive errors from the device.
 *
 * This function reads the receive error count register of the device
 * to obtain information about the number of errors in received messages.
 *
 * @return uint8_t The error count for receive errors.
 */
uint8_t   MCP_errorCountRX(void);

/**
 * @brief Reads the error count for transmit errors from the device.
 *
 * This function reads the transmit error count register of the device
 * to obtain information about the number of errors in transmitted messages.
 *
 * @return uint8_t The error count for transmit errors.
 */
uint8_t   MCP_errorCountTX(void);

#endif /* INC_MCP2515_H_ */
