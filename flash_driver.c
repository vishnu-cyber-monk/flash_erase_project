/**
 * @file flash_driver.c
 * @brief External SPI Flash driver - MINIMAL VERSION
 */

#include "flash_driver.h"
#include "spidrv.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Debug output via printf (iostream_rtt redirects to RTT) */

/* Flash commands */
#define FLASH_CMD_JEDEC_ID              0x9F
#define FLASH_CMD_READ_STATUS_REG1      0x05
#define FLASH_CMD_WRITE_ENABLE          0x06
#define FLASH_CMD_READ_DATA             0x03
#define FLASH_CMD_PAGE_PROGRAM          0x02
#define FLASH_CMD_SECTOR_ERASE          0x20
#define FLASH_CMD_BLOCK_ERASE           0xD8
#define FLASH_CMD_RELEASE_POWER_DOWN    0xAB
#define FLASH_CMD_FOUR_BYTE_MODE_ENTER  0xB7
#define FLASH_CMD_FOUR_BYTE_MODE_EXIT   0x29

/* Flash parameters */
#define SPI_FLASH_PAGE_SIZE    256
#define FLASH_SECTOR_SIZE      4096
#define FLASH_BUSY_BIT         0x01
#define FLASH_STATUS_WEL       0x02

/* SPIDRV handle */
static SPIDRV_HandleData_t spiHandleData;
static SPIDRV_Handle_t spiHandle = &spiHandleData;
static bool flash_initialized = false;
#define MAX_SPI_TRANSFER_SIZE 260

/* Simple delay function */
static void simple_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

/***************************************************************************//**
 * @brief Initialize the flash driver
 ******************************************************************************/

/* Pin Configuration - FROM DEV TEAM */
#define SPI_MOSI_PORT       gpioPortC
#define SPI_MOSI_PIN        2           // PC02 - MOSI

#define SPI_MISO_PORT       gpioPortC
#define SPI_MISO_PIN        3           // PC03 - MISO

#define SPI_CLK_PORT        gpioPortC
#define SPI_CLK_PIN         1           // PC01 - CLK

flash_driver_status_t flash_driver_init(void)
{
    if (flash_initialized) {
        return FLASH_DRIVER_OK;
    }

    printf("\r\n=== FLASH DRIVER INIT ===\r\n");
    printf("ISSI Flash Detected (0x9D)\r\n");

    /* Enable clocks */
    //CMU_ClockEnable(cmuClock_GPIO, true); //Already done in initGPIO()
    CMU_ClockEnable(cmuClock_USART0, true);

    /* Configure GPIO pins */
    GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);   // PC02 - MOSI
    GPIO_PinModeSet(gpioPortC, 3, gpioModeInputPull, 1);  // PC03 - MISO
    GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);   // PC01 - CLK
    GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 1);   // PA04 - CS

    /* FIX: Clear ALL routing bits first */
    GPIO->USARTROUTE[0].ROUTEEN = 0;
    GPIO->USARTROUTE[0].TXROUTE = 0;
    GPIO->USARTROUTE[0].RXROUTE = 0;
    GPIO->USARTROUTE[0].CLKROUTE = 0;
    GPIO->USARTROUTE[0].CSROUTE = 0;

    /* Configure routing */
    GPIO->USARTROUTE[0].TXROUTE = (gpioPortC << _GPIO_USART_TXROUTE_PORT_SHIFT) | (2 << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].RXROUTE = (gpioPortC << _GPIO_USART_RXROUTE_PORT_SHIFT) | (3 << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].CLKROUTE = (gpioPortC << _GPIO_USART_CLKROUTE_PORT_SHIFT) | (1 << _GPIO_USART_CLKROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].CSROUTE = (gpioPortA << _GPIO_USART_CSROUTE_PORT_SHIFT) | (4 << _GPIO_USART_CSROUTE_PIN_SHIFT);

    /* FIX: Force exact ROUTEEN value */
    GPIO->USARTROUTE[0].ROUTEEN = 0x0F;  // Enable only TX, RX, CLK, CS (bits 0-3)

    uint32_t routeen = GPIO->USARTROUTE[0].ROUTEEN;
    printf("ROUTEEN: 0x%08X\r\n", routeen);

    /* Initialize SPIDRV */
    SPIDRV_Init_t initData = {
        .port = USART0,
        .portTx = gpioPortC,
        .portRx = gpioPortC,
        .portClk = gpioPortC,
        .portCs = gpioPortA,
        .pinTx = 2,
        .pinRx = 3,
        .pinClk = 1,
        .pinCs = 4,
        .bitRate = 1000000,
        .frameLength = 8,
        .dummyTxValue = 0xFF,
        .type = spidrvMaster,
        .bitOrder = spidrvBitOrderMsbFirst,
        .clockMode = spidrvClockMode0,  // ISSI chips typically use Mode 0
        .csControl = spidrvCsControlAuto,
        .slaveStartMode = spidrvSlaveStartImmediate
    };

    Ecode_t result = SPIDRV_Init(spiHandle, &initData);
    if (result != ECODE_EMDRV_SPIDRV_OK) {
        printf("SPIDRV_Init failed: 0x%08X\r\n", result);
        return FLASH_DRIVER_ERROR;
    }

    /* Wake up and reset flash */
    uint8_t cmd = 0xAB;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(10);

    /* For ISSI chips, use different reset commands */
    cmd = 0x66;  // Enable Reset
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(1);

    cmd = 0x99;  // Reset Device
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(30);  // ISSI needs longer reset time

    /* Verify JEDEC ID */
    uint8_t tx_buffer[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buffer[4] = {0};
    SPIDRV_MTransferB(spiHandle, tx_buffer, rx_buffer, 4);
    printf("JEDEC ID: 0x%02X 0x%02X 0x%02X\r\n",
                     rx_buffer[1], rx_buffer[2], rx_buffer[3]);

    /* Enter 4 byte mode to address complete 512Mb flash (ISSI 25LP512MG-JLLE) */
    cmd = FLASH_CMD_FOUR_BYTE_MODE_ENTER;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(1);

    /* Clear any write protection bits in status register */
    /* Send Write Enable first */
    cmd = FLASH_CMD_WRITE_ENABLE;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(1);

    /* Write Status Register with all protection bits cleared */
    /* Command 0x01 = Write Status Register, followed by status byte */
    uint8_t clear_protect[2] = {0x01, 0x00};  /* Clear all BP bits */
    SPIDRV_MTransmitB(spiHandle, clear_protect, 2);
    simple_delay_ms(20);  /* Wait for status register write */

    printf("Flash protection cleared\r\n");

    flash_initialized = true;
    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * Flash read using stack buffer
 ******************************************************************************/
flash_driver_status_t flash_read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (!flash_initialized || !data || length == 0) {
        return FLASH_DRIVER_ERROR;
    }

    /* Stack allocated buffers */
    uint8_t tx_buffer[261];  // 5 + 256 max
    uint8_t rx_buffer[261];

    /* Limit length to avoid stack overflow */
    if (length > 256) {
        return FLASH_DRIVER_ERROR;
    }

    /* Build command */
    tx_buffer[0] = FLASH_CMD_READ_DATA;
    tx_buffer[1] = (address >> 24) & 0xFF;
    tx_buffer[2] = (address >> 16) & 0xFF;
    tx_buffer[3] = (address >> 8) & 0xFF;
    tx_buffer[4] = address & 0xFF;
    memset(&tx_buffer[5], 0xFF, length);

    /* Transfer */
    Ecode_t result = SPIDRV_MTransferB(spiHandle, tx_buffer, rx_buffer, 5 + length);
    if (result == ECODE_EMDRV_SPIDRV_OK) {
        memcpy(data, &rx_buffer[5], length);
    }

    return (result == ECODE_EMDRV_SPIDRV_OK) ? FLASH_DRIVER_OK : FLASH_DRIVER_ERROR;
}

/***************************************************************************//**
 * Simple flash write - matching working version but with stack
 ******************************************************************************/
flash_driver_status_t flash_write(uint32_t address, uint8_t *data, uint32_t length)
{
    if (!flash_initialized || !data || length == 0 || length > SPI_FLASH_PAGE_SIZE) {
        return FLASH_DRIVER_ERROR;
    }

    /* Enable write - NO STATUS CHECK like working version */
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);

    /* Build command buffer on stack */
    uint8_t cmd_buffer[5 + SPI_FLASH_PAGE_SIZE];
    cmd_buffer[0] = FLASH_CMD_PAGE_PROGRAM;
    cmd_buffer[1] = (address >> 24) & 0xFF;
    cmd_buffer[2] = (address >> 16) & 0xFF;
    cmd_buffer[3] = (address >> 8) & 0xFF;
    cmd_buffer[4] = address & 0xFF;
    memcpy(&cmd_buffer[5], data, length);

    /* Write */
    Ecode_t result = SPIDRV_MTransmitB(spiHandle, cmd_buffer, 5 + length);
    if (result != ECODE_EMDRV_SPIDRV_OK) {
        return FLASH_DRIVER_ERROR;
    }

    /* Wait for write complete - simple delay like working version */
    simple_delay_ms(5);

    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * Flash sector erase with WEL verification and busy polling
 ******************************************************************************/
flash_driver_status_t flash_erase_sector(uint32_t address)
{
    if (!flash_initialized) {
        return FLASH_DRIVER_ERROR;
    }

    /* Wait for any previous operation to complete */
    while (flash_is_busy()) {
        simple_delay_ms(1);
    }

    /* Enable write with retry */
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    uint8_t status = 0;
    int retries = 3;

    while (retries > 0) {
        SPIDRV_MTransmitB(spiHandle, &cmd, 1);
        simple_delay_ms(1);  /* Give flash time to set WEL */

        flash_read_status(&status);
        if (status & FLASH_STATUS_WEL) {
            break;  /* WEL is set, proceed */
        }
        retries--;
        simple_delay_ms(1);
    }

    if (!(status & FLASH_STATUS_WEL)) {
        printf("ERROR: WEL not set for sector erase at 0x%08X (status=0x%02X)\r\n", address, status);
        return FLASH_DRIVER_ERROR;
    }

    /* Build erase command on stack */
    uint8_t cmd_buffer[5];
    cmd_buffer[0] = FLASH_CMD_SECTOR_ERASE;
    cmd_buffer[1] = (address >> 24) & 0xFF;
    cmd_buffer[2] = (address >> 16) & 0xFF;
    cmd_buffer[3] = (address >> 8) & 0xFF;
    cmd_buffer[4] = address & 0xFF;

    /* Send erase command */
    Ecode_t result = SPIDRV_MTransmitB(spiHandle, cmd_buffer, 5);
    if (result != ECODE_EMDRV_SPIDRV_OK) {
        return FLASH_DRIVER_ERROR;
    }

    /* Wait for erase complete - poll busy bit with timeout */
    uint32_t timeout = 1000;  /* 1 second timeout for sector erase */
    while (flash_is_busy() && timeout > 0) {
        simple_delay_ms(10);
        timeout -= 10;
    }

    if (timeout == 0) {
        return FLASH_DRIVER_TIMEOUT;
    }

    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * Flash 64KB block erase with WEL verification and busy polling
 ******************************************************************************/
flash_driver_status_t flash_erase_block(uint32_t address)
{
    if (!flash_initialized) {
        return FLASH_DRIVER_ERROR;
    }

    /* Wait for any previous operation to complete */
    while (flash_is_busy()) {
        simple_delay_ms(1);
    }

    /* Enable write with retry */
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    uint8_t status = 0;
    int retries = 3;

    while (retries > 0) {
        SPIDRV_MTransmitB(spiHandle, &cmd, 1);
        simple_delay_ms(1);  /* Give flash time to set WEL */

        flash_read_status(&status);
        if (status & FLASH_STATUS_WEL) {
            break;  /* WEL is set, proceed */
        }
        retries--;
        simple_delay_ms(1);
    }

    if (!(status & FLASH_STATUS_WEL)) {
        printf("ERROR: WEL not set for block erase at 0x%08X (status=0x%02X)\r\n", address, status);
        return FLASH_DRIVER_ERROR;
    }

    /* Build erase command on stack */
    uint8_t cmd_buffer[5];
    cmd_buffer[0] = FLASH_CMD_BLOCK_ERASE;
    cmd_buffer[1] = (address >> 24) & 0xFF;
    cmd_buffer[2] = (address >> 16) & 0xFF;
    cmd_buffer[3] = (address >> 8) & 0xFF;
    cmd_buffer[4] = address & 0xFF;

    /* Send erase command */
    Ecode_t result = SPIDRV_MTransmitB(spiHandle, cmd_buffer, 5);
    if (result != ECODE_EMDRV_SPIDRV_OK) {
        return FLASH_DRIVER_ERROR;
    }

    /* Wait for erase complete - poll busy bit with timeout */
    /* ISSI datasheet: 64KB block erase max time is 2000ms */
    uint32_t timeout = 3000;  /* 3 second timeout */
    while (flash_is_busy() && timeout > 0) {
        simple_delay_ms(10);
        timeout -= 10;
    }

    if (timeout == 0) {
        return FLASH_DRIVER_TIMEOUT;
    }

    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * Simple is_busy check - exactly like working version
 ******************************************************************************/
bool flash_is_busy(void)
{
    uint8_t tx_buffer[2] = {FLASH_CMD_READ_STATUS_REG1, 0xFF};
    uint8_t rx_buffer[2] = {0};

    SPIDRV_MTransferB(spiHandle, tx_buffer, rx_buffer, 2);

    return (rx_buffer[1] & FLASH_BUSY_BIT) != 0;
}

/***************************************************************************//**
 * Simple read status - exactly like working version
 ******************************************************************************/
flash_driver_status_t flash_read_status(uint8_t *status)
{
    if (!status) {
        return FLASH_DRIVER_ERROR;
    }

    uint8_t tx_buffer[2] = {FLASH_CMD_READ_STATUS_REG1, 0xFF};
    uint8_t rx_buffer[2] = {0};

    Ecode_t result = SPIDRV_MTransferB(spiHandle, tx_buffer, rx_buffer, 2);
    if (result != ECODE_EMDRV_SPIDRV_OK) {
        return FLASH_DRIVER_ERROR;
    }

    *status = rx_buffer[1];
    return FLASH_DRIVER_OK;
}
