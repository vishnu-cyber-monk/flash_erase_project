/**
 * @file flash_driver.c
 * @brief Internal Flash driver for BGM220 - using MSC (Memory System Controller)
 *
 * This driver uses the MCU's INTERNAL flash for data storage.
 * The BGM220PC22HNA has 512KB internal flash.
 *
 * Flash layout (typical):
 * - 0x00000000 - 0x00040000: Application code (~256KB)
 * - 0x00040000 - 0x0007FFFF: Data storage area (~256KB)
 */

#include "flash_driver.h"
#include "em_msc.h"
#include "em_cmu.h"
#include "em_system.h"
#include <string.h>
#include <stdio.h>

/* Internal flash parameters for BGM220 */
#define FLASH_PAGE_SIZE         8192    /* 8KB pages on EFR32BG22 */
#define FLASH_BASE_ADDRESS      0x00000000
#define FLASH_SIZE              0x00080000  /* 512KB */

/* Data storage area - use upper portion of flash to avoid code */
#define FLASH_DATA_START        0x00060000  /* Start at 384KB offset */
#define FLASH_DATA_END          0x0007FFFF  /* End at 512KB */

static bool flash_initialized = false;

/***************************************************************************//**
 * @brief Initialize the internal flash driver
 ******************************************************************************/
flash_driver_status_t flash_driver_init(void)
{
    if (flash_initialized) {
        return FLASH_DRIVER_OK;
    }

    printf("\r\n=== INTERNAL FLASH DRIVER INIT ===\r\n");

    /* Get device info */
    printf("Device: BGM220PC22HNA\r\n");
    printf("Flash Size: %lu KB\r\n", (unsigned long)(FLASH_SIZE / 1024));
    printf("Flash Page Size: %lu bytes\r\n", (unsigned long)FLASH_PAGE_SIZE);
    printf("Data Storage Area: 0x%08X - 0x%08X\r\n",
           (unsigned int)FLASH_DATA_START, (unsigned int)FLASH_DATA_END);

    /* Initialize MSC (Memory System Controller) */
    MSC_Init();

    printf("MSC initialized OK\r\n");

    flash_initialized = true;
    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Read data from internal flash
 * @param address Flash address to read from
 * @param data Buffer to store read data
 * @param length Number of bytes to read
 * @return FLASH_DRIVER_OK on success
 ******************************************************************************/
flash_driver_status_t flash_read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (!flash_initialized || !data || length == 0) {
        return FLASH_DRIVER_ERROR;
    }

    /* Validate address range */
    if (address + length > FLASH_SIZE) {
        printf("ERROR: Read address 0x%08X + %lu exceeds flash size\r\n",
               (unsigned int)address, (unsigned long)length);
        return FLASH_DRIVER_ERROR;
    }

    /* Direct memory read - internal flash is memory-mapped */
    memcpy(data, (void *)address, length);

    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Write data to internal flash
 * @param address Flash address to write to (must be word-aligned)
 * @param data Data to write
 * @param length Number of bytes to write
 * @return FLASH_DRIVER_OK on success
 * @note Flash must be erased before writing (bits can only go 1->0)
 ******************************************************************************/
flash_driver_status_t flash_write(uint32_t address, uint8_t *data, uint32_t length)
{
    if (!flash_initialized || !data || length == 0) {
        return FLASH_DRIVER_ERROR;
    }

    /* Validate address range - only allow writes to data area */
    if (address < FLASH_DATA_START || address + length > FLASH_DATA_END) {
        printf("ERROR: Write address 0x%08X outside data area\r\n", (unsigned int)address);
        return FLASH_DRIVER_ERROR;
    }

    /* Address must be word-aligned (4 bytes) */
    if (address & 0x3) {
        printf("ERROR: Write address 0x%08X not word-aligned\r\n", (unsigned int)address);
        return FLASH_DRIVER_ERROR;
    }

    /* Write using MSC */
    MSC_Status_TypeDef status = MSC_WriteWord((uint32_t *)address, data, length);

    if (status != mscReturnOk) {
        printf("ERROR: MSC_WriteWord failed with status %d\r\n", status);
        return FLASH_DRIVER_ERROR;
    }

    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Erase a flash page (8KB on BGM220)
 * @param address Address within the page to erase
 * @return FLASH_DRIVER_OK on success
 ******************************************************************************/
flash_driver_status_t flash_erase_sector(uint32_t address)
{
    if (!flash_initialized) {
        return FLASH_DRIVER_ERROR;
    }

    /* Validate address range - only allow erasing data area */
    if (address < FLASH_DATA_START || address > FLASH_DATA_END) {
        printf("ERROR: Erase address 0x%08X outside data area\r\n", (unsigned int)address);
        return FLASH_DRIVER_ERROR;
    }

    /* Align to page boundary */
    uint32_t page_address = address & ~(FLASH_PAGE_SIZE - 1);

    printf("Erasing page at 0x%08X...\r\n", (unsigned int)page_address);

    /* Erase the page using MSC */
    MSC_Status_TypeDef status = MSC_ErasePage((uint32_t *)page_address);

    if (status != mscReturnOk) {
        printf("ERROR: MSC_ErasePage failed with status %d\r\n", status);
        return FLASH_DRIVER_ERROR;
    }

    printf("Page erased successfully\r\n");
    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Erase a 64KB block (8 pages on BGM220)
 * @param address Address within the block to erase
 * @return FLASH_DRIVER_OK on success
 ******************************************************************************/
flash_driver_status_t flash_erase_block(uint32_t address)
{
    if (!flash_initialized) {
        return FLASH_DRIVER_ERROR;
    }

    /* Validate address range */
    if (address < FLASH_DATA_START || address > FLASH_DATA_END) {
        printf("ERROR: Erase address 0x%08X outside data area\r\n", (unsigned int)address);
        return FLASH_DRIVER_ERROR;
    }

    /* Align to 64KB block boundary */
    uint32_t block_address = address & ~0xFFFF;

    printf("Erasing 64KB block at 0x%08X (8 pages)...\r\n", (unsigned int)block_address);

    /* Erase 8 pages (8 x 8KB = 64KB) */
    for (int i = 0; i < 8; i++) {
        uint32_t page_addr = block_address + (i * FLASH_PAGE_SIZE);

        /* Skip pages outside data area */
        if (page_addr < FLASH_DATA_START || page_addr > FLASH_DATA_END) {
            continue;
        }

        MSC_Status_TypeDef status = MSC_ErasePage((uint32_t *)page_addr);
        if (status != mscReturnOk) {
            printf("ERROR: Failed to erase page %d at 0x%08X\r\n", i, (unsigned int)page_addr);
            return FLASH_DRIVER_ERROR;
        }
    }

    printf("Block erased successfully\r\n");
    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Check if flash is busy (always returns false for internal flash)
 ******************************************************************************/
bool flash_is_busy(void)
{
    /* Internal flash operations are blocking, so never busy when function returns */
    return false;
}

/***************************************************************************//**
 * @brief Read flash status (returns 0 for internal flash - always ready)
 ******************************************************************************/
flash_driver_status_t flash_read_status(uint8_t *status)
{
    if (!status) {
        return FLASH_DRIVER_ERROR;
    }

    /* Internal flash is always ready after operation completes */
    *status = 0x00;  /* Not busy, write enabled */
    return FLASH_DRIVER_OK;
}

/***************************************************************************//**
 * @brief Get flash information
 ******************************************************************************/
void flash_get_info(void)
{
    printf("\r\n=== INTERNAL FLASH INFO ===\r\n");
    printf("Total Flash: %lu KB\r\n", (unsigned long)(FLASH_SIZE / 1024));
    printf("Page Size: %lu bytes\r\n", (unsigned long)FLASH_PAGE_SIZE);
    printf("Data Area Start: 0x%08X\r\n", (unsigned int)FLASH_DATA_START);
    printf("Data Area End: 0x%08X\r\n", (unsigned int)FLASH_DATA_END);
    printf("Data Area Size: %lu KB\r\n",
           (unsigned long)((FLASH_DATA_END - FLASH_DATA_START + 1) / 1024));
}
