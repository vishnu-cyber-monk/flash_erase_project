/**
 * @file flash_data_manager.c
 * @brief Manages logging of sensor data to external flash memory.
 */

#include "flash_data_manager.h"
#include "flash_driver.h"
#include "sl_sleeptimer.h"
#include <string.h> // For memset, memcpy
#include <stdio.h>  // For printf (debugging)

// Interval for updating the header in flash (in number of blocks)
#define HEADER_UPDATE_INTERVAL 10

// --- Global State ---
static flash_header_t g_header;
static flash_data_block_t g_current_block;
static uint8_t g_sample_index = 0;
static bool g_initialized = false;

// --- Private Function Prototypes ---
static uint16_t calculate_crc16(const uint8_t *data, uint32_t length);
static void write_header_to_flash(void);

/***************************************************************************//**
 * @brief Initializes the flash data manager.
 ******************************************************************************/
void flash_data_manager_init(void)
{
    //printf("Flash Data Manager: Initializing...\n");

    //printf("Flash Data Manager: Calling flash_driver_init()...\n"); // NEW PRINT
    if (flash_driver_init() != FLASH_DRIVER_OK) {
        //printf("Flash Data Manager: ERROR - Flash driver init failed\n");
        g_initialized = false;
        return;
    }
    //printf("Flash Data Manager: flash_driver_init() returned OK.\n"); // NEW PRINT

    // Try to read the existing header from flash
    //printf("Flash Data Manager: Attempting to read header...\n"); // NEW PRINT
    if (flash_read(FLASH_HEADER_ADDR, (uint8_t*)&g_header, sizeof(flash_header_t)) != FLASH_DRIVER_OK || g_header.magic != 0xDEADBEEF) {
        //printf("Flash Data Manager: No valid header found. Creating a new one.\n");
        
        // Erase the first few sectors to ensure a clean state
        for (int i = 0; i < 4; i++) {
            //printf("Flash Data Manager: Erasing sector %d at 0x%08lX...\n", i, (unsigned long)(FLASH_HEADER_ADDR + i * FLASH_SECTOR_SIZE)); // NEW PRINT
            flash_erase_sector(FLASH_HEADER_ADDR + i * FLASH_SECTOR_SIZE);
            //printf("Flash Data Manager: Sector %d erased.\n", i); // NEW PRINT
        }

        // Initialize a new header
        memset(&g_header, 0, sizeof(flash_header_t));
        g_header.magic = 0xDEADBEEF;
        g_header.version = 1;
        g_header.write_pointer = FLASH_DATA_START;
        g_header.read_pointer = FLASH_DATA_START;
        g_header.oldest_pointer = FLASH_DATA_START;
        //printf("Flash Data Manager: Calling write_header_to_flash()...\n"); // NEW PRINT
        write_header_to_flash();
        //printf("Flash Data Manager: write_header_to_flash() completed.\n"); // NEW PRINT
    } else {
        printf("Flash Data Manager: Valid header found. Total blocks: %lu\n", (unsigned long)g_header.total_blocks);
    }

    // Prepare the first data block
    memset(&g_current_block, 0, sizeof(flash_data_block_t));
    g_sample_index = 0;

    g_initialized = true;
    //printf("Flash Data Manager: Initialization complete.\n");
}

/***************************************************************************//**
 * @brief Adds a new sensor data sample to the current block.
 ******************************************************************************/
/*bool flash_data_manager_add_sample(sensor_data_t *sample)
{
    if (!g_initialized || !sample) {
        return false;
    }

    // Copy the new sample into the current block buffer
    memcpy(&g_current_block.samples[g_sample_index], sample, sizeof(sensor_data_t));
    //printf("Flash Data Manager: Added sample %u to block. Cstrain1 X: %d, Cstrain2 X: %d\n",
           //g_sample_index, sample->Cstrain1, sample->Cstrain2); // Added debug print
    g_sample_index++;

    // If the block is full, flush it to flash
    if (g_sample_index >= 30) {
        return flash_data_manager_flush_block();
    }

    return true;
}*/

/***************************************************************************//**
 * @brief Writes the current data block to flash memory.
 ******************************************************************************/
/*bool flash_data_manager_flush_block(void)
{
    if (!g_initialized || g_sample_index == 0) {
        return false; // Nothing to flush
    }

    //printf("Flash Data Manager: Flushing block with %u samples.\n", g_sample_index);

    // --- Finalize Block Metadata ---
    g_current_block.block_id = g_header.total_blocks;
    g_current_block.sample_count = g_sample_index;
    g_current_block.timestamp_start = g_current_block.samples[0].timestamp;
    g_current_block.timestamp_end = g_current_block.samples[g_sample_index - 1].timestamp;
    //g_current_block.crc16 = calculate_crc16((uint8_t*)&g_current_block, sizeof(flash_data_block_t) - sizeof(uint16_t));

    // --- Write to Flash ---
    // Check if the write pointer needs to wrap around
    if (g_header.write_pointer + sizeof(flash_data_block_t) > FLASH_DATA_END) {
        g_header.write_pointer = FLASH_DATA_START;
        // Overwriting old data, so advance the oldest_pointer
        g_header.oldest_pointer = g_header.write_pointer + sizeof(flash_data_block_t);
    }

    // Erase the sector if we are at the beginning of a new one
    if ((g_header.write_pointer % FLASH_SECTOR_SIZE) == 0) {
        //printf("Flash Data Manager: Erasing sector at 0x%08lX\n", (unsigned long)g_header.write_pointer);
        if (flash_erase_sector(g_header.write_pointer) != FLASH_DRIVER_OK) {
            printf("Flash Data Manager: ERROR - Sector erase failed!\n");
            return false;
        }
    }

    // Write the completed block
    //printf("Flash Data Manager: Writing block %lu to address 0x%08lX\n",
           //(unsigned long)g_header.total_blocks, (unsigned long)g_header.write_pointer); // Added debug print
    if (flash_write(g_header.write_pointer, (uint8_t*)&g_current_block, sizeof(flash_data_block_t)) != FLASH_DRIVER_OK) {
        printf("Flash Data Manager: ERROR - Block write failed!\n");
        return false;
    }

    // --- Update Header ---
    g_header.write_pointer += sizeof(flash_data_block_t);
    g_header.total_blocks++;
    g_header.unread_blocks++;

    // Write header back to flash periodically
    if ((g_header.total_blocks % HEADER_UPDATE_INTERVAL) == 0) {
        write_header_to_flash();
    }

    // --- Reset for Next Block ---
    memset(&g_current_block, 0, sizeof(flash_data_block_t));
    g_sample_index = 0;

    printf("Flash Data Manager: Block %lu written successfully to 0x%x.\n", (unsigned long)g_header.total_blocks - 1, g_header.write_pointer-sizeof(flash_data_block_t));
    return true;
}*?

/***************************************************************************//**
 * @brief Writes the header to flash memory.
 ******************************************************************************/
static void write_header_to_flash(void)
{
    /* FIX: Was inverted - should return if NOT initialized */
    if (!g_initialized) return;

    g_header.crc32 = 0; // CRC calculation would go here if implemented
    flash_erase_sector(FLASH_HEADER_ADDR);
    flash_write(FLASH_HEADER_ADDR, (uint8_t*)&g_header, sizeof(flash_header_t));
    printf("Flash Data Manager: Header updated in flash.\n");
}

/***************************************************************************//**
 * @brief Calculates a simple CRC16.
 ******************************************************************************/
static uint16_t calculate_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // Standard CRC-16-CCITT polynomial
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// --- Stub Implementations for Other Public Functions ---

bool flash_data_manager_get_next_block(flash_data_block_t *block) {
    // This function would read the block at g_header.read_pointer
    // and advance the pointer.
    (void)block;
    return false;
}

bool flash_data_manager_mark_block_read(uint32_t block_id) {
    // This function would update the header to reflect that a block has been read/synced.
    (void)block_id;
    g_header.unread_blocks--;
    return true;
}

// function unusable as flash_status_t does not contain total_blocks or unread_blocks. It needs to be flash_header_t
/*void flash_data_manager_get_status(flash_status_t *status) {
    if (status) {
        memset(status, 0, sizeof(flash_status_t));
        if (g_initialized) {
            status->total_blocks = g_header.total_blocks;
            status->unread_blocks = g_header.unread_blocks;
            // Add more status details here if needed
        }
    }
}*/

uint32_t flash_data_manager_get_unread_count(void) {
    return g_initialized ? g_header.unread_blocks : 0;
}

uint32_t flash_data_manager_get_total_blocks(void) {
    return g_initialized ? g_header.total_blocks : 0;
}

bool flash_data_manager_erase_all(void) {
    if (!g_initialized) return false;
    printf("Flash Data Manager: ERASING ALL DATA...\n");

    /* Erase all 64KB blocks */
    for(uint32_t erase_address = FLASH_HEADER_ADDR; erase_address < FLASH_DATA_END; erase_address += 0x10000){
        flash_driver_status_t status = flash_erase_block(erase_address);
        if (status != FLASH_DRIVER_OK) {
            printf("ERROR: Erase failed at 0x%x (status=%d)\n", (uint32_t)erase_address, status);
            return false;
        }
        printf("Erased 64Kb at 0x%x\n", (uint32_t)erase_address);
    }

    /* Reset header to initial state */
    memset(&g_header, 0, sizeof(flash_header_t));
    g_header.magic = 0xDEADBEEF;
    g_header.version = 1;
    g_header.write_pointer = FLASH_DATA_START;
    g_header.read_pointer = FLASH_DATA_START;
    g_header.oldest_pointer = FLASH_DATA_START;

    /* Write fresh header to flash */
    write_header_to_flash();

    /* Reset current block state */
    memset(&g_current_block, 0, sizeof(flash_data_block_t));
    g_sample_index = 0;

    printf("Erasing Done - Header reinitialized\n");
    return true;
}
