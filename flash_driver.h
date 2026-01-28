/**
 * @file flash_driver.h
 * @brief Minimal flash driver interface for external SPI flash
 */

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Flash driver status codes */
typedef enum {
    FLASH_DRIVER_OK = 0,
    FLASH_DRIVER_ERROR = 1,
    FLASH_DRIVER_TIMEOUT = 2,
    FLASH_DRIVER_BUSY = 3
} flash_driver_status_t;

/* Flash memory parameters */
#define SPI_FLASH_PAGE_SIZE     256         /* Page size for write operations */
#define FLASH_SECTOR_SIZE       4096        /* Sector size for erase operations */
#define FLASH_BLOCK_SIZE_32K    (32*1024)   /* 32KB block size */
#define FLASH_BLOCK_SIZE_64K    (64*1024)   /* 64KB block size */

/* Basic flash driver functions */
flash_driver_status_t flash_driver_init(void);
flash_driver_status_t flash_read(uint32_t address, uint8_t *data, uint32_t length);
flash_driver_status_t flash_write(uint32_t address, uint8_t *data, uint32_t length);
flash_driver_status_t flash_erase_sector(uint32_t address);
flash_driver_status_t flash_erase_block(uint32_t address);
bool flash_is_busy(void);
flash_driver_status_t flash_read_status(uint8_t *status);

/* Stub functions - not implemented in minimal version */
/*
flash_driver_status_t flash_driver_deinit(void);
flash_driver_status_t flash_wait_ready(uint32_t timeout_ms);
flash_driver_status_t flash_erase_block(uint32_t address, uint32_t size);
flash_driver_status_t flash_erase_chip(void);
flash_driver_status_t flash_power_down(void);
flash_driver_status_t flash_wake_up(void);
flash_driver_status_t flash_write_enable(void);
flash_driver_status_t flash_write_disable(void);
flash_driver_status_t flash_write_status(uint8_t status);
*/

#ifdef __cplusplus
}
#endif

#endif /* FLASH_DRIVER_H */
