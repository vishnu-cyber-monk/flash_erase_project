/**
 * @file flash_data_manager.h
 * @brief Flash-based data logging interface
 */

#ifndef FLASH_DATA_MANAGER_H
#define FLASH_DATA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_types.h"

/* Public functions */
void flash_data_manager_init(void);
bool flash_data_manager_add_sample(sensor_data_t *sample);
bool flash_data_manager_flush_block(void);
bool flash_data_manager_get_next_block(flash_data_block_t *block);
bool flash_data_manager_mark_block_read(uint32_t block_id);
void flash_data_manager_get_status(flash_status_t *status);
uint32_t flash_data_manager_get_unread_count(void);
uint32_t flash_data_manager_get_total_blocks(void);

/* Test/utility functions */
bool flash_data_manager_erase_all(void);

#endif /* FLASH_DATA_MANAGER_H */