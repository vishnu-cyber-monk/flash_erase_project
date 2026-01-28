/**
 * @file sensor_types.h
 * @brief Common sensor data types and definitions
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Status flags for sensor data */
#define STATUS_FLAG_MOTION      0x01
#define STATUS_FLAG_VIBRATION   0x02
#define STATUS_FLAG_TOUCHED     0x04
#define STATUS_FLAG_LOW_BATTERY 0x08
#define STATUS_FLAG_ERROR       0x80

/* Flash memory layout for large external flash (64MB) */
#define FLASH_SECTOR_SIZE       4096
#define FLASH_HEADER_ADDR       0x00000000
#define FLASH_BACKUP_HEADER     0x00001000
#define FLASH_DATA_START        0x00010000
#define FLASH_DATA_END          0x03FFFFFF
//#define FLASH_CONFIG_ADDR       0x03FF0000 //Do not use as data is being stored at address

/* Sensor data structure */
typedef struct {
    uint32_t timestamp;
    //int16_t accel_x;
    //int16_t accel_y;
    //int16_t accel_z;
    //uint16_t strain1;
    //uint16_t strain2;
    uint16_t Cstrain1;
    uint16_t Cstrain2;
    //uint16_t pressure_kpa;
    //uint16_t touch_bitmap;
    //uint8_t slider_position;
    //uint16_t vibration_amplitude;
    //uint16_t battery_mv;
    //int8_t temperature_c;
    //uint8_t touch_count;
    //uint8_t status_flags;
} sensor_data_t;

/* Flash data block structure */
typedef struct {
    uint32_t block_id;
    uint32_t timestamp_start;
    uint32_t timestamp_end;
    uint8_t sample_count;
    sensor_data_t samples[30];
    //uint16_t crc16;
} flash_data_block_t;

/* Flash header structure */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t write_pointer;
    uint32_t read_pointer;
    uint32_t oldest_pointer;
    uint32_t total_blocks;
    uint32_t unread_blocks;
    uint32_t last_sync_time;
    uint32_t crc32;
} flash_header_t;

/* Flash status codes */
typedef enum {
    FLASH_STATUS_OK = 0,
    FLASH_STATUS_ERROR,
    FLASH_STATUS_BUSY,
    FLASH_STATUS_FULL,
    FLASH_STATUS_TB,
    FLASH_STATUS_SRP
} flash_status_t;

/* Note: power_mode_t is defined in power_manager.h - do not redefine here */

#endif /* SENSOR_TYPES_H */
