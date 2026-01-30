/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include <stdio.h>
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "app.h"
#include "flash_data_manager.h"
#include "flash_driver.h"

#include "em_device.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_iadc.h"

#define CLK_SRC_ADC_FREQ        20000000  // CLK_SRC_ADC
#define CLK_ADC_FREQ            40000  // CLK_ADC - 10 MHz max in normal mode

#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortBPin0;
#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortBPin1;
#define IADC_INPUT_2_PORT_PIN     iadcPosInputPortBPin2;
#define IADC_INPUT_3_PORT_PIN     iadcPosInputPortBPin3;
#define IADC_INPUT_4_PORT_PIN     iadcPosInputPortBPin4;

#define IADC_INPUT_0_BUS          BBUSALLOC
#define IADC_INPUT_1_BUS          BBUSALLOC
#define IADC_INPUT_2_BUS          BBUSALLOC
#define IADC_INPUT_3_BUS          BBUSALLOC
#define IADC_INPUT_4_BUS          BBUSALLOC

#define IADC_INPUT_0_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0
#define IADC_INPUT_1_BUSALLOC     GPIO_BBUSALLOC_BODD0_ADC0
#define IADC_INPUT_2_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0
#define IADC_INPUT_3_BUSALLOC     GPIO_BBUSALLOC_BODD0_ADC0
#define IADC_INPUT_4_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0

#define EM2DEBUG                  1

//static uint8_t advertising_set_handle = 0xff;
//static uint32_t tap_count = 0;
//static uint8_t connection_handle = 0xFF;
//uint16_t characteristic_handle = 0;
//uint8_t *data = NULL;
//uint16_t data_length = 0;

//static volatile double scanResult[5];  // Volts
//static volatile int conv_piezo_result, conv_schmitt_result, conv_acc1_result, conv_acc2_result, conv_acc3_result;

//volatile bool data_avail = false;
//volatile bool thres_reset = true;

//static volatile uint32_t thres_high = 200;
//static volatile uint32_t thres_low = 180;

/*void update_piezo_data(ble_connection_handle) {
  //characteristic_26 -> Piezo Data
    sl_bt_gatt_server_send_notification(
        ble_connection_handle, // Connection handle from a connection event
        0x1b,              // Characteristic handle
        4, // Length of the value
        &conv_piezo_result         // Pointer to the new value
    );
}*/

/*void update_acc1_data(ble_connection_handle) {
  //characteristic_26 -> Piezo Data
    sl_bt_gatt_server_send_notification(
        ble_connection_handle, // Connection handle from a connection event
        0x1b,              // Characteristic handle (Handle copied from piezo)
        4, // Length of the value
        &conv_acc1_result         // Pointer to the new value
    );
}*/

/*void update_ad7147_2C_data(ble_connection_handle) {
  //characteristic_26 -> Piezo Data, Now used for AD7147 data
    sl_bt_gatt_server_send_notification(
        ble_connection_handle, // Connection handle from a connection event
        0x1b,              // Characteristic handle (Handle copied from piezo)
        4, // Length of the value
        &conv_ad7147_result2C         // Pointer to the new value
    );
}*/

/*void update_ad7147_2F_data(ble_connection_handle) {
  //characteristic_26 -> Piezo Data, Now used for AD7147 data
    sl_bt_gatt_server_send_notification(
        ble_connection_handle, // Connection handle from a connection event
        0x1b,              // Characteristic handle (Handle copied from piezo)
        4, // Length of the value
        &conv_ad7147_result2F         // Pointer to the new value
    );
}*/

/*void update_tap_data(ble_connection_handle) {
  //characteristic_29 -> Tap Data
    sl_bt_gatt_server_send_notification(
        ble_connection_handle, // Connection handle from a connection event
        0x1e,              // Characteristic handle
        4, // Length of the value
        &tap_count         // Pointer to the new value
    );
}*/

/*void initIADC(void)
{
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT;

  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  init.warmup = iadcWarmupNormal;

  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[0].vRef = 1210;
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;

  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);
  initScan.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID4;
  initScan.showId = true;
*/
  /*
   * Specify the input channel.  When negInput = iadcNegInputGnd, the
   * conversion is single-ended.
   */
/*
  initScanTable.entries[0].posInput = IADC_INPUT_0_PORT_PIN;
  initScanTable.entries[0].negInput = iadcNegInputGnd;
  initScanTable.entries[0].includeInScan = true;

  initScanTable.entries[1].posInput = IADC_INPUT_1_PORT_PIN;
  initScanTable.entries[1].negInput = iadcNegInputGnd;
  initScanTable.entries[1].includeInScan = true;

  initScanTable.entries[2].posInput = IADC_INPUT_2_PORT_PIN;
  initScanTable.entries[2].negInput = iadcNegInputGnd;
  initScanTable.entries[2].includeInScan = true;

  initScanTable.entries[3].posInput = IADC_INPUT_3_PORT_PIN;
  initScanTable.entries[3].negInput = iadcNegInputGnd;
  initScanTable.entries[3].includeInScan = true;

  initScanTable.entries[4].posInput = IADC_INPUT_4_PORT_PIN;
  initScanTable.entries[4].negInput = iadcNegInputGnd;
  initScanTable.entries[4].includeInScan = true;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize a single-channel conversion
  IADC_initScan(IADC0, &initScan, &initScanTable);

  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
  GPIO->IADC_INPUT_2_BUS |= IADC_INPUT_2_BUSALLOC;
  GPIO->IADC_INPUT_3_BUS |= IADC_INPUT_3_BUSALLOC;
  GPIO->IADC_INPUT_4_BUS |= IADC_INPUT_4_BUSALLOC;

  // Clear any previous interrupt flags
  IADC_clearInt(IADC0, _IADC_IF_MASK);

  // Enable Scan interrupts
  IADC_enableInt(IADC0, IADC_IEN_SCANTABLEDONE);

  // Enable IADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}*/

/**************************************************************************//**
 * @brief  IADC IRQ Handler
 *****************************************************************************/
/* void IADC_IRQHandler(void)
{
  IADC_Result_t result = {0, 0};

  // While the FIFO count is non-zero...
  while (IADC_getScanFifoCnt(IADC0))
  {
    result = IADC_pullScanFifoResult(IADC0);
    scanResult[result.id] = result.data * 2.42 / 0xFFF;
  }
  IADC_setScanMask(IADC0, 0x000F);
  data_avail = true;
  IADC_clearInt(IADC0, IADC_IF_SCANTABLEDONE);
}*/

/*static void initGPIO(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIOINT_Init();

  GPIOINT_CallbackRegister(0, (void *) readout_callback);

  GPIO_ExtIntConfig(gpioPortB, 0, 0, false, true, true); //Use Interrupt 1 on PB00
  //GPIO_ExtIntConfig(gpioPortB, 1, 0, false, true, true); // OR Interrupt 2 on PB01

  GPIO_IntEnable(1 << 0);       // Enable interrupt #0, intNo 0 (from GPIO_ExtIntConfig)
}*/

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  #ifdef EM2DEBUG
  #if (EM2DEBUG == 1)
  EMU->CTRL_SET = EMU_CTRL_EM2DBGEN;
  #endif
  #endif

  // Initialize flash driver
  printf("\r\n\r\n");
  printf("########################################################\r\n");
  printf("#   EXTERNAL SPI FLASH (ISSI 64MB) - FULL TEST SUITE   #\r\n");
  printf("########################################################\r\n");

  flash_driver_status_t status;
  int test_num = 0;
  int tests_passed = 0;
  int tests_failed = 0;

  //==========================================================================
  // INITIALIZATION
  //==========================================================================
  printf("\r\n[INIT] Initializing flash driver...\r\n");
  status = flash_driver_init();
  if (status != FLASH_DRIVER_OK) {
      printf("[INIT] FATAL: Flash driver init failed (status=%d)\r\n", status);
      printf("[INIT] Check SPI connections and JEDEC ID\r\n");
      return;
  }
  printf("[INIT] Flash driver initialized successfully\r\n");

  //==========================================================================
  // TEST 1: BASIC READ TEST
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: BASIC READ OPERATION\r\n", test_num);
  printf("========================================\r\n");

  uint8_t read_buf[32];
  memset(read_buf, 0x00, 32);  // Clear buffer first

  printf("Reading 32 bytes from address 0x00000000...\r\n");
  status = flash_read(0x00000000, read_buf, 32);
  printf("Read status: %d (%s)\r\n", status, status == FLASH_DRIVER_OK ? "OK" : "ERROR");

  if (status == FLASH_DRIVER_OK) {
      printf("Data: ");
      for(int i = 0; i < 32; i++) printf("%02X ", read_buf[i]);
      printf("\r\n");

      // Check if we got real data (not all zeros or all FFs indicating SPI failure)
      int all_zero = 1, all_ff = 1;
      for(int i = 0; i < 32; i++) {
          if(read_buf[i] != 0x00) all_zero = 0;
          if(read_buf[i] != 0xFF) all_ff = 0;
      }

      if (all_zero) {
          printf("WARNING: All bytes are 0x00 - possible SPI communication issue\r\n");
      }
      printf("TEST %d RESULT: PASS (Read operation completed)\r\n", test_num);
      tests_passed++;
  } else {
      printf("TEST %d RESULT: FAIL (Read returned error)\r\n", test_num);
      tests_failed++;
  }

  //==========================================================================
  // TEST 2: SECTOR ERASE TEST (4KB)
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: SECTOR ERASE OPERATION (4KB)\r\n", test_num);
  printf("========================================\r\n");

  uint32_t erase_addr = 0x00010000;  // 64KB offset - safe test area
  uint8_t verify_buf[64];

  printf("Target address: 0x%08lX\r\n", (unsigned long)erase_addr);

  // First write some data to ensure sector is not empty
  printf("Step 1: Writing test pattern before erase...\r\n");
  uint8_t pre_erase_data[64];
  for(int i = 0; i < 64; i++) pre_erase_data[i] = (uint8_t)(i ^ 0xAA);
  flash_erase_sector(erase_addr);  // Erase first to allow write
  status = flash_write(erase_addr, pre_erase_data, 64);
  printf("Pre-write status: %d\r\n", status);

  // Verify data was written
  flash_read(erase_addr, verify_buf, 64);
  printf("Data before erase: ");
  for(int i = 0; i < 16; i++) printf("%02X", verify_buf[i]);
  printf("...\r\n");

  // Count non-0xFF bytes
  int non_ff_before = 0;
  for(int i = 0; i < 64; i++) if(verify_buf[i] != 0xFF) non_ff_before++;
  printf("Non-0xFF bytes before erase: %d\r\n", non_ff_before);

  // NOW ERASE
  printf("\r\nStep 2: Erasing 4KB sector...\r\n");
  status = flash_erase_sector(erase_addr);
  printf("Erase status: %d (%s)\r\n", status,
         status == FLASH_DRIVER_OK ? "OK" :
         status == FLASH_DRIVER_TIMEOUT ? "TIMEOUT" : "ERROR");

  // Verify erase
  printf("\r\nStep 3: Verifying erase (reading back)...\r\n");
  flash_read(erase_addr, verify_buf, 64);
  printf("Data after erase:  ");
  for(int i = 0; i < 16; i++) printf("%02X", verify_buf[i]);
  printf("...\r\n");

  int non_ff_after = 0;
  for(int i = 0; i < 64; i++) if(verify_buf[i] != 0xFF) non_ff_after++;
  printf("Non-0xFF bytes after erase: %d\r\n", non_ff_after);

  if (status == FLASH_DRIVER_OK && non_ff_after == 0) {
      printf("TEST %d RESULT: PASS (Sector erased to 0xFF)\r\n", test_num);
      tests_passed++;
  } else {
      printf("TEST %d RESULT: FAIL (Erase incomplete or error)\r\n", test_num);
      tests_failed++;
  }

  //==========================================================================
  // TEST 3: WRITE AND VERIFY TEST
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: WRITE AND VERIFY OPERATION\r\n", test_num);
  printf("========================================\r\n");

  uint32_t write_addr = 0x00020000;  // 128KB offset
  uint8_t write_data[64];
  uint8_t read_back[64];

  printf("Target address: 0x%08lX\r\n", (unsigned long)write_addr);

  // Generate pseudo-random test pattern (LFSR)
  printf("Step 1: Generating 64-byte pseudo-random pattern (LFSR)...\r\n");
  uint8_t lfsr = 0xAC;  // seed
  for(int i = 0; i < 64; i++) {
      write_data[i] = lfsr;
      uint8_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1;
      lfsr = (lfsr >> 1) | (bit << 7);
  }
  printf("Pattern (first 16 bytes): ");
  for(int i = 0; i < 16; i++) printf("%02X", write_data[i]);
  printf("\r\n");

  // Erase sector first
  printf("\r\nStep 2: Erasing sector before write...\r\n");
  status = flash_erase_sector(write_addr);
  printf("Erase status: %d\r\n", status);

  // Write data
  printf("\r\nStep 3: Writing 64 bytes...\r\n");
  status = flash_write(write_addr, write_data, 64);
  printf("Write status: %d (%s)\r\n", status, status == FLASH_DRIVER_OK ? "OK" : "ERROR");

  // Read back
  printf("\r\nStep 4: Reading back 64 bytes...\r\n");
  memset(read_back, 0x00, 64);  // Clear buffer
  flash_read(write_addr, read_back, 64);
  printf("Read data (first 16 bytes): ");
  for(int i = 0; i < 16; i++) printf("%02X", read_back[i]);
  printf("\r\n");

  // Compare
  printf("\r\nStep 5: Comparing written vs read data...\r\n");
  int mismatches = 0;
  for(int i = 0; i < 64; i++) {
      if(write_data[i] != read_back[i]) {
          mismatches++;
          if(mismatches <= 3) {
              printf("  MISMATCH at byte[%d]: wrote 0x%02X, read 0x%02X\r\n",
                     i, write_data[i], read_back[i]);
          }
      }
  }

  if (mismatches == 0) {
      printf("All 64 bytes match!\r\n");
      printf("TEST %d RESULT: PASS\r\n", test_num);
      tests_passed++;
  } else {
      printf("Total mismatches: %d out of 64 bytes\r\n", mismatches);
      printf("TEST %d RESULT: FAIL\r\n", test_num);
      tests_failed++;
  }

  //==========================================================================
  // TEST 4: BLOCK ERASE TEST (64KB)
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: BLOCK ERASE OPERATION (64KB)\r\n", test_num);
  printf("========================================\r\n");

  uint32_t block_addr = 0x00030000;  // 192KB offset

  printf("Target address: 0x%08lX\r\n", (unsigned long)block_addr);

  // Write data at start and end of block to verify full block erase
  printf("Step 1: Writing markers at block boundaries...\r\n");
  uint8_t marker[4] = {0xDE, 0xAD, 0xBE, 0xEF};

  // Erase first to allow writes
  flash_erase_block(block_addr);

  flash_write(block_addr, marker, 4);
  flash_write(block_addr + 0xFFFC, marker, 4);  // End of 64KB block

  // Verify markers written
  uint8_t check[4];
  flash_read(block_addr, check, 4);
  printf("Marker at start: %02X%02X%02X%02X\r\n", check[0], check[1], check[2], check[3]);
  flash_read(block_addr + 0xFFFC, check, 4);
  printf("Marker at end:   %02X%02X%02X%02X\r\n", check[0], check[1], check[2], check[3]);

  // Erase 64KB block
  printf("\r\nStep 2: Erasing 64KB block...\r\n");
  status = flash_erase_block(block_addr);
  printf("Block erase status: %d (%s)\r\n", status,
         status == FLASH_DRIVER_OK ? "OK" :
         status == FLASH_DRIVER_TIMEOUT ? "TIMEOUT" : "ERROR");

  // Verify both markers are erased
  printf("\r\nStep 3: Verifying block erased...\r\n");
  flash_read(block_addr, check, 4);
  printf("Start after erase: %02X%02X%02X%02X\r\n", check[0], check[1], check[2], check[3]);
  int start_ok = (check[0]==0xFF && check[1]==0xFF && check[2]==0xFF && check[3]==0xFF);

  flash_read(block_addr + 0xFFFC, check, 4);
  printf("End after erase:   %02X%02X%02X%02X\r\n", check[0], check[1], check[2], check[3]);
  int end_ok = (check[0]==0xFF && check[1]==0xFF && check[2]==0xFF && check[3]==0xFF);

  if (status == FLASH_DRIVER_OK && start_ok && end_ok) {
      printf("TEST %d RESULT: PASS (Full 64KB block erased)\r\n", test_num);
      tests_passed++;
  } else {
      printf("TEST %d RESULT: FAIL\r\n", test_num);
      tests_failed++;
  }

  //==========================================================================
  // TEST 5: HIGH ADDRESS TEST (4-byte addressing verification)
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: HIGH ADDRESS TEST (4-byte mode)\r\n", test_num);
  printf("========================================\r\n");

  // Test addresses beyond 16MB require 4-byte addressing
  uint32_t high_addresses[] = {
      0x00100000,   // 1MB
      0x01000000,   // 16MB (boundary of 3-byte addressing)
      0x02000000,   // 32MB
      0x03000000    // 48MB
  };
  const char* addr_names[] = {"1MB", "16MB", "32MB", "48MB"};
  int high_addr_tests = sizeof(high_addresses) / sizeof(high_addresses[0]);
  int high_addr_passed = 0;

  for(int t = 0; t < high_addr_tests; t++) {
      uint32_t addr = high_addresses[t];
      uint8_t test_pattern[8] = {0xCA, 0xFE, 0xBA, 0xBE, 0x12, 0x34, 0x56, 0x78};
      uint8_t readback[8] = {0};

      printf("\r\nTesting %s (0x%08lX):\r\n", addr_names[t], (unsigned long)addr);

      // Erase, write, read, verify
      flash_erase_sector(addr);
      flash_write(addr, test_pattern, 8);
      flash_read(addr, readback, 8);

      printf("  Wrote: ");
      for(int i = 0; i < 8; i++) printf("%02X", test_pattern[i]);
      printf("\r\n  Read:  ");
      for(int i = 0; i < 8; i++) printf("%02X", readback[i]);
      printf("\r\n");

      int match = 1;
      for(int i = 0; i < 8; i++) {
          if(test_pattern[i] != readback[i]) match = 0;
      }

      if(match) {
          printf("  Result: PASS\r\n");
          high_addr_passed++;
      } else {
          printf("  Result: FAIL\r\n");
      }
  }

  if (high_addr_passed == high_addr_tests) {
      printf("\r\nTEST %d RESULT: PASS (All %d addresses verified)\r\n", test_num, high_addr_tests);
      tests_passed++;
  } else {
      printf("\r\nTEST %d RESULT: FAIL (%d/%d addresses failed)\r\n",
             test_num, high_addr_tests - high_addr_passed, high_addr_tests);
      tests_failed++;
  }

  //==========================================================================
  // TEST 6: PAGE BOUNDARY WRITE TEST
  //==========================================================================
  test_num++;
  printf("\r\n========================================\r\n");
  printf("TEST %d: PAGE BOUNDARY WRITE (256 bytes)\r\n", test_num);
  printf("========================================\r\n");

  uint32_t page_addr = 0x00040000;  // 256KB offset
  uint8_t page_data[256];
  uint8_t page_verify[256];

  printf("Target address: 0x%08lX\r\n", (unsigned long)page_addr);
  printf("Writing full 256-byte page...\r\n");

  // Generate pattern
  for(int i = 0; i < 256; i++) {
      page_data[i] = (uint8_t)i;  // Sequential pattern 0x00-0xFF
  }

  // Erase and write
  flash_erase_sector(page_addr);
  status = flash_write(page_addr, page_data, 256);
  printf("Write status: %d\r\n", status);

  // Read back
  flash_read(page_addr, page_verify, 256);

  // Verify
  int page_errors = 0;
  for(int i = 0; i < 256; i++) {
      if(page_data[i] != page_verify[i]) page_errors++;
  }

  printf("First 16 bytes: ");
  for(int i = 0; i < 16; i++) printf("%02X", page_verify[i]);
  printf("\r\n");
  printf("Last 16 bytes:  ");
  for(int i = 240; i < 256; i++) printf("%02X", page_verify[i]);
  printf("\r\n");

  if (status == FLASH_DRIVER_OK && page_errors == 0) {
      printf("TEST %d RESULT: PASS (All 256 bytes verified)\r\n", test_num);
      tests_passed++;
  } else {
      printf("TEST %d RESULT: FAIL (%d errors)\r\n", test_num, page_errors);
      tests_failed++;
  }

  //==========================================================================
  // FINAL SUMMARY
  //==========================================================================
  printf("\r\n");
  printf("########################################################\r\n");
  printf("#                  TEST SUMMARY                        #\r\n");
  printf("########################################################\r\n");
  printf("\r\n");
  printf("  Total Tests:  %d\r\n", test_num);
  printf("  Passed:       %d\r\n", tests_passed);
  printf("  Failed:       %d\r\n", tests_failed);
  printf("\r\n");

  if (tests_failed == 0) {
      printf("  *** ALL TESTS PASSED! ***\r\n");
      printf("  Flash driver is fully operational.\r\n");
  } else {
      printf("  *** SOME TESTS FAILED ***\r\n");
      printf("  Check hardware connections and flash chip.\r\n");
  }

  printf("\r\n");
  printf("########################################################\r\n");

  //==========================================================================
  // FLASH MEMORY DUMP - Read contents at key addresses
  //==========================================================================
  printf("\r\n");
  printf("########################################################\r\n");
  printf("#              FLASH MEMORY DUMP                       #\r\n");
  printf("########################################################\r\n");

  uint8_t dump_buf[256];
  uint32_t dump_addresses[] = {
      0x00000000,   // Start of flash
      0x00010000,   // 64KB - Test 2 sector erase area
      0x00020000,   // 128KB - Test 3 write area
      0x00030000,   // 192KB - Test 4 block erase area
      0x00040000,   // 256KB - Test 6 page write area
      0x00100000,   // 1MB - Test 5 high address
      0x01000000,   // 16MB - Test 5 high address
      0x02000000,   // 32MB - Test 5 high address
      0x03000000,   // 48MB - Test 5 high address
  };
  int num_dumps = sizeof(dump_addresses) / sizeof(dump_addresses[0]);

  for (int d = 0; d < num_dumps; d++) {
      uint32_t addr = dump_addresses[d];
      printf("\r\n--- Address 0x%08lX ---\r\n", (unsigned long)addr);

      // Read 64 bytes at this address
      memset(dump_buf, 0, 64);
      flash_read(addr, dump_buf, 64);

      // Print in hex dump format
      for (int row = 0; row < 4; row++) {
          printf("  %08lX: ", (unsigned long)(addr + row * 16));
          for (int col = 0; col < 16; col++) {
              printf("%02X ", dump_buf[row * 16 + col]);
          }
          printf("\r\n");
      }
  }

  printf("\r\n########################################################\r\n");
  printf("#            END OF FLASH DUMP                         #\r\n");
  printf("########################################################\r\n");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
/*SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////

  //i2cspm_app_process_action();

  if (data_avail == true) {
      if(connection_handle != 0xFF){
          //IADC_command(IADC0, iadcCmdStartScan);

          //conv_piezo_result = (int)(scanResult[0]*1000);
          //conv_schmitt_result = (int)(scanResult[1]*1000);
          //conv_acc1_result = (int)(scanResult[2]*1000);
          //conv_acc2_result = (int)(scanResult[3]*1000);
          //conv_acc3_result = (int)(scanResult[4]*1000);

          //if ((conv_piezo_result > ((int)thres_high)) && thres_reset) {
          //    tap_count++;
          //    update_tap_data(connection_handle);
          //    thres_reset = false;
          //}

          //if (conv_piezo_result < ((int)thres_low)){
          //    thres_reset = true;
          //}
          update_ad7147_2C_data(connection_handle); //Send data via bluetooth
          update_ad7147_2F_data(connection_handle); //Send data via bluetooth
          data_avail = false;
      }

  }
}*/

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
/*void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
//  printf("bluetooth event\n");
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!

    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      connection_handle = evt->data.evt_connection_opened.connection;
      //tap_count = 0;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_gatt_server_attribute_value_id:
      {
        characteristic_handle = evt->data.evt_gatt_server_user_write_request.characteristic;
        data = evt->data.evt_gatt_server_user_write_request.value.data;
        data_length = evt->data.evt_gatt_server_user_write_request.value.len;

        if (characteristic_handle == 0x21) {
          memcpy(&thres_low, data, data_length);
         }

        if (characteristic_handle == 0x23) {
          memcpy(&thres_high, data, data_length);
         }

      }
      break;

//    case sl_bt_evt_gatt_server_user_write_request_id:
//      {
//        characteristic_handle = evt->data.evt_gatt_server_user_write_request.characteristic;
//        *data = evt->data.evt_gatt_server_user_write_request.value.data;
//        data_length = evt->data.evt_gatt_server_user_write_request.value.len;
//
//        // Assuming we are dealing with a specific characteristic, update your variable:
//        if (characteristic_handle == 0x21) {
//          // Example: Store the data in a global variable
//          memcpy(&thres_low, data, data_length);
//       }
//
//        if (characteristic_handle == 0x23) {
//          memcpy(&thres_high, data, data_length);
//         }
//
//        // Send a response to acknowledge the write request
//        sc = sl_bt_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection,
//                                                         characteristic_handle,
//                                                         SL_STATUS_OK);
//        app_assert_status(sc);
//      }
//      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}*/
