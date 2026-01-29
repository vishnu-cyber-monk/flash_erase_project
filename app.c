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
  printf("\n\n====================================================\n");
  printf("       FLASH ERASE & READOUT TEST\n");
  printf("====================================================\n");

  flash_driver_status_t init_status = flash_driver_init();
  printf("Flash driver init: %s\n", init_status == FLASH_DRIVER_OK ? "OK" : "FAILED");

  if (init_status != FLASH_DRIVER_OK) {
      printf("ERROR: Flash init failed, aborting test\n");
      return;
  }

  //==========================================================================
  // TEST 1: Read header area to check current state
  //==========================================================================
  printf("\n--- TEST 1: Read Header Area (0x00000000) ---\n");
  uint8_t header_data[32];
  flash_read(0x00000000, header_data, 32);
  printf("Header: ");
  for(int i = 0; i < 32; i++) printf("%02X ", header_data[i]);
  printf("\n");

  //==========================================================================
  // TEST 2: Erase and verify ONE block at test address
  // Using internal flash data area (0x00060000 - 0x0007FFFF)
  //==========================================================================
  uint32_t test_addr = 0x00060000;  // Internal flash data storage area
  uint8_t data_before[64];
  uint8_t data_after[64];

  printf("\n--- TEST 2: Erase Verification at 0x%08X ---\n", (unsigned int)test_addr);

  // Read BEFORE erase
  printf("Reading BEFORE erase...\n");
  flash_read(test_addr, data_before, 64);
  printf("BEFORE: ");
  for(int i = 0; i < 64; i++) printf("%02X", data_before[i]);
  printf("\n");

  // Count non-FF bytes before
  int non_ff_before = 0;
  for(int i = 0; i < 64; i++) if(data_before[i] != 0xFF) non_ff_before++;
  printf("Non-0xFF bytes before: %d\n", non_ff_before);

  // ERASE the block
  printf("\nErasing 64KB block...\n");
  flash_driver_status_t erase_status = flash_erase_block(test_addr);
  printf("Erase returned: %d ", erase_status);
  if (erase_status == FLASH_DRIVER_OK) printf("(OK)\n");
  else if (erase_status == FLASH_DRIVER_ERROR) printf("(ERROR)\n");
  else if (erase_status == FLASH_DRIVER_TIMEOUT) printf("(TIMEOUT)\n");
  else printf("(UNKNOWN)\n");

  // Read AFTER erase
  printf("\nReading AFTER erase...\n");
  flash_read(test_addr, data_after, 64);
  printf("AFTER:  ");
  for(int i = 0; i < 64; i++) printf("%02X", data_after[i]);
  printf("\n");

  // Count non-FF bytes after
  int non_ff_after = 0;
  for(int i = 0; i < 64; i++) if(data_after[i] != 0xFF) non_ff_after++;
  printf("Non-0xFF bytes after: %d\n", non_ff_after);

  //==========================================================================
  // RESULT
  //==========================================================================
  printf("\n====================================================\n");
  printf("                    RESULT\n");
  printf("====================================================\n");
  printf("Before erase: %d non-FF bytes\n", non_ff_before);
  printf("After erase:  %d non-FF bytes\n", non_ff_after);

  if (non_ff_after == 0) {
      printf("\n*** SUCCESS: Erase WORKED! All bytes are 0xFF ***\n");
  } else if (non_ff_after < non_ff_before) {
      printf("\n*** PARTIAL: Some bytes erased, but not all ***\n");
  } else {
      printf("\n*** FAILED: Erase did NOT work! Data unchanged ***\n");
  }
  printf("====================================================\n");

  //==========================================================================
  // TEST 3: Write Test - Write pattern and verify
  //==========================================================================
  printf("\n--- TEST 3: Write Test at 0x%08X ---\n", (unsigned int)test_addr);

  // Create test pattern (must be word-aligned, 4-byte multiples)
  uint8_t write_data[16] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78,
                            0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44};
  uint8_t read_back[16];

  printf("Writing test pattern: ");
  for(int i = 0; i < 16; i++) printf("%02X", write_data[i]);
  printf("\n");

  // Write to flash
  flash_driver_status_t write_status = flash_write(test_addr, write_data, 16);
  printf("Write returned: %d ", write_status);
  if (write_status == FLASH_DRIVER_OK) printf("(OK)\n");
  else printf("(ERROR)\n");

  // Read back to verify
  printf("Reading back...\n");
  flash_read(test_addr, read_back, 16);
  printf("Read back:            ");
  for(int i = 0; i < 16; i++) printf("%02X", read_back[i]);
  printf("\n");

  // Compare
  int match = 1;
  for(int i = 0; i < 16; i++) {
      if(write_data[i] != read_back[i]) {
          match = 0;
          break;
      }
  }

  if (match) {
      printf("*** WRITE TEST PASSED: Data matches! ***\n");
  } else {
      printf("*** WRITE TEST FAILED: Data mismatch! ***\n");
  }

  //==========================================================================
  // TEST 4: Read data area content
  //==========================================================================
  printf("\n--- TEST 4: Data Area Content ---\n");
  uint8_t block_sample[32];
  for(int block = 0; block < 2; block++) {
      uint32_t addr = 0x00060000 + (block * 0x2000);  // Every 8KB (page size)
      flash_read(addr, block_sample, 32);
      printf("0x%08X: ", (unsigned int)addr);
      for(int i = 0; i < 32; i++) printf("%02X", block_sample[i]);
      printf("\n");
  }
  printf("====================================================\n");
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
