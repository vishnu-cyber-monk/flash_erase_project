# Flash Erase Project - SPI Communication Bug Documentation

## Project Overview
- **MCU**: Silicon Labs BGM220PC22HNA (Cortex-M33)
- **Flash Chip**: ISSI IS25LP512MG-JLLE (512Mbit/64MB SPI NOR Flash)
- **IDE**: Simplicity Studio v6
- **SDK**: simplicity_sdk_2025.12.0

---

## Bug Report: SPI Flash Communication Failure

### Symptoms
- All SPI flash reads return `0x19` instead of actual data
- JEDEC ID reads as `0x19 0x19 0x19` (expected: `0x9D 0x60 0x1A`)
- Flash status register always reports "busy" (bit 0 = 1)
- Erase operations timeout waiting for flash to become ready
- No actual data is erased from flash

### Root Cause: DUPLICATE SPIDRV INITIALIZATION

**CRITICAL BUG:** The USART0 peripheral (configured for SPI) is initialized TWICE with different handles, corrupting the SPI configuration.

---

## Technical Analysis

### Initialization Sequence (Problem)

#### Step 1: System Startup (autogen code)
```
main()
  → sl_system_init()
    → sl_driver_init()
      → sl_spidrv_init_instances()  ← FIRST INIT
```

**File:** `autogen/sl_event_handler.c:68`
```c
void sl_driver_init(void)
{
  ...
  sl_spidrv_init_instances();  // Initializes USART0 for SPI
}
```

**File:** `autogen/sl_spidrv_init.c:44-46`
```c
void sl_spidrv_init_instances(void) {
  SPIDRV_Init(sl_spidrv_inst_handle, &sl_spidrv_init_inst);
}
```

**Configuration used:**
- Handle: `sl_spidrv_inst_handle`
- dummyTxValue: `0` (defined in sl_spidrv_init.c:36)
- Peripheral: USART0

#### Step 2: Application Code
```
app_init()
  → flash_driver_init()
    → SPIDRV_Init()  ← SECOND INIT (CONFLICT!)
```

**File:** `flash_driver.c:144`
```c
Ecode_t result = SPIDRV_Init(spiHandle, &initData);
```

**Configuration used:**
- Handle: `spiHandleData` (local static variable)
- dummyTxValue: `0xFF`
- Peripheral: USART0 (SAME!)

---

## SPI Pin Configuration

Both configurations use identical pins:

| Signal | Port | Pin | GPIO | Function |
|--------|------|-----|------|----------|
| TX (MOSI) | C | 2 | PC02 | Master Out, Slave In |
| RX (MISO) | C | 3 | PC03 | Master In, Slave Out |
| CLK | C | 1 | PC01 | SPI Clock |
| CS | A | 4 | PA04 | Chip Select (Active Low) |

**Source files:**
- `config/sl_spidrv_inst_config.h` (SDK configuration)
- `flash_driver.c:69-77` (hardcoded defines)

---

## Why Double Init Causes Failure

1. **First SPIDRV_Init()** configures USART0 correctly
2. **Second SPIDRV_Init()** on same USART0:
   - May fail silently (returns OK but doesn't work)
   - Corrupts internal SPIDRV state
   - DMA channels may conflict
   - GPIO routing may be overwritten incorrectly

3. The flash driver uses its own `spiHandle` which points to corrupted state
4. All SPI transfers fail, returning whatever is on the MISO line (0x19 pattern)

---

## Solution Options

### Option 1: Use Existing SPIDRV Instance (RECOMMENDED)

Modify `flash_driver.c` to use the pre-initialized SDK handle instead of creating its own.

**Changes required in `flash_driver.c`:**

1. **Add include for SDK handle:**
```c
#include "sl_spidrv_instances.h"
```

2. **Remove local handle variables (lines 36-38):**
```c
// DELETE these lines:
static SPIDRV_HandleData_t spiHandleData;
static SPIDRV_Handle_t spiHandle = &spiHandleData;
```

3. **Add macro to use SDK handle:**
```c
// ADD this line:
#define spiHandle sl_spidrv_inst_handle
```

4. **Simplify flash_driver_init() - remove SPIDRV_Init call:**
```c
flash_driver_status_t flash_driver_init(void)
{
    if (flash_initialized) {
        return FLASH_DRIVER_OK;
    }

    printf("\r\n=== FLASH DRIVER INIT ===\r\n");

    // SPIDRV is already initialized by sl_spidrv_init_instances()
    // Just configure flash-specific settings

    /* Wake up flash */
    uint8_t cmd = 0xAB;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(10);

    /* Reset flash */
    cmd = 0x66;  // Enable Reset
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(1);
    cmd = 0x99;  // Reset Device
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(30);

    /* Verify JEDEC ID */
    uint8_t tx_buffer[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buffer[4] = {0};
    SPIDRV_MTransferB(spiHandle, tx_buffer, rx_buffer, 4);
    printf("JEDEC ID: 0x%02X 0x%02X 0x%02X\r\n",
           rx_buffer[1], rx_buffer[2], rx_buffer[3]);

    /* Enter 4-byte address mode */
    cmd = 0xB7;
    SPIDRV_MTransmitB(spiHandle, &cmd, 1);
    simple_delay_ms(1);

    flash_initialized = true;
    return FLASH_DRIVER_OK;
}
```

### Option 2: Disable SDK SPIDRV Instance

Remove the SPIDRV component from the Simplicity Studio project configuration so the autogen code doesn't initialize it.

**Steps:**
1. Open project in Simplicity Studio
2. Go to Project Configuration → Software Components
3. Find and uninstall "SPIDRV" component
4. Regenerate project

**Risk:** May break other components that depend on SPIDRV.

---

## Files Reference

| File | Purpose |
|------|---------|
| `flash_driver.c` | Flash driver with duplicate SPIDRV init |
| `autogen/sl_spidrv_init.c` | SDK auto-generated SPIDRV initialization |
| `autogen/sl_event_handler.c` | Calls sl_spidrv_init_instances() at startup |
| `config/sl_spidrv_inst_config.h` | SPIDRV pin/mode configuration |

---

## Verification Steps

1. **Build** the project after changes
2. **Flash** firmware to device
3. **Monitor RTT output** - should see:
   ```
   === FLASH DRIVER INIT ===
   JEDEC ID: 0x9D 0x60 0x1A   ← Correct ISSI ID
   ```
4. **Test erase** - data should change from `0x19` pattern to `0xFF`

---

## Additional Notes

### dummyTxValue Consideration
- SDK default: `dummyTxValue = 0`
- Flash reads need: `dummyTxValue = 0xFF` (or explicit 0xFF in tx_buffer)
- The flash driver already sends 0xFF explicitly in tx_buffer, so this should work

### Expected JEDEC ID for ISSI IS25LP512M
| Byte | Value | Meaning |
|------|-------|---------|
| 1 | 0x9D | Manufacturer ID (ISSI) |
| 2 | 0x60 | Memory Type |
| 3 | 0x1A | Capacity (512Mbit) |
