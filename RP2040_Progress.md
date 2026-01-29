# RP2040 Receiver Debug Progress

## Summary

Debugging an RP2040-based mLRS receiver that won't connect to a Transmitter on default settings (bind phrase = mlrs.0, 868 MHz, 31 Hz).

## Issues Fixed

### Issue 1: GPIO OUTPUT_PP_HIGH Not Setting Pin State (FIXED)

**Location:** `mLRS/Common/rp-lib/rp-peripherals.h`

**Problem:** The original `gpio_init` function had both `IO_MODE_OUTPUT_PP_LOW` and `IO_MODE_OUTPUT_PP_HIGH` mapped to the same `MODE_GPIO_OUTPUT` enum value, and the function didn't set the initial pin state.

**Fix Applied:**
```c
typedef enum {
    MODE_ANALOG           = 0,
    MODE_GPIO_INPUT       = 1,
    MODE_GPIO_OUTPUT_LOW  = 2,
    MODE_GPIO_OUTPUT_HIGH = 3,
    MODE_GPIO_INPUT_PU    = 4,
    MODE_GPIO_INPUT_PD    = 5,
} IOMODEENUM;

#define IO_MODE_OUTPUT_PP_LOW     MODE_GPIO_OUTPUT_LOW
#define IO_MODE_OUTPUT_PP_HIGH    MODE_GPIO_OUTPUT_HIGH

inline void gpio_init(uint8_t pin, IOMODEENUM mode) {
    // ... handles each mode and sets initial pin state
    } else if (mode == MODE_GPIO_OUTPUT_LOW) {
        pinMode(pin, OUTPUT);
        gpio_low(pin);
    } else if (mode == MODE_GPIO_OUTPUT_HIGH) {
        pinMode(pin, OUTPUT);
        gpio_high(pin);
    }
}
```

**Verification:** GPIO states after init show RESET=1, BUSY=1, DIO1=0 (correct).

## Issues Ruled Out

### SX126x Initialization - WORKING

**Evidence:** Added status byte tracking through Configure() sequence:
```
Configure: start, status=0xA2
Configure: after SetPacketType, status=0xA2
Configure: after SetTxClampConfig, status=0xA2
Configure: after ClearDeviceError, status=0xA2
Configure: calling SetDio3AsTcxoControl...
Configure: after SetDio3AsTcxoControl, status=0xA2
Configure: after CalibrateImage, status=0xA2
Configure: complete, status=0xA2
```

All commands return status 0xA2 = STDBY_RC mode + Command OK. The initial 0xAA status was just the chip state before Configure() ran.

### SPI Communication - WORKING

**Evidence:** During steady state operation, SPI responses show valid status bytes:
- TX:0x82 RX:0xC2 (SetRx command, chip in FS mode)
- TX:0xC1 RX:0xD2 (GetRxBufferStatus, chip in RX mode)

The status byte 0xD2 = RX mode + Data Available, indicating the chip is properly entering RX mode.

## Current Issue: Chip Working But No RF Reception

**Latest findings:**

1. **BUSY timing is CORRECT:**
   ```
   BUSY before reset=LOW
   BUSY immediately after RESET high=HIGH  <-- Chip responds!
   0ms: BUSY=LOW  <-- Chip ready quickly
   ```

2. **Chip enters RX mode correctly:**
   ```
   Mode=RX(0xD2)  <-- Chip is in RX mode
   Mode=FS(0xC2)  <-- Normal frequency switching
   ```

3. **But no packets received:**
   ```
   IRQreg:0x0  <-- No RX_DONE flag ever set
   DIO1=L      <-- DIO1 never goes high
   ```

**Root Cause Hypothesis:**
The SX126x chip is working correctly (enters RX mode), but the **RF path is not receiving**. Possible causes:
- RF switch (RX_EN/TX_EN) not being controlled correctly
- TCXO not providing clock (unlikely since commands work)
- Wrong frequency configuration
- Antenna/hardware issue

**Status byte 0xAA after reset:**
This appears to be normal initial state before Configure() runs. The status changes to 0xA2 once commands are executed, and the chip functions correctly afterward.

### Potential Causes to Investigate

1. **attachInterrupt() not working correctly on RP2040**
   - Location: `mLRS/Common/hal/rp/rx-hal-generic-900-rp2040.h:93`
   - Code: `attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);`
   - The Arduino `attachInterrupt()` may have platform-specific behavior

2. **DIO1 pin configuration**
   - DIO1 is configured as INPUT_PD (pull-down)
   - May need to verify the interrupt is properly attached after chip initialization

3. **SetDioIrqParams configuration**
   - Location: `mLRS/Common/sx-drivers/sx126x_driver.h:258-261`
   - Currently sets DIO1 to trigger on RX_DONE|TX_DONE|RX_TX_TIMEOUT
   - Should be correct, but worth verifying

4. **Timing of interrupt attachment**
   - `sx_dio_enable_exti_isr()` is called at end of StartUp()
   - May need to verify this happens after chip is fully configured

## Debug Instrumentation Added

### 1Hz Status Output (mlrs-rx.cpp)
```c
volatile uint32_t dbg_irq_count = 0;
volatile uint32_t dbg_irq_rxdone = 0;
volatile uint32_t dbg_irq_filtered = 0;
volatile uint16_t dbg_last_raw_irq = 0;
```

### Boot Sequence Debug (sx126x_driver.h)
- Status byte after each Configure() step
- Uses `DBG_BOOT()` macro and `Serial1` for RP2040

### GPIO State Debug (rx-hal-generic-900-rp2040.h)
```c
Serial1.print("  GPIO after init: RESET=");
Serial1.print(digitalRead(SX_RESET));
// etc.
```

## Files Modified

1. `mLRS/Common/rp-lib/rp-peripherals.h` - GPIO fix
2. `mLRS/Common/hal/rp/rx-hal-generic-900-rp2040.h` - GPIO debug
3. `mLRS/Common/sx-drivers/sx126x_driver.h` - Configure() status debug
4. `mLRS/CommonRx/mlrs-rx.cpp` - 1Hz counter debug (both platforms)

## Next Steps

### New Debug Added (ready to test)

The following diagnostics have been added:

1. **BUSY pin monitoring during reset** (`sx126x_driver.h:_reset()`)
   - Shows BUSY state before reset
   - Shows BUSY state immediately after RESET goes high
   - Polls BUSY every 10ms for 100ms to see timing
   - Shows BUSY state after WaitOnBusy() completes

2. **1Hz output now includes** (`mlrs-rx.cpp`):
   - `DIO1=H/L` - Direct pin state (does the pin ever go high?)
   - `IRQreg:0x????` - Manual read of SX126x IRQ status register
   - `Mode=RX/FS/STDBY_RC(0x??)` - Actual chip mode and full status byte

3. **Interrupt attachment debug** (`rx-hal-generic-900-rp2040.h`)
   - Prints when interrupt is attached and to which pin

### What to look for in output

**During boot - BUSY pin timing:**
- BUSY should go HIGH briefly after RESET goes high
- BUSY should go LOW within ~10-50ms
- If BUSY stays HIGH → chip not responding to reset

**1Hz output scenarios:**

| Mode | DIO1 | IRQreg | Meaning |
|------|------|--------|---------|
| STDBY_RC | L | 0x0 | Chip never entered RX mode |
| FS | L | 0x0 | Chip stuck in FS mode |
| RX | L | 0x0 | In RX but no packets (RF issue) |
| RX | L | 0x2+ | Packets received but DIO1 not working |
| RX | H | IRQ:0 | DIO1 works but interrupt not attached |

**Expected for working receiver:**
- Mode=RX
- DIO1 toggles between H and L
- IRQreg shows RX_DONE periodically
- IRQ counter increments ~31/sec for 31Hz mode

### Further investigation if needed

1. **Compare with working Wio E5 Mini**
   - Both platforms now have the same 1Hz debug output
   - Run both simultaneously with same TX to compare behavior

2. **Verify RP2040 interrupt API**
   - On RP2040, may need `digitalPinToInterrupt()` wrapper
   - Check if pin supports interrupts (GPIO0-29 all support on RP2040)

## Reference: Status Byte Decoding

SX126x status byte format (from sx126x.h):
- Bits [6:4] = Chip Mode:
  - 0b010 = STDBY_RC (0x20)
  - 0b011 = STDBY_XOSC (0x30)
  - 0b100 = FS (0x40)
  - 0b101 = RX (0x50)
  - 0b110 = TX (0x60)
- Bits [3:1] = Command Status:
  - 0b001 = Data Available / OK (0x02)
  - 0b011 = Timeout (0x06)
  - 0b100 = Processing Error (0x08)
  - 0b101 = Exec Failure (0x0A)
  - 0b110 = TX Done (0x0C)

Examples:
- 0xA2 = STDBY_RC + OK
- 0xAA = STDBY_RC + Exec Failure
- 0xC2 = FS + OK
- 0xD2 = RX + OK
