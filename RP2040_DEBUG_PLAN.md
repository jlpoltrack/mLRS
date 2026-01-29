# RP2040 Receiver Debug Plan

## Overview

This plan adds comprehensive debug instrumentation to the RP2040 receiver implementation to identify why it's not connecting to a Transmitter. The debug output can be compared against a working Wio E5 Mini receiver.

## Issues Identified During Code Review

### Issue 1: GPIO OUTPUT_PP_HIGH Not Setting Pin High
**Location:** `mLRS/Common/rp-lib/rp-peripherals.h:62-72`

**Problem:** The `gpio_init` function doesn't set initial pin state for OUTPUT modes:
```c
// Current code - both modes do the same thing!
#define IO_MODE_OUTPUT_PP_LOW     MODE_GPIO_OUTPUT
#define IO_MODE_OUTPUT_PP_HIGH    MODE_GPIO_OUTPUT

inline void gpio_init(uint8_t pin, IOMODEENUM mode) {
    // ...
    } else if (mode == MODE_GPIO_OUTPUT) {
        pinMode(pin, OUTPUT);  // Pin state is undefined!
    }
}
```

**Fix:** Differentiate the modes and set initial state:
```c
#define IO_MODE_OUTPUT_PP_LOW     (MODE_GPIO_OUTPUT | 0x00)
#define IO_MODE_OUTPUT_PP_HIGH    (MODE_GPIO_OUTPUT | 0x80)

inline void gpio_init(uint8_t pin, IOMODEENUM mode) {
    if (mode == MODE_GPIO_INPUT) {
        pinMode(pin, INPUT);
    } else if (mode == MODE_GPIO_INPUT_PU) {
        pinMode(pin, INPUT_PULLUP);
    } else if (mode == MODE_GPIO_INPUT_PD) {
        pinMode(pin, INPUT_PULLDOWN);
    } else if ((mode & 0x7F) == MODE_GPIO_OUTPUT) {
        pinMode(pin, OUTPUT);
        if (mode & 0x80) {
            gpio_high(pin);  // Set HIGH for OUTPUT_PP_HIGH
        } else {
            gpio_low(pin);   // Set LOW for OUTPUT_PP_LOW
        }
    }
}
```

### Issue 2: SPI in ISR Context
**Location:** `mLRS/CommonRx/mlrs-rx.cpp:237-262`

**Problem:** The DIO1 interrupt handler calls `sx.GetAndClearIrqStatus()` which performs SPI operations. This can conflict with main loop SPI operations.

**Potential Impact:** Missed or corrupted SPI transactions, leading to missed frames.

---

## Debug Instrumentation Plan

### Phase 1: Add Debug Counters and Flags

Add the following debug variables to track system behavior. Create a new file or add to `mlrs-rx.cpp`:

```c
// Debug counters - add near top of mlrs-rx.cpp after includes
#ifdef ARDUINO_ARCH_RP2040
volatile uint32_t dbg_irq_count = 0;           // Total DIO1 IRQs
volatile uint32_t dbg_irq_rxdone_count = 0;    // RX_DONE IRQs
volatile uint32_t dbg_irq_filtered_count = 0;  // IRQs filtered (sync word mismatch)
volatile uint32_t dbg_postrecv_count = 0;      // doPostReceive triggers
volatile uint32_t dbg_loop_count = 0;          // Main loop iterations
volatile uint32_t dbg_link_state_rx = 0;       // Times in LINK_STATE_RECEIVE
volatile uint32_t dbg_link_state_rxwait = 0;   // Times in LINK_STATE_RECEIVE_WAIT
volatile uint32_t dbg_frame_received = 0;      // Valid frames received
volatile uint32_t dbg_spi_conflict = 0;        // SPI conflict detected
volatile bool dbg_spi_busy = false;            // SPI transaction in progress
#endif
```

### Phase 2: Instrument Critical Code Paths

#### 2.1 Instrument the IRQ Handler

```c
IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
#ifdef ARDUINO_ARCH_RP2040
    dbg_irq_count++;
    if (dbg_spi_busy) {
        dbg_spi_conflict++;
        return;  // Don't do SPI if main loop is using it
    }
#endif
    sx_dio_exti_isr_clearflag();
    irq_status = sx.GetAndClearIrqStatus(SX_IRQ_ALL);
#ifdef ARDUINO_ARCH_RP2040
    if (irq_status & SX_IRQ_RX_DONE) {
        dbg_irq_rxdone_count++;
    }
#endif
    if (irq_status & SX_IRQ_RX_DONE) {
        if (bind.IsInBind()) {
            uint64_t bind_signature;
            sx.ReadBuffer(0, (uint8_t*)&bind_signature, 8);
            if (bind_signature != bind.TxSignature) {
#ifdef ARDUINO_ARCH_RP2040
                dbg_irq_filtered_count++;
#endif
                irq_status = 0;
            }
        } else {
            uint16_t sync_word;
            sx.ReadBuffer(0, (uint8_t*)&sync_word, 2);
            if (sync_word != Config.FrameSyncWord) {
#ifdef ARDUINO_ARCH_RP2040
                dbg_irq_filtered_count++;
#endif
                irq_status = 0;
            }
        }
    }
})
```

#### 2.2 Instrument SPI Functions (rp-spi.h)

```c
#ifdef DEBUG_SPI
extern volatile bool dbg_spi_busy;
extern volatile uint32_t dbg_spi_conflict;
#endif

inline void spi_select(void) {
#ifdef DEBUG_SPI
    if (dbg_spi_busy) dbg_spi_conflict++;
    dbg_spi_busy = true;
#endif
    RX_SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    gpio_low(RX_SPI_NSS);
}

inline void spi_deselect(void) {
    gpio_high(RX_SPI_NSS);
    RX_SPI.endTransaction();
#ifdef DEBUG_SPI
    dbg_spi_busy = false;
#endif
}
```

#### 2.3 Instrument doPostReceive Handler

```c
    if (doPostReceive) {
        doPostReceive = false;
#ifdef ARDUINO_ARCH_RP2040
        dbg_postrecv_count++;
#endif
        // ... rest of handler
    }
```

#### 2.4 Instrument Main Loop State Changes

```c
    switch (link_state) {
    case LINK_STATE_RECEIVE:
#ifdef ARDUINO_ARCH_RP2040
        dbg_link_state_rx++;
#endif
        // ... existing code
        break;

    case LINK_STATE_RECEIVE_WAIT:
#ifdef ARDUINO_ARCH_RP2040
        dbg_link_state_rxwait++;
        dbg_loop_count++;
#endif
        // ... existing code
        break;
    }
```

### Phase 3: Debug Output

Add periodic debug output to the 1Hz tick handler:

```c
        if (!tick_1hz) {
#ifdef ARDUINO_ARCH_RP2040
            Serial1.print("\n[DBG] IRQ:");
            Serial1.print(dbg_irq_count);
            Serial1.print(" RxDone:");
            Serial1.print(dbg_irq_rxdone_count);
            Serial1.print(" Filt:");
            Serial1.print(dbg_irq_filtered_count);
            Serial1.print(" PostRx:");
            Serial1.print(dbg_postrecv_count);
            Serial1.print(" Loop:");
            Serial1.print(dbg_loop_count);
            Serial1.print(" SPI_Confl:");
            Serial1.print(dbg_spi_conflict);
            Serial1.print(" State:");
            Serial1.print(connect_state);
            Serial1.print(" Bind:");
            Serial1.println(bind.IsInBind() ? "Y" : "N");

            // Also print frequency info
            Serial1.print("  Freq:");
            Serial1.print(fhss.GetCurrFreq());
            Serial1.print(" SyncWord:");
            Serial1.println(Config.FrameSyncWord, HEX);

            // Reset some counters for per-second rates
            dbg_loop_count = 0;
#endif
            dbg.puts(".");
        }
```

### Phase 4: Add GPIO State Verification

Add verification after GPIO init to ensure pins are in correct state:

```c
void sx_init_gpio(void)
{
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);

#ifdef ARDUINO_ARCH_RP2040
    // Verify pin states
    Serial1.print("GPIO State: RESET=");
    Serial1.print(digitalRead(SX_RESET));
    Serial1.print(" BUSY=");
    Serial1.print(digitalRead(SX_BUSY));
    Serial1.print(" DIO1=");
    Serial1.println(digitalRead(SX_DIO1));
#endif
}
```

---

## Expected Debug Output Comparison

### Working Receiver (Wio E5 Mini) - Expected Pattern:
```
[DBG] IRQ:31 RxDone:31 Filt:0 PostRx:31 Loop:1000 SPI_Confl:0 State:2 Bind:N
  Freq:868500000 SyncWord:A3B2
```
- IRQ count should match frame rate (31 Hz)
- RxDone should equal IRQ count
- Filtered should be 0 for normal operation
- connect_state should be 2 (CONNECT_STATE_SYNC) or 3 (CONNECT_STATE_CONNECTED)

### Non-Working Receiver (RP2040) - Potential Patterns:

**Pattern A: No IRQs**
```
[DBG] IRQ:0 RxDone:0 Filt:0 PostRx:31 Loop:1000 SPI_Confl:0 State:0 Bind:N
```
Indicates: DIO1 interrupt not firing - check wiring, `attachInterrupt` setup

**Pattern B: IRQs but no RxDone**
```
[DBG] IRQ:100 RxDone:0 Filt:0 PostRx:31 Loop:1000 SPI_Confl:0 State:0 Bind:N
```
Indicates: DIO1 firing on noise/wrong edge, or SX126x not in RX mode

**Pattern C: RxDone but all filtered**
```
[DBG] IRQ:31 RxDone:31 Filt:31 PostRx:31 Loop:1000 SPI_Confl:0 State:0 Bind:N
```
Indicates: Receiving frames but sync word mismatch - check bind phrase, frequency band

**Pattern D: SPI Conflicts**
```
[DBG] IRQ:31 RxDone:15 Filt:0 PostRx:31 Loop:1000 SPI_Confl:16 State:0 Bind:N
```
Indicates: SPI race condition - need to protect ISR SPI calls

---

## Quick Fixes to Try

### Fix 1: GPIO Initial State (Apply First)

In `rp-peripherals.h`, change `gpio_init`:

```c
inline void gpio_init(uint8_t pin, IOMODEENUM mode) {
    if (mode == MODE_GPIO_INPUT) {
        pinMode(pin, INPUT);
    } else if (mode == MODE_GPIO_INPUT_PU) {
        pinMode(pin, INPUT_PULLUP);
    } else if (mode == MODE_GPIO_INPUT_PD) {
        pinMode(pin, INPUT_PULLDOWN);
    } else if (mode == MODE_GPIO_OUTPUT) {
        pinMode(pin, OUTPUT);
        // Default to LOW for safety
        gpio_low(pin);
    }
}

// Add explicit high init function
inline void gpio_init_high(uint8_t pin, IOMODEENUM mode) {
    pinMode(pin, OUTPUT);
    gpio_high(pin);
}
```

Then in `rx-hal-generic-900-rp2040.h`:
```c
void sx_init_gpio(void)
{
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD);
    // Explicitly set RESET high
    pinMode(SX_RESET, OUTPUT);
    gpio_high(SX_RESET);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}
```

### Fix 2: Protect SPI in ISR

Disable interrupts during main loop SPI operations:

```c
inline void spi_select(void) {
    noInterrupts();  // Prevent DIO1 ISR from interrupting
    RX_SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    gpio_low(RX_SPI_NSS);
}

inline void spi_deselect(void) {
    gpio_high(RX_SPI_NSS);
    RX_SPI.endTransaction();
    interrupts();  // Re-enable interrupts
}
```

### Fix 3: Defer ISR SPI Operations

Instead of doing SPI in the ISR, just set a flag and do the SPI in the main loop:

```c
volatile bool dio1_interrupt_pending = false;

IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
    sx_dio_exti_isr_clearflag();
    dio1_interrupt_pending = true;  // Just set flag, don't do SPI
})

// In main loop, after link_state switch:
if (dio1_interrupt_pending) {
    dio1_interrupt_pending = false;
    irq_status = sx.GetAndClearIrqStatus(SX_IRQ_ALL);
    // ... rest of IRQ processing
}
```

---

## Testing Procedure

1. Apply Fix 1 (GPIO state) first and test
2. If still not working, add debug instrumentation
3. Capture debug output for 30 seconds
4. Compare IRQ/RxDone/Filtered counts to identify issue
5. Apply appropriate fix based on pattern observed
6. If SPI conflicts are high, apply Fix 2 or Fix 3
