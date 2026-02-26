//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

#ifdef RX_DIY_2400_RP2040
  #define DEVICE_NAME "RP2040 DIY 2400"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_DIY_2400_RP2350
  #define DEVICE_NAME "RP2350 DIY 2400"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_2400_RP2040
  #define DEVICE_NAME "RP2040 DIY 2400 Tx"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_2400_RP2350
  #define DEVICE_NAME "RP2350 DIY 2400 Tx"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif
