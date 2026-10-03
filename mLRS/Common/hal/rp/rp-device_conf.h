//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

#ifdef RX_DIY_LR2021_RP2040
  #ifdef DEVICE_HAS_DRONECAN_FD
    #define DEVICE_NAME "RP2040 DIY LR2021 FD"
  #else
    #define DEVICE_NAME "RP2040 DIY LR2021"
  #endif
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_LR20xx
  #define FREQUENCY_BAND_915_MHZ_FCC
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_LR2021_RP2350
  #define DEVICE_NAME "RP2350 DIY LR2021 Tx"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR20xx
  #define FREQUENCY_BAND_915_MHZ_FCC
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_2P4_GHZ
#endif
