//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// device configuration splicer for RP2040 targets
// 2026-01-28
//*******************************************************

//-------------------------------------------------------
// RP2040 Boards
//-------------------------------------------------------

//-- Generic 868/915 MHz RP2040 Devices

#ifdef RX_GENERIC_900_RP2040
  #define DEVICE_NAME "GENERIC 900 RP2040"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX126x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif
