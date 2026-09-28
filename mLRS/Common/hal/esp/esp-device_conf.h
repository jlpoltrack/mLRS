//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// device configuration splicer for ESP targets
//*******************************************************

//-------------------------------------------------------
// ESP Boards
//-------------------------------------------------------

//-- ELRS Tx Modules (external)

#ifdef TX_ELRS_RADIOMASTER_RP4TD_2400_ESP32 // receiver used as Tx module, SiK only
  #define DEVICE_NAME "RM RP4TD 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_BETAFPV_MICRO_1W_2400_ESP32
  #define DEVICE_NAME "BetaFPV Micro1W 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_MICRO_900_ESP32
  #define DEVICE_NAME "RM Bandit Micro 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32
  #define DEVICE_NAME "RM Bandit 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_RANGER_2400_ESP32
  #define DEVICE_NAME "RM Ranger 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_NOMAD_ESP32
  #define DEVICE_NAME "RM Nomad"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif


//-- ELRS Internal Tx Modules

#ifdef TX_ELRS_JUMPER_INTERNAL_2400_ESP32
  #define DEVICE_NAME "Jumper Int 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_900_ESP32
  #define DEVICE_NAME "Jumper Int 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_2400_ESP32
  #define DEVICE_NAME "RM Int 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_BOXER_2400_ESP32
  #define DEVICE_NAME "RM Int Boxer 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_TX15_ESP32
  #define DEVICE_NAME "RadioMaster Int TX15"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_GX12_ESP32
  #define DEVICE_NAME "RadioMaster Int GX12"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_GX15_ESP32
  #define DEVICE_NAME "RadioMaster Int GX15"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_TX16SMK3_ESP32
  #define DEVICE_NAME "RM Int TX16S MK3"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_AX12_ESP32
  #define DEVICE_NAME "RadioMaster Int AX12"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_FLYSKY_INTERNAL_PA01_2400_ESP32S3
  #define DEVICE_NAME "Flysky Int PA01 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif
