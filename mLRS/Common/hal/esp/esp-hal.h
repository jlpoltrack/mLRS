//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// rx hal splicer for ESP targets
//*******************************************************

//-------------------------------------------------------
// ESP Boards
//-------------------------------------------------------

//-- ELRS targets, configured by tools/elrs/elrs_targets.py

#ifdef RX_ELRS_TARGET
#include "rx-hal-elrs-generic.h"
#endif

//-- ELRS Selected Devices

#ifdef TX_ELRS_RADIOMASTER_RP4TD_2400_ESP32
#include "tx-hal-radiomaster-rp4td-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_2400_ESP32
#include "tx-hal-radiomaster-int-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_BOXER_2400_ESP32
#include "tx-hal-radiomaster-int-boxer-2400-esp32.h"
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_2400_ESP32
#include "tx-hal-jumper-int-2400-esp32.h"
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_900_ESP32
#include "tx-hal-jumper-int-900-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_TX15_ESP32
#include "tx-hal-radiomaster-int-tx15-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_GX12_ESP32
#include "tx-hal-radiomaster-int-gx12-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_GX15_ESP32
#include "tx-hal-radiomaster-int-gx15-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_TX16SMK3_ESP32
#include "tx-hal-radiomaster-int-tx16smk3-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_AX12_ESP32
#include "tx-hal-radiomaster-int-ax12-esp32.h"
#endif

#ifdef TX_ELRS_BETAFPV_MICRO_1W_2400_ESP32
#include "tx-hal-betafpv-micro-1w-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_MICRO_900_ESP32
#include "tx-hal-radiomaster-bandit-series-900-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32
#include "tx-hal-radiomaster-bandit-series-900-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_RANGER_2400_ESP32
#include "tx-hal-radiomaster-ranger-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_NOMAD_ESP32
#include "tx-hal-radiomaster-nomad-esp32.h"
#endif

#ifdef TX_ELRS_FLYSKY_INTERNAL_PA01_2400_ESP32S3
#include "tx-hal-flysky-int-pa01-2400-esp32s3.h"
#endif

