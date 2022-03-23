//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration types
//*******************************************************
#ifndef SETUP_TX_H
#define SETUP_TX_H
#pragma once


#include <stdint.h>
#include "..\Common\setup.h"


#define SETUP_MSK_MODE              &SetupMetaData.Mode_allowed_mask // this we get from the hal

#define SETUP_OPT_TX_POWER          SetupMetaData.Tx_Power_optstr // this we get from the hal
#define SETUP_OPT_RX_POWER          SetupMetaData.Rx_Power_optstr // this we get from the receiver

// common to Tx,Rx, all options limited dependent on hardware, implementation
#define SETUP_OPT_DIVERSITY         "enabled,antenna1,antenna2"
#define SETUP_MSK_TX_DIVERSITY      &SetupMetaData.Tx_Diversity_allowed_mask // this we get from the hal
#define SETUP_MSK_RX_DIVERSITY      &SetupMetaData.Rx_Diversity_allowed_mask // this we get from the receiver

// Tx only
#define SETUP_MSK_TX_SER_DEST       &SetupMetaData.Tx_SerialDestination_allowed_mask // this we get from the hal
#define SETUP_MSK_TX_CH_SOURCE      &SetupMetaData.Tx_ChannelsSource_allowed_mask // this we get from the hal
#define SETUP_MSK_TX_IN_MODE        &SetupMetaData.Tx_InMode_allowed_mask // this we get from the hal

// Rx only
#define SETUP_MSK_RX_OUT_MODE       &SetupMetaData.Rx_OutMode_allowed_mask // this we get from the receiver

// common to Tx,Rx, all options always allowed
#define SETUP_OPT_CH_ORDER          "AETR,TAER,ETAR"
#define SETUP_OPT_SERIAL_BAUDRATE   "9600,19200,38400,57600,115200"
#define SETUP_OPT_SERIAL_LINK_MODE  "transp.,mavlink"
#define SETUP_OPT_SEND_RADIOSTATUS  "off,on,on w txbuf"

#define MSK_ALL                     nullptr // is converted to UINT16_MAX

// TODO: dummies to account for current deficiencies, should be replaced by MSK_ALL
uint16_t tx_channelorder_allowed_mask = 0b0101;
uint16_t rx_channelorder_allowed_mask = 0b0001;

// Tx parameters must begin with "Tx "
// Rx parameters must begin with "Rx "
#define SETUP_PARAMETER_LIST \
  X( Setup.BindPhrase[0],         STR6,  "Bind Phrase",  "BIND_PHRASE",  0,0,0,"", "", 0)\
  X( Setup.Mode,                  LIST,  "Mode",         "MODE",         0,0,0,"", "50 Hz,31 Hz,19 Hz", SETUP_MSK_MODE )\
  \
  X( Setup.Tx.Power,              LIST,  "Tx Power",     "TX_POWER",     0,0,0,"", SETUP_OPT_TX_POWER, MSK_ALL )\
  X( Setup.Tx.Diversity,          LIST,  "Tx Diversity", "TX_DIVERSITY", 0,0,0,"", SETUP_OPT_DIVERSITY, SETUP_MSK_TX_DIVERSITY )\
  X( Setup.Tx.ChannelsSource,     LIST,  "Tx Ch Source", "TX_CH_SOURCE", 0,0,0,"", "none,mbridge,in,crsf", SETUP_MSK_TX_CH_SOURCE )\
  X( Setup.Tx.ChannelOrder,       LIST,  "Tx Ch Order",  "TX_CH_ORDER",  0,0,0,"", SETUP_OPT_CH_ORDER, &tx_channelorder_allowed_mask )\
  X( Setup.Tx.InMode,             LIST,  "Tx In Mode",   "TX_IN_MODE",   0,0,0,"", "sbus,sbus inv", SETUP_MSK_TX_IN_MODE )\
  X( Setup.Tx.SerialDestination,  LIST,  "Tx Ser Dest",      "TX_SER_DEST",      0,0,0,"", "serial,mbridge", SETUP_MSK_TX_SER_DEST )\
  X( Setup.Tx.SerialBaudrate,     LIST,  "Tx Ser Baudrate",  "TX_SER_BAUD",      0,0,0,"", SETUP_OPT_SERIAL_BAUDRATE, MSK_ALL )\
  X( Setup.Tx.SerialLinkMode,     LIST,  "Tx Ser Link Mode", "TX_SER_LNK_MODE",  0,0,0,"", SETUP_OPT_SERIAL_LINK_MODE, MSK_ALL )\
  X( Setup.Tx.SendRadioStatus,    LIST,  "Tx Snd RadioStat", "TX_SND_RADIOSTAT", 0,0,0,"", SETUP_OPT_SEND_RADIOSTATUS, MSK_ALL )\
  \
  X( Setup.Rx.Power,              LIST,  "Rx Power",     "RX_POWER",     0,0,0,"", SETUP_OPT_RX_POWER, MSK_ALL )\
  X( Setup.Rx.Diversity,          LIST,  "Rx Diversity", "RX_DIVERSITY", 0,0,0,"", SETUP_OPT_DIVERSITY, SETUP_MSK_RX_DIVERSITY )\
  X( Setup.Rx.ChannelOrder,       LIST,  "Rx Ch Order",  "RX_CH_ORDER",  0,0,0,"", SETUP_OPT_CH_ORDER, &rx_channelorder_allowed_mask )\
  X( Setup.Rx.OutMode,            LIST,  "Rx Out Mode",  "RX_OUT_MODE",  0,0,0,"", "sbus,crsf,sbus inv", SETUP_MSK_RX_OUT_MODE )\
  X( Setup.Rx.OutRssiChannelMode, LIST,  "Rx Out Rssi Ch",   "RX_OUT_RSSI_CH",   0,0,0,"", "off,4,5,6,7,8,9,10,11,12", MSK_ALL )\
  X( Setup.Rx.FailsafeMode,       LIST,  "Rx FailSafe Mode", "RX_FAILSAFE_MODE", 0,0,0,"", "no sig,low thr,by cnf,low thr cnt,ch1ch4 cnt", MSK_ALL )\
  X( Setup.Rx.SerialBaudrate,     LIST,  "Rx Ser Baudrate",  "RX_SER_BAUD",      0,0,0,"", SETUP_OPT_SERIAL_BAUDRATE, MSK_ALL )\
  X( Setup.Rx.SerialLinkMode,     LIST,  "Rx Ser Link Mode", "RX_SER_LNK_MODE",  0,0,0,"", SETUP_OPT_SERIAL_LINK_MODE, MSK_ALL )\
  X( Setup.Rx.SendRadioStatus,    LIST,  "Rx Snd RadioStat", "RX_SND_RADIOSTAT", 0,0,0,"", SETUP_OPT_SEND_RADIOSTATUS, MSK_ALL )\
  \
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[0],  INT8, "Rx FS Ch1", "RX_FS_CH1", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[1],  INT8, "Rx FS Ch2", "RX_FS_CH2", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[2],  INT8, "Rx FS Ch3", "RX_FS_CH3", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[3],  INT8, "Rx FS Ch4", "RX_FS_CH4", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[4],  INT8, "Rx FS Ch5", "RX_FS_CH5", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[5],  INT8, "Rx FS Ch6", "RX_FS_CH6", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[6],  INT8, "Rx FS Ch7", "RX_FS_CH7", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[7],  INT8, "Rx FS Ch8", "RX_FS_CH8", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[8],  INT8, "Rx FS Ch9", "RX_FS_CH9", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[9],  INT8, "Rx FS Ch10", "RX_FS_CH10", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[10], INT8, "Rx FS Ch11", "RX_FS_CH11", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[11], INT8, "Rx FS Ch12", "RX_FS_CH12", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[0], LIST, "Rx FS Ch13", "RX_FS_CH13", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[1], LIST, "Rx FS Ch14", "RX_FS_CH14", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[2], LIST, "Rx FS Ch15", "RX_FS_CH15", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[3], LIST, "Rx FS Ch16", "RX_FS_CH16", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\


// this must EXACTLY match MAV_PARAM_TYPE !! Otherwise Mavlink will be broken !!
typedef enum {
    SETUP_PARAM_TYPE_UINT8  = 1, // 8-bit unsigned integer
    SETUP_PARAM_TYPE_INT8   = 2, // 8-bit signed integer
    SETUP_PARAM_TYPE_UINT16 = 3, // 16-bit unsigned integer
    SETUP_PARAM_TYPE_INT16  = 4, // 16-bit signed integer

    // our extensions, cannot be handled with mavlink!
    SETUP_PARAM_TYPE_LIST   = 254,
    SETUP_PARAM_TYPE_STR6   = 255,
} SETUP_PARAM_TYPE;


#define UINT8   uint8_t  // u8 fields are type case into uint16_t
#define INT8    uint8_t  // s8 fields are type case into uint16_t
#define UINT16  uint16_t // u16 fields are type case into uint16_t
#define INT16   uint16_t // s16 fields are type case into uint16_t
#define STR6    char
#define LIST    uint8_t  // list fields are type case into uint16_t


typedef union {
    uint8_t UINT8_value;
    int8_t INT8_value;
    uint16_t UINT16_value;
    int16_t INT16_value;
    char* STR6_value;
    uint8_t LIST_value;
} tSetupParameterValue;


typedef struct {
    void* ptr; // pointer to the field in the Setup structure
    uint8_t type;
    const char* name;
    const char* m_name;
    const char* unit;
    tSetupParameterValue dflt;
    tSetupParameterValue min;
    tSetupParameterValue max;
    const char* optstr;
    uint16_t* allowed_mask_ptr;
} tSetupParameterItem;


const tSetupParameterItem SetupParameter[] = {
    #define X(p,t, n,mn, d,mi,ma,u, s, amp) {.ptr=(t*)&(p), .type=SETUP_PARAM_TYPE_##t, .name=n, .m_name=mn, .unit=u, .dflt={.t##_value=d}, .min={.t##_value=mi}, .max={.t##_value=ma}, .optstr = s, .allowed_mask_ptr = amp },
    SETUP_PARAMETER_LIST
    #undef X
};


#define SETUP_PARAMETER_NUM  sizeof(SetupParameter)/sizeof(tSetupParameterItem)



#endif // SETUP_TX_H