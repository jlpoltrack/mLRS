//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down WS2812 standard library
//*******************************************************
// Interface:
// #define WS2812_NUMBER_OF_LEDS
// #define WS2812_IO
// #define WS2812_IO_AF
// #define WS2812_TIMx
// #define WS2812_TIMno
// #define WS2812_CHno
// #define WS2812_DMA_CHANNEL_x
// #define WS2812_FREQUENCY        // timer clock in MHz
//*******************************************************
#ifndef STDSTM32_WS2812_H
#define STDSTM32_WS2812_H
#ifdef __cplusplus
extern "C" {
#endif


#include "stdstm32-tim-ext.h"


//-------------------------------------------------------
// WS2812 Defines
//-------------------------------------------------------

#define _APPEND(x,y)          x ## y
#define APPEND(x,y)           _APPEND(x,y)

#define _APPEND4(x,y,z,w)     x ## y ## z ## w
#define APPEND4(x,y,z,w)      _APPEND4(x,y,z,w)


#if WS2812_TIMno == 2 || WS2812_TIMno == 5
#error WS2812 needs a timer with 16 bit CCRx ! the DMA does halfword writes
#endif

#define WS2812_DMAx                   GPDMA1 // WS2812_GPDMA_REQ_TIMx_CHy and the clock enable are for GPDMA1 only

#define WS2812_TIM_CHANNEL_CHx        APPEND(LL_TIM_CHANNEL_CH, WS2812_CHno)
#define WS2812_TIM_CCRx               APPEND(CCR, WS2812_CHno)

#define WS2812_GPDMA_REQ_TIMx_CHy     APPEND4(LL_GPDMA1_REQUEST_TIM, WS2812_TIMno, _CH, WS2812_CHno)

#define WS2812_TIM_EnableDMAReq_CCx   APPEND(LL_TIM_EnableDMAReq_CC, WS2812_CHno)
#define WS2812_TIM_DisableDMAReq_CCx  APPEND(LL_TIM_DisableDMAReq_CC, WS2812_CHno)


//-------------------------------------------------------
// WS2812 Peripheral stuff
//-------------------------------------------------------

#define WS2812_TIMLOW            (WS2812_FREQUENCY * 41 / 100) // ~ 0.41 us
#define WS2812_TIMHIGH           (WS2812_FREQUENCY * 82 / 100) // ~ 0.82 us
#define WS2812_PERIOD            (WS2812_FREQUENCY * 5 / 4 - 1) // 1.25 us, this is the ARR value


void ws2812_periph_init(void)
{
    gpio_init_af(WS2812_IO, IO_MODE_OUTPUT_ALTERNATE_PP, WS2812_IO_AF, IO_SPEED_FAST);

    tim_config_up(WS2812_TIMx, WS2812_PERIOD, TIMER_BASE_MAX);
    tim_config_oc(WS2812_TIMx, WS2812_TIM_CHANNEL_CHx);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);

LL_DMA_InitTypeDef DMA_InitTypeDef;
    LL_DMA_StructInit(&DMA_InitTypeDef);
    DMA_InitTypeDef.DestAddress       = (uint32_t)&(WS2812_TIMx->WS2812_TIM_CCRx);
    DMA_InitTypeDef.Direction         = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitTypeDef.BlkHWRequest      = LL_DMA_HWREQUEST_SINGLEBURST;
    DMA_InitTypeDef.SrcIncMode        = LL_DMA_SRC_INCREMENT;
    DMA_InitTypeDef.DestIncMode       = LL_DMA_DEST_FIXED;
    DMA_InitTypeDef.SrcDataWidth      = LL_DMA_SRC_DATAWIDTH_HALFWORD;
    DMA_InitTypeDef.DestDataWidth     = LL_DMA_DEST_DATAWIDTH_HALFWORD;
    DMA_InitTypeDef.Priority          = LL_DMA_HIGH_PRIORITY;
    DMA_InitTypeDef.Request           = WS2812_GPDMA_REQ_TIMx_CHy;
    DMA_InitTypeDef.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
    LL_DMA_Init(WS2812_DMAx, WS2812_DMA_CHANNEL_x, &DMA_InitTypeDef);

    tim_oc_enable(WS2812_TIMx);
    tim_enable(WS2812_TIMx);
}


//-------------------------------------------------------
// WS2812 Handler stuff
//-------------------------------------------------------

typedef uint32_t tWs2812Color; // is GRB !!

#define WS2812_PURPLE   0x003f3f
#define WS2812_RED      0x00ff00
#define WS2812_GREEN    0xff0000
#define WS2812_BLUE     0x00003f


uint16_t ws2812_buf[WS2812_NUMBER_OF_LEDS * 24 + 2]; // the last two stay 0 and end the frame low


void ws2812_fill(uint8_t idx, tWs2812Color grb)
{
    if (idx >= WS2812_NUMBER_OF_LEDS) return;

    uint16_t p = idx * 24;
    for (uint16_t i = 0; i < 24; i++) {
        ws2812_buf[p++] = (grb & 0x00800000) ? WS2812_TIMHIGH : WS2812_TIMLOW;
        grb <<= 1;
    }
}


void ws2812_init(void)
{
    ws2812_periph_init();

    for (uint8_t n = 0; n < WS2812_NUMBER_OF_LEDS; n++) ws2812_fill(n, 0);
}


void ws2812_send(void)
{
    // never restart a frame on the wire, the buffer content goes out with the next send
    if (LL_DMA_IsEnabledChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x)) return; // EN clears at TC or error

    WS2812_TIM_DisableDMAReq_CCx(WS2812_TIMx);
    LL_DMA_DisableChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    LL_DMA_ClearFlag_TC(WS2812_DMAx, WS2812_DMA_CHANNEL_x);

    // the GPDMA counts source address and block length during the transfer, so both are reloaded
    LL_DMA_SetSrcAddress(WS2812_DMAx, WS2812_DMA_CHANNEL_x, (uint32_t)ws2812_buf);
    LL_DMA_SetBlkDataLength(WS2812_DMAx, WS2812_DMA_CHANNEL_x, sizeof(ws2812_buf)); // in bytes
    LL_DMA_EnableChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    WS2812_TIM_EnableDMAReq_CCx(WS2812_TIMx);
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_WS2812_H
