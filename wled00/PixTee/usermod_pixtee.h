#pragma once

#include "wled.h"
#include <driver/rmt.h>


class PixTeeUsermod : public Usermod
{
public:
    static unsigned const _eot_level = 0;
    static rmt_item32_t const _stopTxSymbol;
    static rmt_channel_t const _rmtTxChannelA = rmt_channel_t::RMT_CHANNEL_0;
    static rmt_channel_t const _rmtTxChannelB = rmt_channel_t::RMT_CHANNEL_1;
    static rmt_channel_t const _rmtRxChannel = rmt_channel_t::RMT_CHANNEL_2;
    static unsigned const _numOfTxBuffers = 1;
    static unsigned const _sizeOfTxBuffer = _numOfTxBuffers * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    static unsigned const _numOfRxBuffers = 2;
    static unsigned const _sizeOfRxBuffer = _numOfRxBuffers * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    static unsigned _numRcvdBits;
    static bool _startedTxA;
    static bool _startedTxB;
    static unsigned _rxBufferOffset;
    static unsigned _txBBufferOffset;
    static unsigned _txABufferOffset;
    static unsigned _accumRcvdBits;
    static unsigned _accumRcvdFrames;

    PixTeeUsermod();
    virtual ~PixTeeUsermod();

    virtual void addToConfig(JsonObject& root);
    virtual void appendConfigData();
    virtual void loop();
    virtual bool readFromConfig(JsonObject& root);
    virtual void setup();

    virtual uint16_t getId()
    {
        return USERMOD_ID_PIXTEE;
    }

    static unsigned GetNumOfBitsOnChanA()
    {
        return _numChanABits;
    }

    static unsigned GetNumOfBitsPerPixel()
    {
        return _numOfBitsPerLed * _numOfLedsPerPixel;
    }

private:
    static rmt_isr_handle_t _rmtIsrHandle;
    static gpio_num_t _rxPin;
    static gpio_num_t _txAPin;
    static gpio_num_t _txBPin;
    static gpio_num_t _allocedRxPin;
    static gpio_num_t _allocedTxAPin;
    static gpio_num_t _allocedTxBPin;
    static unsigned _numOfBitsPerLed;
    static unsigned _numOfLedsPerPixel;
    static unsigned _numOfPixelsOnChanA;
    static unsigned _numChanABits;
};
