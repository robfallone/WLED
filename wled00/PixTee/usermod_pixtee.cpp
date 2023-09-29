#include "usermod_pixtee.h"
#include <esp_check.h>
#include <esp_core_dump.h>
#include <hal/rmt_hal.h>
#include <hal/rmt_ll.h>


#ifndef USERMOD_PIXTEE_INPUT_PIN
#define USERMOD_PIXTEE_INPUT_PIN    GPIO_NUM_1
#endif

#ifndef USERMOD_PIXTEE_OUTA_PIN
#define USERMOD_PIXTEE_OUTA_PIN     GPIO_NUM_0
#endif

#ifndef USERMOD_PIXTEE_OUTB_PIN
#define USERMOD_PIXTEE_OUTB_PIN     GPIO_NUM_4
#endif

#define RMT_LL_MAX_FILTER_VALUE             255
#define RMT_LL_MAX_IDLE_VALUE               65535
#define RMT_LL_EVENT_TX_DONE(channel)     (1 << (channel))
#define RMT_LL_EVENT_RX_DONE(channel)     (1 << ((channel) + 2))
#define RMT_LL_EVENT_RX_THRES(channel)    (1 << ((channel) + 10))
#define RMT_RX_CHANNEL_ENCODING_START (SOC_RMT_CHANNELS_PER_GROUP-SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_TX_CHANNEL_ENCODING_END   (SOC_RMT_TX_CANDIDATES_PER_GROUP-1)
#define RMT_DECODE_RX_CHANNEL(encode_chan) ((encode_chan - RMT_RX_CHANNEL_ENCODING_START))
#define RMT_ENCODE_RX_CHANNEL(decode_chan) ((decode_chan + RMT_RX_CHANNEL_ENCODING_START))

typedef enum
{
    RMT_LL_MEM_OWNER_SW = 0,
    RMT_LL_MEM_OWNER_HW = 1,
} rmt_ll_mem_owner_t;

DRAM_ATTR unsigned pixTeeCnts[5] = { 0 };
DRAM_ATTR portMUX_TYPE rmt_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define RMT_ENTER_CRITICAL() // portENTER_CRITICAL_SAFE(&rmt_spinlock)
#define RMT_EXIT_CRITICAL()  // portEXIT_CRITICAL_SAFE(&rmt_spinlock)

DRAM_ATTR rmt_isr_handle_t PixTeeUsermod::_rmtIsrHandle = nullptr;
DRAM_ATTR unsigned PixTeeUsermod::p_numRcvdBits = 0;
DRAM_ATTR unsigned PixTeeUsermod::_accumRcvdBits = 0;
DRAM_ATTR unsigned PixTeeUsermod::_accumRcvdFrames = 0;
DRAM_ATTR unsigned PixTeeUsermod::_numRcvdBits = 0;
DRAM_ATTR unsigned PixTeeUsermod::_rxBufferOffset = 0;
DRAM_ATTR bool PixTeeUsermod::_startedTxA = false;
DRAM_ATTR bool PixTeeUsermod::_startedTxB = false;
DRAM_ATTR unsigned PixTeeUsermod::_txBBufferOffset = 0;
DRAM_ATTR unsigned PixTeeUsermod::_txABufferOffset = 0;
DRAM_ATTR gpio_num_t PixTeeUsermod::_rxPin = USERMOD_PIXTEE_INPUT_PIN;
DRAM_ATTR gpio_num_t PixTeeUsermod::_txAPin = USERMOD_PIXTEE_OUTA_PIN;
DRAM_ATTR gpio_num_t PixTeeUsermod::_txBPin = USERMOD_PIXTEE_OUTB_PIN;
DRAM_ATTR gpio_num_t PixTeeUsermod::_allocedRxPin = GPIO_NUM_NC;
DRAM_ATTR gpio_num_t PixTeeUsermod::_allocedTxAPin = GPIO_NUM_NC;
DRAM_ATTR gpio_num_t PixTeeUsermod::_allocedTxBPin = GPIO_NUM_NC;
DRAM_ATTR unsigned PixTeeUsermod::_numOfBitsPerLed = 8;
DRAM_ATTR unsigned PixTeeUsermod::_numOfLedsPerPixel = 3;
DRAM_ATTR unsigned PixTeeUsermod::_numOfPixelsOnChanA = 3;
DRAM_ATTR unsigned PixTeeUsermod::_numChanABits = _numOfPixelsOnChanA * _numOfBitsPerLed * _numOfLedsPerPixel;

DRAM_ATTR rmt_item32_t const PixTeeUsermod::_stopTxSymbol =
{
    {
        {
            .duration0 = 0, // a RMT word whose duration is zero means a "stop" pattern
            .level0 = _eot_level,
            .duration1 = 0,
            .level1 = _eot_level,
        }
    }
};

__attribute__((always_inline))
static inline void rmt_ll_tx_reset_loop_count(rmt_dev_t *dev, uint32_t channel)
{
    dev->tx_lim[channel].loop_count_reset = 1;
    dev->tx_lim[channel].loop_count_reset = 0;
}

__attribute__((always_inline))
static inline uint32_t rmt_ll_rx_get_memory_writer_offset(uint32_t channel)
{
    return RMT.rx_status[channel].mem_waddr_ex - (channel + 2) * 48;
}

static void IRAM_ATTR my_rmt_copy_symbols(void)
{
    volatile rmt_item32_t const* const rxBuffer = &RMTMEM.chan[PixTeeUsermod::_rmtRxChannel].data32[0];
    volatile rmt_item32_t* const txABuffer = &RMTMEM.chan[PixTeeUsermod::_rmtTxChannelA].data32[0];
    volatile rmt_item32_t* const txBBuffer = &RMTMEM.chan[PixTeeUsermod::_rmtTxChannelB].data32[0];

    unsigned const rxWrOffset = rmt_ll_rx_get_memory_writer_offset(RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel));

    while ((PixTeeUsermod::_rxBufferOffset != rxWrOffset) && (PixTeeUsermod::_numRcvdBits < PixTeeUsermod::GetNumOfBitsOnChanA()))
    {
        txABuffer[PixTeeUsermod::_txABufferOffset].val = rxBuffer[PixTeeUsermod::_rxBufferOffset].val;
        PixTeeUsermod::_numRcvdBits += 1;
        PixTeeUsermod::_rxBufferOffset += 1;

        if (PixTeeUsermod::_rxBufferOffset >= PixTeeUsermod::_sizeOfRxBuffer)
        {
            PixTeeUsermod::_rxBufferOffset = 0;
        }

        PixTeeUsermod::_txABufferOffset += 1;

        if (PixTeeUsermod::_txABufferOffset >= PixTeeUsermod::_sizeOfTxBuffer)
        {
            PixTeeUsermod::_txABufferOffset = 0;
        }
    }

    if (PixTeeUsermod::_numRcvdBits == PixTeeUsermod::GetNumOfBitsOnChanA())
    {
        txABuffer[PixTeeUsermod::_txABufferOffset].val = PixTeeUsermod::_stopTxSymbol.val;
    }

    while (PixTeeUsermod::_rxBufferOffset != rxWrOffset)
    {
        txBBuffer[PixTeeUsermod::_txBBufferOffset].val = rxBuffer[PixTeeUsermod::_rxBufferOffset].val;
        PixTeeUsermod::_numRcvdBits += 1;
        PixTeeUsermod::_rxBufferOffset += 1;

        if (PixTeeUsermod::_rxBufferOffset >= PixTeeUsermod::_sizeOfRxBuffer)
        {
            PixTeeUsermod::_rxBufferOffset = 0;
        }

        PixTeeUsermod::_txBBufferOffset += 1;

        if (PixTeeUsermod::_txBBufferOffset >= PixTeeUsermod::_sizeOfTxBuffer)
        {
            PixTeeUsermod::_txBBufferOffset = 0;
        }
    }
}

__attribute__((always_inline))
static inline void rmt_ll_clear_interrupt_status(uint32_t mask)
{
    RMT.int_clr.val = mask;
}

static void IRAM_ATTR PixTeeRmtIsr(void* args)
{
    uint32_t const txStatus = rmt_ll_get_tx_end_interrupt_status(&RMT);

    if (txStatus & RMT_LL_EVENT_TX_DONE(0))
    {
        pixTeeCnts[0] += 1;
        rmt_ll_tx_reset_pointer(&RMT, 0);
        rmt_ll_clear_tx_end_interrupt(&RMT, 0);
    }

    if (txStatus & RMT_LL_EVENT_TX_DONE(1))
    {
        pixTeeCnts[1] += 1;
        rmt_ll_tx_reset_pointer(&RMT, 1);
        rmt_ll_clear_tx_end_interrupt(&RMT, 1);
    }

    bool const rxThrsh = rmt_ll_get_rx_thres_interrupt_status(&RMT) != 0;

    if (rxThrsh)
    {
        pixTeeCnts[2] += 1;
        rmt_ll_clear_interrupt_status(RMT_LL_EVENT_RX_THRES(RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel)));
    }

    bool const rxDone = rmt_ll_get_rx_end_interrupt_status(&RMT) != 0;

    if (rxDone)
    {
        pixTeeCnts[3] += 1;
        RMT_ENTER_CRITICAL();
        rmt_ll_clear_interrupt_status(RMT_LL_EVENT_RX_DONE(RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel)));
        rmt_ll_rx_enable(&RMT, RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel), false);
        RMT_EXIT_CRITICAL();
    }

    if (rxThrsh || rxDone)
    {
        rmt_ll_rx_set_mem_owner(&RMT, RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel), RMT_LL_MEM_OWNER_SW);
        my_rmt_copy_symbols();
        rmt_ll_rx_set_mem_owner(&RMT, RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel), RMT_LL_MEM_OWNER_HW);

        if ((PixTeeUsermod::_numRcvdBits >= PixTeeUsermod::GetNumOfBitsPerPixel()) && !PixTeeUsermod::_startedTxA)
        {
            PixTeeUsermod::_startedTxA = true;
            rmt_ll_tx_start(&RMT, PixTeeUsermod::_rmtTxChannelA);
        }

        if ((PixTeeUsermod::_numRcvdBits > 80) && !PixTeeUsermod::_startedTxB)
        {
            PixTeeUsermod::_startedTxB = true;
            rmt_ll_tx_start(&RMT, PixTeeUsermod::_rmtTxChannelB);
        }

        if (rxDone)
        {
            volatile rmt_item32_t* const txBBuffer = &RMTMEM.chan[PixTeeUsermod::_rmtTxChannelB].data32[0];
            txBBuffer[PixTeeUsermod::_txBBufferOffset].val = PixTeeUsermod::_stopTxSymbol.val;
            pixTeeCnts[4] += PixTeeUsermod::_numRcvdBits;
            PixTeeUsermod::p_numRcvdBits = PixTeeUsermod::_numRcvdBits;
            PixTeeUsermod::_accumRcvdBits += PixTeeUsermod::_numRcvdBits;
            PixTeeUsermod::_accumRcvdFrames += 1;
            PixTeeUsermod::_numRcvdBits = 0;
            PixTeeUsermod::_rxBufferOffset = 0;
            PixTeeUsermod::_startedTxA = false;
            PixTeeUsermod::_txABufferOffset = 0;
            PixTeeUsermod::_startedTxB = false;
            PixTeeUsermod::_txBBufferOffset = 0;
            rmt_ll_rx_reset_pointer(&RMT, RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel));
            rmt_ll_rx_enable(&RMT, RMT_DECODE_RX_CHANNEL(PixTeeUsermod::_rmtRxChannel), true);
        }
    }
}

PixTeeUsermod::PixTeeUsermod()
{
}

PixTeeUsermod::~PixTeeUsermod()
{
}

void PixTeeUsermod::addToConfig(JsonObject& root)
{
    JsonObject top = root.createNestedObject(FPSTR("PixTee"));
    top[F("Pixels on Output A")] = _numOfPixelsOnChanA;
    top[F("LEDs per Pixel")] = _numOfLedsPerPixel;
    top[F("Bits per LED")] = _numOfBitsPerLed;

    JsonArray pins = top.createNestedArray("pin");
    pins.add(_rxPin);
    pins.add(_txAPin);
    pins.add(_txBPin);
}

void PixTeeUsermod::appendConfigData()
{
    oappend(SET_F("addInfo('PixTee:pin[]',0,'','for Input');"));
    oappend(SET_F("addInfo('PixTee:pin[]',1,'','for Output A');"));
    oappend(SET_F("addInfo('PixTee:pin[]',2,'','for Output B');"));
}

void PixTeeUsermod::loop()
{
    static unsigned lastTime = 0;
    unsigned const currTime = millis();
    unsigned const elapsedTime = currTime - lastTime;
    unsigned const timeThreshold = 10000;

    if (elapsedTime >= timeThreshold)
    {
        unsigned const numRcvdBits = p_numRcvdBits;
        p_numRcvdBits = 0;
        unsigned const numRcvdFrames = p_numRcvdFrames;
        p_numRcvdFrames = 0;
        lastTime = currTime;

        DEBUG_PRINT(F("PixTeeUsermod::p_numRcvdBits = "));
        DEBUG_PRINTLN(numRcvdBits);
        DEBUG_PRINT(F("PixTeeUsermod::p_numRcvdFrames = "));
        DEBUG_PRINTLN(numRcvdFrames);
        DEBUG_PRINT(F("Frames / sec = "));
        DEBUG_PRINTLN(numRcvdFrames * 1000 / timeThreshold);
        DEBUG_PRINT(F("BitsPerFrame = "));
        DEBUG_PRINTLN(numRcvdBits / numRcvdFrames);
        DEBUG_PRINT(F("PixelsPerFrame = "));
        DEBUG_PRINTLN(numRcvdBits / (numRcvdFrames * _numOfBitsPerLed * _numOfLedsPerPixel));
    }
}

bool PixTeeUsermod::readFromConfig(JsonObject& root)
{
    JsonObject top = root[FPSTR("PixTee")];
    bool configComplete = !top.isNull();

    configComplete &= getJsonValue(top["Pixels on Output A"], _numOfPixelsOnChanA, 3);
    configComplete &= getJsonValue(top["LEDs per Pixel"], _numOfLedsPerPixel, 3);
    configComplete &= getJsonValue(top["Bits per LED"], _numOfBitsPerLed, 8);
    configComplete &= getJsonValue(top["pin"][0], _rxPin, USERMOD_PIXTEE_INPUT_PIN);
    configComplete &= getJsonValue(top["pin"][1], _txAPin, USERMOD_PIXTEE_OUTA_PIN);
    configComplete &= getJsonValue(top["pin"][2], _txBPin, USERMOD_PIXTEE_OUTB_PIN);
    // setup();

    return configComplete;
}

void PixTeeUsermod::setup()
{
    DEBUG_PRINTLN(F("PixTeeUsermod --- SETUP"));

    if (_rmtIsrHandle == nullptr)
    {
        rmt_isr_register(PixTeeRmtIsr, nullptr, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, &_rmtIsrHandle);
    }

    if (_rmtIsrHandle != nullptr)
    {
        static uint32_t const RmtClockFreq = 80000000;
        static unsigned const RmtClockDivider = 4;
        static uint64_t const RmtClockResolution = RmtClockFreq / RmtClockDivider;

        rmt_config_t rmtConfig =
        {
            .rmt_mode = rmt_mode_t::RMT_MODE_TX,
            .channel = _rmtTxChannelA,
            .gpio_num = _txAPin,
            .clk_div = RmtClockDivider,
            .mem_block_num = _numOfTxBuffers,
            .flags = 0,
        };

        rmtConfig.tx_config.carrier_freq_hz = 100;
        rmtConfig.tx_config.carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW;
        rmtConfig.tx_config.idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW;
        rmtConfig.tx_config.carrier_duty_percent = 50;
        rmtConfig.tx_config.carrier_en = false;
        rmtConfig.tx_config.loop_en = false;
        rmtConfig.tx_config.idle_output_en = true;

        if (_allocedTxAPin != _txAPin)
        {
            RMT_ENTER_CRITICAL();

            if (_allocedTxAPin != GPIO_NUM_NC)
            {
                pinManager.deallocatePin(_allocedTxAPin, PinOwner::UM_PixTee);
                _allocedTxAPin = GPIO_NUM_NC;
            }

            if (pinManager.allocatePin(_txAPin, true, PinOwner::UM_PixTee) &&
                (rmt_config(&rmtConfig) == ESP_OK))
            {
                _allocedTxAPin = _txAPin;
        #if SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP
                rmt_ll_tx_enable_loop_autostop(&RMT, _rmtTxChannelA, true);
        #endif
                rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(_rmtTxChannelA), true);
            }
            else
            {
                DEBUG_PRINTLN(F("Output pin A allocation failed."));
            }

            RMT_EXIT_CRITICAL();
        }

        if (_allocedTxBPin != _txBPin)
        {
            rmtConfig.channel = _rmtTxChannelB;
            rmtConfig.gpio_num = _txBPin;

            RMT_ENTER_CRITICAL();

            if (_allocedTxBPin != GPIO_NUM_NC)
            {
                pinManager.deallocatePin(_allocedTxBPin, PinOwner::UM_PixTee);
                _allocedTxBPin = GPIO_NUM_NC;
            }

            if (pinManager.allocatePin(_txBPin, true, PinOwner::UM_PixTee) &&
                (rmt_config(&rmtConfig) == ESP_OK))
            {
                _allocedTxBPin = _txBPin;
        #if SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP
                rmt_ll_tx_enable_loop_autostop(&RMT, _rmtTxChannelB, true);
        #endif
                rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(_rmtTxChannelB), true);
            }
            else
            {
                DEBUG_PRINTLN(F("Output pin B allocation failed."));
            }

            RMT_EXIT_CRITICAL();
        }

        if (_allocedRxPin != _rxPin)
        {
            rmtConfig.rmt_mode = rmt_mode_t::RMT_MODE_RX;
            rmtConfig.channel = _rmtRxChannel;
            rmtConfig.gpio_num = _rxPin;
            rmtConfig.mem_block_num = _numOfRxBuffers;
            static unsigned const signal_range_max_ns = 10000;
            uint32_t const idle_reg_value = (RmtClockResolution * signal_range_max_ns) / 1000000000UL;
            assert(idle_reg_value <= RMT_LL_MAX_IDLE_VALUE);
            rmtConfig.rx_config.idle_threshold = idle_reg_value;
            static unsigned const signal_range_min_ns = 100;
            uint32_t const filter_reg_value = (RmtClockResolution * signal_range_min_ns) / 1000000000UL;
            assert(filter_reg_value <= RMT_LL_MAX_FILTER_VALUE);
            rmtConfig.rx_config.filter_ticks_thresh = filter_reg_value;
            rmtConfig.rx_config.filter_en = true;
            rmtConfig.rx_config.rm_carrier = false;
            rmtConfig.rx_config.carrier_freq_hz = 100;
            rmtConfig.rx_config.carrier_duty_percent = 50;
            rmtConfig.rx_config.carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW;

            RMT_ENTER_CRITICAL();

            if (_allocedRxPin != -1)
            {
                pinManager.deallocatePin(_allocedRxPin, PinOwner::UM_PixTee);
                _allocedRxPin = GPIO_NUM_NC;
            }

            if (pinManager.allocatePin(_rxPin, false, PinOwner::UM_PixTee) &&
                (rmt_config(&rmtConfig) == ESP_OK))
            {
                _allocedRxPin = _rxPin;
                _numRcvdBits = 0;
                _rxBufferOffset = 0;
                _startedTxA = false;
                _txABufferOffset = 0;
                _startedTxB = false;
                _txBBufferOffset = 0;

                rmt_ll_rx_enable(&RMT, RMT_DECODE_RX_CHANNEL(_rmtRxChannel), false);
                rmt_ll_rx_reset_pointer(&RMT, RMT_DECODE_RX_CHANNEL(_rmtRxChannel));
                rmt_ll_clear_interrupt_status(RMT_LL_EVENT_RX_DONE(RMT_DECODE_RX_CHANNEL(_rmtRxChannel)));
                rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(RMT_DECODE_RX_CHANNEL(_rmtRxChannel)), true);
            #if SOC_RMT_SUPPORT_RX_PINGPONG
                rmt_ll_rx_set_limit(&RMT, RMT_DECODE_RX_CHANNEL(_rmtRxChannel), _numOfBitsPerLed);
                rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_THRES(RMT_DECODE_RX_CHANNEL(_rmtRxChannel)), true);
            #endif
                rmt_ll_rx_enable(&RMT, RMT_DECODE_RX_CHANNEL(_rmtRxChannel), true);
            }
            else
            {
                DEBUG_PRINTLN(F("Input pin allocation failed."));
            }

            RMT_EXIT_CRITICAL();
        }
    }
}
