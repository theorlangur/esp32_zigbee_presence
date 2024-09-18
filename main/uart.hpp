#ifndef UART_H_
#define UART_H_

#include "driver/uart.h"
#include <expected>
#include "generic_helpers.hpp"

namespace uart
{
    enum class Port: std::underlying_type_t<uart_port_t>
    {
        Port0 = UART_NUM_0,
        Port1 = UART_NUM_1,
    };

    enum class DataBits: std::underlying_type_t<uart_word_length_t>
    {
        Bits5 = UART_DATA_5_BITS,
        Bits6 = UART_DATA_6_BITS,
        Bits7 = UART_DATA_7_BITS,
        Bits8 = UART_DATA_8_BITS,
        BitsMax = UART_DATA_BITS_MAX
    };

    enum class StopBits: std::underlying_type_t<uart_stop_bits_t>
    {
        Bits1 = UART_STOP_BITS_1,
        Bits1_5 = UART_STOP_BITS_1_5,
        Bits2 = UART_STOP_BITS_2,
        BitsMax = UART_STOP_BITS_MAX,
    };

    enum class Parity: std::underlying_type_t<uart_parity_t>
    {
        Disable = UART_PARITY_DISABLE,
        Even = UART_PARITY_EVEN,
        Odd = UART_PARITY_ODD,
    };

    enum class HWFlowCtrl: std::underlying_type_t<uart_hw_flowcontrol_t>
    {
        Disable = UART_HW_FLOWCTRL_DISABLE,
        RTS = UART_HW_FLOWCTRL_RTS,
        CTS = UART_HW_FLOWCTRL_CTS,
        CTS_RTS = UART_HW_FLOWCTRL_CTS_RTS,
        Max = UART_HW_FLOWCTRL_MAX
    };

    class Channel
    {
    public:
        using Ref = std::reference_wrapper<Channel>;
        using ExpectedResult = std::expected<Ref, Err>;

        template<typename V>
        using RetVal = RetValT<Ref, V>;

        template<typename V>
        using ExpectedValue = std::expected<RetVal<V>, Err>;

        static const constexpr duration_ms_t kDefaultWait = duration_ms_t{-1};

        Channel(Port p = Port::Port1, int baud_rate = 115200, Parity parity = Parity::Disable);
        ~Channel();

        Channel& SetPort(Port p);
        Port GetPort() const;

        Channel& SetBaudRate(int rate);
        int GetBaudRate() const;

        Channel& SetParity(Parity parity);
        Parity GetParity() const;

        Channel& SetDataBits(DataBits bits);
        DataBits GetDataBits() const;

        Channel& SetStopBits(StopBits bits);
        StopBits GetStopBits() const;

        Channel& SetHWFlowControl(HWFlowCtrl hw);
        HWFlowCtrl GetHWFlowControl() const;

        Channel& SetRxBufferSize(int rx);
        int GetRxBufferSize() const;

        Channel& SetTxBufferSize(int tx);
        int GetTxBufferSize() const;

        Channel& SetQueueSize(int sz);
        int GetQueueSize() const;

        ExpectedResult Configure();

        ExpectedResult SetPins(int tx, int rx, int rts = UART_PIN_NO_CHANGE, int cts = UART_PIN_NO_CHANGE);

        void SetDefaultWait(duration_ms_t w) { m_DefaultWait = w; }
        duration_ms_t GetDefaultWait() const { return m_DefaultWait; }

        ExpectedResult Open();
        ExpectedResult Close();

        ExpectedValue<size_t> GetReadyToReadDataLen();
        ExpectedValue<size_t> GetReadyToWriteDataLen();
        
        ExpectedResult Send(const uint8_t *pData, size_t len);
        ExpectedResult SendWithBreak(const uint8_t *pData, size_t len, size_t breakLen);

        ExpectedValue<size_t> Read(uint8_t *pBuf, size_t len, duration_ms_t wait=kDefaultWait);
        ExpectedResult Flush();
        ExpectedResult WaitAllSent();
        ExpectedValue<uint8_t> ReadByte(duration_ms_t wait=kDefaultWait);
        ExpectedValue<uint8_t> PeekByte(duration_ms_t wait=kDefaultWait);
    private:
        uart_port_t m_Port;
        uart_config_t m_Config;
        QueueHandle_t m_Handle = nullptr;
        int m_RxBufferSize = 1024;
        int m_TxBufferSize = 1024;
        int m_QueueSize = 10;
        duration_ms_t m_DefaultWait{0};
        union{
            struct{
                uint8_t configured: 1;
                uint8_t pins_set: 1;
            }m_State;
            uint8_t m_StateU8 = 0;
        };
        bool m_HasPeekByte = false;
        uint8_t m_PeekByte = 0;
    };
}
#endif
