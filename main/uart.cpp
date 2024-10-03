#include "uart.hpp"
#include <utility>
#include "functional/functional.hpp"

#define DBG_SEND {\
if (!m_DbgPrintSend) \
{\
    m_DbgPrintSend = true;\
    printf("\nout: ");\
}\
}

#define DBG_READ {\
if (m_DbgPrintSend) \
{\
    m_DbgPrintSend = false;\
    printf("\nin: ");\
}\
}

namespace uart
{
    Channel::Channel(Port p, int baud_rate, Parity parity):
        m_Port(uart_port_t(p)),
        m_Config{
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity    = (uart_parity_t)parity,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
            .flags = {0}
        }
    {
    }

    Channel::~Channel()
    {
        Close();
    }

    Channel& Channel::SetPort(Port p)
    {
        m_Port = uart_port_t(p);
        return *this;
    }

    Port Channel::GetPort() const
    {
        return Port(m_Port);
    }

    Channel& Channel::SetBaudRate(int rate)
    {
        m_Config.baud_rate = rate;
        return *this;
    }

    int Channel::GetBaudRate() const
    {
        return m_Config.baud_rate;
    }

    Channel& Channel::SetParity(Parity parity)
    {
        m_Config.parity = uart_parity_t(parity);
        return *this;
    }

    Parity Channel::GetParity() const
    {
        return Parity{m_Config.parity};
    }

    Channel& Channel::SetDataBits(DataBits bits)
    {
        m_Config.data_bits = uart_word_length_t(bits);
        return *this;
    }

    DataBits Channel::GetDataBits() const
    {
        return DataBits{m_Config.data_bits};
    }

    Channel& Channel::SetStopBits(StopBits bits)
    {
        m_Config.stop_bits = uart_stop_bits_t(bits);
        return *this;
    }

    StopBits Channel::GetStopBits() const
    {
        return StopBits{m_Config.stop_bits};
    }

    Channel& Channel::SetHWFlowControl(HWFlowCtrl hw)
    {
        m_Config.flow_ctrl = uart_hw_flowcontrol_t(hw);
        return *this;
    }

    HWFlowCtrl Channel::GetHWFlowControl() const
    {
        return HWFlowCtrl{m_Config.flow_ctrl};
    }

    Channel& Channel::SetRxBufferSize(int rx)
    {
        m_RxBufferSize = rx;
        return *this;
    }

    int Channel::GetRxBufferSize() const
    {
        return m_RxBufferSize;
    }

    Channel& Channel::SetTxBufferSize(int tx)
    {
        m_TxBufferSize = tx;
        return *this;
    }

    int Channel::GetTxBufferSize() const
    {
        return m_TxBufferSize;
    }

    Channel& Channel::SetQueueSize(int sz)
    {
        m_QueueSize = sz;
        return *this;
    }

    int Channel::GetQueueSize() const 
    {
        return m_QueueSize;
    }

    Channel::ExpectedResult Channel::Configure()
    {
        CALL_ESP_EXPECTED("uart::Channel::Configure", uart_param_config(m_Port, &m_Config));
        m_State.configured = true;
        return std::ref(*this);
    }

    Channel::ExpectedResult Channel::SetPins(int tx, int rx, int rts, int cts)
    {
        if (!m_State.configured)
            return std::unexpected(Err{"uart::Channel::SetPins", ESP_ERR_INVALID_STATE});

        CALL_ESP_EXPECTED("uart::Channel::SetPins", uart_set_pin(m_Port, tx, rx, rts, cts));
        m_State.pins_set = true;
        return std::ref(*this);
    }

    void Channel::uart_event_loop(Channel &c)
    {
        uart_event_t event;
        while (true) {
            if (xQueueReceive(c.m_Handle, &event, pdMS_TO_TICKS(2000))) 
            {
                switch (event.type) 
                {
                    case UART_DATA:
                        if (!c.GetReadyToReadDataLen().value().v)//only if data really available
                            continue;
                        break;
                    case UART_BUFFER_FULL:
                    case UART_FIFO_OVF:
                        xQueueReset(c.m_Handle);
                        break;
                    case UART_EVENT_MAX:
                        return;//we're done
                    default:
                        break;
                }
                c.m_EventCallback(event.type);
            }
        }
    }

    Channel::ExpectedResult Channel::Open()
    {
        if (!m_State.pins_set)
            return std::unexpected(Err{"uart::Channel::Open", ESP_ERR_INVALID_STATE});

        if (m_EventCallback)
        {
            CALL_ESP_EXPECTED("uart::Channel::Open", uart_driver_install(m_Port, m_RxBufferSize, m_TxBufferSize, m_QueueSize, &m_Handle, 0));
            m_QueueTask = thread::start_task({.pName = "uart::events", .stackSize=2048}, uart_event_loop, std::ref(*this));
        }
        else
            CALL_ESP_EXPECTED("uart::Channel::Open no events", uart_driver_install(m_Port, m_RxBufferSize, m_TxBufferSize, 0, nullptr, 0));
        return std::ref(*this);
    }

    Channel::ExpectedResult Channel::Close()
    {
        m_StateU8 = 0;
        return std::ref(*this);
    }

    Channel::ExpectedValue<size_t> Channel::GetReadyToReadDataLen()
    {
        size_t len;
        CALL_ESP_EXPECTED("uart::Channel::GetReadyToReadDataLen", uart_get_buffered_data_len(m_Port, &len));
        return RetVal<size_t>{*this, len};
    }

    Channel::ExpectedValue<size_t> Channel::GetReadyToWriteDataLen()
    {
        size_t len;
        CALL_ESP_EXPECTED("uart::Channel::GetReadyToWriteDataLen", uart_get_tx_buffer_free_size(m_Port, &len));
        return RetVal<size_t>{*this, len};
    }

    Channel::ExpectedResult Channel::Send(const uint8_t *pData, size_t len)
    {
        if (m_Dbg) 
        {
            DBG_SEND;
            for(int i = 0; i < len; ++i)
                printf(" %X", pData[i]);
        }
        int r = uart_write_bytes(m_Port, pData, len);
        if (r < 0)
            return std::unexpected(Err{"uart::Channel::Send", ESP_ERR_INVALID_ARG});
        if (r != len)
            return std::unexpected(Err{"uart::Channel::Send", ESP_ERR_INVALID_SIZE});
        return std::ref(*this);
    }

    Channel::ExpectedResult Channel::SendWithBreak(const uint8_t *pData, size_t len, size_t breakLen)
    {
        if (m_Dbg) 
        {
            DBG_SEND;
            for(int i = 0; i < len; ++i)
                printf(" %X", pData[i]);
        }
        int r = uart_write_bytes_with_break(m_Port, pData, len, breakLen);
        if (r < 0)
            return std::unexpected(Err{"uart::Channel::Send", ESP_ERR_INVALID_ARG});
        if (r != len)
            return std::unexpected(Err{"uart::Channel::Send", ESP_ERR_INVALID_SIZE});

        return std::ref(*this);
    }

    Channel::ExpectedValue<size_t> Channel::Read(uint8_t *pBuf, size_t len, duration_ms_t wait)
    {
        if (!len) return RetVal<size_t>{*this, size_t(0)};
        if (wait == kDefaultWait) wait = m_DefaultWait;
        bool addedPeek = m_HasPeekByte;
        if (m_HasPeekByte)
        {
            pBuf[0] = m_PeekByte;
            m_HasPeekByte = false;
            --len;
            ++pBuf;
            if (!len) return RetVal<size_t>{*this, 1};
        }

        int r = uart_read_bytes(m_Port, pBuf, len, wait.count() / portTICK_PERIOD_MS);
        if (r < 0)
            return std::unexpected(Err{"uart::Channel::Read", ESP_ERR_INVALID_ARG});

        if (m_Dbg) 
        {
            DBG_READ;
            for(int i = 0; i < (int(r) + int(addedPeek)); ++i)
                printf(" %X", pBuf[i]);
        }

        return RetVal<size_t>{*this, size_t(r) + size_t(addedPeek)};
    }

    Channel::ExpectedValue<uint8_t> Channel::ReadByte(duration_ms_t wait)
    {
        using namespace functional;
        if (wait == kDefaultWait) wait = m_DefaultWait;
        uint8_t b;
        CHECK_STACK(3500);
        if (auto e = Read(&b, 1, wait); !e)
            return std::unexpected(e.error());
        else if (auto l = e.value().v; !l)
        {
            CHECK_STACK(100);
            if (m_Dbg)
                printf("Nothing to read. Wait: %d\n", wait.count());
            return std::unexpected(::Err{"Channel::ReadByte no data", ESP_OK});
        }else
            return RetVal{std::ref(*this), b};
    }

    Channel::ExpectedValue<uint8_t> Channel::PeekByte(duration_ms_t wait)
    {
        using namespace functional;
        if (m_HasPeekByte)
            return RetVal{std::ref(*this), m_PeekByte};
        if (wait == kDefaultWait) wait = m_DefaultWait;
        if (auto e = ReadByte(wait); !e) return e;
        else{
            m_HasPeekByte = true;
            m_PeekByte = e.value().v;
            return RetVal{std::ref(*this), m_PeekByte};
        }
    }

    Channel::ExpectedResult Channel::Flush()
    {
        CALL_ESP_EXPECTED("uart::Channel::Flush", uart_flush_input(m_Port));
        m_HasPeekByte = false;
        return std::ref(*this);
    }

    Channel::ExpectedResult Channel::WaitAllSent()
    {
        CALL_ESP_EXPECTED("uart::Channel::WaitAllSent", uart_wait_tx_idle_polling(m_Port));
        return std::ref(*this);
    }
}
