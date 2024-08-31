#include "i2c.hpp"
#include <functional>

#define CALL_ESP_EXPECTED(location, f) \
    if (auto err = f; err != ESP_OK) \
        return std::unexpected(Err{location, err})

namespace i2c
{
    I2CBusMaster::I2CBusMaster(SDAType sda, SCLType slc, I2CPort port):
        m_Config{
            .i2c_port = (int)I2CPort::Auto,
            .sda_io_num = sda.data(),
            .scl_io_num = slc.data(),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 32,
            .flags{0}
        }
    {
    }

    I2CBusMaster::I2CBusMaster(I2CBusMaster &&rhs):
        m_Config(rhs.m_Config),
        m_Handle(rhs.m_Handle)
    {
        rhs.m_Handle = nullptr;
    }

    I2CBusMaster::~I2CBusMaster()
    {
        Close();
    }

    I2CBusMaster& I2CBusMaster::SetSDAPin(SDAType sda)
    {
        m_Config.sda_io_num = sda.data();
        return *this;
    }
    SDAType I2CBusMaster::GetSDAPin() const
    {
        return m_Config.sda_io_num;
    }

    I2CBusMaster& I2CBusMaster::SetSCLPin(SCLType sdc)
    {
        m_Config.scl_io_num = sdc.data();
        return *this;
    }
    SCLType I2CBusMaster::GetSCLPin() const
    {
        return m_Config.scl_io_num;
    }

    I2CBusMaster& I2CBusMaster::SetPort(I2CPort p)
    {
        m_Config.i2c_port = (int)p;
        return *this;
    }

    I2CPort I2CBusMaster::GetPort() const
    {
        return (I2CPort)m_Config.i2c_port;
    }

    I2CBusMaster& I2CBusMaster::SetGlitchIgnoreCount(uint8_t c)
    {
        m_Config.glitch_ignore_cnt = c;
        return *this;
    }

    uint8_t I2CBusMaster::GetGlitchIgnoreCount() const
    {
        return m_Config.glitch_ignore_cnt;
    }

    I2CBusMaster& I2CBusMaster::SetInterruptPriority(int p)
    {
        m_Config.intr_priority = p;
        return *this;
    }

    int I2CBusMaster::GetInterruptPriority() const
    {
        return m_Config.intr_priority;
    }

    I2CBusMaster& I2CBusMaster::SetEnableInternalPullup(bool enable)
    {
        m_Config.flags.enable_internal_pullup = enable;
        return *this;
    }

    bool I2CBusMaster::GetEnableInternalPullup() const
    {
        return m_Config.flags.enable_internal_pullup;
    }

    I2CBusMaster::ExpectedResult I2CBusMaster::Open()
    {
        CALL_ESP_EXPECTED("I2CBusMaster::Open", i2c_new_master_bus(&m_Config, &m_Handle));
        return std::ref(*this);
    }

    I2CBusMaster::ExpectedResult I2CBusMaster::Close()
    {
        if (m_Handle)
        {
            i2c_del_master_bus(m_Handle);
            m_Handle = nullptr;
        }
        return std::ref(*this);
    }

    std::expected<I2CDevice, I2CBusMaster::Err> I2CBusMaster::Add(uint16_t addr) const
    {
        I2CDevice d(*this, addr);
        return d.Open()
            .transform([&](auto &&v){ return std::move(d);})
            .transform_error([](auto &&e)->Err{ return Err{"I2CBusMaster::Add", e.code}; });
    }

    I2CDevice::I2CDevice(const I2CBusMaster &bus, uint16_t addr, uint32_t speed_hz):
        m_Bus(bus),
        m_Config{
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = speed_hz,
            .scl_wait_us = 0,
            .flags{0}
        }
    {
    }

    I2CDevice::I2CDevice(I2CDevice &&rhs):
        m_Bus(rhs.m_Bus),
        m_Handle(rhs.m_Handle),
        m_Config(rhs.m_Config)
    {
        rhs.m_Handle = nullptr;
    }

    I2CDevice& I2CDevice::SetAddress(uint16_t addr)
    {
        m_Config.device_address = addr;
        return *this;
    }

    uint16_t I2CDevice::GetAddress() const
    {
        return m_Config.device_address;
    }

    I2CDevice& I2CDevice::SetSpeedHz(uint32_t hz)
    {
        m_Config.scl_speed_hz = hz;
        return *this;
    }

    uint32_t I2CDevice::GetSpeedHz() const
    {
        return m_Config.scl_speed_hz;
    }

    I2CDevice::ExpectedResult I2CDevice::Open()
    {
        CALL_ESP_EXPECTED("I2CDevice::Open", i2c_master_bus_add_device(m_Bus.m_Handle, &m_Config, &m_Handle));
        return std::ref(*this);
    }

    I2CDevice::ExpectedResult I2CDevice::Close()
    {
        if (m_Handle)
        {
            i2c_master_bus_rm_device(m_Handle);
            m_Handle = nullptr;
        }
        return std::ref(*this);
    }

    I2CDevice::ExpectedResult I2CDevice::Send(const uint8_t *pBuf, std::size_t len, duration_t d)
    {
        if (!m_Handle) return std::unexpected(Err{"I2CDevice::Send", ESP_ERR_INVALID_STATE});

        CALL_ESP_EXPECTED("I2CDevice::Send", i2c_master_transmit(m_Handle, pBuf, len, d.count()));
        return std::ref(*this);
    }

    I2CDevice::ExpectedResult I2CDevice::Recv(uint8_t *pBuf, std::size_t len, duration_t d)
    {
        if (!m_Handle) return std::unexpected(Err{"I2CDevice::Recv", ESP_ERR_INVALID_STATE});

        CALL_ESP_EXPECTED("I2CDevice::Recv", i2c_master_receive(m_Handle, pBuf, len, d.count()));
        return std::ref(*this);
    }

    I2CDevice::ExpectedResult I2CDevice::SendRecv(const uint8_t *pSendBuf, std::size_t sendLen, uint8_t *pRecvBuf, std::size_t recvLen, duration_t d)
    {
        if (!m_Handle) return std::unexpected(Err{"I2CDevice::SendRecv", ESP_ERR_INVALID_STATE});

        CALL_ESP_EXPECTED("I2CDevice::SendRecv", i2c_master_transmit_receive(m_Handle, pSendBuf, sendLen, pRecvBuf, recvLen, d.count()));
        return std::ref(*this);
    }
}
