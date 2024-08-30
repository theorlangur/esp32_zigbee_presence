#include "i2c.hpp"

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

    void I2CBusMaster::SetSDAPin(SDAType sda)
    {
        m_Config.sda_io_num = sda.data();
    }
    SDAType I2CBusMaster::GetSDAPin() const
    {
        return m_Config.sda_io_num;
    }

    void I2CBusMaster::SetSCLPin(SCLType sdc)
    {
        m_Config.scl_io_num = sdc.data();
    }
    SCLType I2CBusMaster::GetSCLPin() const
    {
        return m_Config.scl_io_num;
    }

    void I2CBusMaster::SetPort(I2CPort p)
    {
        m_Config.i2c_port = (int)p;
    }

    I2CPort I2CBusMaster::GetPort() const
    {
        return (I2CPort)m_Config.i2c_port;
    }

    void I2CBusMaster::SetGlitchIgnoreCount(uint8_t c)
    {
        m_Config.glitch_ignore_cnt = c;
    }

    uint8_t I2CBusMaster::GetGlitchIgnoreCount() const
    {
        return m_Config.glitch_ignore_cnt;
    }

    void I2CBusMaster::SetInterruptPriority(int p)
    {
        m_Config.intr_priority = p;
    }

    int I2CBusMaster::GetInterruptPriority() const
    {
        return m_Config.intr_priority;
    }

    void I2CBusMaster::SetEnableInternalPullup(bool enable)
    {
        m_Config.flags.enable_internal_pullup = enable;
    }

    bool I2CBusMaster::GetEnableInternalPullup() const
    {
        return m_Config.flags.enable_internal_pullup;
    }

    std::expected<void, esp_err_t> I2CBusMaster::Open()
    {
        if (auto err = i2c_new_master_bus(&m_Config, &m_Handle); err != ESP_OK)
            return std::unexpected(err);

        return {};
    }

    void I2CBusMaster::Close()
    {
        if (m_Handle)
        {
            i2c_del_master_bus(m_Handle);
            m_Handle = nullptr;
        }

    }

    std::expected<I2CDevice, esp_err_t> I2CBusMaster::Add(uint16_t addr) const
    {
        I2CDevice d(*this, addr);
        return d.Open().transform([&]{ return std::move(d);});
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

    void I2CDevice::SetAddress(uint16_t addr)
    {
        if (!m_Handle)
            m_Config.device_address = addr;
    }

    uint16_t I2CDevice::GetAddress() const
    {
        return m_Config.device_address;
    }

    void I2CDevice::SetSpeedHz(uint32_t hz)
    {
        m_Config.scl_speed_hz = hz;
    }

    uint32_t I2CDevice::GetSpeedHz() const
    {
        return m_Config.scl_speed_hz;
    }

    std::expected<void, esp_err_t> I2CDevice::Open()
    {
        if (auto err = i2c_master_bus_add_device(m_Bus.m_Handle, &m_Config, &m_Handle); err != ESP_OK)
            return std::unexpected(err);

        return {};
    }

    void I2CDevice::Close()
    {
        if (m_Handle)
        {
            i2c_master_bus_rm_device(m_Handle);
            m_Handle = nullptr;
        }
    }
}
