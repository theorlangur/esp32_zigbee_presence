#ifndef I2C_HPP_
#define I2C_HPP_

#include "generic_helpers.hpp"
#include "driver/i2c_master.h"
#include <expected>
#include <chrono>

namespace i2c
{
    using duration_t = std::chrono::duration<int, std::milli>;
    inline static constexpr const duration_t kForever = duration_t(-1);

    class SDAType: public StrongType<gpio_num_t, struct SDATag>//, Comparable
    {
        using StrongType::StrongType;
    };

    class SCLType: public StrongType<gpio_num_t, struct SCLTag>//, Comparable
    {
        using StrongType::StrongType;
    };

    enum class I2CPort: int
    {
        Auto = -1,
        Port0 = 0,
        Port1 = 1,
    };

    class I2CDevice;

    class I2CBusMaster
    {
    public:
        struct Err
        {
            const char *pLocation;
            esp_err_t code;
        };
        using BusRef = std::reference_wrapper<I2CBusMaster>;
        using ExpectedResult = std::expected<BusRef, Err>;

        I2CBusMaster(SDAType sda, SCLType sdc, I2CPort port = I2CPort::Auto);
        I2CBusMaster(const I2CBusMaster &rhs) = delete;
        I2CBusMaster(I2CBusMaster &&rhs);
        ~I2CBusMaster();

        void SetSDAPin(SDAType sda);
        SDAType GetSDAPin() const;

        void SetSCLPin(SCLType sdc);
        SCLType GetSCLPin() const;

        void SetPort(I2CPort p);
        I2CPort GetPort() const;

        void SetGlitchIgnoreCount(uint8_t c = 7);
        uint8_t GetGlitchIgnoreCount() const;

        void SetInterruptPriority(int p = 0);
        int GetInterruptPriority() const;

        void SetEnableInternalPullup(bool enable);
        bool GetEnableInternalPullup() const;

        ExpectedResult Open();
        void Close();

        std::expected<I2CDevice, Err> Add(uint16_t addr) const;
    private:
        i2c_master_bus_config_t m_Config;
        i2c_master_bus_handle_t m_Handle = nullptr;

        friend class I2CDevice;
    };

    class I2CDevice
    {
    public:
        struct Err
        {
            const char *pLocation;
            esp_err_t code;
        };
        using DevRef = std::reference_wrapper<I2CDevice>;
        using ExpectedResult = std::expected<DevRef, Err>;
        I2CDevice(const I2CBusMaster &bus, uint16_t addr = 0xff, uint32_t speed_hz = 100'000);
        I2CDevice(const I2CDevice &rhs) = delete;
        I2CDevice(I2CDevice &&rhs);

        void SetAddress(uint16_t addr);
        uint16_t GetAddress() const;

        void SetSpeedHz(uint32_t hz);
        uint32_t GetSpeedHz() const;

        ExpectedResult Open();
        void Close();

        ExpectedResult Send(const uint8_t *pBuf, std::size_t len, duration_t d = kForever);
        ExpectedResult Recv(uint8_t *pBuf, std::size_t len, duration_t d = kForever);
        ExpectedResult SendRecv(const uint8_t *pSendBuf, std::size_t sendLen, uint8_t *pRecvBuf, std::size_t recvLen, duration_t d = kForever);
    private:
        const I2CBusMaster &m_Bus;
        i2c_master_dev_handle_t m_Handle = nullptr;
        i2c_device_config_t m_Config;
    };
}

#endif
