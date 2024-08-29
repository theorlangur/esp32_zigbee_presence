#ifndef I2C_HPP_
#define I2C_HPP_

#include "generic_helpers.hpp"
#include "driver/i2c_master.h"

namespace i2c
{
    class SDAType: private StrongType<gpio_num_t, struct SDATag>, Comparable
    {
    };

    class SDCType: private StrongType<gpio_num_t, struct SDCTag>, Comparable
    {
    };

    enum class I2CPort: int
    {
        Auto = -1,
        Port0 = 0,
        Port1 = 1,
    };

    enum class I2CClockSource: int
    {
        APB,
        Default
    };

    class I2CBusMaster
    {
    public:
        I2CBusMaster(SDAType sda, SDCType sdc, I2CPort port, I2CClockSource clkSrc);
        ~I2CBusMaster();

        void SetSDAPin(SDAType sda);
        SDAType GetSDAPin() const;

        void SetSDCPin(SDCType sdc);
        SDCType GetSDCPin() const;

        void SetPort(I2CPort p);
        I2CPort GetPort() const;

        void SetClockSource(I2CClockSource src);
        I2CClockSource GetClockSource() const;

        void SetGlitchIgnoreCount(uint8_t c = 7);
        uint8_t GetGlitchIgnoreCount() const;

        void SetInterruptPriority(int p = 0);
        int GetInterruptPriority() const;

        void SetEnableInternalPullup(bool enable);
        bool GetEnableInternalPullup() const;

        bool Open();
        void Close();
    private:
        i2c_master_bus_config_t m_Config;
    };
}

#endif
