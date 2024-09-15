#ifndef AHT21_H_
#define AHT21_H_

#include "i2c.hpp"
#include <atomic>
#include "functional/functional.hpp"

class AHT21
{
public:
    static constexpr const uint8_t kAddress = 0x38;

    enum class ErrorCode: uint8_t
    {
        Ok,
        Init_GetStatus,
        Init_Calibration,
        Measure_IssueComand,
        Measure_ReadResponse,
        Measure_Busy,
    };
    static const char* err_to_str(ErrorCode e);

    using Ref = std::reference_wrapper<AHT21>;
    struct Err
    {
        ::Err i2cErr;
        const char *pLocation;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    struct Measurements
    {
        float temperature;
        float humidity;
    };
    template<typename V>
    using RetValue = RetValT<Ref, V>;
    template<class V>
    using ExpectedValue = std::expected<RetValue<V>, Err>;

    union StatusReg
    {
        uint8_t reg;
        struct{
            uint8_t reserved0: 3;
            uint8_t calibrated: 1;
            uint8_t reserved1: 3;
            uint8_t busy: 1;
        }bits;
    };

    AHT21(i2c::I2CBusMaster &bus);

    ExpectedResult Init();

    ExpectedValue<Measurements> UpdateMeasurements();

    std::optional<Measurements> GetLastMeasurements() const;
private:
    auto AdaptError(const char *pL, ErrorCode ec) { return functional::transform_error([pL,ec](auto &e){ return Err{e, pL, ec}; }); }
    auto AdaptI2C(const char *pL, ErrorCode ec) 
    { 
        return functional::adapt_to<ExpectedResult>(
                [&](auto &&v){ return std::ref(*this); }
                , [pL,ec](auto &e){ return Err{e, pL, ec}; }); 
    }
    void ResetReg(uint8_t reg);
    void ConvertTemperature(uint8_t *pData);
    void ConvertHumidity(uint8_t *pData);
    i2c::I2CDevice m_Device;

    std::atomic<float> m_Temperature{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> m_Humidity{std::numeric_limits<float>::quiet_NaN()};
};

#endif
