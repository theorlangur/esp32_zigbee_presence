#include "aht21.hpp"
#include "functional/functional.hpp"
#include <thread>
#include <cmath>

const char* AHT21::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::Init_GetStatus: return "Init_GetStatus";
        case ErrorCode::Init_Calibration: return "Init_Calibration";
        case ErrorCode::Measure_IssueComand: return "Measure_IssueComand";
        case ErrorCode::Measure_ReadResponse: return "Measure_ReadResponse";
        case ErrorCode::Measure_Busy: return "Measure_Busy";
    }
    return "unknown";
}

AHT21::AHT21(i2c::I2CBusMaster &bus):
    m_Device(bus.Add(kAddress).value())
{
}

void AHT21::ResetReg(uint8_t reg)
{
    m_Device.WriteReg16(reg, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    uint8_t values[3];
    m_Device.Recv(values, sizeof(values));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    values[0] = reg | 0xb0;
    m_Device.Send(values, sizeof(values));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

AHT21::ExpectedResult AHT21::Init()
{
    using namespace functional;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return m_Device.ReadReg8(0x71)
                   | AdaptError("AHT21::Init", ErrorCode::Init_GetStatus)
                   | and_then([&](uint8_t status)->ExpectedResult{
                        if (!(status & 0x04))
                        {
                            //not calibrated
                            return m_Device.WriteReg16(0xbe, 0x0800)
                                       | AdaptError("AHT21::Init", ErrorCode::Init_Calibration)
                                       | and_then([&]{
                                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                            return m_Device.ReadReg8(0x71)
                                                   | AdaptError("AHT21::Init 2", ErrorCode::Init_GetStatus);
                                       })
                                       | and_then([&](uint8_t status)->ExpectedResult{
                                            if ((status & 0x18) != 0x18)
                                            {
                                                ResetReg(0x1b);
                                                ResetReg(0x1c);
                                                ResetReg(0x1e);
                                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                            }
                                            return std::ref(*this);
                                       });
                        }
                        return std::ref(*this);
                    });
}

AHT21::ExpectedValue<AHT21::Measurements> AHT21::UpdateMeasurements()
{
    using namespace functional;
    using R = ExpectedValue<Measurements>;
    return m_Device.WriteReg16(0xAC, 0x3300)
        | AdaptError("AHT21::UpdateMeasurements", ErrorCode::Measure_IssueComand)
        | and_then([]{ std::this_thread::sleep_for(std::chrono::milliseconds(80)); })
        | retry_on_fail(
        /*attempts=  */ 3, 
        /*Target func*/ [&]()->R{
                            uint8_t data[7];
                            return m_Device.Recv(data, sizeof(data))
                                    | AdaptError("AHT21::UpdateMeasurements", ErrorCode::Measure_IssueComand)
                                    | and_then([&]()->R{
                                            StatusReg *pStatus = reinterpret_cast<StatusReg *>(&data[0]);
                                            if (!pStatus->bits.busy)
                                                return std::unexpected(Err{{}, "AHT21::UpdateMeasurements", ErrorCode::Measure_Busy});

                                            ConvertHumidity(data);
                                            ConvertTemperature(data);
                                            return RetValue{*this, *GetLastMeasurements()};
                                      });
                        },
        /*Error handl*/ []{ std::this_thread::sleep_for(std::chrono::milliseconds(10)); return true; }
        );
}

void AHT21::ConvertTemperature(uint8_t *data)
{
    uint32_t temperature   = data[3] & 0x0F;                //20-bit raw temperature data
    temperature <<= 8;
    temperature  |= data[4];
    temperature <<= 8;
    temperature  |= data[5];
    float _t = ((float)temperature / 0x100000) * 200 - 50;
    m_Temperature.store(_t, std::memory_order_relaxed);
}

void AHT21::ConvertHumidity(uint8_t *data)
{
    uint32_t humidity   = data[1];                          //20-bit raw humidity data
    humidity <<= 8;
    humidity  |= data[2];
    humidity <<= 4;
    humidity  |= data[3] >> 4;

    if (humidity > 0x100000) {humidity = 0x100000;}             //check if RH>100, no need to check for RH<0 since "humidity" is "uint"

    float _h = ((float)humidity / 0x100000) * 100;
    m_Humidity.store(_h, std::memory_order_relaxed);
}

std::optional<AHT21::Measurements> AHT21::GetLastMeasurements() const
{
    Measurements r;
    r.temperature = m_Temperature.load(std::memory_order_relaxed);
    r.humidity = m_Humidity.load(std::memory_order_relaxed);
    if (std::isnan(r.temperature) || std::isnan(r.humidity))
        return std::nullopt;
    return r;
}
