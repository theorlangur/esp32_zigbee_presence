#ifndef LD2420_H_
#define LD2420_H_

#include "uart.hpp"

class LD2420: protected uart::Channel
{
public:
    enum class ErrorCode: uint8_t
    {
        Ok,
        Init,
        SendFrame,
        SendFrame_Incomplete,
    };
    static const char* err_to_str(ErrorCode e);

    using Ref = std::reference_wrapper<LD2420>;
    struct Err
    {
        ::Err uartErr;
        const char *pLocation;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    template<typename V>
    using RetVal = RetValT<Ref, V>;

    template<class V>
    using ExpectedValue = std::expected<RetVal<V>, Err>;

    LD2420(uart::Port p, int baud_rate = 115200);

    ExpectedResult Init(int txPin, int rxPin);
private:
    ExpectedResult SendFrame(const uint8_t *pData, uint16_t len);
};

#endif
