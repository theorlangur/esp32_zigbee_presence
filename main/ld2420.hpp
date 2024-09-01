#ifndef LD2420_H_
#define LD2420_H_

#include "uart.hpp"

class LD2420: protected uart::Channel
{
public:
    enum class ErrorCode: uint8_t
    {
        Ok,
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
    struct RetValue
    {
        Ref d;
        V v;
    };
    template<class V>
    using ExpectedValue = std::expected<RetValue<V>, Err>;

    LD2420(uart::Port p, int txPin, int rxPin, int baud_rate = 115200);
private:
};

#endif
