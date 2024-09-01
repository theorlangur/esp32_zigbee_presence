#include "ld2420.hpp"

LD2420::LD2420(uart::Port p, int txPin, int rxPin, int baud_rate):
    uart::Channel(p, baud_rate)
{
}
