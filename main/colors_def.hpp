#ifndef COLORS_DEF_HPP_
#define COLORS_DEF_HPP_

#include "periph/board_led.hpp"

namespace colors
{
    /**********************************************************************/
    /* Colors and patterns                                                */
    /**********************************************************************/
    static constexpr led::Color kColorInfo{255, 128, 0};
    static constexpr led::Color kColorError{255, 0, 0};
    static constexpr led::Color kColorError2{255, 0, 255};
    static constexpr led::Color kColorSpecial{0, 255, 255};
    static constexpr led::Color kColorWhite{255, 255, 255};
    static constexpr led::Color kColorBlue{0, 0, 255};

    static constexpr uint32_t kBlinkPatternFactoryReset = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternZStackError = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternSteeringError = 0x0000F00F;
    static constexpr uint32_t kBlinkPatternCmdError = 0b00000111000111000111000111000111;//5 pulses

    static constexpr led::Color kColorSteering = kColorInfo;
    static constexpr led::Color kSteeringError = kColorError;
    static constexpr led::Color kSteering = kColorInfo;
    static constexpr led::Color kZStackError = kColorError2;
    static constexpr led::Color kLD2412ConfigError = kColorError;
    static constexpr led::Color kCmdError = kColorWhite;
}

#endif
