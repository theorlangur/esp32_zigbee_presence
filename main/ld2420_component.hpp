#ifndef LD2420_COMPONENT_H_
#define LD2420_COMPONENT_H_

#include <cstdint>

namespace ld2420
{
     constexpr const int kLD2420_PresencePin = 13;
     constexpr const float kDistanceReportChangeThreshold = 0.1f;//10cm

    enum class Mode
    {
        Simple,
        Energy
    };

    struct QueueMsg
    {
        enum class Type: std::size_t 
        {
            //commands
            Stop,
            Restart,
            StartCalibrate,
            StopCalibrate,
            //report
            Presence,
            PresenceIntr,
            Distance,
            PresenceAndDistance,
            //config
            SetTimeout,
            SetMinDistance,
            SetMaxDistance,
            SetMode,
        };

        Type m_Type;
        union
        {
            bool m_Presence;
            float m_Distance;
            struct{
                bool m_Presence;
                float m_Distance;
            }m_PresenceAndDistance;
            uint32_t m_Timeout;
            Mode m_Mode;
        };
    };

    bool setup_ld2420();
}

#endif
