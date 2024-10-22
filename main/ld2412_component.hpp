#ifndef LD2412_COMPONENT_H_
#define LD2412_COMPONENT_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <thread>
#include "generic_function.hpp"
#include "ld2412.hpp"

namespace ld2412
{
    class Component
    {
        static constexpr const uint16_t kDistanceReportChangeThreshold = 10;//10cm
        static constexpr const uint16_t kEnergyReportChangeThreshold = 10;//10
        struct QueueMsg;
    public:
        enum class ExtendedState: uint8_t
        {
            Normal,
            RunningDynamicBackgroundAnalysis,
            RunningCalibration
        };
        using MovementCallback = GenericCallback<void(bool detected, LD2412::PresenceResult const& p, ExtendedState exState)>;
        using ConfigUpdateCallback = GenericCallback<void()>;
        struct EnergyMinMax
        {
            uint16_t min;
            uint16_t max;
        };
        struct EnergyReading
        {
            EnergyMinMax move;
            EnergyMinMax still;
        };

        ~Component();

        struct setup_args_t{
            int txPin;
            int rxPin;
            uart::Port port = uart::Port::Port1;
            int presencePin = -1;
        };

        bool Setup(setup_args_t const& args);

        void ChangeMode(LD2412::SystemMode m);
        void ChangeTimeout(uint16_t to);
        void ChangeMinDistance(uint16_t d);
        void ChangeMaxDistance(uint16_t d);
        void ChangeMoveSensitivity(const uint8_t (&sensitivity)[14]);
        void ChangeStillSensitivity(const uint8_t (&sensitivity)[14]);

        void StartCalibration();
        void StopCalibration();
        void ResetEnergyStatistics();

        void Restart();
        void FactoryReset();
        void RunDynamicBackgroundAnalysis();

        LD2412::SystemMode GetMode() const;

        int GetMinDistance() const;
        uint8_t GetMinDistanceRaw() const;

        int GetMaxDistance() const;
        uint8_t GetMaxDistanceRaw() const;

        uint8_t GetMoveThreshold(uint8_t gate) const;
        uint8_t GetStillThreshold(uint8_t gate) const;
        uint8_t GetMeasuredMoveEnergy(uint8_t gate) const;
        uint8_t GetMeasuredStillEnergy(uint8_t gate) const;

        uint16_t GetTimeout() const;
                                                         //
        void SetCallbackOnMovement(MovementCallback cb) { m_MovementCallback = std::move(cb); }
        void SetCallbackOnConfigUpdate(ConfigUpdateCallback cb) { m_ConfigUpdateCallback = std::move(cb); }
    private:
        void ConfigurePresenceIsr();
        void HandleMessage(QueueMsg &msg);

        static void presence_pin_isr(void *param);
        static void fast_loop(Component *pC);
        static void manage_loop(Component *pC);

        bool m_Setup = false;
        LD2412 m_Sensor;
        int m_PresencePin = -1;

        MovementCallback m_MovementCallback;
        ConfigUpdateCallback m_ConfigUpdateCallback;

        QueueHandle_t m_FastQueue = 0;
        std::atomic<QueueHandle_t> m_ManagingQueue{0};

        //std::jthread m_FastTask;
        //std::jthread m_ManagingTask;

        EnergyReading m_MeasuredMinMax[14];

        bool m_CalibrationStarted = false;
        bool m_DynamicBackgroundAnalysis = false;
        LD2412::SystemMode m_ModeBeforeCalibration;
    };
}

#endif
