#ifndef LD2420_COMPONENT_H_
#define LD2420_COMPONENT_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <thread>
#include "ld2420.hpp"

namespace ld2420
{

    enum class Mode
    {
        Simple,
        Energy
    };

    bool setup_ld2420();

    class Component
    {
        static constexpr const float kDistanceReportChangeThreshold = 0.1f;//10cm
        struct QueueMsg;
    public:
        ~Component();

        struct setup_args_t{
            int txPin;
            int rxPin;
            uart::Port port = uart::Port::Port1;
            int presencePin = -1;
        };

        bool Setup(setup_args_t const& args);
    private:
        void ConfigurePresenceIsr();
        void HandleMessage(QueueMsg &msg);

        static void presence_pin_isr(void *param);
        static void fast_loop(Component &c);
        static void manage_loop(Component &c);

        bool m_Setup = false;
        LD2420 m_Sensor;
        int m_PresencePin = -1;

        QueueHandle_t m_FastQueue;
        QueueHandle_t m_ManagingQueue;

        std::jthread m_FastTask;
        std::jthread m_ManagingTask;
    };
}

#endif