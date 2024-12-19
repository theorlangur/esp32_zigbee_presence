#ifndef ZB_ALARM_H_
#define ZB_ALARM_H_
#include "esp_zigbee_core.h"
#include <cstdint>
#include "generic_helpers.hpp"

namespace zb
{
    struct ZbAlarm
    {
        static constexpr uint8_t kCounterOfDeathInactive = 0xff;
        static constexpr uint8_t kCounterOfDeathValue = 6;
        static constexpr uint8_t kLowOnHandlesThreshold = 28;

        static bool g_RunningOutOfHandles;
        static uint8_t g_CounterOfDeath;

        struct TimerList
        {
            static constexpr uint8_t kFull = 33;
            static constexpr uint8_t kMaxSize = kFull - 1;

            using handle_t = uint8_t;
            static constexpr handle_t kInvalidHandle = kFull;

            struct TimerEntry
            {
                ZbAlarm *pAlarm;
                esp_zb_user_callback_t cb;
                void *param;
            };

            struct HandleEntry
            {
                TimerEntry &entry;
                handle_t h;
            };

            std::optional<HandleEntry> Allocate()
            {
                LockGuard l(&g_AlarmLock);
                if (head != kFull)
                {
                    auto r = head;
                    head = entries[head].nextFree;
                    return HandleEntry{entries[r].entry, r};
                }
                return std::nullopt;
            }

            void Free(handle_t h)
            {
                LockGuard l(&g_AlarmLock);
                entries[h].nextFree = head;
                head = h;
            }

            auto& GetEntry(handle_t h) { return entries[h].entry; }

            static constexpr TimerList Create()
            {
                TimerList r;
                r.head = 0;
                for(size_t i = 0; i < kMaxSize; ++i)
                {
                    auto &e = r.entries[i];
                    e.nextFree = i + 1;
                }
                r.entries[kMaxSize - 1].nextFree = kFull;
                return r;
            }

        private:

            union ListEntry
            {
                uint8_t nextFree;
                TimerEntry entry;
            };
            handle_t head;
            ListEntry entries[kMaxSize];

            static SpinLock g_AlarmLock;
        };
        constinit static TimerList g_TimerList;

        const char *pDescr = nullptr;
        TimerList::handle_t h = TimerList::kInvalidHandle;

        bool IsRunning() const { return h != TimerList::kInvalidHandle; }

        void Cancel()
        {
            if (IsRunning())
            {
                //REMOVE_ME:a workaround for esp zigbee sdk memory leak:
                esp_zb_scheduler_alarm_cancel(on_timer, h);
                g_TimerList.Free(h);
                h = TimerList::kInvalidHandle;
            }
        }

        esp_err_t Setup(esp_zb_user_callback_t cb, void *param, uint32_t time)
        {
            Cancel();
            if (auto r = g_TimerList.Allocate())
            {
                auto &v = *r;
                h = v.h;
                v.entry.cb = cb;
                v.entry.param = param;
                v.entry.pAlarm = this;

                if (h >= kLowOnHandlesThreshold)
                {
                    FMT_PRINT("Got back handle {:x} >= threshold of {:x}. Prepare to die\n", h, kLowOnHandlesThreshold);
                    //soon we'll be out of handles - let's restart at a convenient moment
                    g_RunningOutOfHandles = true;
                }
                esp_zb_scheduler_alarm(on_timer, h, time);
                return ESP_OK;
            }else
                return ESP_ERR_NO_MEM;
        }

        static void on_timer(uint8_t param)
        {
            auto e = g_TimerList.GetEntry(param);
            g_TimerList.Free(e.pAlarm->h);
            e.pAlarm->h = TimerList::kInvalidHandle;
            e.cb(e.param);
        }

        static void deactivate_counter_of_death()
        {
            FMT_PRINT("Low on handles: Counter of death deactivated\n");
            g_CounterOfDeath = kCounterOfDeathInactive;
        }

        static void check_counter_of_death()
        {
            if (g_RunningOutOfHandles)
            {
                g_CounterOfDeath = kCounterOfDeathValue;
                FMT_PRINT("Low on handles: Counter of death activated: {} iterations left\n", g_CounterOfDeath);
            }
        }

        static void check_death_count()
        {
            if (g_RunningOutOfHandles && g_CounterOfDeath != kCounterOfDeathInactive)
            {
                if (!(--g_CounterOfDeath))
                {
                    //boom
                    FMT_PRINT("Low on handles: time to die and reborn\n");
                    esp_restart();
                    return;
                }
                FMT_PRINT("Low on handles: tick-tock: {} iterations left\n", ZbAlarm::g_CounterOfDeath);
            }
        }
    };

    inline bool ZbAlarm::g_RunningOutOfHandles = false;
    inline uint8_t ZbAlarm::g_CounterOfDeath = kCounterOfDeathInactive;
    inline SpinLock ZbAlarm::TimerList::g_AlarmLock;
    inline constinit ZbAlarm::TimerList ZbAlarm::g_TimerList = ZbAlarm::TimerList::Create();
}
#endif
