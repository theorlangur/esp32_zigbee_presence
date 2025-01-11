#ifndef LIB_THREAD_HELPER_HPP_
#define LIB_THREAD_HELPER_HPP_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mutex>
#include <condition_variable>

namespace thread
{
    inline int64_t get_us()
    {
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    }
    static constexpr UBaseType_t kPrioIDLE = tskIDLE_PRIORITY;
    static constexpr UBaseType_t kPrioDefault = 5;
    static constexpr UBaseType_t kPrioElevated = 6;
    static constexpr UBaseType_t kPrioHigh = 7;
    struct task_config_t
    {
        const char *pName = "";
        uint32_t stackSize = 2048;
        UBaseType_t prio = kPrioDefault;
    };

    struct args_base_t
    {
        ~args_base_t() = default;

        bool done{false};
        std::mutex done_mtx;
        std::condition_variable cv;
    };

    struct TaskBase
    {
        TaskHandle_t h = nullptr;
        std::unique_ptr<args_base_t> args;

        TaskBase() = default;
        TaskBase(TaskBase const&t) = delete;
        TaskBase(TaskBase &&t): h(t.h), args(std::move(t.args)){ t.h = nullptr; }

        ~TaskBase(){ if (h){ join(); vTaskDelete(h); } }
        
        TaskBase& operator=(TaskBase const&t) = delete;
        TaskBase& operator=(TaskBase &&t)
        {
            h = t.h;
            args = std::move(t.args);
            t.h = nullptr; 
            return *this;
        }
        
        void join()
        {
            if (h && args.get())
            {
                std::unique_lock l(args->done_mtx);
                args->cv.wait(l, [&]{ return args->done; });
                args.reset();
            }
        }

        void detach()
        {
            h = nullptr;
            (void)args.release();
        }
    };

    template<class F, class...Args>
    TaskBase start_task(task_config_t cfg, F f, Args&&... args)
    {
        using args_t = std::tuple<Args...>;
        struct args_with_f_t: args_base_t
        {
            args_with_f_t(F &&f, args_t &&args): f(std::move(f)), args(std::move(args)) {}
            F f;
            args_t args;
        };
        static auto _func = +[](void *_pParams)
        {
            args_with_f_t *pParams = (args_with_f_t *)_pParams;
            [&]<size_t...idx>(std::index_sequence<idx...>){ 
                (pParams->f)(std::get<idx>(pParams->args)...);
                vTaskDelete(nullptr);
            }(std::make_index_sequence<sizeof...(Args)>());
        };

        TaskBase r;
        r.args.reset(new args_with_f_t{std::move(f), std::make_tuple(std::forward<Args>(args)...) });
        xTaskCreate(_func, cfg.pName, cfg.stackSize, r.args.get(), cfg.prio, &r.h);
        return r;
    }

};

#endif
