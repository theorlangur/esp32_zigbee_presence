#ifndef LD2412_H_
#define LD2412_H_

#include "uart.hpp"
#include <span>
#include "functional/functional.hpp"
#include "uart_functional.hpp"

class LD2412: protected uart::Channel
{
public:
    static const constexpr duration_ms_t kRestartTimeout{2000};
    static const constexpr bool kDebugFrame = false;
    static const constexpr bool kDebugCommands = false;
    enum class ErrorCode: uint8_t
    {
        Ok,
        Init,
        SendFrame,
        SendFrame_Incomplete,

        SendCommand_InvalidResponse,
        SendCommand_FailedWrite,
        SendCommand_FailedRead,
        SendCommand_WrongFormat,
        SendCommand_Failed,
        SendCommand_InsufficientSpace,

        RecvFrame_Malformed,
        RecvFrame_Incomplete,

        SimpleData_Malformed,
        EnergyData_Malformed,

        SimpleData_Failure,
        EnergyData_Failure,

        FillBuffer_NoSpace,
        FillBuffer_ReadFailure,

        MatchError,
        RestartFailed,
    };
    static const char* err_to_str(ErrorCode e);

    enum class SystemMode: uint8_t
    {
        Simple = 0x02,
        Energy = 0x01,
    };

    enum class TargetState: uint8_t
    {
        Clear,
        Move,
        Still,
        MoveAndStill
    };

    enum class Drain
    {
        No,
        Try,
        Only,
    };

    using Ref = std::reference_wrapper<LD2412>;
    struct Err
    {
        ::Err uartErr;
        const char *pLocation;
        ErrorCode code;
    };

    using ExpectedResult = std::expected<Ref, Err>;
    struct CmdErr
    {
        Err e;
        uint16_t returnCode;
    };

    template<typename V>
    using RetVal = RetValT<Ref, V>;

    template<class V>
    using ExpectedValue = std::expected<RetVal<V>, Err>;


    /**********************************************************************/
    /* PresenceResult                                                     */
    /**********************************************************************/
#pragma pack(push,1)
    struct PresenceResult
    {
        TargetState m_State = TargetState::Clear;
        uint16_t m_MoveDistance = 0;//cm
        uint8_t m_MoveEnergy = 0;
        uint16_t m_StillDistance = 0;//cm
        uint8_t m_StillEnergy = 0;
    };
#pragma pack(pop)


    /**********************************************************************/
    /* ConfigBlock                                                        */
    /**********************************************************************/
    struct ConfigBlock
    {
        LD2412 &d;

        ConfigBlock(LD2412 &d): d(d) {}
        ConfigBlock(ConfigBlock const&) = delete;
        ConfigBlock(ConfigBlock &&) = delete;
        ConfigBlock& operator=(ConfigBlock const &) = delete;
        ConfigBlock& operator=(ConfigBlock &&) = delete;


        ConfigBlock& SetSystemMode(SystemMode mode);

        ConfigBlock& SetMinDistance(int dist);
        ConfigBlock& SetMinDistanceRaw(uint32_t dist);

        ConfigBlock& SetMaxDistance(int dist);
        ConfigBlock& SetMaxDistanceRaw(uint32_t dist);

        ConfigBlock& SetTimeout(uint32_t t);

        ConfigBlock& SetMoveThreshold(uint8_t gate, uint16_t energy);
        ConfigBlock& SetStillThreshold(uint8_t gate, uint16_t energy);

        ExpectedResult EndChange();
    private:
        SystemMode m_NewMode;
        uint32_t m_NewMinDistance;
        uint32_t m_NewMaxDistance;
        uint32_t m_NewTimeout;
        struct Gate
        {
            uint16_t still;
            uint16_t move;
        };
        Gate m_NewGates[16];

        union{
            struct
            {
                uint32_t Gate0MoveThreshold  : 1;
                uint32_t Gate0StillThreshold : 1;
                uint32_t Gate1MoveThreshold  : 1;
                uint32_t Gate1StillThreshold : 1;
                uint32_t Gate2MoveThreshold  : 1;
                uint32_t Gate2StillThreshold : 1;
                uint32_t Gate3MoveThreshold  : 1;
                uint32_t Gate3StillThreshold : 1;
                uint32_t Gate4MoveThreshold  : 1;
                uint32_t Gate4StillThreshold : 1;
                uint32_t Gate5MoveThreshold  : 1;
                uint32_t Gate5StillThreshold : 1;
                uint32_t Gate6MoveThreshold  : 1;
                uint32_t Gate6StillThreshold : 1;
                uint32_t Gate7MoveThreshold  : 1;
                uint32_t Gate7StillThreshold : 1;
                uint32_t Gate8MoveThreshold  : 1;
                uint32_t Gate8StillThreshold : 1;
                uint32_t Gate9MoveThreshold  : 1;
                uint32_t Gate9StillThreshold : 1;
                uint32_t Gate10MoveThreshold : 1;
                uint32_t Gate10StillThreshold: 1;
                uint32_t Gate11MoveThreshold : 1;
                uint32_t Gate11StillThreshold: 1;
                uint32_t Gate12MoveThreshold : 1;
                uint32_t Gate12StillThreshold: 1;
                uint32_t Gate13MoveThreshold : 1;
                uint32_t Gate13StillThreshold: 1;
                uint32_t Gate14MoveThreshold : 1;
                uint32_t Gate14StillThreshold: 1;
                uint32_t Gate15MoveThreshold : 1;
                uint32_t Gate15StillThreshold: 1;

                uint32_t Mode                : 1;
                uint32_t MinDistance         : 1;
                uint32_t MaxDistance         : 1;
                uint32_t Timeout             : 1;
                uint32_t Unused              : 28;
            }m_Changed;
            
            struct{
                uint32_t m_GateChanges = 0;
                uint32_t m_MiscChanges = 0;
            };
        };
    };

    LD2412(uart::Port p = uart::Port::Port1, int baud_rate = 115200);

    void SetPort(uart::Port p);
    uart::Port GetPort() const;

    ExpectedResult Init(int txPin, int rxPin);

    SystemMode GetSystemMode() const { return m_Mode; }

    int GetMinDistance() const { return m_MinDistance * 75 / 100; }
    uint32_t GetMinDistanceRaw() const { return m_MinDistance; }

    int GetMaxDistance() const { return m_MaxDistance * 75 / 100; }
    uint32_t GetMaxDistanceRaw() const { return m_MaxDistance; }

    auto GetMoveThreshold(uint8_t gate) const { return uint16_t(m_Gates[gate].m_MoveThreshold); }
    auto GetStillThreshold(uint8_t gate) const { return uint16_t(m_Gates[gate].m_StillThreshold); }
    auto GetMeasuredEnergy(uint8_t gate) const { return m_Gates[gate].m_Energy; }

    uint32_t GetTimeout() const { return m_Timeout; }//seconds

    ConfigBlock ChangeConfiguration() { return {*this}; }

    ExpectedResult UpdateMinMaxTimeoutConfig();
    ExpectedResult UpdateSystemMode();

    ExpectedResult ReloadConfig();

    std::string_view GetVersion() const;

    ExpectedResult Restart();

    ExpectedResult FactoryReset();

    PresenceResult GetPresence() const { return m_Presence; }

    ExpectedResult TryReadFrame(int attempts = 3, bool flush = false, Drain drain = Drain::No);

    using Channel::SetEventCallback;
    using Channel::Flush;
    using Channel::GetReadyToReadDataLen;
private:
    enum class Cmd: uint16_t
    {
        ReadVer = 0x0000,
        WriteBaseParams = 0x0002,
        ReadBaseParams = 0x0012,
        EnterEngMode = 0x0062,
        LeaveEngMode = 0x0063,
        WriteADB = 0x0007,
        ReadADB = 0x0008,
        WriteSys = 0x0012,
        ReadSys = 0x0013,

        Restart = 0x0068,

        OpenCmd = 0x00ff,
        CloseCmd = 0x00fe,
    };

    friend Cmd operator|(Cmd r, uint16_t v)
    {
        return Cmd(uint16_t(r) | v);
    }

    struct OpenCmdModeResponse
    {
        uint16_t protocol_version;
        uint16_t buffer_size;
    };

#pragma pack(push,1)
    template<class ParamT>
    struct SetParam
    {
        static_assert(sizeof(ParamT) == 2, "Must be 2 bytes");
        ParamT param;
        uint32_t value;
    };
#pragma pack(pop)

    using OpenCmdModeRetVal = RetValT<Ref, OpenCmdModeResponse>;
    using ExpectedOpenCmdModeResult = std::expected<OpenCmdModeRetVal, CmdErr>;
    using ExpectedGenericCmdResult = std::expected<Ref, CmdErr>;

    constexpr static uint8_t kFrameHeader[] = {0xFD, 0xFC, 0xFB, 0xFA};
    constexpr static uint8_t kFrameFooter[] = {0x04, 0x03, 0x02, 0x01};

    auto AdaptToResult(const char *pLocation, ErrorCode ec)
    {
        return functional::adapt_to<ExpectedResult>(
                      [this](auto &c){ return std::ref(*this); }
                    , [&,pLocation,ec](::Err e){ return Err{e, pLocation, ec}; }
                );
    }

    auto AdaptToCmdResult()
    {
        return functional::adapt_to<ExpectedGenericCmdResult>(
                      [&](auto &c){ return std::ref(*this); }
                    , [&](Err e){ return CmdErr{e, 0}; }
                );
    }

    static auto AdaptError(const char *pLocation, ErrorCode ec) { return functional::transform_error([pLocation,ec](::Err e){ return Err{e, pLocation, ec}; }); }

    template<class...T>
    ExpectedResult SendFrameV2(T&&... args)
    {
        using namespace functional;
        return Send(kFrameHeader, sizeof(kFrameHeader))
            | and_then([&]{ 
                    uint16_t len = (sizeof(args) + ...);
                    return Send((uint8_t const*)&len, sizeof(len)); 
            })
            | uart::write_any(*this, std::forward<T>(args)...)
            | and_then([&]{ return Send(kFrameFooter, sizeof(kFrameFooter)); })
            | AdaptToResult("SendFrameV2", ErrorCode::SendFrame);
    }

    template<class...T>
    ExpectedResult RecvFrameV2(T&&... args)
    {
        using namespace functional;
        uint16_t len;
        constexpr const size_t arg_size = (uart::uart_sizeof<std::remove_cvref_t<T>>() + ...);
        return 
                start_sequence()
                | uart::match_bytes(*this, kFrameHeader)
                | uart::read_into(*this, len)
                | and_then([&]()->Channel::ExpectedResult{
                        if (arg_size > len)
                            return std::unexpected(::Err{"RecvFrameV2 len invalid", ESP_OK}); 
                        return std::ref((Channel&)*this);
                  })
                | uart::read_any_limited(*this, len, std::forward<T>(args)...)
                | if_then(
                  /*if*/   [&]{ return len; }
                  /*then*/,[&]{ return start_sequence() | uart::skip_bytes(*this, len); })
                | uart::match_bytes(*this, kFrameFooter)
                | AdaptToResult("RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    }

    template<class... ToSend> static auto to_send(ToSend&&...args) { return std::forward_as_tuple(std::forward<ToSend>(args)...); }
    template<class... ToRecv> static auto to_recv(ToRecv&&...args) { return std::forward_as_tuple(std::forward<ToRecv>(args)...); }

    template<class CmdT, class... ToSend, class... ToRecv>
    ExpectedGenericCmdResult SendCommandV2(CmdT cmd, std::tuple<ToSend&...> sendArgs, std::tuple<ToRecv&...> recvArgs)
    {
        using namespace functional;
        static_assert(sizeof(CmdT) == 2, "must be 2 bytes");
        SetDefaultWait(duration_ms_t(150));
        uint16_t status;
        auto SendFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){
            return SendFrameV2(cmd, std::get<idx>(sendArgs)...);
        };
        auto RecvFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){ 
            return RecvFrameV2(
                uart::match_t{uint16_t(cmd | 0x100)}, 
                status, 
                uart::callback_t{[&]()->Channel::ExpectedResult{
                    if (status != 0)
                        return std::unexpected(::Err{"SendCommandV2 status", status});
                    return std::ref((Channel&)*this);
                }},
                std::get<idx>(recvArgs)...);
        };
        return Flush() 
                | AdaptError("SendCommandV2", ErrorCode::SendCommand_Failed)
                | and_then([&]{ return SendFrameExpandArgs(std::make_index_sequence<sizeof...(ToSend)>()); })
                | and_then([&]{ return WaitAllSent() | AdaptError("SendCommandV2", ErrorCode::SendCommand_Failed); })
                | and_then([&]{ return RecvFrameExpandArgs(std::make_index_sequence<sizeof...(ToRecv)>()); })
                | AdaptToCmdResult();
    }

    ExpectedOpenCmdModeResult OpenCommandMode();
    ExpectedGenericCmdResult CloseCommandMode();

    ExpectedGenericCmdResult SetSystemModeInternal(SystemMode mode);
    ExpectedGenericCmdResult UpdateVersion();
    //ExpectedGenericCmdResult UpdateSystemMode();
    //ExpectedGenericCmdResult UpdateMinMaxTimeout();
    //ExpectedGenericCmdResult UpdateGate(uint8_t gate);

    ExpectedResult ReadFrame();
    //data
    char m_Version[10];
    SystemMode m_Mode = SystemMode::Simple;
    OpenCmdModeResponse m_ProtoInfo{0, 0};
    uint8_t m_MinDistance = 1;//*75cm to get the distance
    uint8_t m_MaxDistance = 12;//*75cm to get the distance
    uint16_t m_Timeout = 30;
    struct Gate
    {
        uint8_t m_StillThreshold; //config
        uint8_t m_MoveThreshold; //config
        uint16_t m_Energy; //output
    };
    Gate m_Gates[16];

    PresenceResult m_Presence;
    bool m_dbg = false;
};

template<>
struct tools::formatter_t<LD2412::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "Err\\{uart=[{}] at {} with {} }", e.uartErr, e.pLocation, LD2412::err_to_str(e.code));
    }
};

template<>
struct tools::formatter_t<LD2412::CmdErr>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::CmdErr const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "CmdErr\\{{Err=[{}]; return={} }", e.e, e.returnCode);
    }
};

template<>
struct tools::formatter_t<LD2412::TargetState>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::TargetState const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case LD2412::TargetState::Clear: pStr = "Clear"; break;
            case LD2412::TargetState::Still: pStr = "Still"; break;
            case LD2412::TargetState::Move: pStr = "Move"; break;
            case LD2412::TargetState::MoveAndStill: pStr = "MoveAndStill"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<LD2412::SystemMode>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::SystemMode const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case LD2412::SystemMode::Simple: pStr = "Simple"; break;
            case LD2412::SystemMode::Energy: pStr = "Energy"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<LD2412::PresenceResult>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::PresenceResult const& p)
    {
        return tools::format_to(std::forward<Dest>(dst), "[{}; move(dist={}cm; energy={}); still(dist={}cm; energy={})]"
                , p.m_State
                , p.m_MoveDistance, p.m_MoveEnergy
                , p.m_StillDistance, p.m_StillEnergy
            );
    }
};

#endif
