#ifndef LD2412_H_
#define LD2412_H_

#include "uart.hpp"
#include <span>
#include "uart_primitives.hpp"

class LD2412: public uart::Channel
{
public:
    static const constexpr duration_ms_t kRestartTimeout{2000};
    static const constexpr duration_ms_t kDefaultWait{350};
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
        BTFailed,
        FactoryResetFailed,
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

    enum class DistanceRes: uint16_t
    {
        _0_75 = 0,
        _0_20 = 1,
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

private:
#pragma pack(push,1)
    struct BaseConfigData
    {
        uint8_t m_MinDistanceGate;
        uint8_t m_MaxDistanceGate;
        uint16_t m_Duration;//seconds
        uint8_t m_OutputPinPolarity;//0 - high on presence; 1 - low on presence
    };
    struct Configuration
    {
        BaseConfigData m_Base;
        uint8_t m_MoveThreshold[14];
        uint8_t m_StillThreshold[14];
    };
#pragma pack(pop)
public:

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
    struct Engeneering
    {
        uint8_t m_MaxMoveGate;
        uint8_t m_MaxStillGate;
        uint8_t m_MoveEnergy[14];
        uint8_t m_StillEnergy[14];
        uint8_t m_Light;
        uint8_t m_Dummy;
    };
    struct Version
    {
        uint8_t m_Minor;
        uint8_t m_Major;
        uint32_t m_Misc;
    };
#pragma pack(pop)


    /**********************************************************************/
    /* ConfigBlock                                                        */
    /**********************************************************************/
    struct ConfigBlock
    {
        LD2412 &d;

        ConfigBlock(LD2412 &d): d(d), m_Configuration(d.m_Configuration) {}
        ConfigBlock(ConfigBlock const&) = delete;
        ConfigBlock(ConfigBlock &&) = delete;
        ConfigBlock& operator=(ConfigBlock const &) = delete;
        ConfigBlock& operator=(ConfigBlock &&) = delete;


        ConfigBlock& SetSystemMode(SystemMode mode);

        ConfigBlock& SetMinDistance(int dist);
        ConfigBlock& SetMinDistanceRaw(uint8_t dist);

        ConfigBlock& SetMaxDistance(int dist);
        ConfigBlock& SetMaxDistanceRaw(uint8_t dist);

        ConfigBlock& SetTimeout(uint16_t t);

        ConfigBlock& SetOutPinPolarity(bool lowOnPresence);

        ConfigBlock& SetMoveThreshold(uint8_t gate, uint8_t energy);
        ConfigBlock& SetStillThreshold(uint8_t gate, uint8_t energy);

        ExpectedResult EndChange();
    private:
        SystemMode m_NewMode;
        Configuration m_Configuration;

        union{
            struct
            {
                uint32_t Mode             : 1;
                uint32_t MinDistance      : 1;
                uint32_t MaxDistance      : 1;
                uint32_t Timeout          : 1;
                uint32_t OutPin           : 1;
                uint32_t MoveThreshold    : 1;
                uint32_t StillThreshold   : 1;
                uint32_t Unused           : 25;
            }m_Changed;
            
            struct{
                uint32_t m_Changes = 0;
            };
        };
    };

    LD2412(uart::Port p = uart::Port::Port1, int baud_rate = 115200);

    void SetPort(uart::Port p);
    uart::Port GetPort() const;

    ExpectedResult Init(int txPin, int rxPin);

    SystemMode GetSystemMode() const { return m_Mode; }

    int GetMinDistance() const { return m_Configuration.m_Base.m_MinDistanceGate * 75 / 100; }
    uint8_t GetMinDistanceRaw() const { return m_Configuration.m_Base.m_MinDistanceGate; }

    int GetMaxDistance() const { return m_Configuration.m_Base.m_MaxDistanceGate * 75 / 100; }
    uint8_t GetMaxDistanceRaw() const { return m_Configuration.m_Base.m_MaxDistanceGate; }

    auto GetMoveThreshold(uint8_t gate) const { return m_Configuration.m_MoveThreshold[gate]; }
    auto GetStillThreshold(uint8_t gate) const { return m_Configuration.m_StillThreshold[gate]; }
    auto const& GetAllMoveThresholds() const { return m_Configuration.m_MoveThreshold; }
    auto const& GetAllStillThresholds() const { return m_Configuration.m_StillThreshold; }
    auto GetMeasuredMoveEnergy(uint8_t gate) const { return m_Engeneering.m_MoveEnergy[gate]; }
    auto GetMeasuredStillEnergy(uint8_t gate) const { return m_Engeneering.m_StillEnergy[gate]; }

    auto GetTimeout() const { return m_Configuration.m_Base.m_Duration; }//seconds
    bool GetOutPinPolarity() const { return m_Configuration.m_Base.m_OutputPinPolarity; }
    auto GetDistanceRes() const { return m_DistanceRes; }

    ConfigBlock ChangeConfiguration() { return {*this}; }

    ExpectedResult UpdateDistanceRes();

    ExpectedResult ReloadConfig();

    auto const& GetVersion() const { return m_Version; }

    auto const& GetBluetoothMAC() const { return m_BluetoothMAC; }
    ExpectedResult SwitchBluetooth(bool on);

    ExpectedResult Restart();
    ExpectedResult FactoryReset();

    PresenceResult GetPresence() const { return m_Presence; }
    const Engeneering& GetEngeneeringData() const { return m_Engeneering; }

    ExpectedResult TryReadFrame(int attempts = 3, bool flush = false, Drain drain = Drain::No);

    using Channel::SetEventCallback;
    using Channel::Flush;
    using Channel::GetReadyToReadDataLen;
private:
    enum class Cmd: uint16_t
    {
        ReadVer = 0x00a0,
        WriteBaseParams = 0x0002,
        ReadBaseParams = 0x0012,

        EnterEngMode = 0x0062,
        LeaveEngMode = 0x0063,

        SetMoveSensitivity = 0x0003,
        GetMoveSensitivity = 0x0013,

        SetStillSensitivity = 0x0004,
        GetStillSensitivity = 0x0014,

        RunDynamicBackgroundAnalysis = 0x000B,
        QuearyDynamicBackgroundAnalysis = 0x001B,

        FactoryReset = 0x00a2,
        Restart = 0x00a3,

        SwitchBluetooth = 0x00a4,
        GetMAC = 0x00a5,

        OpenCmd = 0x00ff,
        CloseCmd = 0x00fe,

        SetDistanceRes = 0x00aa,
        GetDistanceRes = 0x00ab,
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
    constexpr static uint8_t kDataFrameHeader[] = {0xf4, 0xf3, 0xf2, 0xf1};
    constexpr static uint8_t kDataFrameFooter[] = {0xf8, 0xf7, 0xf6, 0xf5};

    template<class E>
    static ExpectedResult to_result(E &&e, const char* pLocation, ErrorCode ec)
    {
        using PureE = std::remove_cvref_t<E>;
        if constexpr(is_expected_type_v<PureE>)
        {
            if constexpr (std::is_same_v<PureE, ExpectedResult>)
                return std::move(e);
            else
                return to_result(e.error(), pLocation, ec);
        }else if constexpr (std::is_same_v<PureE,::Err>)
            return ExpectedResult(std::unexpected(Err{e, pLocation, ec}));
        else if constexpr (std::is_same_v<PureE,Err>)
            return ExpectedResult(std::unexpected(e));
        else if constexpr (std::is_same_v<PureE,CmdErr>)
            return ExpectedResult(std::unexpected(e.e));
        else
        {
            static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
            return ExpectedResult(std::unexpected(Err{}));
        }
    }

    template<class E>
    static ExpectedGenericCmdResult to_cmd_result(E &&e, const char* pLocation, ErrorCode ec)
    {
        using PureE = std::remove_cvref_t<E>;
        if constexpr(is_expected_type_v<PureE>)
        {
            if constexpr (std::is_same_v<PureE, ExpectedGenericCmdResult>)
                return std::move(e);
            else
                return to_cmd_result(e.error(), pLocation, ec);
        }else if constexpr (std::is_same_v<PureE,::Err>)
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{Err{e, pLocation, ec}, 0}));
        else if constexpr (std::is_same_v<PureE,Err>)
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{e, 0}));
        else if constexpr (std::is_same_v<PureE,CmdErr>)
            return ExpectedGenericCmdResult(std::unexpected(e));
        else
        {
            static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{}));
        }
    }

#define TRY_UART_COMM(f, location, ec) \
    if (auto r = f; !r) \
        return to_result(std::move(r), location, ec)

#define TRY_UART_COMM_CMD(f, location, ec) \
    if (auto r = f; !r) \
        return to_cmd_result(std::move(r), location, ec)

    template<class...T>
    ExpectedResult SendFrameV2(T&&... args)
    {
        using namespace uart::primitives;
        //1. header
        TRY_UART_COMM(Send(kFrameHeader, sizeof(kFrameHeader)), "SendFrameV2", ErrorCode::SendFrame);

        //2. length
        {
            uint16_t len = (sizeof(args) + ...);
            TRY_UART_COMM(Send((uint8_t const*)&len, sizeof(len)), "SendFrameV2", ErrorCode::SendFrame);
        }
        //3. data
        TRY_UART_COMM(uart::primitives::write_any(*this, std::forward<T>(args)...), "SendFrameV2", ErrorCode::SendFrame);

        //4. footer
        TRY_UART_COMM(Send(kFrameFooter, sizeof(kFrameFooter)), "SendFrameV2", ErrorCode::SendFrame);

        return std::ref(*this);
    }

    template<class...T>
    ExpectedResult RecvFrameV2(T&&... args)
    {
        constexpr const size_t arg_size = (uart::primitives::uart_sizeof<std::remove_cvref_t<T>>() + ...);
        if (m_dbg) m_Dbg = true;
        ScopeExit resetDbg = [&]{ if (m_dbg) m_Dbg = false; };
        TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameHeader), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (m_dbg) FMT_PRINT("RecvFrameV2: matched header\n"); 
        uint16_t len;
        TRY_UART_COMM(uart::primitives::read_into(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (m_dbg) FMT_PRINT("RecvFrameV2: len: {}\n", len);
        if (arg_size > len)
            return std::unexpected(Err{{}, "RecvFrameV2 len invalid", ErrorCode::RecvFrame_Malformed}); 

        TRY_UART_COMM(uart::primitives::read_any_limited(*this, len, std::forward<T>(args)...), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (len)
        {
            TRY_UART_COMM(uart::primitives::skip_bytes(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        }
        TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameFooter), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        return std::ref(*this);
    }

    template<class... ToSend> static auto to_send(ToSend&&...args) { return std::forward_as_tuple(std::forward<ToSend>(args)...); }
    template<class... ToRecv> static auto to_recv(ToRecv&&...args) { return std::forward_as_tuple(std::forward<ToRecv>(args)...); }

    template<class CmdT, class... ToSend, class... ToRecv>
    ExpectedGenericCmdResult SendCommandV2(CmdT cmd, std::tuple<ToSend&...> sendArgs, std::tuple<ToRecv&...> recvArgs)
    {
        static_assert(sizeof(CmdT) == 2, "must be 2 bytes");
        if (GetDefaultWait() < kDefaultWait)
            SetDefaultWait(kDefaultWait);
        uint16_t status;
        auto SendFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){
            return SendFrameV2(cmd, std::get<idx>(sendArgs)...);
        };
        auto RecvFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){ 
            return RecvFrameV2(
                uart::primitives::match_t{uint16_t(cmd | 0x100)}, 
                status, 
                uart::primitives::callback_t{[&]()->Channel::ExpectedResult{
                    if (m_dbg) FMT_PRINT("Recv frame resp. Status {}\n", status);
                    if (status != 0)
                        return std::unexpected(::Err{"SendCommandV2 status", status});
                    return std::ref((Channel&)*this);
                }},
                std::get<idx>(recvArgs)...);
        };

        TRY_UART_COMM_CMD(Flush(), "SendCommandV2", ErrorCode::SendCommand_Failed);
        if (m_dbg) FMT_PRINT("Sent cmd {}\n", uint16_t(cmd));
        TRY_UART_COMM_CMD(SendFrameExpandArgs(std::make_index_sequence<sizeof...(ToSend)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
        if (m_dbg) FMT_PRINT("Wait all\n");
        TRY_UART_COMM_CMD(WaitAllSent(), "SendCommandV2", ErrorCode::SendCommand_Failed);
        if (m_dbg) FMT_PRINT("Receiving {} args\n", sizeof...(ToRecv));
        TRY_UART_COMM_CMD(RecvFrameExpandArgs(std::make_index_sequence<sizeof...(ToRecv)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
        return std::ref(*this);
    }

    ExpectedOpenCmdModeResult OpenCommandMode();
    ExpectedGenericCmdResult CloseCommandMode();

    ExpectedGenericCmdResult SetSystemModeInternal(SystemMode mode);
    ExpectedGenericCmdResult UpdateVersion();

    ExpectedResult ReadFrame();
    //data
    Version m_Version;
    SystemMode m_Mode = SystemMode::Simple;
    OpenCmdModeResponse m_ProtoInfo{0, 0};
    Configuration m_Configuration;

    //the data will be read into as is
    PresenceResult m_Presence;
    Engeneering m_Engeneering;

    uint8_t m_BluetoothMAC[6] = {0};
    DistanceRes m_DistanceRes = DistanceRes::_0_75;
public:
    struct DbgNow
    {
        DbgNow(LD2412 *pC): m_Dbg(pC->m_dbg), m_PrevDbg(pC->m_dbg) { m_Dbg = true; }
        ~DbgNow() { 
            printf("\n");
            m_Dbg = m_PrevDbg; 
        }

        bool &m_Dbg;
        bool m_PrevDbg;
    };
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

template<>
struct tools::formatter_t<LD2412::Engeneering>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::Engeneering const& p)
    {
        return tools::format_to(std::forward<Dest>(dst), 
                "Max Move Gate:{} Max Still Gate:{}\n"
                "Move: {}\n"
                "Still: {}\n"
                "Light: {}\n"
                , p.m_MaxMoveGate, p.m_MaxStillGate,
                  p.m_MoveEnergy,
                  p.m_StillEnergy,
                  p.m_Light
            );
    }
};

template<>
struct tools::formatter_t<LD2412::Version>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, LD2412::Version const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "v{}.{}.{}" , v.m_Major, v.m_Minor, v.m_Misc);
    }
};

inline bool operator&(LD2412::TargetState s1, LD2412::TargetState s2)
{
    return (uint8_t(s1) & uint8_t(s2)) != 0;
}
#endif
