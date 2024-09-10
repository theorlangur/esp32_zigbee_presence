#ifndef LD2420_H_
#define LD2420_H_

#include "uart.hpp"
#include <span>
#include "functional/functional.hpp"

class LD2420: protected uart::Channel
{
public:
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
    };
    static const char* err_to_str(ErrorCode e);

    enum class SystemMode: uint32_t
    {
        Simple = 0x64,
        Energy = 0x04,
    };

    using Ref = std::reference_wrapper<LD2420>;
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

    struct PresenceResult
    {
        bool m_Detected = false;
        float m_Distance = 0;
    };

    LD2420(uart::Port p, int baud_rate = 115200);

    ExpectedResult Init(int txPin, int rxPin);

    void SetSystemMode(SystemMode mode);
    SystemMode GetSystemMode() const { return m_Mode; }

    int GetMinDistance() const { return m_MinDistance * 7 / 10; }
    uint32_t GetMinDistanceRaw() const { return m_MinDistance; }

    int GetMaxDistance() const { return m_MaxDistance * 7 / 10; }
    uint32_t GetMaxDistanceRaw() const { return m_MaxDistance; }

    auto GetMoveThreshold(uint8_t gate) const { return m_Gates[gate].m_MoveThreshold; }
    auto GetStillThreshold(uint8_t gate) const { return m_Gates[gate].m_StillThreshold; }
    auto GetMeasuredEnergy(uint8_t gate) const { return m_Gates[gate].m_Energy; }

    uint32_t GetTimeout() const { return m_Timeout; }//seconds
    void SetTimeout(uint32_t t) { m_Timeout = t; }

    ExpectedResult UpdateMinMaxTimeoutConfig();

    ExpectedResult ReloadConfig();

    std::string_view GetVersion() const;

    PresenceResult GetPresence() const { return m_Presence; }
private:
    enum class ADBRegs: uint16_t
    {
        MinDistance = 0,
        MaxDistance = 1,
        ActiveFrameNum = 2,
        InactiveFrameNum = 3,
        Timeout = 4,

        MoveThresholdGateBase = 0x10,
        StillThresholdGateBase = 0x20,
    };

    enum class SysRegs: uint16_t
    {
        Mode = 0
    };

    struct frame_t
    {
        static constexpr size_t kDataOffset = 4 + 2;//header + len
        static constexpr size_t kFooterSize = 4;

        uint8_t data[64] = {0xFD, 0xFC, 0xFB, 0xFA};
        uint16_t len;

        uint16_t TotalSize() const;
        void WriteCustom(std::span<uint8_t> const d);
        void WriteCmd(uint16_t cmd, std::span<uint8_t> const d);
        bool VerifyHeader() const;
        bool VerifyFooter() const;
    };
    template<size_t N>
    struct Params
    {
        uint32_t value[N];
    };
    struct OpenCmdModeResponse
    {
        uint16_t protocol_version;
        uint16_t buffer_size;
    };

#pragma pack(push,1)
    struct SetParam
    {
        uint16_t param;
        uint32_t value;
    };
#pragma pack(pop)

    using ADBParam = SetParam;
    using DataRetVal = RetValT<Ref, std::span<uint8_t>>;
    using StrRetVal = RetValT<Ref, std::string_view>;
    using ExpectedDataResult = std::expected<DataRetVal, CmdErr>;

    using OpenCmdModeRetVal = RetValT<Ref, OpenCmdModeResponse>;
    using ExpectedOpenCmdModeResult = std::expected<OpenCmdModeRetVal, CmdErr>;
    using ExpectedGenericCmdResult = std::expected<Ref, CmdErr>;
    using ExpectedCloseCmdModeResult = ExpectedGenericCmdResult;

    using SingleRawADBRetVal = RetValT<Ref, uint32_t>;
    using ExpectedSingleRawADBResult = std::expected<SingleRawADBRetVal, CmdErr>;

    template<size_t N>
    using MultiRawADBRetVal = RetValT<Ref, Params<N>>;
    template<size_t N>
    using ExpectedMultiRawADBResult = std::expected<MultiRawADBRetVal<N>, CmdErr>;

    ExpectedDataResult SendCommand(uint16_t cmd, const std::span<uint8_t> cmdData, std::span<uint8_t> respData);
    template<class T>
    ExpectedDataResult SendCommand(uint16_t cmd, T &&cmdData, frame_t &respData)
    {
        if constexpr (!std::is_same_v<std::remove_cvref_t<T>, std::span<uint8_t>>)
            return SendCommand(cmd, std::span<uint8_t>((uint8_t*)&cmdData, sizeof(T)), std::span<uint8_t>((uint8_t*)&respData, sizeof(frame_t)));
        else
            return SendCommand(cmd, cmdData, std::span<uint8_t>((uint8_t*)&respData, sizeof(frame_t)));
    }
    ExpectedResult SendFrame(const frame_t &frame);
    ExpectedDataResult RecvFrame(frame_t &frame);

    ExpectedOpenCmdModeResult OpenCommandMode();
    ExpectedCloseCmdModeResult CloseCommandMode();

    ExpectedSingleRawADBResult ReadRawADBSingle(uint16_t param);

    template<size_t N>
    ExpectedMultiRawADBResult<N> ReadRawADBMulti(uint16_t (&regs)[N])
    {
        using namespace functional;
        frame_t f;
        std::span<uint8_t> data((uint8_t*)&(regs[0]), sizeof(uint16_t) * N);
        return SendCommand(0x0008, data, f) 
            | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedMultiRawADBResult<N>{
                    if (val.size() < (4 * N))
                        return std::unexpected(CmdErr{Err{{}, "LD2420::ReadRawADBSingle", ErrorCode::SendCommand_WrongFormat}, (uint16_t)val.size()});
                    MultiRawADBRetVal<N> ret{std::ref(d), {}};
                    uint32_t *pRaw = (uint32_t*)val.data();
                    for(int i = 0; i < N; ++i) ret.v.value[i] = pRaw[i];
                    return ret;
              });
    }

    template<typename... T>
    auto ReadRawADBMulti(T... param)->ExpectedMultiRawADBResult<sizeof...(T)>
    {
        uint16_t buf[sizeof...(T)] = {(uint16_t)param...};
        return ReadRawADBMulti<sizeof...(T)>(buf);
    }

    template<size_t N>
    ExpectedGenericCmdResult WriteRawADBMulti(ADBParam (&pairs)[N])
    {
        using namespace functional;
        frame_t f;
        std::span<uint8_t> data((uint8_t*)&(pairs[0]), sizeof(ADBParam) * N);
        return SendCommand(0x0007, data, f) 
            | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedGenericCmdResult{ return std::ref(d); });
    }

    template<typename... T>
    auto WriteRawADBMulti(T... param)
    {
        ADBParam buf[sizeof...(T)] = {param...};
        return WriteRawADBMulti<sizeof...(T)>(buf);
    }

    template<size_t N>
    using MultiRawSysRetVal = RetValT<Ref, Params<N>>;
    template<size_t N>
    using ExpectedMultiRawSysResult = std::expected<MultiRawSysRetVal<N>, CmdErr>;
    template<size_t N>
    ExpectedMultiRawSysResult<N> ReadRawSysMulti(uint16_t (&regs)[N])
    {
        using namespace functional;
        frame_t f;
        std::span<uint8_t> data((uint8_t*)&(regs[0]), sizeof(uint16_t) * N);
        return SendCommand(0x0013, data, f) 
            | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedMultiRawSysResult<N>{
                    if (val.size() < (4 * N))
                        return std::unexpected(CmdErr{Err{{}, "LD2420::ReadRawSysMulti", ErrorCode::SendCommand_WrongFormat}, (uint16_t)val.size()});
                    MultiRawADBRetVal<N> ret{std::ref(d), {}};
                    uint32_t *pRaw = (uint32_t*)val.data();
                    for(int i = 0; i < N; ++i) ret.v.value[i] = pRaw[i];
                    return ret;
              });
    }

    template<typename... T>
    auto ReadRawSysMulti(T... param)
    {
        uint16_t buf[sizeof...(T)] = {(uint16_t)param...};
        return ReadRawSysMulti<sizeof...(T)>(buf);
    }

    template<size_t N>
    ExpectedGenericCmdResult WriteRawSysMulti(SetParam (&pairs)[N])
    {
        using namespace functional;
        frame_t f;
        std::span<uint8_t> data((uint8_t*)&pairs[0], sizeof(SetParam) * N);
        return SendCommand(0x0012, data, f) 
            | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedGenericCmdResult{ return std::ref(d); });
    }

    template<typename... T>
    auto WriteRawSysMulti(T... param)
    {
        SetParam buf[sizeof...(T)] = {param...};
        return WriteRawSysMulti<sizeof...(T)>(buf);
    }

    ExpectedGenericCmdResult SetSystemModeInternal(SystemMode mode);
    ExpectedGenericCmdResult UpdateVersion();
    ExpectedGenericCmdResult UpdateSystemMode();
    ExpectedGenericCmdResult UpdateMinMaxTimeout();
    ExpectedGenericCmdResult UpdateGate(uint8_t gate);

public:
    ExpectedResult ReadSimpleFrame();
    ExpectedResult TryReadSimpleFrame(int attempts = 3);

    ExpectedResult ReadEnergyFrame();
    ExpectedResult TryReadEnergyFrame(int attempts = 3);
private:
    ExpectedResult TryFillBuffer(size_t s);

    char m_Version[10];
    SystemMode m_Mode = SystemMode::Simple;
    OpenCmdModeResponse m_ProtoInfo{0, 0};
    uint32_t m_MinDistance = 1;//*70cm to get the distance
    uint32_t m_MaxDistance = 12;//*70cm to get the distance
    uint32_t m_Timeout = 30;
    struct Gate
    {
        uint16_t m_StillThreshold; //config
        uint16_t m_MoveThreshold; //config
        uint16_t m_Energy; //output
    };
    Gate m_Gates[16];

    char m_Buffer[64];
    size_t m_BufferReadFrom = 0;
    size_t m_BufferWriteTo = 0;
    bool m_BufferEmpty = true;

    PresenceResult m_Presence;
};

#endif
