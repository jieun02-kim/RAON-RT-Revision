#pragma once
#include <thread>
#include <atomic>
#include <string>
#include <memory>

#include "FullDynControllerRT.h"

class VisualServo
{
public:
    using Pose = CControllerFullDynamicsRT::Pose;

    enum class State {
        IDLE,
        TRACKING,
        TAG_LOST,
        SINGULARITY,
    };

    explicit VisualServo(const std::string& calibDir = "/home/raimlab/RAON-RT/App/CalibUtils/eye_to_hand");
    ~VisualServo();

    bool Init();
    void Start();
    void Stop();
    void Loop();
    bool IsRunning()  const { return m_running.load(); }
    State GetState()  const { return m_state.load(); }

    bool GetGoalPose(Pose& out) const;
    void NotifySingularity();

    int lostFrameThreshold = 10;

private:
    std::string        m_calibDir;
    std::atomic<bool>  m_running{false};
    std::atomic<State> m_state{State::IDLE};

    Pose               m_buf[2];
    std::atomic<int>   m_latest{0};
    std::atomic<bool>  m_valid{false};
    bool               m_initialized{false};
    int                m_lostCount{0};

    // ViSP/RealSense 내부 구현 — 헤더 노출 없이 PIMPL
    struct Impl;
    std::unique_ptr<Impl> m_pImpl;
};
