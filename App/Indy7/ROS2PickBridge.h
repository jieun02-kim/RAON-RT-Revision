/*****************************************************************************
*	Name: ROS2PickBridge.h
*	[kv260-merge] Gate 2b — ROS2 bridge to the KV260 perception pipeline.
*
*	Subscribes:
*	  /pick_target_base  (my_interfaces/PickTarget3D, base_link coords)
*	  /detections        (my_interfaces/DetectionArray, raw detector output)
*	Parameter client:
*	  /pick_logic_node   desired_class (LIVE parameter, D6 interactive menu)
*
*	Threading contract (the whole point of this class):
*	  - All ROS2 I/O, menu printing and the N-frame statistics gate run on
*	    the bridge's own non-RT threads (executor spin + worker).
*	  - The 1 kHz RT loop (proc_main_control → DoInput) may ONLY call the
*	    wait-free methods marked [RT] below: they touch atomics and a
*	    single-producer/single-consumer Goal slot — never a mutex.
*	Operator flow ('p' → digit → 'v'):
*	  'p'    menu of classes seen on /detections in the last ~1 s
*	  digit  → desired_class set on pick_logic → wait for TARGET LOCKED
*	  'v'    collect N frames, gate on std(xyz), publish Goal to the RT side
*	         (z already includes Z-margin; RT runs IK + quintic on it)
*****************************************************************************/
#ifndef __ROS2_PICK_BRIDGE__
#define __ROS2_PICK_BRIDGE__

#include "Defines.h"

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "my_interfaces/msg/pick_target3_d.hpp"
#include "my_interfaces/msg/detection_array.hpp"

class CROS2PickBridge
{
public:
    // Plain-data goal for the RT consumer — no Eigen/ROS types on purpose.
    struct Goal
    {
        double dX, dY, dZ;      // base_link [m]; dZ already includes the margin
        char   szClass[32];
        double dStdMM[3];       // per-axis sample std at the gate [mm]
        int    nSamples;
    };

    CROS2PickBridge();
    ~CROS2PickBridge();

    BOOL Init();
    BOOL DeInit();
    BOOL IsInit() const { return m_bInit; }

    /* ---- [RT] wait-free API for the 1 kHz loop ---- */
    void RequestMenu()     { m_bMenuReq.store(true, std::memory_order_release); }
    void RequestApproach() { m_bApproachReq.store(true, std::memory_order_release); }
    // '0'..'9' — consumed only while a menu is open.
    BOOL FeedDigit(char acDigit);
    // TRUE once per gated approach; producer never rewrites while ready.
    BOOL PopGoal(Goal& astOut);
    BOOL HasLockedTarget() const { return m_bLocked.load(std::memory_order_acquire); }
    // TRUE once after the menu's "record HOME" entry — RT snapshots joints.
    BOOL PopHomeRecord() { return m_bHomeRecordReq.exchange(false, std::memory_order_acq_rel); }

    /* ---- non-RT configuration (call before OP; defaults below) ---- */
    void SetWorkspaceBox(double adXMin, double adXMax, double adYMin,
                         double adYMax, double adZMin, double adZMax);
    void SetZMarginM(double adMargin)   { m_dZMarginM = adMargin; }
    void SetGate(int anSamples, double adStdGateM, double adTimeoutS);

private:
    void SpinLoop();
    void WorkerLoop();
    void OnTarget(const my_interfaces::msg::PickTarget3D::SharedPtr apMsg);
    void OnDetections(const my_interfaces::msg::DetectionArray::SharedPtr apMsg);
    void ShowMenu();
    void HandleDigit(int anSel);
    void StartCollect();
    void TickCollect();
    void TickLockWatch();
    BOOL SetDesiredClass(const std::string& astrClass);

    /* Workspace box for the GOAL (z margin already added). Set from the
     * 2026-07-27 hand-eye calibration: camera sits at base (0.762, -0.085,
     * 0.925) looking down, table extends +X. z_max 0.50 keeps >0.4 m of
     * clearance below the camera; x_max 0.85 is the practical Indy7 reach. */
    static constexpr double DEF_BOX[6]   = {0.30, 0.85, -0.50, 0.45, 0.10, 0.50};
    // Axis-aligned box passes goals the arm can't radially reach (orange at
    // r_xy 0.87 stalled IK 46 mm short, 2026-07-27) — gate the horizontal
    // radius too. Indy7 reach 0.8 m nominal.
    static constexpr double DEF_RMAX_XY  = 0.80;   // [m] from base axis
    static constexpr double DEF_ZMARGIN  = 0.15;   // [m] hover above the object
    static constexpr int    DEF_SAMPLES  = 15;     // ≈1 s @15 Hz
    static constexpr double DEF_STD_GATE = 0.008;  // [m] per-axis std gate
    static constexpr double DEF_TIMEOUT  = 3.0;    // [s] collection timeout
    static constexpr double MENU_WINDOW_S   = 1.0; // aggregation window for 'p'
    static constexpr int    MENU_MIN_COUNT  = 3;   // frames to count as "visible"
    static constexpr double LOCK_MAX_AGE_S  = 1.0; // sample freshness for lock
    static constexpr double LOCK_LOST_DEBOUNCE_S = 1.0; // silence detection flicker

    using SteadyTP = std::chrono::steady_clock::time_point;
    struct Sample
    {
        double dX, dY, dZ;
        std::string strClass;
        bool bValid, bDepth;
        SteadyTP tStamp;
    };

    /* ROS2 side (non-RT threads only) */
    std::shared_ptr<rclcpp::Node> m_pNode;
    rclcpp::Subscription<my_interfaces::msg::PickTarget3D>::SharedPtr   m_pSubTarget;
    rclcpp::Subscription<my_interfaces::msg::DetectionArray>::SharedPtr m_pSubDetections;
    std::shared_ptr<rclcpp::AsyncParametersClient> m_pParamClient;
    std::thread m_thSpin, m_thWorker;
    std::atomic<bool> m_bStop{false};
    BOOL m_bInit{FALSE};
    BOOL m_bOwnContext{FALSE};   // we called rclcpp::init → we shut it down

    /* shared between callbacks and worker (never the RT thread) */
    std::mutex m_Mtx;
    Sample m_stLatest;
    bool   m_bHaveSample{false};
    std::map<std::string, std::pair<int, SteadyTP>> m_mapSeen; // class → (count, last)
    std::vector<Sample> m_vCollect;
    bool        m_bCollecting{false};
    std::string m_strCollectClass;
    SteadyTP    m_tCollectDeadline;
    std::vector<std::string> m_vMenu;   // index → class ("" = auto entry)
    std::string m_strSelected;          // desired_class actually set ("" = auto)
    bool m_bLockedPrev{false};
    // Lock chatter is SESSION-scoped (2026-07-27): announce only between a
    // menu selection and GOAL READY. Without this the bridge narrated every
    // pipeline flicker forever — even at boot, when a desired_class left in
    // pick_logic (LIVE param survives app restarts) made stale targets lock.
    bool     m_bAnnounceLock{false};    // worker-thread only
    bool     m_bShownLocked{false};     // last ANNOUNCED state
    SteadyTP m_tLostSince;              // falling-edge timestamp for debounce

    /* RT handshake (atomics + SPSC slot only) */
    std::atomic<bool> m_bMenuReq{false};
    std::atomic<bool> m_bApproachReq{false};
    std::atomic<bool> m_bMenuOpen{false};
    std::atomic<int>  m_nDigit{-1};
    std::atomic<bool> m_bLocked{false};
    Goal              m_stGoal;
    std::atomic<bool> m_bGoalReady{false};
    std::atomic<bool> m_bHomeRecordReq{false};  // menu → RT: snapshot joints

    /* gate configuration (written before OP only) */
    double m_dBox[6];
    double m_dZMarginM{DEF_ZMARGIN};
    int    m_nGateSamples{DEF_SAMPLES};
    double m_dStdGateM{DEF_STD_GATE};
    double m_dTimeoutS{DEF_TIMEOUT};
};

#endif // __ROS2_PICK_BRIDGE__
