/*****************************************************************************
*	Name: ROS2PickBridge.h
*	[kv260-merge] Gate 2b — ROS2 bridge to the KV260 perception pipeline.
*
*	Subscribes:
*	  /pick_target_base  (my_interfaces/PickTarget3D, base_link coords)
*	  /detections        (my_interfaces/DetectionArray, raw detector output)
*	  /operator_key      (std_msgs/String, [gui] one key from the desktop console)
*	  /operator_gains    (std_msgs/String JSON, [gui] admin tab —
*	                      {"kp":[6],"kd":[6],"kf":[6]}; clamped + idle-gated
*	                      here, applied by the RT loop via PopGains)
*	  /operator_jog      (std_msgs/String JSON, [gui] Joint tab —
*	                      {"axis":N,"q":rad}; clamped + gated here (FullDyn/CT
*	                      + servo + idle only), RT walks q_ref to it at
*	                      0.5 rad/s via PeekJog → SetJogTarget)
*	Publishes:
*	  /joint_states      (sensor_msgs/JointState, [gui] 20 Hz, standard tools)
*	  /robot_state       (std_msgs/String JSON, [gui] 20 Hz — joints + the exact
*	                      flags DoInput's guards read; drives button gating)
*	  /operator_msg      (std_msgs/String, [gui] operator-facing lines — the 'p'
*	                      menu, lock/goal/gate messages — mirrored to the console)
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
#include "WorkspaceBox.h"

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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

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

    // [kv260-merge] Per-approach accuracy record (2026-07-29). One row per
    // completed approach: what we ASKED for vs where the TCP actually ended
    // up, plus the pre-refine position so the plot can show what refinement
    // bought. Plain data on purpose — the RT producer must not touch
    // std::string, FILE or the clock; the worker thread stamps the time and
    // does the IO, exactly like the ready-seed mailbox below.
    struct ApproachReport
    {
        char   szClass[32];
        double dGoal[3];        // commanded base_link target [m] (incl. Z margin)
        double dTcp[3];         // FK TCP once refinement settled [m]
        double dTcpFirst[3];    // FK TCP after the FIRST trajectory (NaN = none)
        double dStdMM[3];       // per-axis vision std at the 'v' gate
        int    nSamples;        // frames behind that std
        int    nRefineIter;     // refinement passes actually run
        double dZMarginM;       // hover margin baked into dGoal[2]
        double dTiltDeg;        // IK diag: tool attitude vs q_ready
        double dGapRad;         //          branch distance from q_ready
        double dIkMs;           //          solve wall time
        double dIkPosErrM;      //          solver's own residual
        int    nSeed;           //          winning seed (0 = q_ready)
        int    nSolves;         //          RBDL calls
        int    nGapAxis;        //          joint carrying dGapRad
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
    // [kv260-merge] Persist the operator's recorded posture so the NEXT boot
    // anchors its IK bootstrap to a demonstrated branch (2026-07-27 field:
    // the synthetic ladder produced a contorted q_ready). RT fills the
    // mailbox wait-free; the worker thread does the file IO.
    void PersistReadySeed(const double* aqRad, int anDof);
    static const char* ReadySeedPath();   // $HOME/.indy7_ready_seed

    // [RT] hand one finished approach to the worker thread (wait-free).
    void LogApproach(const ApproachReport& astRep);
    static const char* ApproachLogPath();  // approach_results/approach_log.csv

    // [kv260-merge 2026-08-03] Tracking sample pipe ('o'). The ROS callback
    // publishes every sample that survives finite → in-box → jump gating,
    // EMA-filtered, into a single seqlock slot. Publication implies
    // target_valid && depth_valid; the RT side detects staleness by counting
    // cycles since the sequence number last changed — no clock on the RT
    // thread. Coordinates are RAW base_link metres (no z margin).
    struct TrackSample
    {
        double dX, dY, dZ;
        bool   bClassMatch;   // == desired_class ("" selected = matches any)
    };
    // [RT] single-attempt seqlock read; FALSE = writer mid-update or torn —
    // just keep the last accepted target, a fresh sample lands within 66 ms.
    // auSeq: same even value as last time means "no new sample".
    BOOL PeekTrack(TrackSample& astOut, uint32_t& auSeq) const;

    /* ---- [gui] operator console (raon_operator_gui on the desktop) ---- */
    // [RT] One remote key, consumed by DoInput exactly like a local keypress.
    // FALSE = nothing pending. Whitelisted in OnKey: /operator_key is a network
    // input that moves a real arm.
    BOOL PopKey(char& acOut);

    // [RT] One clamped gains set from the console's admin tab, already
    // idle-gated by OnGains. Layout: [0..5]=Kp, [6..11]=Kd, [12..17]=Kf.
    // The RT loop applies it per-axis (SetControlGain / SetFrictionFFAxis —
    // plain stores, no allocation).
    static constexpr int GAINS_DOF = 6;
    BOOL PopGains(double* adOut);   // adOut[3 * GAINS_DOF]

    // [RT] Newest console jog command (single-attempt seqlock, exactly
    // PeekTrack's contract): same even auSeq as last time = nothing new.
    // OnJog already clamped and gated it; the controller clamps again.
    struct JogCmd
    {
        int    nAxis;
        double dQ;      // [rad], absolute target for that axis
    };
    BOOL PeekJog(JogCmd& astOut, uint32_t& auSeq) const;

    // [gui] What the console needs to gate its buttons: the joint angles plus
    // the SAME booleans DoInput's refusal guards read. Plain data on purpose —
    // the RT producer fills it wait-free; the 20 Hz worker publishes it as
    // /joint_states + /robot_state (JSON). Do not put derived UI logic here:
    // the console mirrors DoInput, it must never out-think it.
    struct RobotState
    {
        double dQ[8];
        int    nDof;
        bool   bServo;   // every axis IsServoOn()
        bool   bCtrl;    // m_bRTControllerEnabled && controller IsEnabled()
        bool   bGrav;    // control mode == eGravityCompensation
        bool   bTrack;   // m_eTrackState != eTRACK_OFF
        bool   bBusy;    // approach/ISO/RECT running
        bool   bHome;    // user HOME recorded || ready-seed available
        bool   bCalib;   // 's' writes the calibration dataset right now
        // [gui] controller eControlMode as int (-1 = no RT controller) —
        // the console's mode display and OnJog's mode gate read this
        int    nMode;
        // [gui] live controller gains for the admin tab's readback. bGains
        // FALSE (RT controller absent) => arrays not serialized, and the
        // console blocks sending — no readback means no proven channel.
        bool   bGains;
        double dKp[8];
        double dKd[8];
        double dKf[8];
    };
    // [RT] hand the cycle's snapshot to the worker. Wait-free seqlock.
    void PushState(const RobotState& astState);

    /* ---- non-RT configuration (call before OP; defaults below) ---- */
    void SetWorkspaceBox(double adXMin, double adXMax, double adYMin,
                         double adYMax, double adZMin, double adZMax);
    void SetZMarginM(double adMargin)   { m_dZMarginM = adMargin; }
    double GetZMarginM() const          { return m_dZMarginM; }
    void SetGate(int anSamples, double adStdGateM, double adTimeoutS);
    // EMA alpha per 15 Hz vision frame (clamped 0.05..1.0; 1.0 = filter off).
    void SetTrackAlpha(double adA)
    {
        m_dTrackAlpha = adA < 0.05 ? 0.05 : (adA > 1.0 ? 1.0 : adA);
    }

    /* The measured box and its radius cap now live in WorkspaceBox.h so the
     * ROS-free consumers (the app's IK probes, the offline reach-map tool)
     * can read them without rclcpp. The rationale for each number is there;
     * these names are kept because the app and bridge both spell them this
     * way. Public: the app derives the ready-seed IK probes from the box. */
    static constexpr double DEF_BOX[6]   = {kv260::WS_BOX[0], kv260::WS_BOX[1],
                                            kv260::WS_BOX[2], kv260::WS_BOX[3],
                                            kv260::WS_BOX[4], kv260::WS_BOX[5]};
    static constexpr double DEF_RMAX_XY  = kv260::WS_RMAX_XY;   // [m] from base axis

private:
    void SpinLoop();
    void WorkerLoop();
    void OnTarget(const my_interfaces::msg::PickTarget3D::SharedPtr apMsg);
    void OnDetections(const my_interfaces::msg::DetectionArray::SharedPtr apMsg);
    void OnKey(const std_msgs::msg::String::SharedPtr apMsg);
    void OnGains(const std_msgs::msg::String::SharedPtr apMsg);
    void OnJog(const std_msgs::msg::String::SharedPtr apMsg);
    void TickStatePub();
    // [gui] operator-facing line: robot terminal (printf, unchanged) AND
    // /operator_msg for the console log. Worker/ROS-callback threads ONLY —
    // never the RT thread (publish allocates).
    void Say(const char* acFmt, ...) __attribute__((format(printf, 2, 3)));
    void ShowMenu();
    void HandleDigit(int anSel);
    void StartCollect();
    void TickCollect();
    void TickLockWatch();
    void TickSeedPersist();
    void TickApproachLog();
    BOOL SetDesiredClass(const std::string& astrClass);

    // seed-persist mailbox (single producer = RT, single consumer = worker)
    double           m_adSeedMail[8] = {0};
    std::atomic<int> m_nSeedMailN{0};

    // approach-report mailbox (same SPSC contract; approaches are seconds
    // apart and the worker ticks at 50 ms, so one slot is plenty)
    ApproachReport    m_stRepMail{};
    std::atomic<bool> m_bRepReady{false};

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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             m_pSubKey;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr         m_pPubJoints;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                m_pPubState;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                m_pPubMsg;
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

    // [gui] remote key slot (writer = ROS callback, reader = RT PopKey).
    // ' ' = empty, same sentinel CRobotIndy7::m_cKeyPress uses.
    std::atomic<char> m_cRemoteKey{' '};

    // [gui] admin gains mailbox (writer = OnGains on the spin thread, reader
    // = RT PopGains — same SPSC contract as the goal slot: the producer never
    // rewrites while ready)
    double            m_adGainMail[3 * GAINS_DOF] = {0};
    std::atomic<bool> m_bGainReady{false};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_pSubGains;

    // [gui] jog slot: seqlock like the tracking slot (writer = OnJog on the
    // spin thread, reader = RT PeekJog). Newest command wins — a slider drag
    // is a stream where only the latest target matters.
    JogCmd                m_stJogSlot{};
    std::atomic<uint32_t> m_uJogSeq{0};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_pSubJog;

    // [gui] state slot: seqlock the other way round (writer = RT PushState,
    // reader = worker TickStatePub). Same contract as the tracking slot below.
    RobotState            m_stStateSlot{};
    std::atomic<uint32_t> m_uStateSeq{0};

    // tracking slot: seqlock (writer = ROS callback, reader = RT PeekTrack).
    // Even seq = consistent; the writer holds it odd across the slot copy.
    TrackSample           m_stTrackSlot{};
    std::atomic<uint32_t> m_uTrackSeq{0};
    // callback-thread-only filter state
    double   m_dTrackAlpha{0.4};
    double   m_adTrackEma[3]{};
    bool     m_bTrackSeeded{false};
    SteadyTP m_tTrackAccept{};      // last accept time (gap → reseed)
    static constexpr double TRACK_GAP_RESEED_S = 1.0;
    static constexpr double TRACK_BOX_EPS_M    = 0.02;

    /* gate configuration (written before OP only) */
    double m_dBox[6];
    double m_dZMarginM{DEF_ZMARGIN};
    int    m_nGateSamples{DEF_SAMPLES};
    double m_dStdGateM{DEF_STD_GATE};
    double m_dTimeoutS{DEF_TIMEOUT};
};

#endif // __ROS2_PICK_BRIDGE__
