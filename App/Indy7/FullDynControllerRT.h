/*****************************************************************************
*	Name: FullDynControllerRT.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Real-time optimized full dynamics controller using RBDL
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __CONTROLLER_RT_FULL_DYNAMICS__
#define __CONTROLLER_RT_FULL_DYNAMICS__

#include "Controller.h"  // Include the parent class
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Kinematics.h>
#include <Eigen/Core>
#include <sys/mman.h>
#include <sched.h>
#include <vector>
#include <string>
#include <cstring>

#include "posix_rt.h"

#define _ALL_AXIS 0xFFFFFFFF

class CControllerFullDynamicsRT : public CController
{
public:
    CControllerFullDynamicsRT(const TSTRING& astrURDFPath, unsigned int auDOF);
    virtual ~CControllerFullDynamicsRT();

    // Inherited from CController - must implement
    virtual BOOL Init() override;
    virtual BOOL Update(const std::vector<double>& avCurrentPos, 
                       const std::vector<double>& avCurrentVel,
                       const std::vector<double>& avCurrentTor,
                       std::vector<double>& avOutputTorque) override;
    virtual BOOL Reset() override;
    virtual void SetGains(const std::vector<double>& avGains) override;

    // RT-specific methods
    void SetReferenceTrajectory(const RigidBodyDynamics::Math::VectorNd& avQ_ref,
                               const RigidBodyDynamics::Math::VectorNd& avQd_ref,
                               const RigidBodyDynamics::Math::VectorNd& avQdd_ref);
    
    void SetControlGains(const std::vector<double>& avKp, const std::vector<double>& avKd);
    void SetGravity(const RigidBodyDynamics::Math::Vector3d& avGravity);
    void EnableRTMode(BOOL abEnable) { m_bRTMode = abEnable; }
    BOOL SetReferencePos(UINT auAxis, double adPos);
    BOOL SetControlGain(UINT auAxis, double adKp, double adKd);
    BOOL GetControlGain(UINT auAxis, double* adKp, double* adKd);
    UINT GetDOF() const { return m_uDOF; }

    void SetTrajectoryDuration(double adT_sec) { m_traj_duration = adT_sec; }
    bool IsTrajectoryDone() const { return !m_traj.active; }
    // [kv260-merge] "reference generation finished" — unlike IsTrajectoryDone,
    // does NOT require the settle criterion (every joint within 0.15 rad of
    // q_goal). Under steady-state droop that criterion can never be met, so
    // completion logic gated on IsTrajectoryDone would wait forever.
    bool IsTrajectoryRefDone() const
    { return !m_traj.active || m_traj.t_elapsed >= m_traj.T; }
    const RigidBodyDynamics::Math::VectorNd& GetTrajGoal() const { return m_traj.q_goal; }
    double GetTrajT() const { return m_traj.T; }

    // [kv260-merge] IK sanity gates (2026-07-27 table strike, merge.md E13).
    // A solution jumping a joint by more than IK_DQ_MAX_RAD from the current
    // configuration is a branch flip / wrist unwind — refuse it instead of
    // sweeping the arc. T is stretched so the quintic's PEAK joint speed
    // (1.875*dq/T) never exceeds IK_QD_PEAK_RADPS.
    static constexpr double IK_DQ_MAX_RAD    = 2.0;
    static constexpr double IK_QD_PEAK_RADPS = 0.6;
    static constexpr double IK_T_MAX_S       = 10.0;
    // Orientation is a SOFT preference (hard keep-R made the solution — and
    // the dq gate — hypersensitive to the hand-parked start pose): position
    // must land within IK_POS_TOL_M, orientation bends only where the arm
    // must. Acceptance is checked by FK, not by RBDL's convergence flag,
    // because an unsatisfiable soft residual keeps that flag false.
    static constexpr float  IK_ORI_WEIGHT    = 0.3f;
    static constexpr double IK_POS_TOL_M     = 0.002;
    // Newton budget, written to RBDL's CS.max_steps.
    //
    // ⚠ The trap: CS.num_steps is an OUTPUT ("the number of iterations
    // performed"); CS.max_steps is the input limit (RBDL default 300). The
    // pre-existing `CS.num_steps = 1000  // 수렴 기회 늘리기` never did
    // anything, and neither did a first attempt to CAP the ladder there.
    //
    // ⚠ And do not try to cap it now that the right field is being written:
    // that was tried (40 steps for exploratory rungs, 2026-07-28 17:18) and it
    // fixed the timing while WRECKING the solutions. A ladder rung asks "does
    // this weight converge?", and a starved solver answers wrong in both
    // directions — rungs that would have converged fail, and under-converged
    // points pass the 2 mm position test while being poor solutions. Measured:
    // mustard went from (w=0.01, dq 1.11 rad, direct, refine 3 passes to
    // 7.0 mm) to (w=0.10, dq 2.98 rad, staged, staged leg refused) and
    // (w=0.00, tilt 3 deg, refine oscillating to 16.7 mm).
    //
    // So the budget stays full and the ladder costs up to ~32 ms. That is a
    // real 1 kHz violation — the honest fix is to move the solve OFF the RT
    // thread (the bridge already owns a non-RT worker; it would need its own
    // RBDL model instance, the model not being thread-safe) and hand RT a
    // finished joint vector. Until then the WARN below marks the event; it is
    // one-shot at operator command with the arm static in grav-comp, and the
    // bus has been measured unharmed through it.
    static constexpr int    IK_SOLVE_STEPS   = 300;
    // Far-reach fallback (2026-07-27 E23): the soft-R solve can equilibrate
    // SHORT of a far target (E18's gradient stall — banana at r 0.83
    // "missed" by 235 mm while mustard at r 0.86 missed by 62 mm: the
    // residual measures where the solver stalled, not the true shortfall).
    // SolveReadyIK then retries pure-position from the same q_ready seed and
    // accepts only if the tool attitude stays within this cone of the ready
    // posture; fold/limits/dq gates still guard the output.
    static constexpr double APPROACH_RDEV_MAX_DEG = 60.0;

    // [kv260-merge] Sticky-float hold: grav-comp + a weak per-joint spring to
    // an anchor that DRAGS when pushed past the dead-band and freezes when
    // released. Stops the residual-model sink (the model can't be perfect —
    // link mass % errors + friction) while staying hand-guidable; the spring
    // force is bounded by construction (max = frac*Kp*DB per joint).
    // Toggle with 'k' (default ON).
    static constexpr double HOLD_DB_RAD  = 0.03;   // dead-band [rad]
    static constexpr double HOLD_KP_FRAC = 0.15;   // of the joint Kp
    static constexpr double HOLD_KD_FRAC = 0.03;   // of the joint Kd (feel)
    // Per-joint velocity gate: while a joint is clearly being hand-guided
    // (|qd| above this) its anchor just follows — guiding feels like pure
    // grav-comp. Creep from model residual is far slower than this, so the
    // spring still catches sinking. (Operator feedback 2026-07-27: the
    // always-on spring made guiding feel stiff.)
    static constexpr double HOLD_UNLOCK_QD = 0.08; // [rad/s]
    std::atomic<bool> m_bStickyEnable{true};
    BOOL StartJointTrajectory(const RigidBodyDynamics::Math::VectorNd& q_goal, double T);

    // [kv260-merge] Deterministic ready-seed IK (2026-07-27, operator request).
    // RBDL IK is a LOCAL iterative solver whose only branch-selection input is
    // the seed; seeding from the live posture made approach success depend on
    // where the operator happened to park the arm (field failures: branch-flip
    // REFUSED / soft-R local-minimum residual). The seed is a COMPUTATIONAL
    // start point — it does not need to be the physical posture — so we
    // bootstrap ONE posture from the measured workspace box at init, verify it
    // against the box corners, and solve every approach from it: same target →
    // same solution, regardless of the start posture. The arm then only has to
    // TRAVEL (direct if the solution is within IK_DQ_MAX_RAD, staged via
    // q_ready otherwise — the app decides).
    BOOL ComputeReadySeed(const std::vector<RigidBodyDynamics::Math::Vector3d>& avProbes,
                          const RigidBodyDynamics::Math::Vector3d& avCenter,
                          const RigidBodyDynamics::Math::VectorNd* apAnchorSeed = NULL);
    // Re-base q_ready on a REAL posture (the operator's recorded HOME): a
    // branch a human demonstrated is the ONLY trusted anchor. Field lessons
    // 2026-07-27: synthetic ladder bootstraps produced kinematically-valid
    // but contorted postures TWICE (J4 2.61 wrist fold; then J2 -2.80 elbow
    // fold) — reach+limits+dq alone don't make a posture sane, so the ladder
    // was removed: no anchor, no q_ready (live-posture seeding until HOME is
    // recorded).
    BOOL SetReadyAnchor(const RigidBodyDynamics::Math::VectorNd& aqAnchor);
    // Express a joint goal in the LIVE counter frame: fold each joint to the
    // 2π-equivalent nearest the reference. The multiturn counters can hold
    // whole extra turns (E17 / E-stop bus-power cycles re-reference them:
    // J1 -6.2, J4 -18.9 rad observed with the arm physically near zero), and
    // a goal expressed in another lap would command REAL full-turn rotations.
    // Folding toward the live q caps every joint's commanded travel at <= π.
    static void FoldTowardRef(RigidBodyDynamics::Math::VectorNd& aq,
                              const RigidBodyDynamics::Math::VectorNd& aqRef,
                              unsigned int auDof);
    // Record-time coverage feedback: after SetReadyAnchor the workspace
    // probes are re-verified ONE PER CYCLE in the RT loop (a 9-solve burst
    // would stall the 1 kHz task) and a summary is logged — the operator
    // learns immediately whether the posture they just recorded covers the
    // box, instead of finding out next boot.
    void TickAnchorVerify();
    BOOL HasReadySeed() const { return m_bReadySet; }
    const RigidBodyDynamics::Math::VectorNd& GetReadyQ() const { return m_QReady; }
    // Pure solve, no motion: seed = q_ready, soft-R = q_ready's own FK
    // orientation (fixed → deterministic). Folds the solution to the 2π-branch
    // nearest q_ready and checks URDF joint limits. adDqMaxFromCur/anDqAxis
    // report the largest travel from the CURRENT posture for the caller's
    // direct-vs-stage decision.
    BOOL SolveReadyIK(const RigidBodyDynamics::Math::Vector3d& avTarget,
                      RigidBodyDynamics::Math::VectorNd& aqSol,
                      double& adPosErrM, double& adDqMaxFromCur, int& anDqAxis);
    // E13 speed cap as a reusable rule: stretch T so the quintic peak joint
    // speed (1.875*dq/T) never exceeds IK_QD_PEAK_RADPS, capped at IK_T_MAX_S.
    static double ScaledTrajTime(double adDqMax, double adTBase);


    // Controller modes
    enum eControlMode {
        eGravityCompensation = 0,
        eFullDynamics,
        eComputedTorque,
        eAdaptiveControl,
        eInverseKinematics,       // 'i' : Jacobian-based IK
        eInverseKinematics_6dof   // 'm' : RBDL IK (position + orientation) + CTC
    };
    
    void SetControlMode(eControlMode aeMode) { m_eControlMode = aeMode; }
    eControlMode GetControlMode() const { return m_eControlMode; }
    
    // Performance monitoring
    struct RTPerformance {
        uint64_t max_compute_time_ns;
        uint64_t avg_compute_time_ns;
        uint64_t total_compute_time_ns;
        uint64_t cycle_count;
        uint64_t rt_violations;
        uint64_t deadline_ns;
        
        void Reset() {
            max_compute_time_ns = 0;
            avg_compute_time_ns = 0;
            total_compute_time_ns = 0;
            cycle_count = 0;
            rt_violations = 0;
        }
    };
    
    RTPerformance GetPerformance() const { return m_rt_perf; }
    void ResetPerformance() { m_rt_perf.Reset(); }
    void SetDeadline(uint64_t deadline_ns) { m_rt_perf.deadline_ns = deadline_ns; }
    
    // RT optimization control
    BOOL InitRTOptimizations();

    //====================================================================
    // jieun 
    //====================================================================
    struct Pose
    {
        RigidBodyDynamics::Math::Vector3d m_position, m_rotation_rpy;
        RigidBodyDynamics::Math::Matrix3d m_rotation;

        Pose()
        {
            m_position.setZero();
            m_rotation.setIdentity();
            m_rotation_rpy.setZero();
            
        }
    };

    // RigidBodyDynamics::Math::Vector3d tcp_local_point{0.0, 0.0, 0.07};
    RigidBodyDynamics::Math::Vector3d tcp_local_point{0.0, 0.0, 0.0};
    
    void CheckIKConvergence();
    BOOL ComputeTcpFK();
    BOOL SetTcpReferencePose();
    BOOL ComputeJacobianBasedInverseKinematics(std::vector<double>& avOutputTorque);
    BOOL RunISOCubeIKValidation();

    Pose tcpPose;
    const Pose& GetTcpPose() const { return tcpPose; }
    
    
    // for goal pose
    Pose goal_tcpPose;          //
    Pose fir_tcpPose, fin_tcpPose;;

    // verification
    Pose m_tcpStartPose;        // 
    Pose m_tcpFinalPose;        //
    Pose m_goalTcpPoseForCheck; //

    BOOL m_bIkReady;            // IK 해가 설정됐는지
    BOOL m_bVerifyDone;
    BOOL m_bIkMotionStarted;    // IK 시작 후 실제로 움직임이 감지됐는지
    int  m_nStableCount;
    unsigned int m_body_id;

    
    const RigidBodyDynamics::Math::VectorNd& GetQRef() const { return     m_Q_ref; }    

    // logging
    std::atomic<bool> 		 m_bIkTrigger{false};
    std::atomic<bool>        m_bSetRefPoseTrigger{false};

    // Full IK (Jacobian-based)
    RigidBodyDynamics::Math::MatrixNd m_J;       // 6 x DOF Jacobian
    RigidBodyDynamics::Math::MatrixNd m_JJt;     // 6 x 6
    RigidBodyDynamics::Math::MatrixNd m_J_pinv;  // DOF x 6
    RigidBodyDynamics::Math::VectorNd m_e_task;  // 6D Cartesian error [angular; linear]
    double m_Kp_task_pos = 1.0;                  // task-space position gain
    double m_Kp_task_rot = 0.2;                  // task-space orientation gain (작게 → flip 방지)
    static constexpr double m_dt = 0.001;        // 1kHz → 1ms
    static constexpr double m_lambda = 0.01;     // DLS damping factor
    
    // static constexpr double m_lambda = 0.01;     // DLS damping factor

    BOOL GetCurrentPose(Pose& astCurrPose);
    BOOL SetTargetPose(Pose astTargetPose);
    // adOriWeight: soft keep-R weight. Default for approaches; pass 0.0 for
    // small LOCAL re-targets (refine) — with a large bias the soft-R trade
    // stalls at a position/orientation equilibrium and the FK acceptance
    // rejects it (2026-07-27 regression). The dq gate still applies.
    // abQuiet: suppress the refusal logs. Used by the E26 refine ladder, where
    // a rung failing is normal — only the last rung's failure is real news.
    BOOL SetTargetPosePositionOnly(Pose astTargetPose,
                                   double adOriWeight = IK_ORI_WEIGHT,
                                   BOOL abQuiet = FALSE);
    // Tool attitude (body->base, the Pose::m_rotation convention) at an
    // arbitrary joint vector. E26: lets the caller anchor refine on the
    // attitude the approach SOLUTION reaches instead of on whatever attitude
    // the arm happened to be parked in when the goal was accepted.
    RigidBodyDynamics::Math::Matrix3d ToolRotAt(
        const RigidBodyDynamics::Math::VectorNd& aq);
    BOOL SetTargetPose_Jacobian();           // visual servoing용 (매 사이클 목표 추종)
    BOOL LogDistanceError(Pose astTargetPose);

    static RigidBodyDynamics::Math::Matrix3d RPYToRot(double roll, double pitch, double yaw);


    //====================================================================
    //====================================================================


private:
    // [kv260-merge] ready-seed state (init or SetReadyAnchor; main-control
    // thread only)
    RigidBodyDynamics::Math::VectorNd m_QReady;
    RigidBodyDynamics::Math::Matrix3d m_EReady;   // base->body FK R at q_ready
    bool m_bReadySet{false};
    std::vector<RigidBodyDynamics::Math::Vector3d> m_vProbeStore;
    int m_nAnchorVerifyIdx{-1};    // -1 = idle; else next probe to verify
    int m_nAnchorVerifyPass{0};
    // Canonicalize IN PLACE (fold toward mid-range zero) and check the URDF
    // limits — validity is judged on the PHYSICAL posture, never on a wound
    // counter representation. Returns false if any joint has no in-limit
    // representative.
    bool CheckLimitsPhysical(RigidBodyDynamics::Math::VectorNd& aq) const;
    // One position-IK solve from an explicit seed (apEOri: optional base->body
    // soft-R target). Acceptance by FK residual, not RBDL's flag.
    BOOL SolvePositionIK(const RigidBodyDynamics::Math::VectorNd& aqSeed,
                         const RigidBodyDynamics::Math::Vector3d& avTarget,
                         double adOriWeight,
                         const RigidBodyDynamics::Math::Matrix3d* apEOri,
                         RigidBodyDynamics::Math::VectorNd& aqSol,
                         double& adPosErrM,
                         int anMaxSteps = 1000);
    // Largest per-joint distance from q_ready after folding aq toward it, with
    // the offending axis. A solution is only useful if it sits on the SAME
    // branch as q_ready: the solver can reach an identical point with the
    // shoulder swung ~180 deg round and the remaining joints folded back, so
    // the TOOL ATTITUDE barely moves and APPROACH_RDEV_MAX_DEG waves it
    // through (measured: J0 3.13 rad away, tool tilt 1 deg). Staging cannot
    // rescue that — staging moves the arm TO q_ready, which is exactly where
    // this gap is measured from.
    double ReadyBranchGap(const RigidBodyDynamics::Math::VectorNd& aq,
                          int& anAxis);
    // Angle (deg) between the tool attitude at aq and the ready attitude
    // m_EReady. This is the quantity APPROACH_RDEV_MAX_DEG gates on, and the
    // one the E25 polish pass tries to shrink.
    double AttitudeDevDeg(const RigidBodyDynamics::Math::VectorNd& aq);

    // RBDL model and dynamics
    RigidBodyDynamics::Model m_rbdlModel;
    TSTRING m_strURDFPath;
    eControlMode m_eControlMode;
    
    // Pre-allocated RBDL vectors (RT-safe)
    RigidBodyDynamics::Math::VectorNd m_Q;              // Current position
    RigidBodyDynamics::Math::VectorNd m_Qd;             // Current velocity
    RigidBodyDynamics::Math::VectorNd m_Qdd;            // Current acceleration
    RigidBodyDynamics::Math::VectorNd m_Q_ref;          // Reference position
    RigidBodyDynamics::Math::VectorNd m_Qd_ref;         // Reference velocity
    RigidBodyDynamics::Math::VectorNd m_Qd_ref_prev;   // Previous cycle reference velocity (for acc limit)
    RigidBodyDynamics::Math::VectorNd m_Qdd_ref;        // Reference acceleration
    RigidBodyDynamics::Math::VectorNd m_zero_vector;    // Zero vector
    RigidBodyDynamics::Math::VectorNd m_Q_ik_target;    // RBDL IK solution (final target)
    
    // Dynamics components
    RigidBodyDynamics::Math::MatrixNd m_M;              // Inertia matrix M(q)
    RigidBodyDynamics::Math::VectorNd m_h;              // Nonlinear effects h(q,qd)
    RigidBodyDynamics::Math::VectorNd m_g;              // Gravity vector g(q)
    RigidBodyDynamics::Math::VectorNd m_c;              // Coriolis forces c(q,qd)
    
    // Control terms
    RigidBodyDynamics::Math::VectorNd m_tau_M;          // M(q) * qdd_ref
    RigidBodyDynamics::Math::VectorNd m_tau_feedback;   // Feedback control
    RigidBodyDynamics::Math::VectorNd m_tau_total;      // Total control torque
    
    // Error vectors
    RigidBodyDynamics::Math::VectorNd m_pos_error;      // Position error
    RigidBodyDynamics::Math::VectorNd m_vel_error;      // Velocity error
    
    // Control gains
    std::vector<double> m_Kp;                           // Position gains
    std::vector<double> m_Kd;                           // Velocity gains

    // sticky-float anchor (grav-comp only; reset on mode change)
    RigidBodyDynamics::Math::VectorNd m_Q_hold;
    bool m_bHoldAnchored{false};

    // 5th-order polynomial trajectory
    struct TrajState {
        RigidBodyDynamics::Math::VectorNd q_start;
        RigidBodyDynamics::Math::VectorNd q_goal;
        double T;
        double t_elapsed;
        bool   active;
    };
    TrajState m_traj;
    double    m_traj_duration;
    void      UpdateTrajectory();
    
    // RT performance monitoring
    RTPerformance m_rt_perf;
    BOOL m_bRTMode;
    BOOL m_bRTOptimized;
    
    // RT utility methods
    BOOL LoadURDF(const TSTRING& astrURDFPath);
    BOOL InitRTMemoryOptimizations();
    BOOL InitRTProcessorOptimizations();
    void CheckRTViolation(uint64_t computation_time_ns);
    
    // Control computation methods
    BOOL ComputeGravityCompensation(std::vector<double>& avOutputTorque);
    BOOL ComputeFullDynamics(std::vector<double>& avOutputTorque);
    BOOL ComputeComputedTorque(std::vector<double>& avOutputTorque);

    // jieun
    //BOOL ComputeTcpFK(std::vector<double>& avOutputTorque);
    //Pose tcpPose;
    BOOL ComputeInverseKinematics_6dof(std::vector<double>& avOutputTorque);

    BOOL IsJointSettled(double vel_threshold);
    BOOL IsJointStopped(void);
    void PrintTcpVerificationResult();
    
};

#endif // __CONTROLLER_RT_FULL_DYNAMICS__