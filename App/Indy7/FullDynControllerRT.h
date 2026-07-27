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
    BOOL SetTargetPosePositionOnly(Pose astTargetPose,
                                   double adOriWeight = IK_ORI_WEIGHT);
    BOOL SetTargetPose_Jacobian();           // visual servoing용 (매 사이클 목표 추종)
    BOOL LogDistanceError(Pose astTargetPose);

    static RigidBodyDynamics::Math::Matrix3d RPYToRot(double roll, double pitch, double yaw);


    //====================================================================
    //====================================================================


private:
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