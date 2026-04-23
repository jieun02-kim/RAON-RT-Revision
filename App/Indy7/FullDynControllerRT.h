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
  

    // Controller modes
    enum eControlMode {
        eGravityCompensation = 0,
        eFullDynamics,
        eComputedTorque,
        eAdaptiveControl,
        eInverseKinematics
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
        RigidBodyDynamics::Math::Vector3d m_position;
        RigidBodyDynamics::Math::Matrix3d m_rotation;
        Pose()
        {
            m_position.setZero();
            m_rotation.setIdentity();
        }
    };

    //RigidBodyDynamics::Math::Vector3d tcp_local_point{0.0, 0.0, 0.07};
    
    RigidBodyDynamics::Math::Vector3d tcp_local_point{0.0, 0.0, 0.0};
    BOOL ComputeTcpFK();
    Pose tcpPose;
    const Pose& GetTcpPose() const { return tcpPose; }
    
    
    // for goal pose
    Pose goal_tcpPose;
    Pose fir_tcpPose, fin_tcpPose;;

    // verification
    Pose m_tcpStartPose;
    Pose m_tcpFinalPose;
    Pose m_goalTcpPoseForCheck;

    BOOL m_bIkReady;          // IK 해가 설정됐는지
    BOOL m_bVerifyDone;
    BOOL m_bIkMotionStarted;  // IK 시작 후 실제로 움직임이 감지됐는지
    int  m_nStableCount;

    void CheckIKConvergence();

    
    unsigned int m_body_id;

    const RigidBodyDynamics::Math::VectorNd& GetQRef() const { return     m_Q_ref; }    

    // logging
    std::atomic<bool> 		 m_bIkTrigger{false};

    // Full IK (Jacobian-based)
    RigidBodyDynamics::Math::MatrixNd m_J;       // 6 x DOF Jacobian
    RigidBodyDynamics::Math::MatrixNd m_JJt;     // 6 x 6
    RigidBodyDynamics::Math::MatrixNd m_J_pinv;  // DOF x 6
    RigidBodyDynamics::Math::VectorNd m_e_task;  // 6D Cartesian error [angular; linear]
    double m_Kp_task_pos = 1.0;                  // task-space position gain
    double m_Kp_task_rot = 1.0;                  // task-space orientation gain
    static constexpr double m_dt = 0.001;        // 1kHz → 1ms
    static constexpr double m_lambda = 0.01;     // DLS damping factor

    BOOL ComputeJacobianBasedInverseKinematics(std::vector<double>& avOutputTorque);

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
    RigidBodyDynamics::Math::VectorNd m_Qdd_ref;        // Reference acceleration
    RigidBodyDynamics::Math::VectorNd m_zero_vector;    // Zero vector
    
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
    BOOL ComputeInverseKinematics(std::vector<double>& avOutputTorque);
    
    BOOL IsJointSettled(double vel_threshold);
    void PrintTcpVerificationResult();
    
};

#endif // __CONTROLLER_RT_FULL_DYNAMICS__