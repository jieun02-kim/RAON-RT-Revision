/*****************************************************************************
*	Name: FullDynControllerRT.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of RT-optimized full dynamics controller
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "FullDynControllerRT.h"
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <errno.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
/* [kv260-merge] SSE headers exist only on x86; the FTZ/DAZ intrinsics they
 * serve are already __SSE__-guarded at the call sites (no aarch64 effect). */
#if defined(__SSE__)
#include <xmmintrin.h>
#include <pmmintrin.h>
#endif

// RPY → 회전행렬 (ZYX convention: Rz*Ry*Rx)
RigidBodyDynamics::Math::Matrix3d
CControllerFullDynamicsRT::RPYToRot(double roll, double pitch, double yaw)
{
    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);
    RigidBodyDynamics::Math::Matrix3d R;
    R << cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
         sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
        -sp,     cp*sr,             cp*cr;
    return R;
}

// 회전행렬 → RPY (ZYX convention)
static void RotToRPY(const RigidBodyDynamics::Math::Matrix3d& R,
                     double& roll, double& pitch, double& yaw)
{
    pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    roll  = atan2(R(2,1), R(2,2));
    yaw   = atan2(R(1,0), R(0,0));
}


CControllerFullDynamicsRT::CControllerFullDynamicsRT(const TSTRING& astrURDFPath, unsigned int auDOF)
    : CController(eControllerFullDynamics, auDOF)  // Use MPC type for full dynamics
    , m_strURDFPath(astrURDFPath)
    , m_eControlMode(eGravityCompensation)
    , m_bRTMode(FALSE)
    , m_bRTOptimized(FALSE)
{
    // Initialize performance monitoring
    memset(&m_rt_perf, 0, sizeof(m_rt_perf));
    m_rt_perf.deadline_ns = 500000;  // 500µs deadline for 1kHz control
    
    // Pre-allocate all RBDL vectors
    m_Q.resize(auDOF);
    m_Qd.resize(auDOF);
    m_Qdd.resize(auDOF);
    m_Q_ref.resize(auDOF);
    m_Qd_ref.resize(auDOF);
    m_Qd_ref_prev.resize(auDOF);
    m_Qdd_ref.resize(auDOF);
    m_zero_vector.resize(auDOF);
    m_Q_ik_target.resize(auDOF);

    m_M.resize(auDOF, auDOF);
    m_h.resize(auDOF);
    m_g.resize(auDOF);
    m_c.resize(auDOF);
    
    m_tau_M.resize(auDOF);
    m_tau_feedback.resize(auDOF);
    m_tau_total.resize(auDOF);
    
    m_pos_error.resize(auDOF);
    m_vel_error.resize(auDOF);
    
    // Initialize all vectors to zero
    m_Q.setZero();
    m_Qd.setZero();
    m_Qdd.setZero();
    m_Q_ref.setZero();
    m_Qd_ref.setZero();
    m_Qd_ref_prev.setZero();
    m_Qdd_ref.setZero();
    m_zero_vector.setZero();
    m_Q_ik_target.setZero();

    m_M.setZero();
    m_h.setZero();
    m_g.setZero();
    m_c.setZero();
    
    m_tau_M.setZero();
    m_tau_feedback.setZero();
    m_tau_total.setZero();
    
    m_pos_error.setZero();
    m_vel_error.setZero();
    
    // Initialize control gains
    m_Kp.resize(auDOF, 100.0);  // Default position gains
    m_Kd.resize(auDOF, 10.0);   // Default velocity gains

    // Initialize trajectory state
    m_traj_duration    = 3.0;
    m_traj.T           = m_traj_duration;
    m_traj.t_elapsed   = 0.0;
    m_traj.active      = false;
    m_traj.q_start.resize(auDOF);
    m_traj.q_goal.resize(auDOF);
    m_traj.q_start.setZero();
    m_traj.q_goal.setZero();


    // jieun
    tcpPose.m_position.setZero();
    tcpPose.m_rotation.setZero();
    m_bIkTrigger = FALSE;
    m_bIkReady   = FALSE;
    m_bVerifyDone  = FALSE;
    m_nStableCount = 0;

    // Full IK
    m_J.resize(6, auDOF);
    m_J.setZero();
    m_JJt.resize(6, 6);
    m_JJt.setZero();
    m_J_pinv.resize(auDOF, 6);
    m_J_pinv.setZero();
    m_e_task.resize(6);
    m_e_task.setZero();

    DBG_LOG_INFO("(%s) Constructor completed for %u DOF", "CControllerFullDynamicsRT", auDOF);
}

CControllerFullDynamicsRT::~CControllerFullDynamicsRT()
{
    if (m_bRTMode && m_bRTOptimized) {
        // Unlock memory
        munlockall();
    }

    DBG_LOG_INFO("(%s) Destructor completed", "CControllerFullDynamicsRT");
}

BOOL 
CControllerFullDynamicsRT::Init()
{
    DBG_LOG_INFO("(%s) Initializing controller...", "CControllerFullDynamicsRT");
    
    // Load URDF model
    if (!LoadURDF(m_strURDFPath)) {
        DBG_LOG_ERROR("(%s) Failed to load URDF: %s", "CControllerFullDynamicsRT", m_strURDFPath.c_str());
        return FALSE;
    }
    
    // Set default gravity
    SetGravity(RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81));
    
    // Verify DOF consistency
    if (m_rbdlModel.qdot_size != m_uDOF) {
        DBG_LOG_ERROR("(%s) DOF mismatch: Model=%u, Expected=%u", 
                     "CControllerFullDynamicsRT", m_rbdlModel.qdot_size, m_uDOF);
        return FALSE;
    }
    
    // Initialize RT optimizations if enabled
    if (m_bRTMode) {
        if (!InitRTOptimizations()) {
            DBG_LOG_ERROR("(%s) Failed to initialize RT optimizations", "CControllerFullDynamicsRT");
            return FALSE;
        }
    }
    
    m_bInitialized = TRUE;
    DBG_LOG_INFO("(%s) Controller initialized successfully with %u DOF", 
                 "CControllerFullDynamicsRT", m_uDOF);
    
    // jieun
    m_body_id = m_rbdlModel.GetBodyId("tcp");

    // 기본 목표 위치 — 실수로 다른 모드 진입 시 로봇이 안전한 위치로 이동
    goal_tcpPose.m_position[0] = -0.011930;
    goal_tcpPose.m_position[1] = -0.188431;
    goal_tcpPose.m_position[2] =  1.326710;

    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::Update(const std::vector<double>& avCurrentPos, 
                                      const std::vector<double>& avCurrentVel,
                                      const std::vector<double>& avCurrentTor,
                                      std::vector<double>& avOutputTorque)
{
    if (!m_bInitialized || !m_bEnabled) {
        std::fill(avOutputTorque.begin(), avOutputTorque.end(), 0.0);
        return FALSE;
    }
    
    // Start RT timing
    uint64_t t_start = read_timer();
    
    // Input validation
    if (avCurrentPos.size() != m_uDOF || avCurrentVel.size() != m_uDOF || 
        avOutputTorque.size() != m_uDOF) {
        DBG_LOG_ERROR("(%s) Size mismatch: Pos=%zu, Vel=%zu, Output=%zu, DOF=%u", 
                     "CControllerFullDynamicsRT", avCurrentPos.size(), avCurrentVel.size(), 
                     avOutputTorque.size(), m_uDOF);
        return FALSE;
    }
    
    // Copy current state to RBDL vectors (RT-safe)
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_Q[i] = avCurrentPos[i];
        m_Qd[i] = avCurrentVel[i];
    }

    SetTcpReferencePose();

    // sticky-float anchor is only meaningful while grav-comp is the ACTIVE
    // mode — re-anchor fresh on the next grav-comp entry
    if (m_eControlMode != eGravityCompensation)
        m_bHoldAnchored = false;

    // Compute control based on selected mode
    BOOL result = FALSE;
    switch (m_eControlMode) {
        case eGravityCompensation:
            result = ComputeGravityCompensation(avOutputTorque);
            break;
        case eFullDynamics:
            result = ComputeFullDynamics(avOutputTorque);
            break;
        case eComputedTorque:
            result = ComputeComputedTorque(avOutputTorque);
            break;
        case eInverseKinematics:
            result = ComputeJacobianBasedInverseKinematics(avOutputTorque);
            break;
        case eInverseKinematics_6dof:
            UpdateTrajectory();
            result = ComputeComputedTorque(avOutputTorque);
            break;

        default:
            result = ComputeGravityCompensation(avOutputTorque);
            break;
    }
    
    // RT performance monitoring
    uint64_t t_end = read_timer();
    uint64_t computation_time = t_end - t_start;
    
    // Update performance statistics
    m_rt_perf.total_compute_time_ns += computation_time;
    m_rt_perf.cycle_count++;
    
    if (computation_time > m_rt_perf.max_compute_time_ns) {
        m_rt_perf.max_compute_time_ns = computation_time;
    }
    
    m_rt_perf.avg_compute_time_ns = m_rt_perf.total_compute_time_ns / m_rt_perf.cycle_count;
    
    // Check for RT violations
    if (m_bRTMode) {
        CheckRTViolation(computation_time);
    }
    
    return result;
}

BOOL 
CControllerFullDynamicsRT::ComputeGravityCompensation(std::vector<double>& avOutputTorque)
{
    // Gravity compensation: τ = g(q)
    RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);

    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutputTorque[i] = m_g[i];
    }

    // [kv260-merge] Sticky-float hold (see header). Only when grav-comp is
    // the ACTIVE mode — this function also serves as a fallback for other
    // modes, which must stay pure g(q).
    if (m_eControlMode == eGravityCompensation &&
        m_bStickyEnable.load(std::memory_order_relaxed))
    {
        if (!m_bHoldAnchored)
        {
            m_Q_hold = m_Q;
            m_bHoldAnchored = true;
        }
        for (unsigned int i = 0; i < m_uDOF; ++i)
        {
            if (std::fabs(m_Qd[i]) > HOLD_UNLOCK_QD)
            {
                // clearly being hand-guided — anchor follows, no spring
                m_Q_hold[i] = m_Q[i];
                continue;
            }
            double dErr = m_Q[i] - m_Q_hold[i];
            if (dErr > HOLD_DB_RAD)          // pushed past the dead-band:
                m_Q_hold[i] = m_Q[i] - HOLD_DB_RAD;   // the anchor drags along
            else if (dErr < -HOLD_DB_RAD)
                m_Q_hold[i] = m_Q[i] + HOLD_DB_RAD;
            dErr = m_Q[i] - m_Q_hold[i];
            avOutputTorque[i] += -HOLD_KP_FRAC * m_Kp[i] * dErr
                                 - HOLD_KD_FRAC * m_Kd[i] * m_Qd[i];
        }
    }

    return TRUE;
}

// this is same with CTC, but more explicitly defined 
BOOL 
CControllerFullDynamicsRT::ComputeFullDynamics(std::vector<double>& avOutputTorque)
{
    // Full dynamics control: τ = M(q)q̈_ref + h(q,q̇) + K_feedback

    // Inertia Matrix M(q)
    // return m_M
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, m_Q, m_M);
    
    // Nonlinear Effects h(q,qd) = C(q,qd)*qd + g(q)
    // m_h == tau
    RigidBodyDynamics::NonlinearEffects(m_rbdlModel, m_Q, m_Qd, m_h);
    
    // Gravity Vector g(q) 
    RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);
    
    // Coriolis forces c(q,qd) = h(q,qd) - g(q)
    m_c = m_h - m_g;
    
    // Position and velocity errors
    m_pos_error = m_Q_ref - m_Q;
    m_vel_error = m_Qd_ref - m_Qd;
    
    // Feedforward term: M(q) * q̈_ref
    m_tau_M = m_M * m_Qdd_ref;
    
    // Feedback control
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_tau_feedback[i] = m_Kp[i] * m_pos_error[i] + m_Kd[i] * m_vel_error[i];
    }
    
    m_tau_total = m_tau_M + m_h + m_tau_feedback;
    
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutputTorque[i] = m_tau_total[i];
    }
    
    return TRUE;
}

BOOL CControllerFullDynamicsRT::ComputeComputedTorque(std::vector<double>& avOutputTorque)
{
    // Computed torque control: τ = M(q)[q̈_ref + Kp*e_pos + Kd*e_vel] + h(q,q̇)

    // Compute dynamics
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, m_Q, m_M);
    RigidBodyDynamics::NonlinearEffects(m_rbdlModel, m_Q, m_Qd, m_h);
    
    // Compute errors
    m_pos_error = m_Q_ref - m_Q;
    m_vel_error = m_Qd_ref - m_Qd;

    // Desired acceleration with feedback
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_Qdd[i] = m_Qdd_ref[i] + m_Kp[i] * m_pos_error[i] + m_Kd[i] * m_vel_error[i];
    }
    
    // Computed torque: τ = M(q) * q̈_desired + h(q,q̇)
    m_tau_M = m_M * m_Qdd;
    m_tau_total = m_tau_M + m_h;
    
    // Copy result to output
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutputTorque[i] = m_tau_total[i];
    }
    
    return TRUE;
}

//===================================================================
// jieun


BOOL CControllerFullDynamicsRT::IsJointSettled(double vel_threshold)
{
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (fabs(m_Qd[i]) > vel_threshold) {
            return FALSE;
        }
    }
    return TRUE;
}

BOOL CControllerFullDynamicsRT::IsJointStopped(void)
{
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (fabs(m_Qd[i]) > 0.04) {
            return FALSE;
        }
    }
    return TRUE;
}

// RPY → 회전행렬 (ZYX convention: Rz*Ry*Rx)

void CControllerFullDynamicsRT::PrintTcpVerificationResult()
{
    double dx = m_tcpFinalPose.m_position[0] - m_tcpStartPose.m_position[0];
    double dy = m_tcpFinalPose.m_position[1] - m_tcpStartPose.m_position[1];
    double dz = m_tcpFinalPose.m_position[2] - m_tcpStartPose.m_position[2];

    double ex = m_goalTcpPoseForCheck.m_position[0] - m_tcpFinalPose.m_position[0];
    double ey = m_goalTcpPoseForCheck.m_position[1] - m_tcpFinalPose.m_position[1];
    double ez = m_goalTcpPoseForCheck.m_position[2] - m_tcpFinalPose.m_position[2];

    double err_norm = sqrt(ex*ex + ey*ey + ez*ez);

    double r_start, p_start, y_start;
    double r_goal,  p_goal,  y_goal;
    double r_final, p_final, y_final;
    RotToRPY(m_tcpStartPose.m_rotation,        r_start, p_start, y_start);
    RotToRPY(m_goalTcpPoseForCheck.m_rotation,  r_goal,  p_goal,  y_goal);
    RotToRPY(m_tcpFinalPose.m_rotation,         r_final, p_final, y_final);

    DBG_LOG_INFO("========== TCP Verification ==========");
    DBG_LOG_INFO("TCP Start : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f (rad)",
        m_tcpStartPose.m_position[0], m_tcpStartPose.m_position[1], m_tcpStartPose.m_position[2],
        r_start, p_start, y_start);
    DBG_LOG_INFO("TCP Goal  : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f (rad)",
        m_goalTcpPoseForCheck.m_position[0], m_goalTcpPoseForCheck.m_position[1], m_goalTcpPoseForCheck.m_position[2],
        r_goal, p_goal, y_goal);
    DBG_LOG_INFO("TCP Final : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f (rad)",
        m_tcpFinalPose.m_position[0], m_tcpFinalPose.m_position[1], m_tcpFinalPose.m_position[2],
        r_final, p_final, y_final);
    // 회전 오차: R_err = R_goal * R_final^T → axis-angle (vee map)
    RigidBodyDynamics::Math::Matrix3d R_err =
        m_goalTcpPoseForCheck.m_rotation * m_tcpFinalPose.m_rotation.transpose();
    double er     = 0.5 * (R_err(2,1) - R_err(1,2));
    double ep     = 0.5 * (R_err(0,2) - R_err(2,0));
    double ey_rot = 0.5 * (R_err(1,0) - R_err(0,1));
    double rot_err_norm = sqrt(er*er + ep*ep + ey_rot*ey_rot);

    DBG_LOG_INFO("Delta     : dX=%.6f dY=%.6f dZ=%.6f", dx, dy, dz);
    DBG_LOG_INFO("Error     : eX=%.6f eY=%.6f eZ=%.6f", ex, ey, ez);
    DBG_LOG_INFO("Norm Error: %.6f m", err_norm);
    DBG_LOG_INFO("Rot Error : eR=%.6f eP=%.6f eY=%.6f (rad) | Norm=%.6f", er, ep, ey_rot, rot_err_norm);
    DBG_LOG_INFO("======================================");

    // CSV에 오차 기록 (rt_ik_error_log/ 폴더에 누적 append)
    const char* log_dir  = "rt_ik_error_log";
    const char* csv_path = "rt_ik_error_log/ik_accuracy_log.csv";
    mkdir(log_dir, 0755);  // 폴더 없으면 생성

    bool write_header = false;
    {
        std::ifstream check(csv_path);
        write_header = !check.good();
    }
    std::ofstream csv(csv_path, std::ios::app);
    if (csv.is_open())
    {
        if (write_header)
            csv << "goal_x,goal_y,goal_z,"
                << "final_x,final_y,final_z,"
                << "err_x,err_y,err_z,norm_err,"
                << "goal_r,goal_p,goal_y,"
                << "final_r,final_p,final_y,"
                << "err_r,err_p,err_y,rot_norm_err\n";
        csv << m_goalTcpPoseForCheck.m_position[0] << ","
            << m_goalTcpPoseForCheck.m_position[1] << ","
            << m_goalTcpPoseForCheck.m_position[2] << ","
            << m_tcpFinalPose.m_position[0] << ","
            << m_tcpFinalPose.m_position[1] << ","
            << m_tcpFinalPose.m_position[2] << ","
            << ex << "," << ey << "," << ez << "," << err_norm << ","
            << r_goal << "," << p_goal << "," << y_goal << ","
            << r_final << "," << p_final << "," << y_final << ","
            << er << "," << ep << "," << ey_rot << "," << rot_err_norm << "\n";
    }
}


void
CControllerFullDynamicsRT::CheckIKConvergence()
{
    if (m_bVerifyDone) return;

    // 한 번이라도 움직임 감지 (관절 속도 > 0.02 rad/s)
    if (!m_bIkMotionStarted && !IsJointSettled(0.02))
        m_bIkMotionStarted = TRUE;

    //움직임 감지 후 속도 조건을 5ms 연속 유지해야 정지 판정
    if (m_bIkMotionStarted) {
        if (IsJointSettled(0.01))
            m_nStableCount++;
        else
            m_nStableCount = 0;

        if (m_nStableCount >= 5000) {
            ComputeTcpFK();
            m_tcpFinalPose = tcpPose;
            PrintTcpVerificationResult();
            m_bVerifyDone = TRUE;
        }
    }


    // if(m_bIkMotionStarted){
    //     if(IsJointStopped())
    //        m_nStableCount++;
    //     else
    //         m_nStableCount = 0;
    //     if (m_nStableCount >= 2000) {
    //         ComputeTcpFK();
    //         m_tcpFinalPose = tcpPose;
    //         PrintTcpVerificationResult();
    //         m_bVerifyDone = TRUE;
    //     }

    //     }
}


BOOL 
CControllerFullDynamicsRT::SetTcpReferencePose()
{
    if(m_bSetRefPoseTrigger==TRUE)
    {
        ComputeTcpFK();
        double r_set, p_set, y_set;
        RotToRPY(tcpPose.m_rotation, r_set, p_set, y_set);

        DBG_LOG_INFO("========== Current TCP ==========");
        DBG_LOG_INFO("TCP Setting : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f (rad)",
        tcpPose.m_position[0], tcpPose.m_position[1], tcpPose.m_position[2],
        r_set, p_set, y_set);
        
        goal_tcpPose = tcpPose;

        DBG_LOG_INFO("========== Setting TCP  ==========");
        m_bSetRefPoseTrigger = FALSE;

    }
    return TRUE;
}








BOOL
CControllerFullDynamicsRT::GetCurrentPose(Pose& astCurrPose)
{
    RigidBodyDynamics::Math::Vector3d p_tcp;
    RigidBodyDynamics::Math::Matrix3d R_base_to_body, R_body_to_base;

    // tcp position 
    p_tcp = RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, m_Q, m_body_id, tcp_local_point);
    
    // tcp orientation - (R_base_to_body : base -> body) 
    R_base_to_body = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, m_Q, m_body_id);
    R_body_to_base = R_base_to_body.transpose();
    
    // base coordinates
    astCurrPose.m_position = p_tcp;
    astCurrPose.m_rotation = R_body_to_base;
   
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::SetTargetPose(Pose astTargetPose)
{
    RigidBodyDynamics::Math::VectorNd vjointspace = m_Q;
    RigidBodyDynamics::InverseKinematicsConstraintSet CS;
    CS.num_steps      = 1000;   // default 100 → 수렴 기회 늘리기
    CS.step_tol       = 1.0e-10;
    CS.constraint_tol = 1.0e-8;
    CS.AddFullConstraint(m_body_id, tcp_local_point, astTargetPose.m_position, astTargetPose.m_rotation.transpose());
    BOOL is_ok = RigidBodyDynamics::InverseKinematics(m_rbdlModel, m_Q, CS, vjointspace);
    if (!is_ok)
    {
        DBG_LOG_WARN("(SetTargetPose) IK failed!");
        return FALSE;
    }

    m_traj.q_start   = m_Q;
    m_traj.q_goal    = vjointspace;
    m_traj.T         = m_traj_duration;
    m_traj.t_elapsed = 0.0;
    m_traj.active    = true;
    goal_tcpPose     = astTargetPose;
    DBG_LOG_INFO("(SetTargetPose) Trajectory start, T=%.2f s", m_traj.T);

    return TRUE;
}

RigidBodyDynamics::Math::Matrix3d
CControllerFullDynamicsRT::ToolRotAt(const RigidBodyDynamics::Math::VectorNd& aq)
{
    // CalcBodyWorldOrientation returns base->body; Pose::m_rotation is
    // body->base (SetTargetPosePositionOnly transposes it back for RBDL).
    return RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, aq,
                                                       m_body_id).transpose();
}

BOOL
CControllerFullDynamicsRT::SetTargetPosePositionOnly(Pose astTargetPose,
                                                     double adOriWeight,
                                                     BOOL abQuiet)
{
    RigidBodyDynamics::Math::VectorNd vjointspace = m_Q;
    RigidBodyDynamics::InverseKinematicsConstraintSet CS;
    // abQuiet marks an exploratory refine rung: cap its Newton budget for the
    // same reason SolveReadyIK caps its ladder — a stalling solve that runs to
    // 1000 steps costs ~8 ms inside the 1 kHz cycle. The final rung is the one
    // whose verdict counts, so it keeps the full budget.
    CS.num_steps      = abQuiet ? (unsigned int)IK_PROBE_STEPS : 1000u;
    CS.step_tol       = 1.0e-10;
    CS.constraint_tol = 1.0e-8;
    // [kv260-merge] E13 (2026-07-27 table strike): a point-only constraint
    // leaves a 3-dim nullspace, so RBDL was free to return a configuration
    // far from m_Q (elbow flip / wrist unwind) and the joint-space quintic
    // then swept an arbitrary Cartesian arc through the table. Position is
    // hard; the caller's orientation (m_rotation is body->base, RBDL wants
    // base->body) enters as a SOFT regularizer that keeps the solution on
    // the current branch without making it hypersensitive to the start pose.
    CS.AddPointConstraint(m_body_id, tcp_local_point, astTargetPose.m_position);
    if (adOriWeight > 0.0)
        CS.AddOrientationConstraint(m_body_id, astTargetPose.m_rotation.transpose(),
                                    (float)adOriWeight);
    (void)RigidBodyDynamics::InverseKinematics(m_rbdlModel, m_Q, CS, vjointspace);
    // 2π fold: a "wound" return (J3 at 10.42 rad) is the in-range solution in
    // disguise — folding by 2π steps leaves the FK pose EXACTLY unchanged.
    // Validity is judged on the PHYSICAL (zero-folded) posture; execution is
    // then re-expressed in the LIVE counter frame (the counters themselves
    // can be wound whole turns — E17/E-stop legacy) so the quintic never
    // commands real extra turns.
    {
        RigidBodyDynamics::Math::VectorNd vqPhys = vjointspace;
        if (!CheckLimitsPhysical(vqPhys))
        {
            if (!abQuiet)
                DBG_LOG_ERROR("(SetTargetPosePositionOnly) REFUSED: solution "
                              "outside joint limits even after 2π fold");
            return FALSE;
        }
        vjointspace = vqPhys;
    }
    FoldTowardRef(vjointspace, m_Q, m_uDOF);
    // Soft residual keeps RBDL's flag false — judge by the FK position error.
    const RigidBodyDynamics::Math::Vector3d vPosSol =
        RigidBodyDynamics::CalcBodyToBaseCoordinates(
            m_rbdlModel, vjointspace, m_body_id, tcp_local_point);
    const double dPosErr = (vPosSol - astTargetPose.m_position).norm();
    if (dPosErr > IK_POS_TOL_M)
    {
        if (!abQuiet)
            DBG_LOG_WARN("(SetTargetPosePositionOnly) IK failed — residual "
                         "%.1f mm > %.1f mm", dPosErr * 1e3, IK_POS_TOL_M * 1e3);
        return FALSE;
    }

    double dDqMax = 0.0;
    int    nDqAxis = -1;
    for (unsigned int i = 0; i < m_uDOF; i++)
    {
        const double dDq = std::fabs(vjointspace[i] - m_Q[i]);
        if (dDq > dDqMax) { dDqMax = dDq; nDqAxis = (int)i; }
    }
    if (dDqMax > IK_DQ_MAX_RAD)
    {
        if (!abQuiet)
            DBG_LOG_ERROR("(SetTargetPosePositionOnly) REFUSED: J%d jumps %.2f rad "
                          "(> %.1f) — IK branch flip. Reposition the arm (grav-comp) "
                          "closer to the target posture and retry.",
                          nDqAxis, dDqMax, IK_DQ_MAX_RAD);
        return FALSE;
    }

    double dT = m_traj_duration;
    const double dT_need = 1.875 * dDqMax / IK_QD_PEAK_RADPS;  // quintic peak vel
    if (dT_need > dT)
    {
        dT = (dT_need < IK_T_MAX_S) ? dT_need : IK_T_MAX_S;
        DBG_LOG_INFO("(SetTargetPosePositionOnly) T stretched %.2f → %.2f s "
                     "(dq_max %.2f rad)", m_traj_duration, dT, dDqMax);
    }

    // how far the soft constraint let the orientation drift (info only)
    const RigidBodyDynamics::Math::Matrix3d E_sol =
        RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, vjointspace,
                                                    m_body_id);
    const RigidBodyDynamics::Math::Matrix3d R_err = E_sol * astTargetPose.m_rotation;
    double dCosA = (R_err.trace() - 1.0) / 2.0;
    if (dCosA > 1.0) dCosA = 1.0; else if (dCosA < -1.0) dCosA = -1.0;
    const double dOriDevDeg = std::acos(dCosA) * 180.0 / M_PI;

    m_traj.q_start   = m_Q;
    m_traj.q_goal    = vjointspace;
    m_traj.T         = dT;
    m_traj.t_elapsed = 0.0;
    m_traj.active    = true;
    goal_tcpPose     = astTargetPose;
    DBG_LOG_INFO("(SetTargetPosePositionOnly) Trajectory start, T=%.2f s, "
                 "dq_max %.2f rad (J%d), w=%.2f, R held within %.1f deg",
                 m_traj.T, dDqMax, nDqAxis, adOriWeight, dOriDevDeg);

    return TRUE;
}

BOOL
CControllerFullDynamicsRT::StartJointTrajectory(
    const RigidBodyDynamics::Math::VectorNd& q_goal, double T)
{
    m_traj.q_start   = m_Q;
    m_traj.q_goal    = q_goal;
    m_traj.T         = T;
    m_traj.t_elapsed = 0.0;
    m_traj.active    = true;
    DBG_LOG_INFO("(StartJointTrajectory) T=%.2f s", T);
    return TRUE;
}

//============================================================================
// [kv260-merge] Deterministic ready-seed IK — see header for the rationale.
// URDF joint limits (indy7.urdf <limit> tags): J1-J5 ±175°, J6 ±215°.
// RBDL's IK iteration is limit-blind, so the check lives here.
//============================================================================
static const double s_adJointLo[6] = {-3.0543261909900767, -3.0543261909900767,
                                      -3.0543261909900767, -3.0543261909900767,
                                      -3.0543261909900767, -3.7524578917878086};
static const double s_adJointHi[6] = { 3.0543261909900767,  3.0543261909900767,
                                       3.0543261909900767,  3.0543261909900767,
                                       3.0543261909900767,  3.7524578917878086};

double
CControllerFullDynamicsRT::ScaledTrajTime(double adDqMax, double adTBase)
{
    double dT = adTBase;
    const double dT_need = 1.875 * adDqMax / IK_QD_PEAK_RADPS;  // quintic peak
    if (dT_need > dT)
        dT = (dT_need < IK_T_MAX_S) ? dT_need : IK_T_MAX_S;
    return dT;
}

void
CControllerFullDynamicsRT::FoldTowardRef(
    RigidBodyDynamics::Math::VectorNd& aq,
    const RigidBodyDynamics::Math::VectorNd& aqRef,
    unsigned int auDof)
{
    for (unsigned int i = 0; i < auDof && (int)i < aq.size() &&
                             (int)i < aqRef.size(); i++)
        aq[i] -= 2.0 * M_PI * std::round((aq[i] - aqRef[i]) / (2.0 * M_PI));
}

bool
CControllerFullDynamicsRT::CheckLimitsPhysical(
    RigidBodyDynamics::Math::VectorNd& aq) const
{
    bool bOK = true;
    for (unsigned int i = 0; i < m_uDOF && i < 6; i++)
    {
        double dQ = aq[i] - 2.0 * M_PI *
                    std::round(aq[i] / (2.0 * M_PI));       // fold toward 0
        if (dQ < s_adJointLo[i] && dQ + 2.0 * M_PI <= s_adJointHi[i])
            dQ += 2.0 * M_PI;
        else if (dQ > s_adJointHi[i] && dQ - 2.0 * M_PI >= s_adJointLo[i])
            dQ -= 2.0 * M_PI;
        aq[i] = dQ;
        if (dQ < s_adJointLo[i] || dQ > s_adJointHi[i]) bOK = false;
    }
    return bOK;
}

BOOL
CControllerFullDynamicsRT::SolvePositionIK(
    const RigidBodyDynamics::Math::VectorNd& aqSeed,
    const RigidBodyDynamics::Math::Vector3d& avTarget,
    double adOriWeight,
    const RigidBodyDynamics::Math::Matrix3d* apEOri,
    RigidBodyDynamics::Math::VectorNd& aqSol,
    double& adPosErrM,
    int anMaxSteps)
{
    aqSol = aqSeed;
    RigidBodyDynamics::InverseKinematicsConstraintSet CS;
    CS.num_steps      = (unsigned int)anMaxSteps;
    CS.step_tol       = 1.0e-10;
    CS.constraint_tol = 1.0e-8;
    CS.AddPointConstraint(m_body_id, tcp_local_point, avTarget);
    if (adOriWeight > 0.0 && apEOri != NULL)
        CS.AddOrientationConstraint(m_body_id, *apEOri, (float)adOriWeight);
    (void)RigidBodyDynamics::InverseKinematics(m_rbdlModel, aqSeed, CS, aqSol);
    const RigidBodyDynamics::Math::Vector3d vPosSol =
        RigidBodyDynamics::CalcBodyToBaseCoordinates(
            m_rbdlModel, aqSol, m_body_id, tcp_local_point);
    adPosErrM = (vPosSol - avTarget).norm();
    return (adPosErrM <= IK_POS_TOL_M) ? TRUE : FALSE;
}

BOOL
CControllerFullDynamicsRT::ComputeReadySeed(
    const std::vector<RigidBodyDynamics::Math::Vector3d>& avProbes,
    const RigidBodyDynamics::Math::Vector3d& avCenter,
    const RigidBodyDynamics::Math::VectorNd* apAnchorSeed)
{
    // Non-RT: runs once at init, before the control threads start.
    // ANCHOR-ONLY (v3): q_ready is only ever an operator-demonstrated
    // posture — either the persisted file (a previous session's recorded
    // HOME) here, or a live SetReadyAnchor at record time. The synthetic
    // seed ladder was removed after producing two contorted postures.
    (void)avCenter;
    m_vProbeStore = avProbes;   // kept for record-time anchor verification
    m_bReadySet   = false;

    if (apAnchorSeed == NULL || (unsigned int)apAnchorSeed->size() < m_uDOF)
    {
        DBG_LOG_WARN("[SEED] no operator seed on file — approaches use "
                     "live-posture seeding until HOME is recorded "
                     "('p' menu, last entry)");
        return FALSE;
    }

    RigidBodyDynamics::Math::VectorNd vq(m_uDOF);
    for (unsigned int i = 0; i < m_uDOF; i++) vq[i] = (*apAnchorSeed)[i];
    const RigidBodyDynamics::Math::VectorNd vqRaw = vq;
    if (!CheckLimitsPhysical(vq))
    {
        DBG_LOG_WARN("[SEED] persisted seed invalid (outside limits after "
                     "fold) — ignoring; record HOME to re-create");
        return FALSE;
    }
    for (unsigned int i = 0; i < m_uDOF; i++)
    {
        const int nTurns = (int)std::round((vqRaw[i] - vq[i]) / (2.0 * M_PI));
        if (nTurns != 0)
            DBG_LOG_INFO("[SEED] note: J%d in the seed file was wound %+d "
                         "turn(s) (E17/E-stop counter legacy) — folded to the "
                         "physical posture", (int)i, nTurns);
    }

    m_QReady    = vq;
    m_EReady    = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel,
                                                              m_QReady,
                                                              m_body_id);
    m_bReadySet = true;

    // Inline coverage verification (init is non-RT — a full pass is fine
    // here; record-time re-verification is amortized in TickAnchorVerify).
    int nPass = 0;
    for (size_t p = 0; p < avProbes.size(); p++)
    {
        RigidBodyDynamics::Math::VectorNd vqSol;
        double dErr = 0.0;
        if (!SolvePositionIK(m_QReady, avProbes[p], IK_ORI_WEIGHT, &m_EReady,
                             vqSol, dErr))
            continue;
        if (!CheckLimitsPhysical(vqSol))
            continue;
        double dDqMax = 0.0;
        for (unsigned int i = 0; i < m_uDOF; i++)
        {
            const double dDq = std::fabs(vqSol[i] - m_QReady[i]);
            if (dDq > dDqMax) dDqMax = dDq;
        }
        if (dDqMax <= IK_DQ_MAX_RAD) nPass++;
    }

    DBG_LOG_INFO("[SEED] ready posture q = [%.2f %.2f %.2f %.2f %.2f %.2f] "
                 "rad [operator anchor] — %d/%d workspace probes reachable",
                 m_QReady[0], m_QReady[1], m_QReady[2],
                 m_QReady[3], m_QReady[4], m_QReady[5],
                 nPass, (int)avProbes.size());
    if (nPass < (int)avProbes.size())
        DBG_LOG_WARN("[SEED] %d probe(s) near the box edge unreachable — "
                     "fine unless objects sit there",
                     (int)avProbes.size() - nPass);
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::SetReadyAnchor(
    const RigidBodyDynamics::Math::VectorNd& aqAnchor)
{
    if ((unsigned int)aqAnchor.size() < m_uDOF) return FALSE;
    RigidBodyDynamics::Math::VectorNd vq(m_uDOF);
    for (unsigned int i = 0; i < m_uDOF; i++) vq[i] = aqAnchor[i];
    const RigidBodyDynamics::Math::VectorNd vqRaw = vq;
    // canonicalize: the LIVE counters may be wound whole turns (E17/E-stop
    // legacy — J1 -6.2 / J4 -18.9 rad observed with the arm physically near
    // zero), so the anchor is stored as the PHYSICAL posture.
    if (!CheckLimitsPhysical(vq))
    {
        DBG_LOG_WARN("[SEED] anchor posture outside joint limits — keeping "
                     "the previous seed");
        return FALSE;
    }
    for (unsigned int i = 0; i < m_uDOF; i++)
    {
        const int nTurns = (int)std::round((vqRaw[i] - vq[i]) / (2.0 * M_PI));
        if (nTurns != 0)
            DBG_LOG_INFO("[SEED] note: J%d counter is wound %+d turn(s) "
                         "(E17/E-stop legacy) — folded; motions stay in the "
                         "live counter frame", (int)i, nTurns);
    }
    m_QReady    = vq;
    m_EReady    = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel,
                                                              m_QReady,
                                                              m_body_id);
    m_bReadySet = true;
    DBG_LOG_INFO("[SEED] ready anchor re-based to operator posture "
                 "q = [%.2f %.2f %.2f %.2f %.2f %.2f] — approaches solve "
                 "from here",
                 m_QReady[0], m_QReady[1], m_QReady[2],
                 m_QReady[3], m_QReady[4], m_QReady[5]);
    if (!m_vProbeStore.empty())
    {
        m_nAnchorVerifyIdx  = 0;      // amortized coverage check starts now
        m_nAnchorVerifyPass = 0;
    }
    return TRUE;
}

void
CControllerFullDynamicsRT::TickAnchorVerify()
{
    if (m_nAnchorVerifyIdx < 0 || !m_bReadySet) return;
    if (m_nAnchorVerifyIdx >= (int)m_vProbeStore.size())
    {
        m_nAnchorVerifyIdx = -1;
        return;
    }

    const RigidBodyDynamics::Math::Vector3d vProbe =
        m_vProbeStore[m_nAnchorVerifyIdx];
    RigidBodyDynamics::Math::VectorNd vqSol;
    double dErr = 0.0;
    bool   bOK  = false;
    if (SolvePositionIK(m_QReady, vProbe, IK_ORI_WEIGHT, &m_EReady,
                        vqSol, dErr) &&
        CheckLimitsPhysical(vqSol))
    {
        double dDqMax = 0.0;
        for (unsigned int i = 0; i < m_uDOF; i++)
        {
            const double dDq = std::fabs(vqSol[i] - m_QReady[i]);
            if (dDq > dDqMax) dDqMax = dDq;
        }
        bOK = (dDqMax <= IK_DQ_MAX_RAD);
    }
    if (bOK) m_nAnchorVerifyPass++;
    else
        DBG_LOG_WARN("[SEED] probe (%.2f, %.2f, %.2f) unreachable from the "
                     "recorded posture", vProbe[0], vProbe[1], vProbe[2]);

    m_nAnchorVerifyIdx++;
    if (m_nAnchorVerifyIdx >= (int)m_vProbeStore.size())
    {
        DBG_LOG_INFO("[SEED] anchor coverage: %d/%d workspace probes "
                     "reachable from the recorded posture (misses are box "
                     "corners — fine unless objects sit there)",
                     m_nAnchorVerifyPass, (int)m_vProbeStore.size());
        m_nAnchorVerifyIdx = -1;
    }
}

double
CControllerFullDynamicsRT::AttitudeDevDeg(
    const RigidBodyDynamics::Math::VectorNd& aq)
{
    const RigidBodyDynamics::Math::Matrix3d E_sol =
        RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, aq, m_body_id);
    const RigidBodyDynamics::Math::Matrix3d R_err = E_sol * m_EReady.transpose();
    double dCosA = (R_err.trace() - 1.0) / 2.0;
    if (dCosA > 1.0) dCosA = 1.0; else if (dCosA < -1.0) dCosA = -1.0;
    return std::acos(dCosA) * 180.0 / M_PI;
}

BOOL
CControllerFullDynamicsRT::SolveReadyIK(
    const RigidBodyDynamics::Math::Vector3d& avTarget,
    RigidBodyDynamics::Math::VectorNd& aqSol,
    double& adPosErrM, double& adDqMaxFromCur, int& anDqAxis)
{
    if (!m_bReadySet) return FALSE;

    const uint64_t tIkStart = read_timer();

    // E25: walk the orientation weight DOWN a ladder instead of dropping it.
    // E23 tried IK_ORI_WEIGHT and then jumped straight to 0, so any target the
    // soft constraint could not satisfy fell through to a pure-position solve
    // whose attitude is an accident of the descent path. That is not a reach
    // limit: banana at 0.877 m from the base origin was refused at 87 deg
    // while the FARTHER mustard at 0.912 m passed at 36 deg. Each rung keeps
    // as much of the ready attitude as the reach still allows.
    static const double adLadder[] = { IK_ORI_WEIGHT, 0.1, 0.03, 0.01 };
    static const int    nLadder    = (int)(sizeof(adLadder) / sizeof(adLadder[0]));

    double dWUsed  = -1.0;
    int    nSolves = 0;
    // Warm start: a failed rung stops where the position and attitude
    // gradients balance. That point is far closer to the next (weaker) rung's
    // answer than q_ready is, and it was itself reached FROM q_ready, so the
    // branch — the whole reason q_ready exists — is preserved and the chain
    // stays deterministic. Combined with the IK_PROBE_STEPS cap this is what
    // keeps a descending ladder from costing 32 ms.
    RigidBodyDynamics::Math::VectorNd vqSeed = m_QReady;
    for (int i = 0; i < nLadder; i++)
    {
        nSolves++;
        if (SolvePositionIK(vqSeed, avTarget, adLadder[i], &m_EReady,
                            aqSol, adPosErrM, IK_PROBE_STEPS))
        { dWUsed = adLadder[i]; break; }
        vqSeed = aqSol;             // stalled iterate seeds the next rung
    }

    if (dWUsed < 0.0)
    {
        // Bottom rung: position only, attitude unconstrained. This one has to
        // answer "is it reachable at all", so it gets the full Newton budget.
        nSolves++;
        if (!SolvePositionIK(vqSeed, avTarget, 0.0, NULL, aqSol, adPosErrM))
        {
            DBG_LOG_WARN("(SolveReadyIK) no solution — residual %.1f mm even "
                         "position-only (genuinely out of reach)",
                         adPosErrM * 1e3);
            return FALSE;
        }
        dWUsed = 0.0;

        // E25 polish: position is satisfied now, but the attitude is wherever
        // the descent happened to stop. Re-solve FROM THAT SOLUTION with a
        // light weight — the point constraint is already met, so the solver
        // has only the nullspace left and spends it pulling the tool back
        // toward the ready attitude. The strongest weight that still holds
        // position wins; the unpolished solution is kept if none does, or if
        // the polish does not actually improve the deviation.
        double dDevBest = AttitudeDevDeg(aqSol);
        for (int k = 1; k < nLadder; k++)
        {
            RigidBodyDynamics::Math::VectorNd vqPol = aqSol;
            double dErrPol = 0.0;
            nSolves++;
            if (!SolvePositionIK(aqSol, avTarget, adLadder[k], &m_EReady,
                                 vqPol, dErrPol, IK_PROBE_STEPS))
                continue;                   // this weight loses the position
            const double dDevPol = AttitudeDevDeg(vqPol);
            if (dDevPol < dDevBest)
            {
                DBG_LOG_INFO("(SolveReadyIK) polish w=%.2f pulled the tool "
                             "%.0f -> %.0f deg (pos %.1f mm)",
                             adLadder[k], dDevBest, dDevPol, dErrPol * 1e3);
                aqSol     = vqPol;
                adPosErrM = dErrPol;
                dDevBest  = dDevPol;
            }
            break;      // first weight that holds position is the strongest
        }
    }

    // The solves all run inside the 1 kHz control cycle. Measure rather than
    // guess: if this ever exceeds the budget the ladder has to be amortized
    // one rung per cycle, the way the anchor coverage check already is.
    const double dIkMs = (double)(read_timer() - tIkStart) / 1.0e6;
    if (dIkMs > 0.8)
        DBG_LOG_WARN("(SolveReadyIK) %d IK solve(s) took %.2f ms — over the "
                     "1 kHz cycle budget, amortize the ladder", nSolves, dIkMs);
    else
        DBG_LOG_INFO("(SolveReadyIK) ladder settled at w=%.2f after %d solve(s), "
                     "%.2f ms", dWUsed, nSolves, dIkMs);

    if (!CheckLimitsPhysical(aqSol))    // validity on the PHYSICAL posture
    {
        DBG_LOG_WARN("(SolveReadyIK) solution violates joint limits — refused");
        return FALSE;
    }
    if (dWUsed < (double)IK_ORI_WEIGHT)     // any rung below nominal is relaxed
    {
        const double dDevDeg = AttitudeDevDeg(aqSol);
        if (dDevDeg > APPROACH_RDEV_MAX_DEG)
        {
            DBG_LOG_WARN("(SolveReadyIK) reachable only with extreme tool "
                         "tilt (%.0f deg > %.0f) — refused; move the object "
                         "closer", dDevDeg, APPROACH_RDEV_MAX_DEG);
            return FALSE;
        }
        DBG_LOG_INFO("(SolveReadyIK) far-reach: w relaxed to %.2f, tool tilts "
                     "%.0f deg from the ready attitude", dWUsed, dDevDeg);
    }
    // Execution frame: fold to the lap nearest the LIVE counters so the
    // commanded travel is the true physical delta (never extra turns).
    FoldTowardRef(aqSol, m_Q, m_uDOF);
    adDqMaxFromCur = 0.0;
    anDqAxis      = -1;
    for (unsigned int i = 0; i < m_uDOF; i++)
    {
        const double dDq = std::fabs(aqSol[i] - m_Q[i]);
        if (dDq > adDqMaxFromCur) { adDqMaxFromCur = dDq; anDqAxis = (int)i; }
    }
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::LogDistanceError(Pose astTargetPose)
{
    RigidBodyDynamics::Math::Vector3d current_tcp, current_rpy;
    RigidBodyDynamics::Math::Matrix3d R_base_to_body;

    current_tcp = RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, m_Q, m_body_id, tcp_local_point);
    R_base_to_body = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, m_Q, m_body_id);

    RotToRPY(R_base_to_body.transpose(), current_rpy[0], current_rpy[1], current_rpy[2]);
    RotToRPY(astTargetPose.m_rotation, astTargetPose.m_rotation_rpy[0], astTargetPose.m_rotation_rpy[1], astTargetPose.m_rotation_rpy[2]);

    // 오차 계산
    double ex  = astTargetPose.m_position[0]     - current_tcp[0];
    double ey  = astTargetPose.m_position[1]     - current_tcp[1];
    double ez  = astTargetPose.m_position[2]     - current_tcp[2];
    double er  = astTargetPose.m_rotation_rpy[0] - current_rpy[0];
    double ep  = astTargetPose.m_rotation_rpy[1] - current_rpy[1];
    double eyw = astTargetPose.m_rotation_rpy[2] - current_rpy[2];

    double pos_norm = std::sqrt(ex*ex + ey*ey + ez*ez);
    double rot_norm = std::sqrt(er*er + ep*ep + eyw*eyw);

    DBG_LOG_INFO("========== DistanceError ==========");
    DBG_LOG_INFO("Target_pos    : X=%.4f Y=%.4f Z=%.4f",
        astTargetPose.m_position[0], astTargetPose.m_position[1], astTargetPose.m_position[2]);
    DBG_LOG_INFO("Target_rpy|raw    : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f",
        astTargetPose.m_rotation_rpy[0], astTargetPose.m_rotation_rpy[1], astTargetPose.m_rotation_rpy[2],
        astTargetPose.m_rotation(0,0), astTargetPose.m_rotation(1,0), astTargetPose.m_rotation(2,0));
       
    DBG_LOG_INFO("Actual    : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f",
        current_tcp[0], current_tcp[1], current_tcp[2],
        current_rpy[0], current_rpy[1], current_rpy[2]);
    DBG_LOG_INFO("Error     : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f",
        ex, ey, ez, er, ep, eyw);
    DBG_LOG_INFO("Norm      : Pos=%.4f m\nRot=%.4f rad (%.2f deg)",
        pos_norm, rot_norm, rot_norm * 180.0 / M_PI);
    
    return TRUE;

}

void
CControllerFullDynamicsRT::UpdateTrajectory()
{
    if (!m_traj.active) return;

    double s = m_traj.t_elapsed / m_traj.T;
    if (s > 1.0) s = 1.0;
    double s2 = s*s, s3 = s2*s, s4 = s3*s, s5 = s4*s;

    // 5th-order polynomial: q(s) = q0 + dq*(10s³ - 15s⁴ + 6s⁵)
    double p   =  10.0*s3 - 15.0*s4 +  6.0*s5;
    double pd  = (30.0*s2 - 60.0*s3 + 30.0*s4) / m_traj.T;
    double pdd = (60.0*s  - 180.0*s2 + 120.0*s3) / (m_traj.T * m_traj.T);

    for (unsigned int i = 0; i < m_uDOF; ++i) {
        double dq    = m_traj.q_goal[i] - m_traj.q_start[i];
        m_Q_ref[i]   = m_traj.q_start[i] + dq * p;
        m_Qd_ref[i]  = dq * pd;
        m_Qdd_ref[i] = dq * pdd;
    }

    m_traj.t_elapsed += m_dt;

    if (m_traj.t_elapsed >= m_traj.T) {
        // settling phase: q_goal을 향해 MAX_REF_LEAD로 클램핑
        m_Q_ref   = m_traj.q_goal;
        m_Qd_ref  = m_zero_vector;
        m_Qdd_ref = m_zero_vector;

        static constexpr double MAX_REF_LEAD = 0.15;  // rad
        bool settled = true;
        for (unsigned int i = 0; i < m_uDOF; ++i) {
            double lead = m_Q_ref[i] - m_Q[i];
            if (fabs(lead) > MAX_REF_LEAD) {
                m_Q_ref[i] = m_Q[i] + (lead > 0 ? MAX_REF_LEAD : -MAX_REF_LEAD);
                settled = false;
            }
        }

        if (settled) {
            m_traj.active = false;
            DBG_LOG_INFO("(Trajectory) Done. Use 'D' to check error.");
        }
    }
}

BOOL
CControllerFullDynamicsRT::SetTargetPose_Jacobian()
{
    // 2. 6D Jacobian 계산 [angular(0-2); linear(3-5)] × DOF
    m_J.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(m_rbdlModel, m_Q, m_body_id, tcp_local_point, m_J);

    // 3. Cartesian 오차 계산 (RBDL Jacobian 순서에 맞춰 [angular; linear])
    // 위치 오차
    m_e_task[3] = goal_tcpPose.m_position[0] - tcpPose.m_position[0];
    m_e_task[4] = goal_tcpPose.m_position[1] - tcpPose.m_position[1];
    m_e_task[5] = goal_tcpPose.m_position[2] - tcpPose.m_position[2];

    // 자세 오차: TCP z축 방향 정렬만 수행 (roll 무시)
    // e_rot = curZ × goalZ — z축을 goalZ로 회전시키는 각속도 방향, magnitude = sin(θ)
    {
        double pos_err_raw = sqrt(m_e_task[3]*m_e_task[3] +
                                  m_e_task[4]*m_e_task[4] +
                                  m_e_task[5]*m_e_task[5]);
        if (pos_err_raw < 0.08) {
            RigidBodyDynamics::Math::Vector3d curZ  = tcpPose.m_rotation.col(2);
            RigidBodyDynamics::Math::Vector3d goalZ = goal_tcpPose.m_rotation.col(2);
            RigidBodyDynamics::Math::Vector3d e_rot = curZ.cross(goalZ);
            m_e_task[0] = m_Kp_task_rot * e_rot[0];
            m_e_task[1] = m_Kp_task_rot * e_rot[1];
            m_e_task[2] = m_Kp_task_rot * e_rot[2];
        } else {
            m_e_task[0] = 0.0;
            m_e_task[1] = 0.0;
            m_e_task[2] = 0.0;
        }
    }

    m_e_task[3] *= m_Kp_task_pos;
    m_e_task[4] *= m_Kp_task_pos;
    m_e_task[5] *= m_Kp_task_pos;

    // 각속도 클램핑 (rad/s)
    {
        double rot_mag = sqrt(m_e_task[0]*m_e_task[0] +
                              m_e_task[1]*m_e_task[1] +
                              m_e_task[2]*m_e_task[2]);
        const double MAX_ROT_VEL = 0.30; // rad/s
        if (rot_mag > MAX_ROT_VEL) {
            double s = MAX_ROT_VEL / rot_mag;
            m_e_task[0] *= s;
            m_e_task[1] *= s;
            m_e_task[2] *= s;
        }
    }

    // 선속도 클램핑 제거 — 관절 속도 한계(MAX_JOINT_VEL)로만 제한

    // 6. q_ref 적분용 위치 오차 계산 (gain 적용 전 raw 오차)
    double pos_err_now = sqrt(m_e_task[3]*m_e_task[3] +
                              m_e_task[4]*m_e_task[4] +
                              m_e_task[5]*m_e_task[5]) / m_Kp_task_pos;

    // 4. DLS pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹
    // manipulability 기반 adaptive damping
    m_JJt.noalias() = m_J * m_J.transpose();
    double w = std::sqrt(std::max(0.0, m_JJt.determinant()));
    const double w_threshold   = 0.01;
    const double lambda_high   = 0.05;
    const double lambda_low    = 0.0005;
    double lambda;
    if      (w < w_threshold)    lambda = lambda_high;
    else if (pos_err_now < 0.01) lambda = lambda_low;
    else                         lambda = m_lambda;
    m_JJt.diagonal().array() += lambda * lambda;
    m_J_pinv.noalias() = m_J.transpose() * m_JJt.inverse();

    // 5. 관절 속도 레퍼런스: q̇_ref = J⁺ * e_task
    m_Qd_ref.noalias() = m_J_pinv * m_e_task;

    // 진단: 1000 사이클마다 위치/자세 오차와 예측 속도 출력
    {
        static int _dbg = 0;
        if (++_dbg >= 1000) {
            _dbg = 0;
            RigidBodyDynamics::Math::VectorNd v6 = m_J * m_Qd_ref;
            double e_pos = sqrt(m_e_task[3]*m_e_task[3]+m_e_task[4]*m_e_task[4]+m_e_task[5]*m_e_task[5]);
            double e_rot = sqrt(m_e_task[0]*m_e_task[0]+m_e_task[1]*m_e_task[1]+m_e_task[2]*m_e_task[2]);
            printf("[VS-DBG] e_x=%.4f pred_vx=%.4f tcp_x=%.4f | e_z=%.4f pred_vz=%.4f tcp_z=%.4f | e_rot=%.4f\n",
                   m_e_task[3], v6[3], tcpPose.m_position[0],
                   m_e_task[5], v6[5], tcpPose.m_position[2], e_rot);
        }
    }

    // 가속도 제한 (속도 변화율 클램핑)
    const double MAX_JOINT_ACC = 2.0;  // rad/s²
    const double max_delta_vel = MAX_JOINT_ACC * m_dt;
    for (unsigned int i = 0; i < m_uDOF; ++i)
    {
        double dv = m_Qd_ref[i] - m_Qd_ref_prev[i];
        if (dv >  max_delta_vel) m_Qd_ref[i] = m_Qd_ref_prev[i] + max_delta_vel;
        if (dv < -max_delta_vel) m_Qd_ref[i] = m_Qd_ref_prev[i] - max_delta_vel;
    }

    // 속도 상한 (절대 안전 한계)
    const double MAX_JOINT_VEL = 0.6;  // rad/s
    for (unsigned int i = 0; i < m_uDOF; ++i)
        m_Qd_ref[i] = std::max(-MAX_JOINT_VEL, std::min(MAX_JOINT_VEL, m_Qd_ref[i]));

    m_Qd_ref_prev = m_Qd_ref;

    // 적분 제거: 매 주기 m_Q 기준으로 Q_ref 재계산 (드리프트 없음)
    m_Q_ref = m_Q;
    m_Q_ref += m_Qd_ref * m_dt;

    // m_Q_ref가 실제 m_Q보다 MAX_REF_LEAD 이상 앞서지 못하도록 제한 (오버슈트 방지)
    const double MAX_REF_LEAD = 0.10;  // rad
    for (unsigned int i = 0; i < m_uDOF; ++i)
    {
        double lead = m_Q_ref[i] - m_Q[i];
        if (fabs(lead) > MAX_REF_LEAD)
            m_Q_ref[i] = m_Q[i] + (lead > 0 ? MAX_REF_LEAD : -MAX_REF_LEAD);
    }

    // 7. 참조 가속도 0
    m_Qdd_ref = m_zero_vector;

    return TRUE;
}



BOOL
CControllerFullDynamicsRT::Reset()
{
    // Reset all vectors to zero
    m_Q.setZero();
    m_Qd.setZero();
    m_Qdd.setZero();
    m_Q_ref.setZero();
    m_Qd_ref.setZero();
    m_Qdd_ref.setZero();
    
    m_M.setZero();
    m_h.setZero();
    m_g.setZero();
    m_c.setZero();
    
    m_tau_M.setZero();
    m_tau_feedback.setZero();
    m_tau_total.setZero();
    
    m_pos_error.setZero();
    m_vel_error.setZero();
    
    // Reset performance counters
    ResetPerformance();
    
    DBG_LOG_INFO("(%s) Controller reset", "CControllerFullDynamicsRT");
    return TRUE;
}

void 
CControllerFullDynamicsRT::SetGains(const std::vector<double>& avGains)
{
    if (avGains.size() == m_uDOF * 2) {
        // Assume first half is Kp, second half is Kd
        for (unsigned int i = 0; i < m_uDOF; ++i) {
            m_Kp[i] = avGains[i];
            m_Kd[i] = avGains[i + m_uDOF];
        }
    }
    m_vGains = avGains;
}

void 
CControllerFullDynamicsRT::SetReferenceTrajectory(const RigidBodyDynamics::Math::VectorNd& avQ_ref,
                                                      const RigidBodyDynamics::Math::VectorNd& avQd_ref,
                                                      const RigidBodyDynamics::Math::VectorNd& avQdd_ref)
{
    if (avQ_ref.size() == m_uDOF && avQd_ref.size() == m_uDOF && avQdd_ref.size() == m_uDOF) {
        m_Q_ref = avQ_ref;
        m_Qd_ref = avQd_ref;
        m_Qdd_ref = avQdd_ref;
    }
}

BOOL
CControllerFullDynamicsRT::SetReferencePos(UINT auAxis, double adPos)
{
    if (auAxis >= m_uDOF && auAxis != _ALL_AXIS) 
    {
        DBG_LOG_ERROR("(%s) Cannot set Reference Pos - Invalid axis %u for DOF %u", "CControllerFullDynamicsRT", auAxis, m_uDOF);
        return FALSE;
    }
    else if (auAxis == _ALL_AXIS) 
    {
        // Set all axes to the same position
        for (unsigned int i = 0; i < m_uDOF; ++i) 
        {
            m_Q_ref[i] = adPos;
        }
        return TRUE;
    }
    m_Q_ref[auAxis] = adPos;
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::GetControlGain(UINT auAxis, double* adKp, double* adKd)
{
    if (auAxis >= m_uDOF) 
    {
        DBG_LOG_ERROR("(%s) Cannot get Control Gain - Invalid axis %u for DOF %u", "CControllerFullDynamicsRT", auAxis, m_uDOF);
        return FALSE;
    }
    *adKp = m_Kp[auAxis];
    *adKd = m_Kd[auAxis];
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::SetControlGain(UINT auAxis, double adKp, double adKd)
{
    if (auAxis >= m_uDOF && auAxis != _ALL_AXIS) 
    {
        DBG_LOG_ERROR("(%s) Cannot set Control Gain - Invalid axis %u for DOF %u", "CControllerFullDynamicsRT", auAxis, m_uDOF);
        return FALSE;
    }
    else if (auAxis == _ALL_AXIS) 
    {
        // Set all axes to the same gains
        for (unsigned int i = 0; i < m_uDOF; ++i) 
        {
            m_Kp[i] = adKp;
            m_Kd[i] = adKd;
        }
        return TRUE;
    }
    m_Kp[auAxis] = adKp;
    m_Kd[auAxis] = adKd;
    return TRUE;
}


void 
CControllerFullDynamicsRT::SetControlGains(const std::vector<double>& avKp, const std::vector<double>& avKd)
{
    if (avKp.size() == m_uDOF && avKd.size() == m_uDOF) {
        m_Kp = avKp;
        m_Kd = avKd;
    }
}

void 
CControllerFullDynamicsRT::SetGravity(const RigidBodyDynamics::Math::Vector3d& avGravity)
{
    m_rbdlModel.gravity = avGravity;
    DBG_LOG_INFO("(%s) Gravity set to [%.3f, %.3f, %.3f]", 
                 "CControllerFullDynamicsRT", avGravity[0], avGravity[1], avGravity[2]);
}

BOOL 
CControllerFullDynamicsRT::LoadURDF(const TSTRING& astrURDFPath)
{
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(astrURDFPath.c_str(), &m_rbdlModel, false, true)) {
        DBG_LOG_ERROR("(%s) Failed to load URDF: %s", "CControllerFullDynamicsRT", astrURDFPath.c_str());
        return FALSE;
    }
    
    DBG_LOG_INFO("(%s) URDF loaded successfully: %s (DOF=%u)", 
                 "CControllerFullDynamicsRT", astrURDFPath.c_str(), m_rbdlModel.qdot_size);
    
    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::InitRTOptimizations()
{
    if (m_bRTOptimized) {
        return TRUE;  // Already optimized
    }
    
    // Initialize RT memory optimizations
    if (!InitRTMemoryOptimizations()) {
        DBG_LOG_ERROR("(%s) Failed to initialize RT memory optimizations", "CControllerFullDynamicsRT");
        return FALSE;
    }
    
    // Initialize RT processor optimizations
    if (!InitRTProcessorOptimizations()) {
        DBG_LOG_ERROR("(%s) Failed to initialize RT processor optimizations", "CControllerFullDynamicsRT");
        return FALSE;
    }
    
    m_bRTOptimized = TRUE;
    DBG_LOG_INFO("(%s) RT optimizations initialized", "CControllerFullDynamicsRT");
    
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::InitRTMemoryOptimizations()
{
    DBG_LOG_INFO("(%s) Prefaulting controller heap memory...", "CControllerFullDynamicsRT");
    
    // Prefault all pre-allocated RBDL vectors by touching them
    // This ensures they are in physical memory before RT loop starts
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        // Touch all vector elements
        m_Q[i] = 0.0;
        m_Qd[i] = 0.0;
        m_Qdd[i] = 0.0;
        m_Q_ref[i] = 0.0;
        m_Qd_ref[i] = 0.0;
        m_Qdd_ref[i] = 0.0;
        m_zero_vector[i] = 0.0;
        
        m_h[i] = 0.0;
        m_g[i] = 0.0;
        m_c[i] = 0.0;
        m_tau_M[i] = 0.0;
        m_tau_feedback[i] = 0.0;
        m_tau_total[i] = 0.0;
        m_pos_error[i] = 0.0;
        m_vel_error[i] = 0.0;
        
        // Touch matrix elements (most important for RT performance)
        for (unsigned int j = 0; j < m_uDOF; ++j) {
            m_M(i, j) = 0.0;
        }
    }
    
    // Prefault std::vector containers
    std::fill(m_Kp.begin(), m_Kp.end(), 0.0);
    std::fill(m_Kd.begin(), m_Kd.end(), 0.0);
    
    // Force page faults for RBDL model data structures
    // This is crucial for RT performance
    if (m_rbdlModel.q_size > 0) {
        RigidBodyDynamics::Math::VectorNd temp_q(m_rbdlModel.q_size);
        RigidBodyDynamics::Math::VectorNd temp_qdot(m_rbdlModel.qdot_size);
        RigidBodyDynamics::Math::VectorNd temp_tau(m_rbdlModel.qdot_size);
        RigidBodyDynamics::Math::MatrixNd temp_M(m_rbdlModel.qdot_size, m_rbdlModel.qdot_size);
        
        temp_q.setZero();
        temp_qdot.setZero();
        temp_tau.setZero();
        temp_M.setZero();
        
        // Perform one dummy computation to prefault RBDL internal structures
        try {
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, temp_q, temp_M);
            RigidBodyDynamics::InverseDynamics(m_rbdlModel, temp_q, temp_qdot, temp_qdot, temp_tau);
            RigidBodyDynamics::NonlinearEffects(m_rbdlModel, temp_q, temp_qdot, temp_tau);
        } catch (...) {
            DBG_LOG_WARN("(%s) Dummy RBDL computation failed during prefault", "CControllerFullDynamicsRT");
        }
    }
    
    DBG_LOG_INFO("(%s) RT memory optimizations completed (using existing memory lock)", 
                 "CControllerFullDynamicsRT");
    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::InitRTProcessorOptimizations()
{
    // 1. Eigen RT optimizations - disable malloc
    #if defined(EIGEN_RUNTIME_NO_MALLOC) && (__EIGEN_MAJOR_VERSION__>=3 && __EIGEN_MINOR_VERSION__>=4)
    Eigen::internal::set_is_malloc_allowed(false);
    DBG_LOG_INFO("(%s) Eigen malloc disabled for RT", "CControllerFullDynamicsRT");
    #endif
    
    // 2. SSE optimizations - flush-to-zero and denormals-are-zero
    #ifdef __SSE__
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    DBG_LOG_INFO("(%s) SSE optimizations enabled", "CControllerFullDynamicsRT");
    #endif
    
    return TRUE;
}

void 
CControllerFullDynamicsRT::CheckRTViolation(uint64_t computation_time_ns)
{
    if (computation_time_ns > m_rt_perf.deadline_ns) {
        m_rt_perf.rt_violations++;
        
        // Log only occasional violations to avoid RT impact
        if (m_rt_perf.rt_violations % 100 == 0) {
            DBG_LOG_WARN("(%s) RT violation #%llu: %llu µs (deadline: %llu µs)", 
                        "CControllerFullDynamicsRT", 
                        (unsigned long long)m_rt_perf.rt_violations,
                        (unsigned long long)(computation_time_ns / 1000),
                        (unsigned long long)(m_rt_perf.deadline_ns / 1000));
        }
    }
}






//jieun
//trash code! spaghetti code!!

//   - ComputeTcpFK                                                                                                  
//   - ComputeInverseKinematics_6dof                                                                              
//   - ComputeJacobianBasedInverseKinematics                                                                         
//   - RunISOCubeIKValidation  
//===========================================================================================

BOOL 
CControllerFullDynamicsRT::ComputeTcpFK()
{
    RigidBodyDynamics::Math::Vector3d p_tcp;
    RigidBodyDynamics::Math::Matrix3d R_base_to_body, R_body_to_base;

    // tcp position 
    p_tcp = RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, m_Q, m_body_id, tcp_local_point);
    
    // tcp orientation - (R_base_to_body : base -> body) 
    R_base_to_body = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, m_Q, m_body_id);
    R_body_to_base = R_base_to_body.transpose();
    
    // base coordinates
    tcpPose.m_position = p_tcp;
    tcpPose.m_rotation = R_body_to_base;
   
    return TRUE;

}



// Lagacy function
BOOL
CControllerFullDynamicsRT::ComputeInverseKinematics_6dof(std::vector<double>& avOutputTorque)
{
    if (m_bIkTrigger) {
        ComputeTcpFK();
        m_Q_ref       = m_Q;
        m_Q_ik_target = m_Q;
        m_Qd_ref      = m_zero_vector;
        m_Qdd_ref     = m_zero_vector;

        m_tcpStartPose = tcpPose;

        // 'p' 키 누른 시점의 orientation을 목표로 설정 (position은 's' 키 저장값 유지)
        goal_tcpPose.m_rotation = tcpPose.m_rotation;

        m_goalTcpPoseForCheck = goal_tcpPose;

        m_bVerifyDone      = FALSE;
        m_nStableCount     = 0;
        m_bIkMotionStarted = FALSE;
        m_bIkTrigger       = FALSE;

        double r_goal, p_goal, y_goal;
        RotToRPY(goal_tcpPose.m_rotation, r_goal, p_goal, y_goal);
        DBG_LOG_INFO("========== 6DOF IK Command ==========");
        DBG_LOG_INFO("Start : X=%.4f Y=%.4f Z=%.4f", m_tcpStartPose.m_position[0], m_tcpStartPose.m_position[1], m_tcpStartPose.m_position[2]);
        DBG_LOG_INFO("Goal  : X=%.4f Y=%.4f Z=%.4f | R=%.4f P=%.4f Y=%.4f",
            goal_tcpPose.m_position[0], goal_tcpPose.m_position[1], goal_tcpPose.m_position[2],
            r_goal, p_goal, y_goal);

        // 6DOF IK: position + orientation 동시 제약 → m_Q_ik_target에 저장
        RigidBodyDynamics::InverseKinematicsConstraintSet CS;
        CS.num_steps      = 1000;   // default 100 → 수렴 기회 늘리기
        CS.step_tol       = 1.0e-10;
        CS.constraint_tol = 1.0e-8;
        CS.AddFullConstraint(m_body_id, tcp_local_point, goal_tcpPose.m_position, goal_tcpPose.m_rotation.transpose());
        BOOL is_ok = RigidBodyDynamics::InverseKinematics(m_rbdlModel, m_Q, CS, m_Q_ik_target);

        if (!is_ok) {
            DBG_LOG_WARN("(ComputeInverseKinematics_6dof) IK failed - goal pos=(%.4f, %.4f, %.4f)",
                goal_tcpPose.m_position[0], goal_tcpPose.m_position[1], goal_tcpPose.m_position[2]);
            m_bIkReady = FALSE;
        }
        else {
            m_bIkReady = TRUE;
        }
    }

    if (!m_bIkReady)
        return ComputeGravityCompensation(avOutputTorque);

    // 점진적 추종: m_Q_ref → m_Q_ik_target (속도 제한)
    const double MAX_JOINT_VEL = 0.5;   // rad/s
    const double MAX_REF_LEAD  = 0.10;  // rad
    const double max_step      = MAX_JOINT_VEL * m_dt;

    for (unsigned int i = 0; i < m_uDOF; ++i) {
        double step = m_Q_ik_target[i] - m_Q_ref[i];
        step = std::max(-max_step, std::min(max_step, step));
        m_Q_ref[i] += step;

        double lead = m_Q_ref[i] - m_Q[i];
        if (fabs(lead) > MAX_REF_LEAD)
            m_Q_ref[i] = m_Q[i] + (lead > 0 ? MAX_REF_LEAD : -MAX_REF_LEAD);
    }

    m_Qdd_ref = m_zero_vector;

    ComputeTcpFK();
    ComputeComputedTorque(avOutputTorque);
    CheckIKConvergence();
    return TRUE;
}


BOOL
CControllerFullDynamicsRT::ComputeJacobianBasedInverseKinematics(std::vector<double>& avOutputTorque)
{
    // 1. FK로 현재 TCP pose 업데이트
    ComputeTcpFK();

    if (m_bIkTrigger) {
        m_Q_ref        = m_Q;
        m_Qd_ref       = m_zero_vector;
        m_Qdd_ref      = m_zero_vector;

        m_bVerifyDone       = FALSE;
        m_bIkMotionStarted  = FALSE;
        m_nStableCount      = 0;
        m_bIkReady          = TRUE;

        m_tcpStartPose = tcpPose;

        // [ORIGINAL] 현재 위치에서 X +5cm
        // goal_tcpPose          = tcpPose;
        // goal_tcpPose.m_position[0] += 0.05;

        // // TODO: 목표 TCP 위치 (절대 좌표, 단위: m)
        // goal_tcpPose.m_position[0] = 0.5;   // X
        // goal_tcpPose.m_position[1] = 0.0;   // Y
        // goal_tcpPose.m_position[2] = 0.8;   // Z

        // 현재 자세 유지: 'i' 키 누른 시점의 orientation을 목표로 설정
        goal_tcpPose.m_rotation = tcpPose.m_rotation;

        // TODO: 목표 TCP 자세 (RPY, 단위: rad) - 로그의 Final RPY 참고해서 설정
        // double goal_roll  = -0.6045;
        // double goal_pitch = -0.6636;
        // double goal_yaw   = -1.2223;
        // goal_tcpPose.m_rotation = RPYToRot(goal_roll, goal_pitch, goal_yaw);

        m_goalTcpPoseForCheck = goal_tcpPose;

        m_bIkTrigger = FALSE;

        DBG_LOG_INFO("========== IK Command ==========");
        DBG_LOG_INFO("Start : X=%.4f Y=%.4f Z=%.4f",
            m_tcpStartPose.m_position[0],
            m_tcpStartPose.m_position[1],
            m_tcpStartPose.m_position[2]);
        DBG_LOG_INFO("Goal  : X=%.4f Y=%.4f Z=%.4f",
            goal_tcpPose.m_position[0],
            goal_tcpPose.m_position[1],
            goal_tcpPose.m_position[2]);
    }

    if (!m_bIkReady)
        return ComputeGravityCompensation(avOutputTorque);


    // 2. 6D Jacobian 계산 [angular(0-2); linear(3-5)] × DOF
    m_J.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(m_rbdlModel, m_Q, m_body_id, tcp_local_point, m_J);

    // 3. Cartesian 오차 계산 (RBDL Jacobian 순서에 맞춰 [angular; linear])
    // 위치 오차
    m_e_task[3] = goal_tcpPose.m_position[0] - tcpPose.m_position[0];
    m_e_task[4] = goal_tcpPose.m_position[1] - tcpPose.m_position[1];
    m_e_task[5] = goal_tcpPose.m_position[2] - tcpPose.m_position[2];

    // 자세 오차 비활성화 (XYZ만 제어)
    RigidBodyDynamics::Math::Matrix3d R_err = goal_tcpPose.m_rotation * tcpPose.m_rotation.transpose();
    m_e_task[0] = m_Kp_task_rot * 0.5 * (R_err(2,1) - R_err(1,2));
    m_e_task[1] = m_Kp_task_rot * 0.5 * (R_err(0,2) - R_err(2,0));
    m_e_task[2] = m_Kp_task_rot * 0.5 * (R_err(1,0) - R_err(0,1));
    // m_e_task[0] = 0.0;
    // m_e_task[1] = 0.0;
    // m_e_task[2] = 0.0;

    m_e_task[3] *= m_Kp_task_pos;
    m_e_task[4] *= m_Kp_task_pos;
    m_e_task[5] *= m_Kp_task_pos;


    

    // 6. q_ref 적분용 위치 오차 계산 (gain 적용 전 raw 오차)
    double pos_err_now = sqrt(m_e_task[3]*m_e_task[3] +
                              m_e_task[4]*m_e_task[4] +
                              m_e_task[5]*m_e_task[5]) / m_Kp_task_pos;

    // 4. DLS pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹
    // 목표 근접 시 λ를 줄여 수렴 정확도 향상 (adaptive damping)
    double lambda = (pos_err_now < 0.01) ? 0.0005 : m_lambda;
    m_JJt.noalias() = m_J * m_J.transpose();
    m_JJt.diagonal().array() += lambda * lambda;
    m_J_pinv.noalias() = m_J.transpose() * m_JJt.inverse();

    // 5. 관절 속도 레퍼런스: q̇_ref = J⁺ * e_task
    m_Qd_ref.noalias() = m_J_pinv * m_e_task;

    // velocity clamping
    const double MAX_JOINT_VEL = 0.5;  // rad/s
    for (unsigned int i = 0; i < m_uDOF; ++i)
        m_Qd_ref[i] = std::max(-MAX_JOINT_VEL, std::min(MAX_JOINT_VEL, m_Qd_ref[i]));

    // 적분 제거: 매 주기 m_Q 기준으로 Q_ref 재계산 (드리프트 없음)
    m_Q_ref = m_Q;
    m_Q_ref += m_Qd_ref * m_dt;

    // m_Q_ref가 실제 m_Q보다 MAX_REF_LEAD 이상 앞서지 못하도록 제한 (오버슈트 방지)
    const double MAX_REF_LEAD = 0.10;  // rad
    for (unsigned int i = 0; i < m_uDOF; ++i)
    {
        double lead = m_Q_ref[i] - m_Q[i];
        if (fabs(lead) > MAX_REF_LEAD)
            m_Q_ref[i] = m_Q[i] + (lead > 0 ? MAX_REF_LEAD : -MAX_REF_LEAD);
    }

    // 7. 참조 가속도 0
    m_Qdd_ref = m_zero_vector;

    // 8. CTC로 토크 계산
    ComputeComputedTorque(avOutputTorque);
    CheckIKConvergence();




    return TRUE;
}



BOOL
CControllerFullDynamicsRT::RunISOCubeIKValidation()
{
    DBG_LOG_INFO("========== ISO Cube IK Validation Start ==========");

    // Load separate RBDL model — RT loop keeps using m_rbdlModel (gravity comp safe)
    RigidBodyDynamics::Model local_model;
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(m_strURDFPath.c_str(), &local_model, false, true)) {
        DBG_LOG_ERROR("RunISOCubeIKValidation: Failed to load URDF");
        return FALSE;
    }
    local_model.gravity = RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81);
    unsigned int local_body_id = local_model.GetBodyId("tcp");

    // Center pose — current TCP + 2cm in X
    RigidBodyDynamics::Math::Vector3d c_pos =
        RigidBodyDynamics::CalcBodyToBaseCoordinates(local_model, m_Q, local_body_id, tcp_local_point);
    c_pos[0] += 0.02;
    RigidBodyDynamics::Math::Matrix3d c_rot =
        RigidBodyDynamics::CalcBodyWorldOrientation(local_model, m_Q, local_body_id).transpose();

    // 5 ISO cube points: P1=center, P2~P5=ISO 9283 diagonal plane (250mm cube, 0.1L inset)
    const double a = 0.100;  // 0.8 * (L/2) = 0.8 * 0.125 = 0.10, L=0.25m
    const double offsets[4][3] = {
        { a,  a, -a},   // P2: C1 근처 (+,+,-)
        {-a,  a, -a},   // P3: C2 근처 (-,+,-)
        {-a, -a,  a},   // P4: C7 근처 (-,-,+)
        { a, -a,  a}
    };

    Pose targets[6];
    targets[0].m_position = c_pos;
    targets[0].m_rotation = c_rot;
    for (int i = 1; i < 5; ++i) {
        targets[i].m_position = c_pos + RigidBodyDynamics::Math::Vector3d(offsets[i-1][0], offsets[i-1][1], offsets[i-1][2]);
        targets[i].m_rotation = c_rot;
    }
    // P6 = clearance (center + Z+0.08)
    targets[5].m_position = c_pos + RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.08);
    targets[5].m_rotation = c_rot;

    // Print target points
    for (int i = 0; i < 5; ++i)
        DBG_LOG_INFO("P%d target: X=%.4f Y=%.4f Z=%.4f", i+1,
            targets[i].m_position[0], targets[i].m_position[1], targets[i].m_position[2]);
    DBG_LOG_INFO("Clearance: X=%.4f Y=%.4f Z=%.4f",
        targets[5].m_position[0], targets[5].m_position[1], targets[5].m_position[2]);

    // Open CSV
    mkdir("/home/raimlab/RAON-RT/App/Indy7/iso_csv", 0777);
    mkdir("/home/raimlab/RAON-RT/App/Indy7/iso_csv/virtual", 0777);
    const char* csv_path = "/home/raimlab/RAON-RT/App/Indy7/iso_csv/virtual/iso_cube_ik_validation.csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        DBG_LOG_ERROR("RunISOCubeIKValidation: Cannot open %s", csv_path);
        return FALSE;
    }
    csv << "point,start_type,success,time_ms,pos_err_mm,rot_err_deg,"
        << "goal_x,goal_y,goal_z,fk_x,fk_y,fk_z\n";

    RigidBodyDynamics::Math::VectorNd q_home = RigidBodyDynamics::Math::VectorNd::Zero(m_uDOF);

    for (int pt = 0; pt < 6; ++pt) {  // 0~4: P1~P5, 5: clearance
        const Pose& tgt = targets[pt];
        const char* pt_label = (pt < 5) ? std::to_string(pt+1).c_str() : "C";

        // cold(0) and warm(1) start
        for (int sw = 0; sw < 2; ++sw) {
            RigidBodyDynamics::Math::VectorNd q_seed = (sw == 0) ? q_home : m_Q;
            RigidBodyDynamics::Math::VectorNd q_sol  = q_seed;

            RigidBodyDynamics::InverseKinematicsConstraintSet CS;
            CS.num_steps      = 1000;
            CS.step_tol       = 1.0e-10;
            CS.constraint_tol = 1.0e-8;
            CS.AddFullConstraint(local_body_id, tcp_local_point, tgt.m_position, tgt.m_rotation.transpose());

            uint64_t t0 = read_timer();
            BOOL ok = RigidBodyDynamics::InverseKinematics(local_model, q_seed, CS, q_sol);
            uint64_t t1 = read_timer();
            double time_ms = (double)(t1 - t0) / 1e6;

            // FK of solution
            RigidBodyDynamics::Math::Vector3d p_fk =
                RigidBodyDynamics::CalcBodyToBaseCoordinates(local_model, q_sol, local_body_id, tcp_local_point);
            RigidBodyDynamics::Math::Matrix3d R_fk =
                RigidBodyDynamics::CalcBodyWorldOrientation(local_model, q_sol, local_body_id).transpose();

            // Position error (mm)
            double pos_err_mm = (p_fk - tgt.m_position).norm() * 1000.0;

            // Rotation error (deg)
            RigidBodyDynamics::Math::Matrix3d R_err = tgt.m_rotation * R_fk.transpose();
            double er = 0.5 * (R_err(2,1) - R_err(1,2));
            double ep = 0.5 * (R_err(0,2) - R_err(2,0));
            double ey = 0.5 * (R_err(1,0) - R_err(0,1));
            double rot_err_deg = sqrt(er*er + ep*ep + ey*ey) * 180.0 / M_PI;

            DBG_LOG_INFO("P%s [%s] %s  time=%.2fms  pos_err=%.4fmm  rot_err=%.4fdeg",
                pt_label, sw==0?"cold":"warm", ok?"OK  ":"FAIL",
                time_ms, pos_err_mm, rot_err_deg);

            csv << pt_label << ","
                << (sw==0 ? "cold" : "warm") << ","
                << (ok ? 1 : 0) << ","
                << time_ms << ","
                << pos_err_mm << ","
                << rot_err_deg << ","
                << tgt.m_position[0] << "," << tgt.m_position[1] << "," << tgt.m_position[2] << ","
                << p_fk[0] << "," << p_fk[1] << "," << p_fk[2] << "\n";
        }
    }

    csv.close();
    DBG_LOG_INFO("========== ISO Cube IK Validation Done ==========");
    DBG_LOG_INFO("Results: %s", csv_path);

    system("python3 /home/raimlab/RAON-RT/App/Indy7/iso_csv/plot_iso_virtual.py");
    DBG_LOG_INFO("Plots saved to: /home/raimlab/RAON-RT/App/Indy7/iso_csv/virtual/");
    return TRUE;
}


//===========================================================================================

