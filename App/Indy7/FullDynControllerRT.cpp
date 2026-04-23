/*****************************************************************************
*	Name: FullDynControllerRT.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of RT-optimized full dynamics controller
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "FullDynControllerRT.h"
#include <algorithm>
#include <cmath>
#include <errno.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <xmmintrin.h>
#include <pmmintrin.h>



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
    m_Qdd_ref.resize(auDOF);
    m_zero_vector.resize(auDOF);
    
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
    m_Qdd_ref.setZero();
    m_zero_vector.setZero();
    
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
            //result = ComputeInverseKinematics(avOutputTorque);
            result = ComputeJacobianBasedInverseKinematics(avOutputTorque);
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

BOOL
CControllerFullDynamicsRT::ComputeInverseKinematics(std::vector<double>& avOutputTorque)
{
    if (m_bIkTrigger) {
        m_Q_ref    = m_Q;
        m_Qd_ref   = m_zero_vector;
        m_Qdd_ref  = m_zero_vector;

        m_tcpStartPose     = tcpPose;
        goal_tcpPose       = tcpPose;
        goal_tcpPose.m_position[0] += 0.05;
        m_goalTcpPoseForCheck = goal_tcpPose;

        m_bVerifyDone  = FALSE;
        m_nStableCount = 0;
        m_bIkTrigger   = FALSE;

        std::vector<unsigned int>                      body_ids;
        std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
        std::vector<RigidBodyDynamics::Math::Vector3d> target_positions;
        body_ids.push_back(m_body_id);
        body_points.push_back(tcp_local_point);
        target_positions.push_back(goal_tcpPose.m_position);

        BOOL is_ok = RigidBodyDynamics::InverseKinematics(
            m_rbdlModel, m_Q, body_ids, body_points, target_positions, m_Q_ref);

        if (!is_ok) {
            DBG_LOG_WARN("(ComputeInverseKinematics) IK failed - falling back to gravity compensation");
            m_bIkReady = FALSE;
            return ComputeGravityCompensation(avOutputTorque);
        }
        m_bIkReady = TRUE;
    }

    if (!m_bIkReady)
        return ComputeGravityCompensation(avOutputTorque);

    ComputeTcpFK();
    ComputeComputedTorque(avOutputTorque);
    CheckIKConvergence();
    return TRUE;
}

void
CControllerFullDynamicsRT::CheckIKConvergence()
{
    if (m_bVerifyDone) return;

    // 한 번이라도 움직임 감지 (관절 속도 > 0.02 rad/s)
    if (!m_bIkMotionStarted && !IsJointSettled(0.02))
        m_bIkMotionStarted = TRUE;

    // 움직임 감지 후 완전히 멈추면 수렴 판정
    if (m_bIkMotionStarted && IsJointSettled(0.01)) {
        ComputeTcpFK();
        m_tcpFinalPose = tcpPose;
        PrintTcpVerificationResult();
        m_bVerifyDone = TRUE;
    }
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

        // TODO: 목표 TCP 위치 (절대 좌표, 단위: m)
        goal_tcpPose.m_position[0] = 0.5;   // X
        goal_tcpPose.m_position[1] = 0.0;   // Y
        goal_tcpPose.m_position[2] = 0.8;   // Z

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

    // 자세 오차 (rotation matrix → axis-angle 근사)
    // TODO: 자세 제어 활성화 시 위치 드리프트 발생 확인 필요 → 현재 비활성화
    // RigidBodyDynamics::Math::Matrix3d R_err = goal_tcpPose.m_rotation * tcpPose.m_rotation.transpose();
    // m_e_task[0] = m_Kp_task_rot * 0.5 * (R_err(2,1) - R_err(1,2));
    // m_e_task[1] = m_Kp_task_rot * 0.5 * (R_err(0,2) - R_err(2,0));
    // m_e_task[2] = m_Kp_task_rot * 0.5 * (R_err(1,0) - R_err(0,1));
    m_e_task[0] = 0.0;
    m_e_task[1] = 0.0;
    m_e_task[2] = 0.0;

    m_e_task[3] *= m_Kp_task_pos;
    m_e_task[4] *= m_Kp_task_pos;
    m_e_task[5] *= m_Kp_task_pos;

    // 4. DLS pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹
    // 특이점 근처에서도 안정적으로 동작 (Damped Least Squares)
    m_JJt.noalias() = m_J * m_J.transpose();
    m_JJt.diagonal().array() += m_lambda * m_lambda;
    m_J_pinv.noalias() = m_J.transpose() * m_JJt.inverse();

    // 5. 관절 속도 레퍼런스: q̇_ref = J⁺ * e_task
    m_Qd_ref.noalias() = m_J_pinv * m_e_task;

    // 안전을 위한 관절 속도 클램핑
    const double MAX_JOINT_VEL = 0.2;  // rad/s
    for (unsigned int i = 0; i < m_uDOF; ++i)
        m_Qd_ref[i] = std::max(-MAX_JOINT_VEL, std::min(MAX_JOINT_VEL, m_Qd_ref[i]));

    // 6. q_ref 적분: 오차가 2mm 이상일 때만 적분 (windup 방지)
    double pos_err_now = sqrt(m_e_task[3]*m_e_task[3] +
                              m_e_task[4]*m_e_task[4] +
                              m_e_task[5]*m_e_task[5]);
    if (pos_err_now > 0.002)
        m_Q_ref += m_Qd_ref * m_dt;

    // m_Q_ref가 실제 m_Q보다 MAX_REF_LEAD 이상 앞서지 못하도록 제한 (오버슈트 방지)
    const double MAX_REF_LEAD = 0.15;  // rad
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


    CheckIKConvergence();


    return TRUE;
}


BOOL CControllerFullDynamicsRT::IsJointSettled(double vel_threshold)
{
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (fabs(m_Qd[i]) > vel_threshold) {
            return FALSE;
        }
    }
    return TRUE;
}

void CControllerFullDynamicsRT::PrintTcpVerificationResult()
{
    double dx = m_tcpFinalPose.m_position[0] - m_tcpStartPose.m_position[0];
    double dy = m_tcpFinalPose.m_position[1] - m_tcpStartPose.m_position[1];
    double dz = m_tcpFinalPose.m_position[2] - m_tcpStartPose.m_position[2];

    double ex = m_goalTcpPoseForCheck.m_position[0] - m_tcpFinalPose.m_position[0];
    double ey = m_goalTcpPoseForCheck.m_position[1] - m_tcpFinalPose.m_position[1];
    double ez = m_goalTcpPoseForCheck.m_position[2] - m_tcpFinalPose.m_position[2];

    double err_norm = sqrt(ex*ex + ey*ey + ez*ez);

    DBG_LOG_INFO("========== TCP Verification ==========");
    DBG_LOG_INFO("TCP Start : X=%.6f Y=%.6f Z=%.6f",
        m_tcpStartPose.m_position[0],
        m_tcpStartPose.m_position[1],
        m_tcpStartPose.m_position[2]);

    DBG_LOG_INFO("TCP Goal  : X=%.6f Y=%.6f Z=%.6f",
        m_goalTcpPoseForCheck.m_position[0],
        m_goalTcpPoseForCheck.m_position[1],
        m_goalTcpPoseForCheck.m_position[2]);

    DBG_LOG_INFO("TCP Final : X=%.6f Y=%.6f Z=%.6f",
        m_tcpFinalPose.m_position[0],
        m_tcpFinalPose.m_position[1],
        m_tcpFinalPose.m_position[2]);

    DBG_LOG_INFO("Delta     : dX=%.6f dY=%.6f dZ=%.6f", dx, dy, dz);
    DBG_LOG_INFO("Error     : eX=%.6f eY=%.6f eZ=%.6f", ex, ey, ez);
    DBG_LOG_INFO("Norm Error: %.6f m", err_norm);
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
                << "err_x,err_y,err_z,norm_err\n";
        csv << m_goalTcpPoseForCheck.m_position[0] << ","
            << m_goalTcpPoseForCheck.m_position[1] << ","
            << m_goalTcpPoseForCheck.m_position[2] << ","
            << m_tcpFinalPose.m_position[0] << ","
            << m_tcpFinalPose.m_position[1] << ","
            << m_tcpFinalPose.m_position[2] << ","
            << ex << "," << ey << "," << ez << "," << err_norm << "\n";
    }
}

//===================================================================


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