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
            result = ComputeInverseKinematics(avOutputTorque);
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
        m_tcpStartPose = tcpPose;
        //goal_tcpPose = tcpPose;
        
                
        // init goal_tcpPose : mode is t 
        // goal_tcpPose.m_position[0] = 0.042177;
        // goal_tcpPose.m_position[1] = -0.145970;
        // goal_tcpPose.m_position[2] = 1.331485;

        // goal_tcpPose.m_rotation <<
        //     0.806490, -0.585035,  0.085484,
        //     0.585444,  0.810391,  0.022838,
        //     -0.082636,  0.031628,  0.996078;
 

        goal_tcpPose.m_position[0] += 0.05;
        m_goalTcpPoseForCheck = goal_tcpPose;
        

        m_bVerifyInit = TRUE;
        m_bVerifyDone = FALSE;
        m_nStableCount = 0;
        //DBG_LOG_INFO("IK Target Reset: Current X + 5cm");  
        
        std::vector<unsigned int> body_ids;
        std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
        std::vector<RigidBodyDynamics::Math::Vector3d> target_positions;

        body_ids.push_back(m_body_id);
        body_points.push_back(tcp_local_point);
        target_positions.push_back(goal_tcpPose.m_position);

        BOOL is_ok = RigidBodyDynamics::InverseKinematics(
            m_rbdlModel, 
            m_Q, 
            body_ids, 
            body_points, 
            target_positions, 
            m_Q_ref
        );

         if(!ComputeTcpFK()){return FALSE;}
        if (!ComputeComputedTorque(avOutputTorque)) return FALSE;


        if(!is_ok)
        {
            return FALSE;
        }
        m_Qd_ref = m_zero_vector;
        m_Qdd_ref = m_zero_vector;
        m_bIkTrigger = FALSE;   // for once execution

    }

    // ---------- Convengence Check & Data Logging ----------
    if (m_bVerifyInit && !m_bVerifyDone) {
        if (IsJointSettled(0.01)) {
            m_nStableCount++;
        } else {
            m_nStableCount = 0;
        }

        // Considered converged if sufficiently slow for 100 consecutive cycles.
        if (m_nStableCount > 100) {
            if (ComputeTcpFK()) {
                m_tcpFinalPose = tcpPose;
                PrintTcpVerificationResult();
                m_bVerifyDone = TRUE;
                
            }
        }
    }
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