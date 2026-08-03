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
    m_Kf.resize(auDOF, 0.0);    // Friction FF off unless the cfg says so

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
        case eTrackingServo:
            // per-cycle servo needs a FRESH tcpPose (SetTargetPose_Jacobian
            // reads it but does not compute it), then the torque law
            ComputeTcpFK();
            SetTargetPose_Jacobian();
            result = ComputeComputedTorque(avOutputTorque);
            break;

        default:
            result = ComputeGravityCompensation(avOutputTorque);
            break;
    }

    // Torque backstop: nothing downstream saturates (SaturateOutput is never
    // wired and MoveTorque writes drive current unchecked). URDF effort
    // limits per joint; any non-finite value means the dynamics are poisoned
    // — zero ALL joints and fail the cycle (the app's FALSE path already
    // writes zero torque) rather than emit full-scale current.
    {
        static constexpr double s_adTauMax[6] =
            {431.97, 431.97, 197.23, 79.79, 79.79, 79.79};
        for (unsigned int i = 0; i < m_uDOF; ++i) {
            if (!std::isfinite(avOutputTorque[i])) {
                std::fill(avOutputTorque.begin(), avOutputTorque.end(), 0.0);
                static uint32_t s_uNanCnt = 0;
                if ((s_uNanCnt++ % 1000) == 0)
                    DBG_LOG_ERROR("(FullDynRT) non-finite torque on J%u — "
                                  "output zeroed (x%u)", i, s_uNanCnt);
                result = FALSE;
                break;
            }
            const double dMax = (i < 6) ? s_adTauMax[i] : 79.79;
            if (avOutputTorque[i] >  dMax) avOutputTorque[i] =  dMax;
            if (avOutputTorque[i] < -dMax) avOutputTorque[i] = -dMax;
        }
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

    // [kv260-merge 2026-07-31] Coulomb friction feedforward. The rigid-body
    // terms above contain no friction, so on the harmonic-drive wrists the
    // whole breakaway torque (j3 ~7-10 Nm measured) lands on the PD - whose
    // authority is M_jj-scaled and loses by an order of magnitude (E-series
    // field logs: j4 parked 0.15 rad short every approach). Add the expected
    // friction torque while a move is COMMANDED: Kf*sat(qd_ref/0.01). Ramps
    // in smoothly (no torque step at motion start/end) and is exactly zero
    // whenever qd_ref is zero - so it cannot buzz at rest and cannot hunt
    // like an integrator. Kf defaults to 0 (cfg KF_n absent = term off).
    // m_dKfScale: per-trajectory gate — 1 for gross transport, 0 for the
    // refine mini-moves (the leash-released joint overruns a slow short
    // reference; see SetTargetPosePositionOnly's adKfScale note).
    // [2026-07-31] E35 lag gate: FF only while the joint TRAILS the reference
    // in the commanded direction. On mustard's small j3 leg (0.28 rad) the
    // ungated FF rode through the decel phase, overran the final ref by
    // 0.13 rad, and settle stiction held it there (qd_ref=0 -> FF=0 -> no
    // push-back): +36 mm y-miss with the SIGN FLIPPED vs every pre-FF log.
    // Trailing is the only state where friction is the enemy; caught-up or
    // ahead, friction braking is what we want. 0.002 rad ramp keeps the term
    // continuous (real mid-move lag is 0.01-0.13 rad, solidly gate=1).
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (m_Kf[i] > 0.0 && m_dKfScale > 0.0) {
            double dS = m_Qd_ref[i] * 100.0;            // /0.01 rad/s
            if (dS > 1.0) dS = 1.0; else if (dS < -1.0) dS = -1.0;
            double dLag = (m_Q_ref[i] - m_Q[i]) * (dS >= 0.0 ? 1.0 : -1.0);
            double dG = dLag / 0.002;
            if (dG > 1.0) dG = 1.0; else if (dG < 0.0) dG = 0.0;
            m_tau_total[i] += m_Kf[i] * m_dKfScale * dS * dG;
        }
    }

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
    // ⚠ num_steps is an RBDL *output* (iterations performed); the input limit
    // is max_steps (default 300). This line has never had any effect. Do NOT
    // "fix" it to max_steps=1000: at ~35 us/step that is a 35 ms stall inside
    // the 1 kHz cycle. 300 is what this path has always run at, and it works.
    CS.num_steps      = 1000;
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
    m_dKfScale       = 1.0;
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

RigidBodyDynamics::Math::Vector3d
CControllerFullDynamicsRT::ToolPosAt(const RigidBodyDynamics::Math::VectorNd& aq)
{
    // Offline use only (fk_replay) — see the header note.
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, aq,
                                                        m_body_id,
                                                        tcp_local_point);
}

BOOL
CControllerFullDynamicsRT::SetTargetPosePositionOnly(Pose astTargetPose,
                                                     double adOriWeight,
                                                     BOOL abQuiet,
                                                     double adKfScale)
{
    RigidBodyDynamics::Math::VectorNd vjointspace = m_Q;
    RigidBodyDynamics::InverseKinematicsConstraintSet CS;
    // abQuiet only suppresses this rung's refusal logs — NOT its Newton budget.
    // Starving an exploratory rung was tried and it corrupted the solutions
    // (see IK_SOLVE_STEPS). max_steps is the limit; num_steps is RBDL's output.
    CS.max_steps      = (unsigned int)IK_SOLVE_STEPS;
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
    m_dKfScale       = (adKfScale < 0.0) ? 0.0
                     : (adKfScale > 1.0) ? 1.0 : adKfScale;
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
    m_dKfScale       = 1.0;
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
    // CS.num_steps is an OUTPUT (iterations performed); CS.max_steps is the
    // input limit. Writing num_steps, as this code did, capped nothing.
    CS.max_steps      = (unsigned int)anMaxSteps;
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
CControllerFullDynamicsRT::ReadyBranchGap(
    const RigidBodyDynamics::Math::VectorNd& aq, int& anAxis)
{
    RigidBodyDynamics::Math::VectorNd vq = aq;
    FoldTowardRef(vq, m_QReady, m_uDOF);   // 2π laps are not branch changes
    double dMax = 0.0;
    anAxis = -1;
    for (unsigned int i = 0; i < m_uDOF; i++)
    {
        const double d = std::fabs(vq[i] - m_QReady[i]);
        if (d > dMax) { dMax = d; anAxis = (int)i; }
    }
    return dMax;
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

// [kv260-merge] Record what the solve did — see IkDiag in the header. Called
// at every SolveReadyIK exit; the two attitude/branch numbers are one FK and
// one fold each, which is why this is affordable on the pass path too.
void
CControllerFullDynamicsRT::StampIkDiag(
    eIkVerdict aeVerdict, double adW, int anSolves, uint64_t atStart,
    const RigidBodyDynamics::Math::VectorNd& aq, double adPosErrM,
    int anSeed, int anCandidates)
{
    m_stIkDiag.eVerdict = aeVerdict;
    m_stIkDiag.dW       = adW;
    m_stIkDiag.nSolves  = anSolves;
    m_stIkDiag.dMs      = (double)(read_timer() - atStart) / 1.0e6;
    m_stIkDiag.dPosErrM = adPosErrM;
    m_stIkDiag.dTiltDeg = AttitudeDevDeg(aq);
    m_stIkDiag.dBranchGap = ReadyBranchGap(aq, m_stIkDiag.nBranchAxis);
    m_stIkDiag.nSeed       = anSeed;
    m_stIkDiag.nCandidates = anCandidates;
}

// [kv260-merge] Branch seeds — see IK_NUM_SEEDS in the header.
//
// Indy7's joint axes at q=0 are Z, Y, Y, Z, Y, Z: J0 is base yaw, J1/J2 are a
// parallel pair forming a planar arm in the vertical plane, and J3/J4/J5 are
// the wrist. That gives the three classic branch choices, applied here as
// discrete transforms of the reference posture:
//
//   bit 0  base    J0 += pi with the planar pair mirrored — reach the same
//                  point from the opposite side of the base
//   bit 1  elbow   negate the elbow (up <-> down) in the planar pair
//   bit 2  wrist   J3 += pi, J4 = -J4, J5 += pi — exact for an intersecting
//                  Z-Y-Z wrist, approximate here (the 183 mm J3-J5 offset)
//
// These are START POINTS, not answers. They only have to be closer to the
// target basin than q_ready is; the solver walks the rest. Index 0 is the
// identity so the fast path stays bit-identical to the previous solver.
void
CControllerFullDynamicsRT::BuildBranchSeeds(
    const RigidBodyDynamics::Math::VectorNd& aqRef,
    std::vector<RigidBodyDynamics::Math::VectorNd>& avSeeds) const
{
    avSeeds.clear();
    avSeeds.reserve(IK_NUM_SEEDS);
    for (int nMask = 0; nMask < IK_NUM_SEEDS; nMask++)
    {
        RigidBodyDynamics::Math::VectorNd vq = aqRef;
        if ((nMask & 1) && m_uDOF > 3)          // base flip
        {
            vq[0] += M_PI;
            vq[1]  = -vq[1];
            vq[2]  = -vq[2];
            vq[3]  = -vq[3];
        }
        if ((nMask & 2) && m_uDOF > 2)          // elbow up <-> down
        {
            const double dElbow = vq[2];
            vq[2] = -dElbow;
            vq[1] =  vq[1] + dElbow;            // keep the hand roughly put
        }
        if ((nMask & 4) && m_uDOF > 5)          // wrist flip
        {
            vq[3] += M_PI;
            vq[4]  = -vq[4];
            vq[5] += M_PI;
        }
        avSeeds.push_back(vq);
    }
}

BOOL
CControllerFullDynamicsRT::SolveReadyIK(
    const RigidBodyDynamics::Math::Vector3d& avTarget,
    RigidBodyDynamics::Math::VectorNd& aqSol,
    double& adPosErrM, double& adDqMaxFromCur, int& anDqAxis)
{
    m_stIkDiag = IkDiag{eIkNoSeed, -1.0, 0, 0.0, 0.0, 0.0, 0.0, -1, -1, 0};
    if (!m_bReadySet) return FALSE;

    const uint64_t tIkStart = read_timer();
    int  nSolves    = 0;
    bool bReachable = false;      // some solve met the 2 mm test, somewhere

    // Every candidate is judged by the same gates, in one place: joint limits,
    // distance from the q_ready branch, and — only where the attitude was
    // relaxed — the tool tilt cone. (Position is inside SolvePositionIK.) E28
    // happened because ONE path, polish, skipped the branch gate; a single
    // gate function is how that stops recurring.
    struct Cand {
        RigidBodyDynamics::Math::VectorNd q;
        double dPosErr, dW, dTilt, dGap;
        int    nAxis, nSeed;
    };
    auto Accept = [&](RigidBodyDynamics::Math::VectorNd& avq, double adW,
                      double adErr, int anSeed, Cand& astOut) -> bool
    {
        if (!CheckLimitsPhysical(avq)) return false;
        int nAxis = -1;
        const double dGap = ReadyBranchGap(avq, nAxis);
        if (dGap > IK_DQ_MAX_RAD) return false;
        const double dTilt = AttitudeDevDeg(avq);
        if (adW < (double)IK_ORI_WEIGHT && dTilt > APPROACH_RDEV_MAX_DEG)
            return false;
        astOut = Cand{avq, adErr, adW, dTilt, dGap, nAxis, anSeed};
        return true;
    };

    // Position is already satisfied, so a light weight spends only the
    // nullspace pulling the tool back toward the ready attitude. Goes through
    // Accept() like everything else (E28), and is kept only if the tilt really
    // improves.
    auto Polish = [&](Cand& astC)
    {
        if (astC.dW > 0.0) return;
        static const double adPolishW[] = { 0.1, 0.03, 0.01 };
        for (int k = 0; k < 3; k++)
        {
            nSolves++;
            RigidBodyDynamics::Math::VectorNd vq;
            double dErr = 0.0;
            Cand   stP;
            if (!SolvePositionIK(astC.q, avTarget, adPolishW[k], &m_EReady,
                                 vq, dErr, IK_SOLVE_STEPS))
                continue;                   // this weight loses the point
            if (Accept(vq, adPolishW[k], dErr, astC.nSeed, stP) &&
                stP.dTilt < astC.dTilt)
            {
                DBG_LOG_INFO("(SolveReadyIK) polish w=%.2f pulled the tool "
                             "%.0f -> %.0f deg", adPolishW[k], astC.dTilt,
                             stP.dTilt);
                astC = stP;
            }
            break;      // first weight that holds position is the strongest
        }
    };

    // Express the answer in the LIVE counter frame so the commanded travel is
    // the true physical delta (never extra turns), and report the largest
    // travel for the caller's direct-vs-stage decision.
    auto Finish = [&](RigidBodyDynamics::Math::VectorNd& avq)
    {
        FoldTowardRef(avq, m_Q, m_uDOF);
        adDqMaxFromCur = 0.0;
        anDqAxis       = -1;
        for (unsigned int i = 0; i < m_uDOF; i++)
        {
            const double dDq = std::fabs(avq[i] - m_Q[i]);
            if (dDq > adDqMaxFromCur) { adDqMaxFromCur = dDq; anDqAxis = (int)i; }
        }
    };

    Cand stBest;
    bool bHave = false;

    // ---- Stage 1: the q_ready ladder — unchanged from the previous solver ---
    // Runs FIRST and in full, because it is not just a fallback: the ladder
    // walks the attitude down gradually and the warm start carries that into
    // the w=0 rung, which is where the good attitudes come from. Solving w=0
    // straight from q_ready instead — the first version of this branch did —
    // still reached the targets but left the tool at 53-57 deg where the
    // ladder gives 3. So Stage 1 stays whole, and multi-start below is what
    // happens only when it comes back with nothing.
    {
        static const double adLadder[] = { IK_ORI_WEIGHT, 0.1, 0.03, 0.01 };
        RigidBodyDynamics::Math::VectorNd vqSeed = m_QReady;
        for (int i = 0; i < 4 && !bHave; i++)
        {
            nSolves++;
            RigidBodyDynamics::Math::VectorNd vq;
            double dErr = 0.0;
            const bool bHit = SolvePositionIK(vqSeed, avTarget, adLadder[i],
                                              &m_EReady, vq, dErr,
                                              IK_SOLVE_STEPS);
            if (bHit) bReachable = true;
            if (bHit && Accept(vq, adLadder[i], dErr, 0, stBest)) bHave = true;
            else vqSeed = vq;               // stalled iterate seeds the next
        }
        if (!bHave)
        {
            nSolves++;
            RigidBodyDynamics::Math::VectorNd vq;
            double dErr = 0.0;
            if (SolvePositionIK(vqSeed, avTarget, 0.0, NULL, vq, dErr,
                                IK_SOLVE_STEPS))
            {
                bReachable = true;
                if (Accept(vq, 0.0, dErr, 0, stBest)) { Polish(stBest); bHave = true; }
            }
        }
    }

    // ---- Stage 1b: position-only, straight from q_ready ---------------------
    // The warm start that makes Stage 1's attitudes good also drags the w=0
    // rung along the chain, and on the E27 targets that chain ends in the
    // flipped basin (gap 2.9 rad). Solving w=0 from q_ready UNWARMED lands in
    // the near branch instead (gap 0.87) — a worse attitude, ~55 deg against
    // 3, but inside the cone and on the right branch, so it is a real answer
    // where Stage 1 had none. One solve, so it is tried before the seed fan-out.
    if (!bHave)
    {
        nSolves++;
        RigidBodyDynamics::Math::VectorNd vq;
        double dErr = 0.0;
        if (SolvePositionIK(m_QReady, avTarget, 0.0, NULL, vq, dErr,
                            IK_SOLVE_STEPS))
        {
            bReachable = true;
            if (Accept(vq, 0.0, dErr, 0, stBest)) { Polish(stBest); bHave = true; }
        }
    }

    // ---- Stage 2: multi-start ----------------------------------------------
    // Stage 1 reached the point but could not use the answer — off the
    // q_ready branch, or past the tilt cone. Relaxing the weight further
    // cannot fix that, because the basin is what is wrong and the weight does
    // not change basins. Re-solve from seeds planted in the OTHER branches and
    // see whether one of them descends into a usable near-branch solution.
    // These seeds are not there to USE a far branch — Accept() still refuses
    // those — they are there to find a near one the first descent missed.
    if (!bHave)
    {
        std::vector<RigidBodyDynamics::Math::VectorNd> vSeeds;
        BuildBranchSeeds(m_QReady, vSeeds);
        std::vector<Cand> vCands;
        static const double adRoundW[] = { IK_ORI_WEIGHT, 0.0 };

        for (int r = 0; r < 2 && vCands.empty(); r++)
        {
            const double dW = adRoundW[r];
            for (size_t s = 1; s < vSeeds.size(); s++)   // 0 was Stage 1
            {
                nSolves++;
                RigidBodyDynamics::Math::VectorNd vq;
                double dErr = 0.0;
                Cand   stC;
                const bool bHit = SolvePositionIK(vSeeds[s], avTarget, dW,
                                                  (dW > 0.0) ? &m_EReady : NULL,
                                                  vq, dErr, IK_SEED_STEPS);
                if (bHit) bReachable = true;
                if (bHit && Accept(vq, dW, dErr, (int)s, stC))
                    vCands.push_back(stC);
            }
        }

        if (!vCands.empty())
        {
            // A TOTAL order, so the answer never depends on an iteration
            // accident: stay on the q_ready branch first, then keep the tool
            // attitude, then land accurately, and break exact ties by seed
            // index (the seed list is fixed, so the call is reproducible).
            size_t nBest = 0;
            for (size_t i = 1; i < vCands.size(); i++)
            {
                const Cand& a = vCands[i];
                const Cand& b = vCands[nBest];
                bool bBetter;
                if      (a.dGap    < b.dGap    - 1e-9)  bBetter = true;
                else if (a.dGap    > b.dGap    + 1e-9)  bBetter = false;
                else if (a.dTilt   < b.dTilt   - 1e-9)  bBetter = true;
                else if (a.dTilt   > b.dTilt   + 1e-9)  bBetter = false;
                else if (a.dPosErr < b.dPosErr - 1e-12) bBetter = true;
                else if (a.dPosErr > b.dPosErr + 1e-12) bBetter = false;
                else                                    bBetter = (a.nSeed < b.nSeed);
                if (bBetter) nBest = i;
            }
            stBest = vCands[nBest];
            Polish(stBest);
            bHave  = true;
            m_stIkDiag.nCandidates = (int)vCands.size();
            DBG_LOG_INFO("(SolveReadyIK) multi-start rescued this target: "
                         "%d candidate(s), seed %d wins", (int)vCands.size(),
                         stBest.nSeed);
        }
    }

    if (!bHave)
    {
        // Report the distinction the operator can act on: out of reach, versus
        // reached but only by flipping the arm or tipping the tool over.
        RigidBodyDynamics::Math::VectorNd vq;
        double dErr = 0.0;
        nSolves++;
        const bool bHit = SolvePositionIK(m_QReady, avTarget, 0.0, NULL, vq,
                                          dErr, IK_SOLVE_STEPS);
        int nAxis = -1;
        const double dGap  = ReadyBranchGap(vq, nAxis);
        const double dTilt = AttitudeDevDeg(vq);
        const eIkVerdict e = (!bHit && !bReachable) ? eIkNoSolution
                           : (dGap > IK_DQ_MAX_RAD) ? eIkBranch : eIkTilt;
        StampIkDiag(e, -1.0, nSolves, tIkStart, vq, dErr, -1, 0);
        if (e == eIkNoSolution)
            DBG_LOG_WARN("(SolveReadyIK) no solution — residual %.1f mm even "
                         "position-only (genuinely out of reach)", dErr * 1e3);
        else if (e == eIkBranch)
            DBG_LOG_WARN("(SolveReadyIK) reachable only on a different arm "
                         "branch (J%d %.2f rad from q_ready) and none of the "
                         "%d seeds found a near-branch solution — refused",
                         nAxis, dGap, IK_NUM_SEEDS);
        else
            DBG_LOG_WARN("(SolveReadyIK) reachable only with extreme tool tilt "
                         "(%.0f deg > %.0f) — refused; move the object closer",
                         dTilt, APPROACH_RDEV_MAX_DEG);
        return FALSE;
    }

    const int nCands = (m_stIkDiag.nCandidates > 0) ? m_stIkDiag.nCandidates : 1;
    aqSol     = stBest.q;
    adPosErrM = stBest.dPosErr;
    StampIkDiag(eIkPass, stBest.dW, nSolves, tIkStart, aqSol, adPosErrM,
                stBest.nSeed, nCands);
    DBG_LOG_INFO("(SolveReadyIK) w=%.2f, seed %d, %d solve(s), gap %.2f rad "
                 "on J%d, tilt %.0f deg, %.1f mm, %.2f ms",
                 stBest.dW, stBest.nSeed, nSolves, stBest.dGap, stBest.nAxis,
                 stBest.dTilt, stBest.dPosErr * 1e3, m_stIkDiag.dMs);
    if (m_stIkDiag.dMs > 0.8)
        DBG_LOG_WARN("(SolveReadyIK) %d IK solve(s) took %.2f ms — over the "
                     "1 kHz cycle budget", nSolves, m_stIkDiag.dMs);

    Finish(aqSol);
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
    // [2026-08-03] hysteresis (on 0.06 / off 0.10): a MOVING goal crosses a
    // single threshold repeatedly, and each crossing was a 0.3 rad/s
    // rot-velocity step softened only by the accel clamp.
    {
        double pos_err_raw = sqrt(m_e_task[3]*m_e_task[3] +
                                  m_e_task[4]*m_e_task[4] +
                                  m_e_task[5]*m_e_task[5]);
        if (m_bZAlignOn) { if (pos_err_raw > 0.10) m_bZAlignOn = false; }
        else             { if (pos_err_raw < 0.06) m_bZAlignOn = true;  }
        if (m_bZAlignOn) {
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

    // [2026-08-03] Cartesian speed hard cap — a person stands next to the arm
    // during tracking (E17). Kp_task_pos = 1.0 makes e_task[3..5] the
    // commanded velocity in m/s, so scaling the error vector IS the limit.
    {
        double dLin = sqrt(m_e_task[3]*m_e_task[3] +
                           m_e_task[4]*m_e_task[4] +
                           m_e_task[5]*m_e_task[5]);
        if (dLin > m_dMaxLinVel) {
            const double dS = m_dMaxLinVel / dLin;
            m_e_task[3] *= dS;
            m_e_task[4] *= dS;
            m_e_task[5] *= dS;
        }
    }

    // 4. DLS pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹
    // manipulability 기반 adaptive damping
    // [2026-08-03] two-state with hysteresis (was a 3-way ladder): the raw
    // w threshold flipped λ by 100x cycle-to-cycle at the boundary, and the
    // near-goal λ=0.0005 high-gain regime bought nothing — accuracy is
    // stiction-bound (~10-15 mm) — while keeping near-singular amplification.
    m_JJt.noalias() = m_J * m_J.transpose();
    double w = std::sqrt(std::max(0.0, m_JJt.determinant()));
    if (m_bNearSingular) { if (w > 0.012) m_bNearSingular = false; }
    else                 { if (w < 0.008) m_bNearSingular = true;  }
    const double lambda = m_bNearSingular ? 0.05 : m_lambda;
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

    // [2026-08-03] joint soft limits — the servo path has no IK acceptance
    // gate (CheckLimitsPhysical never runs here); kill outward velocity
    // within 0.1 rad of a hard stop instead of leaning on it indefinitely.
    // [E36 fix, same day] judge in the PHYSICAL frame: m_Q is the ENCODER
    // COUNTER frame, wound by 2π·k laps (E20 — live session showed J1 −5.8,
    // J4 −12.6). Comparing raw counters blocked J1's negative velocity
    // permanently → TCP followed +y but never −y. Fold toward zero first,
    // exactly like CheckLimitsPhysical. (J6's ±3.75 range exceeds the fold
    // window so the gate is inert there — roll, no position effect.)
    for (unsigned int i = 0; i < m_uDOF && i < 6; ++i)
    {
        const double dQph = m_Q[i] - 2.0 * M_PI *
                            std::round(m_Q[i] / (2.0 * M_PI));
        if (dQph > s_adJointHi[i] - 0.1 && m_Qd_ref[i] > 0.0) m_Qd_ref[i] = 0.0;
        if (dQph < s_adJointLo[i] + 0.1 && m_Qd_ref[i] < 0.0) m_Qd_ref[i] = 0.0;
    }

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

// [2026-08-03] Tracking-servo entry/exit. RT thread only — DoInput and the
// tracking SM both run inside proc_main_control, so nothing interleaves;
// mode is still set LAST as hygiene (references primed before dispatch).
void
CControllerFullDynamicsRT::StartTrackingServo()
{
    ComputeTcpFK();
    goal_tcpPose = tcpPose;      // zero initial error; R FROZEN here — the
                                 // z-align term now HOLDS this attitude
    m_Q_ref   = m_Q;
    m_Qd_ref.setZero();
    m_Qdd_ref.setZero();
    m_Qd_ref_prev.setZero();     // stale from a past session = first-cycle
                                 // lurch through the accel clamp (S5)
    m_dKfScale = 0.0;            // friction FF hard-off: servo-mode lag is
                                 // |qd_ref|·1ms ≤ 0.3 of the E35 gate ramp —
                                 // an untuned regime; stiction costs ~10-15 mm
                                 // under a 200 mm hover margin
    m_bNearSingular = false;
    m_bZAlignOn     = false;
    m_traj.active   = false;     // no quintic fighting per-cycle references
    m_eControlMode  = eTrackingServo;
}

void
CControllerFullDynamicsRT::StopTrackingServo()
{
    // freeze-and-hold, the post-approach idiom: with m_traj inactive,
    // UpdateTrajectory no-ops and CTC holds the parked reference
    m_Q_ref   = m_Q;
    m_Qd_ref.setZero();
    m_Qdd_ref.setZero();
    m_traj.active  = false;
    m_eControlMode = eInverseKinematics_6dof;
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
CControllerFullDynamicsRT::SetFrictionFF(const std::vector<double>& avKf)
{
    if (avKf.size() != m_uDOF)
        return;
    // Clamp to [0, 12] Nm: a cfg typo must not become a torque source. 12 is
    // above the largest measured breakaway (j3 ~10.6 Nm) yet well under the
    // wrist axes' 21 Nm rated limit.
    bool bClamped = false;
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        double d = avKf[i];
        if (d < 0.0 || d > 12.0) { d = (d < 0.0) ? 0.0 : 12.0; bClamped = true; }
        m_Kf[i] = d;
    }
    if (bClamped)
        DBG_LOG_WARN("(%s) Friction FF value out of [0,12] Nm - clamped",
                     "CControllerFullDynamicsRT");
    if (m_uDOF == 6)
        DBG_LOG_INFO("(%s) Friction FF [Nm]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                     "CControllerFullDynamicsRT",
                     m_Kf[0], m_Kf[1], m_Kf[2], m_Kf[3], m_Kf[4], m_Kf[5]);
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
        // ⚠ num_steps is an RBDL *output* (iterations performed); the input limit
    // is max_steps (default 300). This line has never had any effect. Do NOT
    // "fix" it to max_steps=1000: at ~35 us/step that is a 35 ms stall inside
    // the 1 kHz cycle. 300 is what this path has always run at, and it works.
    CS.num_steps      = 1000;
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
            // ⚠ output field, not a limit — see the note above. Effective
            // budget here is RBDL's default max_steps = 300.
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

