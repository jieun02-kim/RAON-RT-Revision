/*****************************************************************************
*	Name: AxisRTMuJoCo.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Hybrid RT control with MuJoCo visualization implementation
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "AxisRTMuJoCo.h"
#include <mujoco/mujoco.h>
#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#endif

CAxisRTMuJoCo::CAxisRTMuJoCo(eAxisType aeAxisType, int mjJointId, double adEncRes, 
							 double adGearRatio, double adTransRatio, 
							 BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled)
	: CAxisRT(aeAxisType, adEncRes, adGearRatio, adTransRatio, abAbsoluteEncoder, abCCW, abEnabled)
{
	m_nMjJointId = mjJointId;
	m_pMjModel = nullptr;
	m_pMjData = nullptr;
	
	m_bVisualizationEnabled = false;
	m_bMuJoCoInitialized = false;
	
	m_dLastVizUpdateTime = 0.0;
	m_dVizUpdateRate = 60.0;  // 60Hz visualization rate
}

CAxisRTMuJoCo::~CAxisRTMuJoCo()
{
	// Don't delete MuJoCo objects here - they're managed externally
}

bool CAxisRTMuJoCo::InitMuJoCo(mjModel* model, mjData* data)
{
	if (!model || !data) return false;
	
	if (m_nMjJointId < 0 || m_nMjJointId >= model->njnt) return false;
	
	m_pMjModel = model;
	m_pMjData = data;
	m_bMuJoCoInitialized = true;
	
	return true;
}

// CRITICAL: This runs in RT thread - must be deterministic
void CAxisRTMuJoCo::StepSimulation(double dt)
{
	// First, update any external forces from visualization thread
	UpdateExternalForcesFromVisualization();
	
	// Run the RT-safe simulation step
	CAxisRT::StepSimulation(dt);
	
	// Publish state to visualization thread (lock-free)
	if (m_bVisualizationEnabled.load())
	{
		PublishStateToVisualization();
	}
}

// RT-safe: No locks, just atomic writes
void CAxisRTMuJoCo::PublishStateToVisualization()
{
m_rtToVizState.joint_positions[0].store(m_motorDynamics.position);
m_rtToVizState.joint_velocities[0].store(m_motorDynamics.velocity);
m_rtToVizState.joint_torques[0].store(m_dMotorTorque);
m_rtToVizState.timestamp.store(m_dSimTime);
m_rtToVizState.data_updated.store(true);
}

// RT-safe: No locks, just atomic reads
void CAxisRTMuJoCo::UpdateExternalForcesFromVisualization()
{
if (m_vizToRTState.data_updated.load())
{
	// No external_torque/external_force fields in RTVisualizationData; skip or implement as needed
	m_vizToRTState.data_updated.store(false);
}
}

// Called from visualization thread - can use MuJoCo
void CAxisRTMuJoCo::UpdateMuJoCoVisualization()
{
#ifdef MUJOCO_ENABLED
	if (!m_bMuJoCoInitialized.load() || !m_pMjData) return;
	
// Check if RT thread has new data
if (!m_rtToVizState.data_updated.load()) return;
	
	// Get state from RT thread (lock-free)
	double pos, vel, torque, timestamp;
	GetVisualizationState(pos, vel, torque, timestamp);
	
	// Apply to MuJoCo for visualization
	m_pMjData->qpos[m_nMjJointId] = pos;
	m_pMjData->qvel[m_nMjJointId] = vel;
	m_pMjData->ctrl[m_nMjJointId] = torque;
	
// Mark as processed
m_rtToVizState.data_updated.store(false);
#endif
}

// Called from visualization thread
void CAxisRTMuJoCo::StepMuJoCoPhysics()
{
#ifdef MUJOCO_ENABLED
	if (!m_bMuJoCoInitialized.load()) return;
	
	// Step MuJoCo to compute external forces (contacts, gravity, etc.)
	mj_step(m_pMjModel, m_pMjData);
	
	// Extract external forces and send to RT thread
	double externalTorque = m_pMjData->qfrc_applied[m_nMjJointId];
	SetExternalForcesFromMuJoCo(externalTorque, 0, 0, 0);
#endif
}

void CAxisRTMuJoCo::GetVisualizationState(double& pos, double& vel, double& torque, double& time)
{
pos = m_rtToVizState.joint_positions[0].load();
vel = m_rtToVizState.joint_velocities[0].load();
torque = m_rtToVizState.joint_torques[0].load();
time = m_rtToVizState.timestamp.load();
}

void CAxisRTMuJoCo::SetExternalForcesFromMuJoCo(double torque, double fx, double fy, double fz)
{
// No external_torque/external_force fields in RTVisualizationData; skip or implement as needed
m_vizToRTState.data_updated.store(true);
}

// Multi-axis robot controller implementation
CRobotRTMuJoCo::CRobotRTMuJoCo()
{
	m_bRTInitialized = false;
	m_bMuJoCoReady = false;
	m_nNumAxes = 0;
}

void CRobotRTMuJoCo::SetAxes(std::vector<std::unique_ptr<CAxisRTMuJoCo>>&& axes)
{
	m_nNumAxes = axes.size();
	m_axes = std::move(axes);
}
CRobotRTMuJoCo::~CRobotRTMuJoCo()
{
	Shutdown();
}

bool CRobotRTMuJoCo::InitRT()
{
	// Configure axes with INDY7-like parameters
	struct GearConfig {
		double gear;
		double torque_const;
		double current_ratio;
		double torque_limit;
	};
	std::vector<GearConfig> configs = {
		{121.0, 0.0884, 48.0, 117.73},  // Joint 0-1
		{121.0, 0.0884, 48.0, 117.73},
		{121.0, 0.087,  96.0, 47.5},    // Joint 2
		{101.0, 0.058,  96.0, 21.0},    // Joint 3-5
		{101.0, 0.058,  96.0, 21.0},
		{101.0, 0.058,  96.0, 21.0}
	};

	printf("[CRobotRTMuJoCo::InitRT] m_nNumAxes = %d\n", m_nNumAxes);
	for (int i = 0; i < m_nNumAxes && i < (int)configs.size(); ++i)
	{
		auto& axis = m_axes[i];
		printf("[CRobotRTMuJoCo::InitRT] Axis %d: gear=%.2f, torque_const=%.4f, current_ratio=%.2f, torque_limit=%.2f\n", i, configs[i].gear, configs[i].torque_const, configs[i].current_ratio, configs[i].torque_limit);

		// Set motor parameters
		axis->SetTorqueConstant(configs[i].torque_const);
		axis->SetCurrentRatio(configs[i].current_ratio);
		axis->SetTorqueLimits(configs[i].torque_limit);

		// Set position limits (degrees)
		axis->SetPositionLimits(-35.0, 35.0, false);
		axis->SetVelocityLimits(150.0);

		// Set motion parameters
		axis->SetVelocity(130.0);
		axis->SetAcceleration(10000.0);

		// Set homing
		axis->SetHomingMethod(eAxisHomeStartPos);

		// Initialize
		bool axisInit = axis->Init();
		printf("[CRobotRTMuJoCo::InitRT] axis->Init() for axis %d returned %s\n", i, axisInit ? "true" : "false");
		if (!axisInit)
		{
			printf("[CRobotRTMuJoCo::InitRT] FAILURE: axis %d failed to initialize!\n", i);
			return false;
		}

		// Auto servo on
		axis->ServoOn();
	}

	m_bRTInitialized = true;
	printf("[CRobotRTMuJoCo::InitRT] All axes initialized successfully.\n");
	return true;
}

bool CRobotRTMuJoCo::InitMuJoCo(mjModel* model, mjData* data)
{
	if (!model || !data) return false;
	
	// Initialize MuJoCo for all axes
	for (int i = 0; i < m_nNumAxes; ++i)
	{
		if (!m_axes[i]->InitMuJoCo(model, data))
		{
			return false;
		}
		m_axes[i]->SetVisualizationEnabled(true);
	}
	
	m_bMuJoCoReady = true;
	return true;
}

// CRITICAL: Call this from RT thread at 1kHz
void CRobotRTMuJoCo::StepRTSimulation(double dt)
{
	if (!m_bRTInitialized.load()) return;
	
	for (auto& axis : m_axes)
	{
		axis->StepSimulation(dt);
	}
}

// Call this from visualization thread at ~60Hz
void CRobotRTMuJoCo::UpdateVisualization()
{
	if (!m_bMuJoCoReady.load()) return;
	
	for (auto& axis : m_axes)
	{
		axis->UpdateMuJoCoVisualization();
	}
}

void CRobotRTMuJoCo::StepMuJoCoPhysics()
{
	if (!m_bMuJoCoReady.load()) return;
	
	// Only need to step once for the whole model
	if (!m_axes.empty())
	{
		m_axes[0]->StepMuJoCoPhysics();
	}
}

// RT-safe joint commands
void CRobotRTMuJoCo::SetJointCommand(int jointId, double position)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		m_axes[jointId]->MoveAxis(position);
	}
}

void CRobotRTMuJoCo::SetJointVelocityCommand(int jointId, double velocity)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		m_axes[jointId]->MoveVelocity(velocity);
	}
}

void CRobotRTMuJoCo::SetJointTorqueCommand(int jointId, double torque)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		m_axes[jointId]->MoveTorque(torque);
	}
}

// RT-safe state queries
double CRobotRTMuJoCo::GetJointPosition(int jointId)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		return m_axes[jointId]->GetCurrentPos();
	}
	return 0.0;
}

double CRobotRTMuJoCo::GetJointVelocity(int jointId)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		return m_axes[jointId]->GetCurrentVel();
	}
	return 0.0;
}

double CRobotRTMuJoCo::GetJointTorque(int jointId)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		return m_axes[jointId]->GetTargetTorq();
	}
	return 0.0;
}

bool CRobotRTMuJoCo::IsJointReady(int jointId)
{
	if (jointId >= 0 && jointId < m_nNumAxes)
	{
		return m_axes[jointId]->IsServoOn();
	}
	return false;
}

void CRobotRTMuJoCo::SetVisualizationEnabled(bool enabled)
{
	for (auto& axis : m_axes)
	{
		axis->SetVisualizationEnabled(enabled);
	}
}

void CRobotRTMuJoCo::UpdateMuJoCoFromRT()
{
	// This is called from proc_ethercat_control - handles communication 
	// between RT simulation and MuJoCo model for visualization
	if (!m_bMuJoCoReady.load())
		return;
		
	// Update each axis visualization state from RT simulation
	for (auto& axis : m_axes)
	{
		if (axis)
		{
			axis->PublishStateToVisualization();
		}
	}
}

void CRobotRTMuJoCo::Shutdown()
{
	for (auto& axis : m_axes)
	{
		axis->ServoOff();
		axis->DeInit();
	}
	m_bRTInitialized = false;
	m_bMuJoCoReady = false;
}
