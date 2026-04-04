/*****************************************************************************
*	Name: AxisMuJoCo.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of the CAxisMuJoCo class - MuJoCo simulation motor model
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "AxisMuJoCo.h"
#include <algorithm>
#include <cmath>

CAxisMuJoCo::CAxisMuJoCo(eAxisType aeAxisType, int mjJointId, double adEncRes, double adGearRatio, 
						 double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled)
	: CAxis((eCommType)0x20, aeAxisType, abAbsoluteEncoder, abCCW, abEnabled)  // Use eAxisMuJoCo (0x20)
{
	m_nMjJointId = mjJointId;
	m_pMjModel = nullptr;
	m_pMjData = nullptr;
	
	SetResolution(adEncRes, adGearRatio, adTransRatio);
	
	// Initialize control state
	m_eControlMode = CONTROL_POSITION;
	m_bServoOn = FALSE;
	m_dLastUpdateTime = 0.0;
	m_dSimulatedEncoderPos = 0.0;
	
	// Initialize motion profiles (similar to your real motor settings)
	m_dProfileVel = 130.0;  // deg/s from INDY7.cfg
	m_dProfileAcc = 10000.0; // deg/s^2
	m_dProfileDec = 10000.0;
	
	// Initialize PID controllers with reasonable defaults
	// Position PID
	m_pidPosition.kp = 1000.0;  // Adjust based on your system
	m_pidPosition.ki = 0.0;
	m_pidPosition.kd = 50.0;
	m_pidPosition.output_limit = 1000.0; // Nm
	
	// Velocity PID  
	m_pidVelocity.kp = 100.0;
	m_pidVelocity.ki = 0.0;
	m_pidVelocity.kd = 10.0;
	m_pidVelocity.output_limit = 1000.0; // Nm
	
	// Initialize filtered commands
	m_dFilteredTargetPos = 0.0;
	m_dFilteredTargetVel = 0.0;
	m_dFilteredTargetTorque = 0.0;
	
	if (eAxisRevolute != aeAxisType)
	{
		// For non-revolute joints, remember to set appropriate OneTurnRef
	}
}

CAxisMuJoCo::~CAxisMuJoCo()
{
	DeInit();
}

BOOL CAxisMuJoCo::Init(mjModel* model, mjData* data)
{
	if (eAxisInit <= GetState()) return TRUE;
	
	if (!model || !data)
	{
		SetState(eAxisError);
		return FALSE;
	}
	
	if (m_nMjJointId < 0 || m_nMjJointId >= model->njnt)
	{
		SetState(eAxisError);
		return FALSE;
	}
	
	// Check if axis limits are configured
	if (!IsLimitConfigured())
	{
		SetState(eAxisError);
		return FALSE;
	}
	
	m_pMjModel = model;
	m_pMjData = data;
	m_dLastUpdateTime = data->time;
	
	// Initialize simulated encoder position from MuJoCo
	m_dSimulatedEncoderPos = ConvertRadMM2Res(data->qpos[m_nMjJointId]);
	
	// Set initial home position if using start position homing
	if (GetHomingMethod() == eAxisHomeStartPos)
	{
		SetHomePosition(m_dSimulatedEncoderPos);
	}
	
	SetState(eAxisInit);
	return TRUE;
}

BOOL CAxisMuJoCo::DeInit()
{
	m_pMjModel = nullptr;
	m_pMjData = nullptr;
	m_bServoOn = FALSE;
	SetState(eAxisDeinit);
	return TRUE;
}

BOOL CAxisMuJoCo::ServoOn()
{
	if (!m_pMjModel || !m_pMjData || !IsEnabled())
		return FALSE;
		
	m_bServoOn = TRUE;
	SetState(eAxisIdle);
	return TRUE;
}

BOOL CAxisMuJoCo::ServoOff()
{
	m_bServoOn = FALSE;
	SetState(eAxisInit);
	return TRUE;
}

BOOL CAxisMuJoCo::IsServoOn()
{
	return m_bServoOn;
}

BOOL CAxisMuJoCo::DoHoming()
{
	if (!IsServoOn()) return FALSE;
	
	SetState(eAxisHoming);
	
	switch (GetHomingMethod())
	{
	case eAxisHomeStartPos:
		// Use current position as home
		SetHomePosition(GetCurrentRawPos());
		SetState(eAxisIdle);
		return TRUE;
		
	case eAxisHomeSearch:
		// Simulate homing search - move to a reference position
		if (IsHomeReferenceSet())
		{
			// Move to home reference position
			MoveAxis(GetHomeReference(), TRUE);
			SetHomePosition(ConvertRadMM2Res(GetHomeReference()));
			SetState(eAxisIdle);
			return TRUE;
		}
		break;
		
	case eAxisHomeManual:
		// Manual homing - use preset home position
		if (IsHomeSet())
		{
			MoveAxis(ConvertRes2RadMM(GetHomePosition()), TRUE);
			SetState(eAxisIdle);
			return TRUE;
		}
		break;
		
	default:
		SetState(eAxisError);
		return FALSE;
	}
	
	SetState(eAxisError);
	return FALSE;
}

BOOL CAxisMuJoCo::MoveAxis(double adTarget, BOOL abForce, BOOL abAbs)
{
	if (!IsServoOn() && !abForce) return FALSE;
	
	// Check position limits
	if (!abForce && !IsAllowablePosition(adTarget))
		return FALSE;
	
	double targetPos = adTarget;
	
	// Handle absolute vs relative positioning
	if (abAbs && IsHomeSet())
	{
		// Absolute positioning from home
		targetPos = adTarget;
	}
	else if (!abAbs)
	{
		// Relative positioning
		targetPos = GetCurrentPos() + adTarget;
	}
	
	m_dTargetPos = targetPos;
	m_eControlMode = CONTROL_POSITION;
	SetState(eAxisRunning);
	
	return TRUE;
}

BOOL CAxisMuJoCo::MoveHome(BOOL abForced)
{
	if (!IsHomeSet()) return FALSE;
	
	double homePos = ConvertRes2RadMM(GetHomePosition());
	return MoveAxis(homePos, abForced);
}

BOOL CAxisMuJoCo::StopAxis()
{
	if (!IsServoOn()) return FALSE;
	
	// Set current position as target to stop motion
	m_dTargetPos = GetCurrentPos();
	m_dTargetVel = 0.0;
	m_dTargetTorq = 0.0;
	
	SetState(eAxisStopped);
	return TRUE;
}

BOOL CAxisMuJoCo::EmgStopAxis()
{
	// Emergency stop - immediately set zero torque
	m_dTargetTorq = 0.0;
	m_dFilteredTargetTorque = 0.0;
	
	if (m_pMjData)
	{
		m_pMjData->ctrl[m_nMjJointId] = 0.0;
	}
	
	SetState(eAxisEmergency);
	return TRUE;
}

BOOL CAxisMuJoCo::MoveVelocity(double adVel)
{
	if (!IsServoOn()) return FALSE;
	
	if (!IsAllowableVelocity(adVel))
		return FALSE;
	
	m_dTargetVel = adVel;
	m_eControlMode = CONTROL_VELOCITY;
	SetState(eAxisRunning);
	
	return TRUE;
}

BOOL CAxisMuJoCo::MoveTorque(double adTorque)
{
	// if (!IsServoOn()) return FALSE;
	
	// if (!IsAllowableTorque(adTorque))
		// return FALSE;
	
	m_dTargetTorq = adTorque;
	// m_eControlMode = CONTROL_TORQUE;
	// SetState(eAxisRunning);
	
	return TRUE;
}

BOOL CAxisMuJoCo::SetVelocity(double adVel)
{
	m_dProfileVel = std::abs(adVel);
	return TRUE;
}

BOOL CAxisMuJoCo::SetAcceleration(double adAcc)
{
	m_dProfileAcc = std::abs(adAcc);
	return TRUE;
}

BOOL CAxisMuJoCo::SetDeceleration(double adDec)
{
	m_dProfileDec = std::abs(adDec);
	return TRUE;
}

void CAxisMuJoCo::SetMuJoCoModel(mjModel* model, mjData* data)
{
	m_pMjModel = model;
	m_pMjData = data;
}

void CAxisMuJoCo::UpdateFromMuJoCo()
{
	if (!m_pMjModel || !m_pMjData) return;
	
	double currentTime = m_pMjData->time;
	double dt = currentTime - m_dLastUpdateTime;
	m_dLastUpdateTime = currentTime;
	
	// Update simulated encoder position
	UpdateSimulatedEncoder();
	
	// Update current parameters from MuJoCo state
	double currentPos = m_pMjData->qpos[m_nMjJointId];
	double currentVel = m_pMjData->qvel[m_nMjJointId];
	double currentTorque = m_pMjData->qfrc_actuator[m_nMjJointId];
	
	// Convert to encoder units and update
	INT32 rawPos = ConvertRadMM2Res(currentPos);
	INT32 rawVel = ConvertRadMM2Res(currentVel);
	INT32 rawTor = ConvertTor2Res(currentTorque);
	
	UpdateCurrentParams(rawPos, rawVel, rawTor);
	
	// Update state based on motion
	if (IsServoOn())
	{
		if (std::abs(currentVel) < 1e-6)  // Very small velocity threshold
		{
			if (GetState() == eAxisRunning)
				SetState(eAxisIdle);
		}
		else
		{
			if (GetState() == eAxisIdle)
				SetState(eAxisRunning);
		}
	}
}

void CAxisMuJoCo::ApplyToMuJoCo()
{
	if (!m_pMjModel || !m_pMjData || !IsServoOn()) return;
	
	double dt = m_pMjData->time - m_dLastUpdateTime;
	if (dt <= 0) return;
	
	double outputTorque = 0.0;
	
	switch (m_eControlMode)
	{
	case CONTROL_POSITION:
		ComputePIDPosition(dt);
		outputTorque = m_pidPosition.integral;  // PID output
		break;
		
	case CONTROL_VELOCITY:
		ComputePIDVelocity(dt);
		outputTorque = m_pidVelocity.integral;  // PID output
		break;
		
	case CONTROL_TORQUE:
		outputTorque = m_dTargetTorq;
		break;
	}
	
	// Apply torque limits
	ApplyTorqueLimits(outputTorque);
	
	// Apply to MuJoCo control
	m_pMjData->ctrl[m_nMjJointId] = outputTorque;
}

void CAxisMuJoCo::ComputePIDPosition(double dt)
{
	double currentPos = GetCurrentPos();
	double error = m_dTargetPos - currentPos;
	
	// Proportional term
	double pTerm = m_pidPosition.kp * error;
	
	// Integral term
	m_pidPosition.integral += m_pidPosition.ki * error * dt;
	
	// Derivative term
	double dTerm = m_pidPosition.kd * (error - m_pidPosition.prev_error) / dt;
	m_pidPosition.prev_error = error;
	
	// Combine PID terms
	double output = pTerm + m_pidPosition.integral + dTerm;
	
	// Apply output limits
	output = std::max(-m_pidPosition.output_limit, std::min(m_pidPosition.output_limit, output));
	
	m_pidPosition.integral = output;  // Store for ApplyToMuJoCo
}

void CAxisMuJoCo::ComputePIDVelocity(double dt)
{
	double currentVel = GetCurrentVel();
	double error = m_dTargetVel - currentVel;
	
	// Proportional term
	double pTerm = m_pidVelocity.kp * error;
	
	// Integral term  
	m_pidVelocity.integral += m_pidVelocity.ki * error * dt;
	
	// Derivative term
	double dTerm = m_pidVelocity.kd * (error - m_pidVelocity.prev_error) / dt;
	m_pidVelocity.prev_error = error;
	
	// Combine PID terms
	double output = pTerm + m_pidVelocity.integral + dTerm;
	
	// Apply output limits
	output = std::max(-m_pidVelocity.output_limit, std::min(m_pidVelocity.output_limit, output));
	
	m_pidVelocity.integral = output;  // Store for ApplyToMuJoCo
}

void CAxisMuJoCo::ApplyTorqueLimits(double& torque)
{
	ST_AXIS_LIMITS limits = GetAxisLimits();
	if (limits.stTorque.bIsSet)
	{
		torque = std::max(limits.stTorque.dLower, std::min(limits.stTorque.dUpper, torque));
	}
}

void CAxisMuJoCo::UpdateSimulatedEncoder()
{
	if (!m_pMjData) return;
	
	// Update simulated encoder position from MuJoCo joint position
	double mjPos = m_pMjData->qpos[m_nMjJointId];
	m_dSimulatedEncoderPos = ConvertRadMM2Res(mjPos);
}

void CAxisMuJoCo::SetState(eAxisState aeState)
{
	eAxisState ePrevState = GetState();
	CAxis::SetState(aeState);
	
	// Add MuJoCo-specific state change handling if needed
	if (ePrevState != aeState)
	{
		// Could add logging or additional state-specific actions here
	}
}

void CAxisMuJoCo::SetPositionControl()
{
	m_eControlMode = CONTROL_POSITION;
}

void CAxisMuJoCo::SetVelocityControl()
{
	m_eControlMode = CONTROL_VELOCITY;
}

void CAxisMuJoCo::SetTorqueControl()
{
	m_eControlMode = CONTROL_TORQUE;
}

double CAxisMuJoCo::GetMuJoCoTime()
{
	return m_pMjData ? m_pMjData->time : 0.0;
}
