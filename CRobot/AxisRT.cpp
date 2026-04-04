/*****************************************************************************
*	Name: AxisRT.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Real-time axis simulation implementation
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "AxisRT.h"
#include <algorithm>
#include <cmath>

CAxisRT::CAxisRT(eAxisType aeAxisType, double adEncRes, double adGearRatio, 
				 double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled)
	: CAxis((eCommType)0x20, aeAxisType, abAbsoluteEncoder, abCCW, abEnabled)  // Use eAxisMuJoCo (0x20)
{
	SetResolution(adEncRes, adGearRatio, adTransRatio);
	
	// Initialize control state
	m_eControlMode = CONTROL_POSITION;
	m_bServoOn = FALSE;
	m_dSimTime = 0.0;
	m_dLastUpdateTime = 0.0;
	m_dSimulatedEncoderPos = 0.0;
	m_dExternalTorque = 0.0;
	m_dMotorTorque = 0.0;
	m_dMotorCurrent = 0.0;
	
	// Initialize motion profiles (similar to your real motor settings)
	m_dProfileVel = 130.0;  // deg/s from INDY7.cfg
	m_dProfileAcc = 10000.0; // deg/s^2
	m_dProfileDec = 10000.0;
	
	// Initialize PID controllers with reasonable defaults for RT
	// Position PID - tuned for responsiveness without overshoot
	m_pidPosition.kp = 800.0;  // From your INDY7.cfg KP values
	m_pidPosition.ki = 0.1;    // Small integral gain
	m_pidPosition.kd = 80.0;   // From your INDY7.cfg KD values
	m_pidPosition.output_limit = 100.0; // Nm
	m_pidPosition.integral_limit = 10.0;
	
	// Velocity PID - similar to position but faster response
	m_pidVelocity.kp = 50.0;
	m_pidVelocity.ki = 0.05;
	m_pidVelocity.kd = 5.0;
	m_pidVelocity.output_limit = 100.0; // Nm
	m_pidVelocity.integral_limit = 10.0;
	
	// Initialize motor dynamics - typical values for robot joints
	m_motorDynamics.inertia = 0.001;  // kg*m^2 - will be set based on robot
	m_motorDynamics.damping = 0.01;   // N*m*s/rad
	m_motorDynamics.friction = 0.001; // N*m
	
	// Initialize filtered commands
	m_dFilteredTargetPos = 0.0;
	m_dFilteredTargetVel = 0.0;
	m_dFilteredTargetTorque = 0.0;
	
	if (eAxisRevolute != aeAxisType)
	{
		// For non-revolute joints, remember to set appropriate OneTurnRef
	}
}

CAxisRT::~CAxisRT()
{
	DeInit();
}

BOOL CAxisRT::Init()
{
	printf("[CAxisRT::Init] Called\n");
	if (eAxisInit <= GetState()) {
		printf("[CAxisRT::Init] Already initialized (state=%d)\n", GetState());
		return TRUE;
	}
	// Print axis limits
	ST_AXIS_LIMITS lim = GetAxisLimits();
	printf("[CAxisRT::Init] Axis limits: Pos(bIsSet=%d, [%.2f, %.2f]), Vel(bIsSet=%d, [%.2f, %.2f]), Acc(bIsSet=%d, [%.2f, %.2f]), Dec(bIsSet=%d, [%.2f, %.2f]), Jerk(bIsSet=%d, [%.2f, %.2f]), Torque(bIsSet=%d, [%.2f, %.2f])\n",
		lim.stPos.bIsSet, lim.stPos.dLower, lim.stPos.dUpper,
		lim.stVel.bIsSet, lim.stVel.dLower, lim.stVel.dUpper,
		lim.stAcc.bIsSet, lim.stAcc.dLower, lim.stAcc.dUpper,
		lim.stDec.bIsSet, lim.stDec.dLower, lim.stDec.dUpper,
		lim.stJerk.bIsSet, lim.stJerk.dLower, lim.stJerk.dUpper,
		lim.stTorque.bIsSet, lim.stTorque.dLower, lim.stTorque.dUpper);
	// Check if axis limits are configured
	if (!IsLimitConfigured())
	{
		printf("[CAxisRT::Init] FAILURE: Axis limits not configured!\n");
		SetState(eAxisError);
		return FALSE;
	}
	// Initialize simulation time
	m_dSimTime = 0.0;
	m_dLastUpdateTime = 0.0;
	// Initialize motor position to current encoder position
	m_motorDynamics.position = ConvertRes2RadMM(GetCurrentRawPos());
	m_motorDynamics.velocity = 0.0;
	m_dSimulatedEncoderPos = GetCurrentRawPos();
	// Set initial home position if using start position homing
	if (GetHomingMethod() == eAxisHomeStartPos)
	{
		SetHomePosition(m_dSimulatedEncoderPos);
	}
	SetState(eAxisInit);
	printf("[CAxisRT::Init] SUCCESS\n");
	return TRUE;
}

BOOL CAxisRT::DeInit()
{
	m_bServoOn = FALSE;
	SetState(eAxisDeinit);
	return TRUE;
}

BOOL CAxisRT::ServoOn()
{
	if (!IsEnabled())
		return FALSE;
		
	m_bServoOn = TRUE;
	
	// Reset PID controllers when servo turns on
	m_pidPosition.reset();
	m_pidVelocity.reset();
	
	SetState(eAxisIdle);
	return TRUE;
}

BOOL CAxisRT::ServoOff()
{
	m_bServoOn = FALSE;
	m_dMotorTorque = 0.0;
	m_dMotorCurrent = 0.0;
	SetState(eAxisInit);
	return TRUE;
}

BOOL CAxisRT::IsServoOn()
{
	return m_bServoOn;
}

BOOL CAxisRT::DoHoming()
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

BOOL CAxisRT::MoveAxis(double adTarget, BOOL abForce, BOOL abAbs)
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
// Only the debug version of CAxisRT::Init() should remain (already present)
// ...existing code...
	
	SetState(eAxisEmergency);
	return TRUE;
}

BOOL CAxisRT::MoveVelocity(double adVel)
{
	if (!IsServoOn()) return FALSE;
	
	if (!IsAllowableVelocity(adVel))
		return FALSE;
	
	m_dTargetVel = adVel;
	m_eControlMode = CONTROL_VELOCITY;
	SetState(eAxisRunning);
	
	return TRUE;
}

BOOL CAxisRT::MoveTorque(double adTorque)
{
    // if (!IsServoOn()) return FALSE;
	
	// if (!IsAllowableTorque(adTorque))
	// 	return FALSE;
	
    // printf("[CAxisRT::MoveTorque] Moving to torque: %lf Nm\n", adTorque);
	m_dTargetTorq = adTorque;
    // m_pstAxisParams->dTor = adTorque; // Update current parameters
    
	// m_eControlMode = CONTROL_TORQUE;
	SetState(eAxisRunning);
	
	return TRUE;
}

BOOL CAxisRT::SetVelocity(double adVel)
{
	m_dProfileVel = std::abs(adVel);
	return TRUE;
}

BOOL CAxisRT::SetAcceleration(double adAcc)
{
	m_dProfileAcc = std::abs(adAcc);
	return TRUE;
}

BOOL CAxisRT::SetDeceleration(double adDec)
{
	m_dProfileDec = std::abs(adDec);
	return TRUE;
}

void CAxisRT::SetExternalTorque(double torque)
{
	m_dExternalTorque = torque;
}

void CAxisRT::SetMotorParameters(double inertia, double damping, double friction)
{
	m_motorDynamics.inertia = inertia;
	m_motorDynamics.damping = damping;
	m_motorDynamics.friction = friction;
}

// CRITICAL: This is the RT-safe simulation step
void CAxisRT::StepSimulation(double dt)
{
	if (!IsServoOn() || dt <= 0) return;
	
	m_dSimTime += dt;
	
	// Compute control output based on mode
	double controlTorque = 0.0;
	
	switch (m_eControlMode)
	{
	case CONTROL_POSITION:
		{
			double posError = m_dTargetPos - m_motorDynamics.position;
			controlTorque = m_pidPosition.compute(posError, dt);
		}
		break;
		
	case CONTROL_VELOCITY:
		{
			double velError = m_dTargetVel - m_motorDynamics.velocity;
			controlTorque = m_pidVelocity.compute(velError, dt);
		}
		break;
		
	case CONTROL_TORQUE:
		controlTorque = m_dTargetTorq;
		break;
	}
	
	// Apply torque limits
	ApplyTorqueLimits(controlTorque);
	m_dMotorTorque = controlTorque;
	
	// Update motor dynamics (simplified but realistic)
	UpdateMotorDynamics(dt);
	
	// Update simulated encoder
	UpdateSimulatedEncoder();
	
	// Convert motor current from torque
	m_dMotorCurrent = ConvertTor2Cur(m_dMotorTorque);
	
	// Update current parameters for base class
	INT32 rawPos = ConvertRadMM2Res(m_motorDynamics.position);
	INT32 rawVel = ConvertRadMM2Res(m_motorDynamics.velocity);
	INT32 rawTor = ConvertTor2Res(m_dMotorTorque);
	
	UpdateCurrentParams(rawPos, rawVel, rawTor);
	
	// Update state based on motion
	if (IsServoOn())
	{
		if (std::abs(m_motorDynamics.velocity) < 1e-6)  // Very small velocity threshold
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

void CAxisRT::UpdateMotorDynamics(double dt)
{
	// Simple motor dynamics: J*a = T_motor + T_external - B*v - T_friction
	double frictionTorque = (m_motorDynamics.velocity > 0) ? -m_motorDynamics.friction : 
						   (m_motorDynamics.velocity < 0) ? m_motorDynamics.friction : 0;
	
	double totalTorque = m_dMotorTorque + m_dExternalTorque - 
						 m_motorDynamics.damping * m_motorDynamics.velocity - frictionTorque;
	
	// Compute acceleration
	m_motorDynamics.acceleration = totalTorque / m_motorDynamics.inertia;
	
	// Integrate velocity
	m_motorDynamics.velocity += m_motorDynamics.acceleration * dt;
	
	// Integrate position
	m_motorDynamics.position += m_motorDynamics.velocity * dt;
	
	// Apply position limits (hard stops)
	ST_AXIS_LIMITS limits = GetAxisLimits();
	if (limits.stPos.bIsSet)
	{
		if (m_motorDynamics.position > limits.stPos.dUpper)
		{
			m_motorDynamics.position = limits.stPos.dUpper;
			if (m_motorDynamics.velocity > 0) m_motorDynamics.velocity = 0;
		}
		if (m_motorDynamics.position < limits.stPos.dLower)
		{
			m_motorDynamics.position = limits.stPos.dLower;
			if (m_motorDynamics.velocity < 0) m_motorDynamics.velocity = 0;
		}
	}
}

void CAxisRT::ApplyTorqueLimits(double& torque)
{
	ST_AXIS_LIMITS limits = GetAxisLimits();
	if (limits.stTorque.bIsSet)
	{
		torque = std::max(limits.stTorque.dLower, std::min(limits.stTorque.dUpper, torque));
	}
}

void CAxisRT::UpdateSimulatedEncoder()
{
	// Update simulated encoder position from motor dynamics
	m_dSimulatedEncoderPos = ConvertRadMM2Res(m_motorDynamics.position);
}

void CAxisRT::SetState(eAxisState aeState)
{
	eAxisState ePrevState = GetState();
	CAxis::SetState(aeState);
	
	// Add RT-specific state change handling if needed
	if (ePrevState != aeState)
	{
		// Could add logging or additional state-specific actions here
	}
}

void CAxisRT::SetPositionControl()
{
	m_eControlMode = CONTROL_POSITION;
	m_pidPosition.reset();  // Reset PID state when switching modes
}

void CAxisRT::SetVelocityControl()
{
	m_eControlMode = CONTROL_VELOCITY;
	m_pidVelocity.reset();  // Reset PID state when switching modes
}

void CAxisRT::SetTorqueControl()
{
	m_eControlMode = CONTROL_TORQUE;
}

// --- Linker error fix: Provide required method definitions ---
BOOL CAxisRT::MoveHome(BOOL abForced) {
	// Default stub or call base if needed
	return FALSE;
}

BOOL CAxisRT::StopAxis() {
	// Default stub or call base if needed
	return FALSE;
}

BOOL CAxisRT::EmgStopAxis() {
	// Default stub or call base if needed
	return FALSE;
}