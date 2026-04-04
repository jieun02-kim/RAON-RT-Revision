/*****************************************************************************
*	Name: AxisMuJoCo.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CAxisMuJoCo class - MuJoCo simulation motor model
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __AXIS__MUJOCO__
#define __AXIS__MUJOCO__

#include "Axis.h"
// Note: MuJoCo headers should be included in the implementation file
// #include <mujoco/mujoco.h>

// Forward declarations for MuJoCo types
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

class CAxisMuJoCo : public CAxis
{
public:
	CAxisMuJoCo(eAxisType aeAxisType, int mjJointId, double adEncRes = 65536, double adGearRatio = 1.0, 
				double adTransRatio = 1.0, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE);
	virtual ~CAxisMuJoCo();

public:
	// Override core axis functions for MuJoCo simulation
	virtual	BOOL	Init					(mjModel* model, mjData* data);
	virtual	BOOL	DeInit					();
	virtual BOOL	DoHoming				();
	virtual BOOL	MoveAxis				(double adTarget, BOOL abForce = FALSE, BOOL abAbs = TRUE);
	virtual BOOL	MoveHome				(BOOL abForced = FALSE);
	virtual BOOL	StopAxis				();
	virtual BOOL	EmgStopAxis				();
	virtual BOOL	MoveVelocity			(double adVel);
	virtual BOOL	MoveTorque				(double adTorque);
	
	// Servo control (simulated)
	virtual BOOL	ServoOn					();
	virtual BOOL	ServoOff				();
	virtual BOOL	IsServoOn				();
	
	// Motion parameter setters
	virtual BOOL	SetVelocity				(double adVel);
	virtual BOOL	SetAcceleration			(double adAcc);
	virtual BOOL	SetDeceleration			(double adDec);
	
	// Real-time MuJoCo interface methods
	void			SetMuJoCoModel			(mjModel* model, mjData* data);
	void			UpdateFromMuJoCo		();  // Update axis state from MuJoCo simulation (RT-safe)
	void			ApplyToMuJoCo			();  // Apply commands to MuJoCo simulation (RT-safe)
	
	// Alternative: Direct state interface (bypassing MuJoCo for RT)
	void			SetSimulatedState		(double pos, double vel, double torque, double time);
	void			GetControlOutput		(double& torque_out);
	
	// Control mode setters
	void			SetPositionControl		();
	void			SetVelocityControl		();
	void			SetTorqueControl		();
	
	// Target getters (for debugging/monitoring)
	virtual double	GetTargetPos			() { return m_dTargetPos; }
	virtual double	GetTargetVel			() { return m_dTargetVel; }
	virtual double	GetTargetTorq			() { return m_dTargetTorq; }
	
	// MuJoCo state accessors
	int				GetMuJoCoJointId		() { return m_nMjJointId; }
	double			GetMuJoCoTime			();
	
protected:
	// Internal control methods
	void			ComputePIDPosition		(double dt);
	void			ComputePIDVelocity		(double dt);
	void			ApplyTorqueLimits		(double& torque);
	void			UpdateSimulatedEncoder	();
	
	// Override state management
	virtual void	SetState				(eAxisState aeState);

protected:
	// MuJoCo interface
	mjModel*		m_pMjModel;
	mjData*			m_pMjData;
	int				m_nMjJointId;			// MuJoCo joint ID
	
	// Control state
	enum eControlMode {
		CONTROL_POSITION,
		CONTROL_VELOCITY, 
		CONTROL_TORQUE
	} m_eControlMode;
	
	// PID controllers for position and velocity
	struct PIDController {
		double kp, ki, kd;
		double integral;
		double prev_error;
		double output_limit;
		
		PIDController() : kp(0), ki(0), kd(0), integral(0), prev_error(0), output_limit(1000) {}
	};
	
	PIDController	m_pidPosition;
	PIDController	m_pidVelocity;
	
	// Simulation state
	BOOL			m_bServoOn;
	double			m_dLastUpdateTime;
	double			m_dSimulatedEncoderPos;	// Simulated encoder position
	
	// Motion profiles (for trajectory generation)
	double			m_dProfileVel;
	double			m_dProfileAcc;
	double			m_dProfileDec;
	
	// Command filtering
	double			m_dFilteredTargetPos;
	double			m_dFilteredTargetVel;
	double			m_dFilteredTargetTorque;

}; // CAxisMuJoCo

#endif // __AXIS__MUJOCO__
