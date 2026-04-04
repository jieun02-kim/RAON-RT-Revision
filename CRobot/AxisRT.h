/*****************************************************************************
*	Name: AxisRT.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Real-time axis simulation without MuJoCo dependencies
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __AXIS__RT__
#define __AXIS__RT__

#include "Axis.h"

// Real-time safe motor simulation
class CAxisRT : public CAxis
{
public:
	CAxisRT(eAxisType aeAxisType, double adEncRes = 65536, double adGearRatio = 1.0, 
			double adTransRatio = 1.0, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE);
	virtual ~CAxisRT();

public:
	// Override core axis functions for RT simulation
	virtual	BOOL	Init					();
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
	
	// Real-time simulation methods
	virtual void	StepSimulation			(double dt);  // RT-safe simulation step
	void			SetExternalTorque		(double torque);  // From physics engine
	void			SetMotorParameters		(double inertia, double damping, double friction);
	
	// Control mode setters
	void			SetPositionControl		();
	void			SetVelocityControl		();
	void			SetTorqueControl		();
	
	// Target getters (for debugging/monitoring)
	virtual double	GetTargetPos			() { return m_dTargetPos; }
	virtual double	GetTargetVel			() { return m_dTargetVel; }
	virtual double	GetTargetTorq			() { return m_dTargetTorq; }
	
	// RT state accessors
	double			GetSimulationTime		() { return m_dSimTime; }
	double			GetMotorCurrent			() { return m_dMotorCurrent; }
	
protected:
	// RT-safe control methods
	void			ComputePIDPosition		(double dt);
	void			ComputePIDVelocity		(double dt);
	void			ApplyTorqueLimits		(double& torque);
	void			UpdateMotorDynamics		(double dt);
	void			UpdateSimulatedEncoder	();
	
	// Override state management
	virtual void	SetState				(eAxisState aeState);

protected:
	// Control state
	enum eControlMode {
		CONTROL_POSITION,
		CONTROL_VELOCITY, 
		CONTROL_TORQUE
	} m_eControlMode;
	
	// PID controllers for position and velocity (RT-safe)
	struct PIDController {
		double kp, ki, kd;
		double integral;
		double prev_error;
		double output_limit;
		double integral_limit;  // Anti-windup
		
		PIDController() : kp(0), ki(0), kd(0), integral(0), prev_error(0), 
						  output_limit(1000), integral_limit(100) {}
		
		// RT-safe PID computation
		double compute(double error, double dt) {
			double p_term = kp * error;
			
			integral += ki * error * dt;
			// Anti-windup
			if (integral > integral_limit) integral = integral_limit;
			if (integral < -integral_limit) integral = -integral_limit;
			
			double d_term = kd * (error - prev_error) / dt;
			prev_error = error;
			
			double output = p_term + integral + d_term;
			
			// Apply limits
			if (output > output_limit) output = output_limit;
			if (output < -output_limit) output = -output_limit;
			
			return output;
		}
		
		void reset() {
			integral = 0;
			prev_error = 0;
		}
	};
	
	PIDController	m_pidPosition;
	PIDController	m_pidVelocity;
	
	// Motor dynamics (simplified but RT-safe)
	struct MotorDynamics {
		double inertia;      // kg*m^2
		double damping;      // N*m*s/rad
		double friction;     // N*m (constant friction)
		double position;     // rad
		double velocity;     // rad/s
		double acceleration; // rad/s^2
		
		MotorDynamics() : inertia(0.001), damping(0.01), friction(0.001),
						  position(0), velocity(0), acceleration(0) {}
	} m_motorDynamics;
	
	// Simulation state
	BOOL			m_bServoOn;
	double			m_dSimTime;
	double			m_dLastUpdateTime;
	double			m_dSimulatedEncoderPos;	// Simulated encoder position
	double			m_dExternalTorque;		// From physics/external forces
	double			m_dMotorTorque;			// Motor output torque
	double			m_dMotorCurrent;		// Simulated motor current
	
	// Motion profiles (for trajectory generation)
	double			m_dProfileVel;
	double			m_dProfileAcc;
	double			m_dProfileDec;
	
	// Command filtering (RT-safe)
	double			m_dFilteredTargetPos;
	double			m_dFilteredTargetVel;
	double			m_dFilteredTargetTorque;

}; // CAxisRT

#endif // __AXIS__RT__
