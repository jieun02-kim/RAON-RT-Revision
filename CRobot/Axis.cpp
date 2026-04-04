/*****************************************************************************
*	Name: Axis.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CAxis class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Axis.h"


/////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CAxis::CAxis(eCommType aeCommType, eAxisType aeAxisType, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled)
{
	m_bEnabled = abEnabled;
	m_pstAxisInfo = new ST_AXIS_INFO();
	m_pstAxisLimits = new ST_AXIS_LIMITS();
	m_pstAxisParams = new ST_AXIS_PARAMS();
	m_pstAxisRawParams = new ST_AXIS_RAW_PARAMS();
	m_pstAxisStartRawParams = new ST_AXIS_RAW_PARAMS();

	SetCommType(aeCommType);
	SetAxisType(aeAxisType);
	SetState(eAxisUnknownState);

	m_dResolution = 1.;
	m_dGearRatio = 1.;
	m_dEncoderRatio = 1.;
	m_dTorqueConstant = 1.;
	m_dCurrentRatio = 1.;

	m_usAxisAlias = 0;
	m_usAxisPos = 0;
	m_usAxisID = 0;
	
	m_nHomePosition = 0;
	m_bHomeSet = FALSE;
	m_bHomeRefSet = FALSE;

	/* Call the method SetOneTurnRef if the axis is not revolute */
	m_dOneTurnRef = (eAxisRevolute == aeAxisType) ? TWO_M_PI : 1.;
	
	/* normal polarity = CCW, inverse polarity = CW */
	m_nDirection = 1;
	if (FALSE == abCCW) m_nDirection = -1;

	m_bAbsEncoder = abAbsoluteEncoder;
	m_dHomeReference = 0.;
	m_nPosBeforeExit = 0;
	m_bIsHoming = FALSE;
	m_bIsAutoServoOn = FALSE;
	
	m_dTargetTorq = 0.;
	m_dTargetPos = 0.;
	m_dTargetVel = 0.;
}

CAxis::~CAxis()
{
	if (m_pstAxisInfo)
	{
		delete m_pstAxisInfo;
		m_pstAxisInfo = NULL;
	}

	if (m_pstAxisLimits)
	{
		delete m_pstAxisLimits;
		m_pstAxisLimits = NULL;
	}

	if (m_pstAxisParams)
	{
		delete m_pstAxisParams;
		m_pstAxisParams = NULL;
	}

	if (m_pstAxisRawParams)
	{
		delete m_pstAxisRawParams;
		m_pstAxisRawParams = NULL;
	}

	if (m_pstAxisStartRawParams)
	{
		delete m_pstAxisStartRawParams;
		m_pstAxisStartRawParams = NULL;
	}
}
//////////////////////////////////////////////////////////////////////
BOOL
CAxis::Init()
{

	SetState(eAxisInit);
	return TRUE;
}

BOOL
CAxis::DeInit()
{
	SetState(eAxisDeinit);
	return TRUE;
}

BOOL 
CAxis::InitHW()
{

	return FALSE;
}

BOOL 
CAxis::InitSW()
{

	return FALSE;
}

BOOL 
CAxis::DeInitHW()
{

	return FALSE;
}

BOOL 
CAxis::DeinitSW()
{

	return FALSE;
}

BOOL
CAxis::ServoOn(	)
{
	return FALSE;
}

BOOL
CAxis::ServoOff()
{
	return FALSE;
}

BOOL
CAxis::DoHoming()
{
	return FALSE;
}

BOOL
CAxis::MoveHome(BOOL abForced)
{
	return MoveAxis(m_nHomePosition, abForced);
}

BOOL	
CAxis::MovePosLimit(BOOL abForced)
{
	return MoveAxis(m_pstAxisLimits->stPos.dUpper, abForced);
}

BOOL	
CAxis::MoveNegLimit(BOOL abForced)
{
	return MoveAxis(m_pstAxisLimits->stPos.dLower, abForced);
}

BOOL
CAxis::SetHomePosition(INT32 anHomePos)
{
	m_nHomePosition = anHomePos;
	m_bHomeSet = TRUE;
	return TRUE;
}

BOOL
CAxis::SetHomeReference(double adHomeRef)
{
	m_dHomeReference = adHomeRef;
	m_bHomeRefSet = TRUE;
	return TRUE;
}

INT32
CAxis::GetHomePosition(	)
{
	return m_nHomePosition;
}

BOOL
CAxis::MoveAxis(double adPosition, BOOL abForce, BOOL abAbs)
{
	return FALSE;
}

BOOL
CAxis::StopAxis()
{
	return FALSE;
}

BOOL
CAxis::EmgStopAxis()
{
	return FALSE;
}

/* This sets the position upper and lower limits explicitly
*  Default is degrees
*/
BOOL
CAxis::SetPositionLimits(double adLower, double adUpper, BOOL abIsRad)
{
	double dLower = adLower; double dUpper = adUpper;
	if (FALSE == abIsRad && eAxisRevolute == GetAxisType())
	{
		dLower = ConvertDeg2Rad(dLower);
		dUpper = ConvertDeg2Rad(dUpper);
	}
	return SetLimits(m_pstAxisLimits->stPos, dLower, dUpper);
}

/* This sets opposite values as position limits */
BOOL
CAxis::SetPositionLimits(double adPosNegLimit, BOOL abIsRad)
{
	double dLimit = adPosNegLimit;
	if (FALSE == abIsRad && eAxisRevolute == GetAxisType())
	{
		dLimit = ConvertDeg2Rad(dLimit);
	}
	return SetPositionLimits(-dLimit, dLimit);
}


BOOL
CAxis::SetVelocity(double adVel)
{
	return FALSE;
}

BOOL
CAxis::SetVelocityLimits(double adVelPosNeg)
{
	return SetVelocityLimits(-adVelPosNeg, adVelPosNeg);
}

BOOL
CAxis::SetVelocityLimits(double adLower, double adUpper)
{	
	
	return SetLimits(m_pstAxisLimits->stVel, adLower, adUpper);
}

BOOL
CAxis::SetAcceleration(double adAcc)
{
	return FALSE;
}

BOOL
CAxis::SetAccelerationLimits(double adAccPosNeg)
{
	return SetAccelerationLimits(-adAccPosNeg, adAccPosNeg);
}

BOOL
CAxis::SetAccelerationLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stAcc, adLower, adUpper);
}

BOOL
CAxis::SetDeceleration(double adDec)
{
	return FALSE;
}

BOOL
CAxis::SetDecelerationLimits(double adDecPosNeg)
{
	return SetDecelerationLimits(-adDecPosNeg, adDecPosNeg);
}

BOOL
CAxis::SetDecelerationLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stDec, adLower, adUpper);
}

BOOL
CAxis::SetJerk(double adJerk)
{
	return FALSE;
}

BOOL
CAxis::SetJerkLimits(double adJerkPosNeg)
{
	return SetJerkLimits(-adJerkPosNeg, adJerkPosNeg);
}

BOOL
CAxis::SetJerkLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stJerk, adLower, adUpper);
}

BOOL
CAxis::SetTorque(double adJerk)
{
	return FALSE;
}

BOOL
CAxis::SetTorqueLimits(double adTorquePosNeg)
{
	return SetTorqueLimits(-adTorquePosNeg, adTorquePosNeg);
}

BOOL
CAxis::SetTorqueLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stTorque, adLower, adUpper);
}

BOOL
CAxis::SetTorqueConstant(double adTorqueConst)
{
	m_dTorqueConstant = adTorqueConst;
	return TRUE;
}

BOOL
CAxis::MoveVelocity(double adVel)
{
	return FALSE;
}

BOOL
CAxis::MoveTorque(double adTorque)
{
	return FALSE;
}

void
CAxis::UpdateStartRawParams(INT32 anPos, INT32 anVel, INT32 anTor)
{
	m_pstAxisStartRawParams->nPos = anPos;
	m_pstAxisStartRawParams->nVel = anVel;
	m_pstAxisStartRawParams->nTor = anTor;
}

void
CAxis::UpdateCurrentParams(INT32 anPos, INT32 anVel, INT32 anTor)
{
	m_pstAxisRawParams->nPos = anPos;
	m_pstAxisRawParams->nVel = anVel;
	m_pstAxisRawParams->nTor = anTor;
	
	/* Calculate position from the home or starting position, depending whether Home Position is configured
	*/
	INT32 nCurrentPos = m_pstAxisRawParams->nPos - GetHomePosition();
	if (FALSE == IsHomeSet())
		nCurrentPos = m_pstAxisRawParams->nPos - GetStartRawPos();
	
	double dPos = ConvertRes2RadMM(nCurrentPos);
	double dVel = ConvertRes2RadMM(m_pstAxisRawParams->nVel);
	double dTor = ConvertRes2Tor(m_pstAxisRawParams->nTor);

	UpdateCurrentParams(dPos, dVel, dTor);

	// ST_AXIS_LIMITS stLimits = GetAxisLimits();

	// BOOL bLimited = FALSE;
	// bLimited = CheckLimits(stLimits.stPos, m_pstAxisParams->dPos);
	// if (bLimited == FALSE)
	// {
	// 	if (IsMoving() && !IsAllowablePosition(GetTargetPos()))
	// 		StopAxis();
		
	// 	DBG_LOG_WARN("(Axis[%d:%d] UpdateCurrentParams - Exceeded Position Limit! Pos: %lf, LL: %lf, UL: %lf", 
	// 		m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dPos, stLimits.stPos.dLower, stLimits.stPos.dUpper);

	// }
	// bLimited = CheckLimits(stLimits.stVel, m_pstAxisParams->dVel);
	// if (bLimited == FALSE)
	// {
	// 	// if (IsMoving())
	// 	// 	StopAxis();
			
	// 	DBG_LOG_WARN("(Axis[%d:%d] UpdateCurrentParams - Exceeded Velocity Limit! Vel: %lf, LL: %lf, UL: %lf", 
	// 		m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dVel, stLimits.stVel.dLower, stLimits.stVel.dUpper);
	// }
	// bLimited = CheckLimits(stLimits.stTorque, m_pstAxisParams->dTor);
	// if (bLimited == FALSE)
	// {
	// 	// if (IsMoving()) // && !IsAllowableTorque(GetTargetTorq()))
	// 	// 	StopAxis();

	// 	DBG_LOG_WARN("(Axis[%d:%d] UpdateCurrentParams - Exceeded Torque Limit! Torq: %lf, LL: %lf, UL: %lf", 
	// 		m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dTor, stLimits.stTorque.dLower, stLimits.stTorque.dUpper);
	// }	
}

void
CAxis::UpdateCurrentParams(double adPos, double adVel, double adTor)
{
	m_pstAxisParams->dPos = adPos;
	m_pstAxisParams->dVel = adVel;
	m_pstAxisParams->dTor = adTor;
}

INT32
CAxis::GetCurrentRawPos()
{
	return m_pstAxisRawParams->nPos;
}


INT32
CAxis::GetCurrentRawVel()
{
	return m_pstAxisRawParams->nVel;
}

INT32
CAxis::GetCurrentRawTor()
{
	return m_pstAxisRawParams->nTor;
}

INT32
CAxis::GetStartRawPos()
{
	return m_pstAxisStartRawParams->nPos;
}

INT32
CAxis::GetStartRawVel()
{
	return m_pstAxisStartRawParams->nVel;
}

INT32
CAxis::GetStartRawTor()
{
	return m_pstAxisStartRawParams->nTor;
}

double
CAxis::GetCurrentPos()
{
	return (m_pstAxisParams->dPos);
}

double
CAxis::GetCurrentPosD()
{
	return (ConvertRad2Deg(GetCurrentPos()));
}

double
CAxis::GetCurrentVel()
{
	return m_pstAxisParams->dVel;
}

double
CAxis::GetCurrentTor()
{
	// jieun
	//printf("[CAxis::GetCurrentTor] Current Torque: %lf\n", m_pstAxisParams->dTor);
	return m_pstAxisParams->dTor;
}

ST_AXIS_LIMITS
CAxis::GetAxisLimits()
{
	return *m_pstAxisLimits;
}

ST_AXIS_PARAMS
CAxis::GetCurrentParams()
{
	return *m_pstAxisParams;
}

ST_AXIS_RAW_PARAMS
CAxis::GetCurrentRawParams()
{
	return *m_pstAxisRawParams;
}

void
CAxis::SetState(eAxisState aeState)
{
	m_pstAxisInfo->eState = aeState;
}

/* This should be called explicitly if the joint is prismatic, otherwise pre-set values are configured */
BOOL
CAxis::SetOneTurnRef(double adOneTurnRef)
{
	if (eAxisRevolute == GetAxisType())
		return FALSE;

	m_dOneTurnRef = adOneTurnRef;
	return TRUE;
}

/* Sets the alias and position of the axis from the main controller (this is necessary for EtherCAT) */
void
CAxis::SetAliasPos(UINT16 ausAlias, UINT16 ausPos)
{
	m_usAxisAlias = ausAlias;
	m_usAxisPos = ausPos;
}

/* Sets the encoder resolution with the actual resolution, encoder ratio (X1, X2, X4), and the gear ratio (if necessary)  */
void
CAxis::SetResolution(double adRes, double adGearRatio/*=1*/, double adTransRatio/*=1*/)
//CAxis::SetResolution(double adRes, double adGearRatio/*=1*/, double adEncRatio/*=1*/)
{
	m_dResolution = adRes;
	m_dTransRatio = adTransRatio;
	m_dGearRatio = adGearRatio;
}

BOOL
CAxis::IsLimitConfigured()
{
	/*  20221228 : raim.delgado : delete IsHomeSet from the limits list.
	*  m_bHomeSet will be handled by the state machine.
	*/
	BOOL bLimits[] = {m_pstAxisLimits->stPos.bIsSet, m_pstAxisLimits->stVel.bIsSet, m_pstAxisLimits->stAcc.bIsSet,
		m_pstAxisLimits->stDec.bIsSet, m_pstAxisLimits->stJerk.bIsSet, m_pstAxisLimits->stTorque.bIsSet };

	for (int nCnt = 0; nCnt < (int)sizeof(bLimits); nCnt++)
	{
		if (FALSE == bLimits[nCnt]) return FALSE;
	}
	return TRUE;
}

BOOL
CAxis::IsMovable()
{
	/* ensure that the motor only moves when the axis is enabled and the status is not in error
	* also for the meantime, check whether all of the limits are configured
	*/
	if ((eAxisInit < GetState()) && (TRUE == IsLimitConfigured()) && (TRUE == IsEnabled()))
		return TRUE;

	return FALSE;
}

BOOL
CAxis::IsServoOn()
{
	return FALSE;
}

BOOL 
CAxis::IsAllowablePosition(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stPos, adTarget);
}

BOOL
CAxis::IsAllowableVelocity(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stVel, adTarget);
}

BOOL
CAxis::IsAllowableAcceleration(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stAcc, adTarget);
}

BOOL
CAxis::IsAllowableDeceleration(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stDec, adTarget);
}

BOOL
CAxis::IsAllowableJerk(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stJerk, adTarget);
}

BOOL
CAxis::IsAllowableTorque(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stTorque, adTarget);
}

BOOL
CAxis::IsActuator()
{
	if (eAxisSensor <= GetAxisType())
		return FALSE;

	return TRUE;
}

BOOL
CAxis::SetLimits(ST_LIMITS& astLimit, double adLower, double adUpper)
{
	astLimit.dUpper = adUpper;
	astLimit.dLower = adLower;
	astLimit.bIsSet = TRUE;
	return TRUE;
}

BOOL
CAxis::CheckLimits(ST_LIMITS& astLimit, double adTarget)
{
	if ((astLimit.dLower <= adTarget) && (astLimit.dUpper >= adTarget))
		return TRUE;

	return FALSE;
}

/* Converts radians or millimeters to encoder resolution */
INT32
CAxis::ConvertRadMM2Res(double adPos)
{
	return (INT32)((adPos / m_dOneTurnRef) * m_dTransRatio * m_dGearRatio * m_dResolution * m_nDirection);
}

/* Converts encoder resolution to either radians or millimeters */
double
CAxis::ConvertRes2RadMM(INT32 adRes)
{	
	
	return (double)(m_dOneTurnRef * (adRes / m_dResolution / m_dGearRatio / m_dTransRatio) * m_nDirection);
	
}

/* 20250714 raim.delgado: select between torque constant, or rated torque/current for conversion */
double  
CAxis::ConvertCur2Tor(double adCurrent, BOOL abUseTConstant)
{
	if (abUseTConstant)
		return adCurrent * GetTorqueConstant();
	// If not using torque constant, use rated values
	
	if (GetRatedCurrent() == 0.0) 
	{	
		DBG_LOG_ERROR("ConvertCur2Tor - Rated current is zero, cannot convert current to torque.");
		return 0.0;
	}	
	return adCurrent * GetRatedTorque() / GetRatedCurrent();
}

/* 20250714 raim.delgado: select between torque constant, or rated torque/current for conversion */
double  
CAxis::ConvertTor2Cur(double adTorque,  BOOL abUseTConstant)
{
	if (abUseTConstant)
		return adTorque / GetTorqueConstant();
	
	if (GetRatedTorque() == 0.0) 
	{
		DBG_LOG_ERROR("ConvertTor2Cur - Rated torque is zero, cannot convert torque to current.");
		return 0.0;	
	}

	return adTorque * GetRatedCurrent() / GetRatedTorque();
}		

/* Calculation may change accoding to the motor driver */
INT32
CAxis::ConvertTor2Res(double adTor)
{
	/* 20250714 raim.delgado: add sign of torque using m_nDirection*/
	INT32 res = (int)( m_nDirection * ConvertTor2Cur(adTor) * 1000.0 / GetRatedCurrent());
	
	return  (INT32)adTor;
}

double
CAxis::ConvertRes2Tor(INT32 adRes)
{
	/* 20250714 raim.delgado: add sign of torque using m_nDirection*/
	double torque = m_nDirection * adRes * GetRatedTorque() / 1000.0;
	return (double)adRes;
}

double
CAxis::GetMaxPos()
{
	return m_pstAxisLimits->stPos.dUpper;
}

double
CAxis::GetMinPos()
{
	return m_pstAxisLimits->stPos.dLower;
}

BOOL 
CAxis::IsMoving()
{
	if (fabs(GetCurrentVel()) > 0.0) {
		return TRUE;
	}

	return FALSE;
}

BOOL
CAxis::ChangeDriveMode(INT8 abtDriveMode)
{
	// if (FALSE == IsServoOn()) return FALSE;
	return FALSE;
}