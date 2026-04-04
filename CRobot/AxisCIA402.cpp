/*****************************************************************************
*	Name: AxisCIA402.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CAxisCIA402 class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "AxisCIA402.h"
#include <algorithm>
#include <functional>
#include "posix_rt.h"

/* adEncRes should include the encoding type of the encoder (X1, X2, X4)
* For joint types other than revolute, the adOneTurnRef should be given explicitly
* Ex) Screw-type prismatic joint, one turn is equal to the lead of the screw, thus adOneTurnRef should be equal to the lead (pitch * number of starts) of the screw.
*/
CAxisCIA402::CAxisCIA402(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
	: CAxis(eAxisEtherCAT, aeAxisType, abAbsoluteEncoder, abCCW, abEnabled)
{
	m_bConnected = abConnected;
	if (FALSE == m_bConnected) SetEnabled(FALSE);

	SetResolution(adEncRes, adGearRatio, adTransRatio);

	if (eAxisRevolute != aeAxisType)
	{
		DBG_LOG_WARN("Axis type is not revolute, remember to set appropriate OneTurnRef");
	}

	m_bFirstHome = TRUE;
	m_pEcMaster = NULL;
	m_nPosBeforeDisconnect = 0;
}

CAxisCIA402::~CAxisCIA402()
{
	if (NULL != m_pEcMaster)
	{
		// m_pEcMaster = NULL;
	}
}

/* Registers the EtherCAT master and initializes the slave. 
*  The axis will then attempt to change the CIA402 status to OPERATIONAL (Servo ON) if auto servo on is enabled and then set the drive mode according to abtDrivemode.
*/
// ensure that the EtherCAT master instance is not changed
BOOL 
CAxisCIA402::Init(CEcatMaster& apEcmaster, INT8 abtDriveMode)
{
	/* Pre-conditions 
	*  Each condition is separated to provide exact error type.
	*  todo: make this fucntion return int and provide error code?
	*/
	/* return TRUE when the axis is already initialized */
	if (eAxisInit <= GetState())	return TRUE;

	/* check if the axis limits are configured before initializing */
	if (FALSE == IsLimitConfigured())
	{
		DBG_LOG_ERROR("(%s) Axis Limits are not configured!", "CAxisCIA402");
		DeInit();
		return FALSE;
	}

	eHomingMethod ehoming = GetHomingMethod();
	switch (ehoming)
	{
	case eAxisHomeStartPos:
		break;
	case eAxisHomeSearch:
		if (FALSE == IsHomeReferenceSet())
		{
			DBG_LOG_ERROR("(%s) HomeSearch Method is Selected, but Reference is not configured!", "CAxisCIA402");
			DeInit();
			return FALSE;
		}
		break;
	case eAxisHomeManual:
		if (FALSE == IsHomeSet())
		{
			DBG_LOG_ERROR("(%s) HomeManual Method is Selected, but Home Position is not configured!", "CAxisCIA402");
			DeInit();
			return FALSE;
		}
		break;
	case eAxisHomeBuiltin: // todo: consider this later
	case eAxisHomingNotSet:
	default:
		DBG_LOG_ERROR("(%s) Invalid Homing Method!", "CAxisCIA402");
		DeInit();
		return FALSE;
	}

	if (TRUE == IsConnected())
	{
		m_pEcMaster = &apEcmaster;
		m_pEcMaster->AddSlave(&m_cEcSlave);
		UINT16 usAlias = m_cEcSlave.GetAlias();
		UINT16 usPosition = m_cEcSlave.GetPosition();
		SetAliasPos(usAlias, usPosition);

		if (TRUE == IsEnabled())
		{
			// register callback functions only when the axis is enabled
			m_cEcSlave.RegisterCallbackStateChange(std::bind(&CAxisCIA402::OnSlaveStatusChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
			m_cEcSlave.RegisterCallbackParamUpdate(std::bind(&CAxisCIA402::OnSlaveUpdateRawParam, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

			BOOL bRet = FALSE;
			if (TRUE == IsAutoServoOn())
				bRet = ServoOn(abtDriveMode);
			else
			{
				m_cEcSlave.SetDriveMode(abtDriveMode);
				bRet = TRUE;
			}
			
			if (TRUE == bRet)
			{
				TSTRING szDriveMode = GetCIA402DriveMode((int)abtDriveMode);
				DBG_LOG_INFO("(%s) Axis[%d:%d] Initialized! DriveMode: %s", "CAxisCIA402", m_usAxisAlias, m_usAxisPos, szDriveMode.c_str());
				return CAxis::Init(); // change the state to eAxisInit
			}
			else
			{
				DBG_LOG_WARN("(%s) Axis[%d:%d] Initialized, but ServoOn Failed!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);
				DeInit();
				return FALSE; 
			}
		}
		DBG_LOG_WARN("(%s) Axis[%d:%d] Initialized, but Axis is Disabled!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);
		return CAxis::Init(); // change the state to eAxisInit
	}
	DBG_LOG_WARN("(%s) Axis is not connected!", "CAxisCIA402");
	return DeInit(); // return TRUE but change the state to eAxisDeInit
}

BOOL 
CAxisCIA402::DeInit()
{
	return CAxis::DeInit();
}

BOOL
CAxisCIA402::DoHoming()
{
	return FALSE;
}

/* radians */
BOOL
CAxisCIA402::MoveAxis(double adTarget, BOOL abForce, BOOL abAbs)
{
	/* only allow this when in any of the position-based operation mode 
	*/
	if ((CIA402_CYCLIC_POSITION != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_POSITION != m_cEcSlave.GetActualDriveMode()))
	{
		/*DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Not Position Mode!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);*/
		return FALSE;
	}
	
	/* check if limits are configured and if the status is IDLE 
	*  bypass this condition when forced.
	*/
	if ((FALSE == abForce) && (FALSE == IsMovable()))
	{
		DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Not Moveable!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);
		return FALSE;
	}
		
	/*
	* Absolute motion is only allowable when home position is configured
	*/
	INT32 nReferencePos = 0;
	if (TRUE == abAbs && TRUE == IsHomeSet())
	{
		nReferencePos = GetHomePosition();
		if (FALSE == IsAllowablePosition(adTarget)) 
		{
			DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Exceeded Limit! Target: %lf, Cur Pos: %lf, LL: %lf, UL: %lf", 
                "CAxisCIA402", m_usAxisAlias, m_usAxisPos, adTarget, GetCurrentPos(), 
                GetAxisLimits().stPos.dLower, GetAxisLimits().stPos.dUpper);
			return FALSE;
		}
	}
	else
	{
		nReferencePos = GetCurrentRawPos();
		if (TRUE == IsHomeSet())
		{
			if (FALSE == IsAllowablePosition(GetCurrentPos() + adTarget))
			{
				DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Exceeded Limit!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);
				return FALSE;
			}
		}
	}
	// DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - TargetPos! %d %d", "CAxisCIA402", m_usAxisAlias, m_usAxisPos, nReferencePos, nReferencePos + ConvertRadMM2Res(adTarget));
	return m_cEcSlave.SetTargetPos(nReferencePos + ConvertRadMM2Res(adTarget));
}

BOOL
CAxisCIA402::MoveHome(BOOL abForce)
{
	/*todo: shall we add force here? */
	if ((FALSE == IsMovable())) return FALSE;

	/* only allow this when in any of the position-based operation mode */
	if ((CIA402_CYCLIC_POSITION != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_POSITION != m_cEcSlave.GetActualDriveMode()))
		return FALSE;

	return m_cEcSlave.SetTargetPos(GetHomePosition(), abForce);
}

BOOL
CAxisCIA402::StopAxis()
{
	if (FALSE == IsMovable()) return FALSE;
	BOOL bRet = m_cEcSlave.SetHalt(TRUE);

	return bRet;
}

BOOL
CAxisCIA402::EmgStopAxis()
{
	return  m_cEcSlave.SetEmgStop();
}

/* rad/s */
BOOL
CAxisCIA402::MoveVelocity(double adVel)
{
	if ((FALSE == IsMovable()) || (FALSE == IsAllowableVelocity(adVel))) return FALSE;

	/* only allow this when in any of the velocity-based operation mode */
	if ((CIA402_CYCLIC_VELOCITY != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_VELOCITY != m_cEcSlave.GetActualDriveMode()))
		return FALSE;

	m_cEcSlave.SetTargetVel(ConvertRadMM2Res(adVel));

	return TRUE;
}

BOOL
CAxisCIA402::MoveTorque(double adTor)
{
	if (FALSE == IsMovable()) return FALSE;

	/* only allow this when in any of the torque-based operation mode */
	if ((CIA402_CYCLIC_TORQUE != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_TORQUE != m_cEcSlave.GetActualDriveMode()))
		return FALSE;

	m_cEcSlave.SetTargetTor((INT16)ConvertTor2Res(adTor));

	return TRUE;
}


BOOL
CAxisCIA402::ServoOn(INT8 abtDriveMode)
{
	if ((FALSE == IsConnected()) && (FALSE == IsEnabled())) return FALSE;

	return m_cEcSlave.SetServoOn(abtDriveMode);
}

BOOL
CAxisCIA402::ChangeDriveMode(INT8 abtDriveMode)
{
	// if (FALSE == IsServoOn()) return FALSE;

	m_cEcSlave.SetDriveMode(abtDriveMode);

	return TRUE;
}

BOOL
CAxisCIA402::ServoOff(	)
{
	return m_cEcSlave.SetServoOff();
}

BOOL
CAxisCIA402::IsServoOn()
{
	return m_cEcSlave.IsServoOn();
}

BOOL	
CAxisCIA402::SetVelocity(double adRPM)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableVelocity(adRPM))
		return FALSE;
	
	//check unit used in EPOS4
	m_cEcSlave.SetProfileVel((INT32)adRPM);
	m_cEcSlave.SetMaxProfileVel((INT32)adRPM);

	return TRUE;
}

BOOL	
CAxisCIA402::SetAcceleration(double adRPMS)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableAcceleration(adRPMS))
		return FALSE;

	m_cEcSlave.SetProfileAcc((INT32)adRPMS);
	return TRUE;
}

BOOL	
CAxisCIA402::SetDeceleration(double adRPMS)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableDeceleration(adRPMS))
		return FALSE;

	m_cEcSlave.SetProfileDec((INT32)adRPMS);
	return TRUE;
}

INT32
CAxisCIA402::GetAcceleration()
{
	return m_cEcSlave.GetActualProfileAcc();
}

INT32
CAxisCIA402::GetDeceleration()
{
	return m_cEcSlave.GetActualProfileDec();
}

INT32
CAxisCIA402::GetTargetRawPos()
{
	return m_cEcSlave.GetTargetPos();
}

INT32
CAxisCIA402::GetTargetRawVel()
{
	return m_cEcSlave.GetTargetVel();
}

INT32
CAxisCIA402::GetTargetRawTor()
{
	return (INT32)m_cEcSlave.GetTargetTor();
}

INT32
CAxisCIA402::GetVelocity()
{
	return m_cEcSlave.GetActualProfileVel();
}

INT32
CAxisCIA402::GetMaxVel()
{
	return m_cEcSlave.GetActualMaxProfileVel();
}

INT32
CAxisCIA402::GetMaxAcc()
{
	return m_cEcSlave.GetActualMaxProfileAcc();
}

double CAxisCIA402::GetAdditionalPos()
{
	
	return m_cEcSlave.GetAdditionalPos();
}

INT32
CAxisCIA402::GetQuickStopDec()
{
	return m_cEcSlave.GetActualQuickStopDec();
}

UINT8
CAxisCIA402::GetDriveMode()
{
	return (UINT8)m_cEcSlave.GetActualDriveMode();
}

UINT16
CAxisCIA402::GetStatusWord()
{
	return m_cEcSlave.GetRawStatusWord();
}

void	
CAxisCIA402::SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode)
{
	m_cEcSlave.SetVendorInfo(auVendorID, auProductCode);
}
void	
CAxisCIA402::SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime)
{
	m_cEcSlave.SetDCInfo(abDCSupported, ausActivateWord, anShiftTime);
}

UINT16
CAxisCIA402::GetControlWord()
{
	return m_cEcSlave.GetControlWord();
}

void 
CAxisCIA402::SetState(eAxisState aeState)
{
	TSTRING			szState = "";
	BOOL			bPrintState = FALSE;
	eAxisState ePrevState = GetState();
	eAxisState eCurrState = aeState;
	m_pstAxisInfo->eState = eCurrState;

	if (ePrevState != eCurrState)
	{
		/* in case disconnected due to power loss
		*/
		if (eAxisDisconnected == ePrevState)
		{
			/* todo: in-position error should be considered */
			if (GetCurrentRawPos() != m_nPosBeforeDisconnect)
			{
				// ClearHomeSet();
				return;
			}
		}

		// if (eAxisError == ePrevState)
		// {
		// 	DBG_LOG_TRACE("(%s) Axis[%d:%d] Fault Reset: TTTTargetPos: %d, RRRRawPos:%d", "CAxisCIA402", m_usAxisAlias, m_usAxisPos, GetTargetRawPos(), GetCurrentRawPos());
		// }
		
		switch (eCurrState)
		{
		case eAxisEmergency:
			szState = "EMERGENCY";
			break;
		case eAxisError:
			// DBG_LOG_TRACE("(%s) Axis[%d:%d] Fault : TTTTargetPos: %d, RRRRawPos:%d", "CAxisCIA402", m_usAxisAlias, m_usAxisPos, GetTargetRawPos(), GetCurrentRawPos());
			szState = "ERROR";
			break;
		case eAxisDisconnected:
			szState = "DISCONNECTED";
			m_nPosBeforeDisconnect = GetCurrentRawPos(); 
			break;
		case eAxisDeinit:
			szState = "DEINIT";
			break;
		case eAxisInit:
			szState = "INIT";
			break;
		case eAxisIdle:
			szState = "IDLE";
			break;
		case eAxisHoming:
			szState = "HOMING";
			break;
		case eAxisRunning:
			szState = "RUNNING";
			break;
		case eAxisStopped:
			szState = "STOPPED";
			break;
		case eAxisUnknownState:
		default:
			szState = "UNKNOWN";
			break;
		}
		/* do not print state change IDLE<->RUNNING because it may flood STDOUT */
		if (bPrintState)
		{
			DBG_LOG_INFO("(%s) Axis[%d:%d] Current State is : %s", "CAxisCIA402", m_usAxisAlias, m_usAxisPos, szState.c_str());

			if (eAxisDisconnected == ePrevState)
			{
				DBG_LOG_INFO("(%s) Axis[%d:%d] Connection recovered!", "CAxisCIA402", m_usAxisAlias, m_usAxisPos);
			}
		}
	}
}

void 
CAxisCIA402::SetHomingFlag(BOOL abFlag)
{
	m_bIsHoming = abFlag;
	if (TRUE == abFlag)
		SetState(eAxisHoming);
}


/* Callback function for servo status change */
void
CAxisCIA402::OnSlaveStatusChanged(PVOID apSlave, PVOID apStatus, PVOID apReserved1, PVOID apReserved2)
{
	CSlaveCIA402Base cEcatSlave = *(CSlaveCIA402Base*)apSlave;
	eSERVO_STATUS	eServoStatus = *(eSERVO_STATUS*)apStatus;
	
	switch (eServoStatus)
	{
	case eServoOff:
		//SetState(eAxisInit);
		//m_bFirstHome = TRUE; 
		break;
	case eServoIdle:
		if (TRUE == m_bFirstHome)
		{
			INT32 nStartPos = cEcatSlave.GetActualPos();
			if (eAxisHomeStartPos == GetHomingMethod())
			{
				SetHomePosition(nStartPos - GetPositionBeforeExit());
			}
			UpdateCurrentParams(nStartPos, cEcatSlave.GetStartVel(), cEcatSlave.GetStartTor());
			/* todo: do we need this? */
			UpdateStartRawParams(nStartPos, cEcatSlave.GetStartVel(), cEcatSlave.GetStartTor());
			
			m_bFirstHome = FALSE;
		}
		if (eAxisHoming == GetState())
			m_bIsHoming = FALSE;

		if (TRUE == IsHomeSet())
			SetState(eAxisIdle);
		else
			SetState(eAxisInit);
		break;
	case eServoHoming:
		SetState(eAxisHoming);
		break;
	case eServoRunning:
		SetState(eAxisRunning);
		break;
	case eServoStopped:
		SetState(eAxisStopped);
		break;
	case eServoFault:
		SetState(eAxisError);
		break;
	case eServoDisconnected:
		SetState(eAxisDisconnected);
		break;
	default:
		break;
	}
}

/* Callback function for updating raw data from the servo driver.
*  This will be called everytime a new EtherCAT packet has been parsed.
*/
void
CAxisCIA402::OnSlaveUpdateRawParam(PVOID apSlave, PVOID apParams, PVOID apStatus, PVOID apReserved1)
{
	CSlaveCIA402Base	cEcatSlave	 = *(CSlaveCIA402Base*)apSlave;
	ST_RAW_DATA			stRawParams	 = *(ST_RAW_DATA*)apParams;
	// eSERVO_STATUS		eServoStatus = *(eSERVO_STATUS*)apStatus;

	UpdateCurrentParams(stRawParams.nActualPos, stRawParams.nActualVel, stRawParams.nActualTor);
}
