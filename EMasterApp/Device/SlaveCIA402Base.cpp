/*****************************************************************************
*	Name: SlaveCIA402Base.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CSlaveCIA402Base child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "SlaveCIA402Base.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CSlaveCIA402Base::CSlaveCIA402Base()
{
	m_bIsServoOn = FALSE;
	m_bIsHomingAborted = FALSE;
	m_bIsSetServoOnOff = FALSE;
	m_bIsReachedHome = FALSE;
	m_bIsReachedTarget = FALSE;
	m_bIsSpeedZero = FALSE;
	m_bIsSpeedSaturated = FALSE;
	m_bIsHomeSet = FALSE;
	m_bIsHalt = FALSE;
	m_bIsSetPointAck = FALSE;
	m_bIsFollowingError = FALSE;
	m_bIsSetGoHome = FALSE;
	m_bTriggerGoHome = FALSE;
	m_bIsEndlessMovement = FALSE;
	m_bFault = FALSE;

	m_nStartPos = 0;
	m_nStartVel = 0;
	m_uStartTor = 0;

	m_ePosMode = eAbsolute;
	m_pStateChange = NULL;
	m_pUpdateParam = NULL;

	m_eServoStatus = eServoDisconnected;
	// m_eServoStatus = eServoOff;

	CEcatSlaveBase::SetDeviceType(eCIA402);
}

CSlaveCIA402Base::~CSlaveCIA402Base()
{

}

void
CSlaveCIA402Base::DeInit()
{
	SetServoOff();
	SetEmgStop();
}


void
CSlaveCIA402Base::InitSyncs()
{
	cia402_syncs[0] = { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
	cia402_syncs[1] = { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
	cia402_syncs[2] = { 2, EC_DIR_OUTPUT, 1, cia402_pdos + 0, EC_WD_ENABLE };
	cia402_syncs[3] = { 3, EC_DIR_INPUT,  1, cia402_pdos + 1, EC_WD_DISABLE };
	cia402_syncs[4] = { 0xff };
	CSlaveCIA402Base::SetEcatPdoSync(cia402_syncs);
}

UINT16
CSlaveCIA402Base::AnalyzeStatusWord(UINT16 ausRawStatusWord, BOOL abExtend)
{
	
	if (TRUE == abExtend)
	{
		INT8 bDriveMode = GetActualDriveMode();

		/* refer to CIA402 Manual */
		m_bIsHomeSet = (0 != (ausRawStatusWord & 0x8000)) ? TRUE : FALSE;

		/* todo: how do we process following/homing error */
		m_bIsFollowingError = (0 != (ausRawStatusWord & 0x2000)) ? TRUE : FALSE;


		switch (bDriveMode)
		{
		case CIA402_PROFILE_POSITION:
			m_bIsReachedTarget = FALSE;
			m_bIsSetPointAck = FALSE;

			if (TRUE == IsHalt())
			{
				SetServoStatus(eServoStopped);
			}
			else
			{
				/* Set Point Ackowledged */
				if (0 != (ausRawStatusWord & 0x1000))
				{
					SetServoStatus(eServoRunning);
					//m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x30);
					m_bIsSetPointAck = TRUE;
				}
				/* Target Reached */
				if (0 != (ausRawStatusWord & 0x0400))
				{
					SetServoStatus(eServoIdle);
					m_bIsReachedTarget = TRUE;
				}
			}
			break;
		case CIA402_HOMING:
			if (TRUE == m_bIsSetGoHome)
			{
				m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x10);
				m_bIsSetGoHome = FALSE;
			}

			m_bIsReachedHome = (0 != (ausRawStatusWord & 0x1000)) ? TRUE : FALSE;
			m_bIsHomingAborted = (0 != (ausRawStatusWord & 0x0400)) ? TRUE : FALSE; /* Homing Aborted */

			if (TRUE == m_bIsReachedHome)
			{
				SetServoStatus(eServoIdle);
				SetDriveMode(CIA402_PROFILE_POSITION);
			}
			else if ((TRUE == m_bIsHomingAborted) && (FALSE == m_bIsReachedHome))
			{
				SetServoStatus(eServoStopped);
			}
			else
			{
				if (TRUE == IsServoOn())
					SetServoStatus(eServoHoming);
			}
			break;
		case CIA402_PROFILE_VELOCITY:
			if (TRUE == IsHalt())
			{
				SetServoStatus(eServoStopped);
			}
			else
			{
				/* Target Reached */
				if (0 != (ausRawStatusWord & 0x0400))
				{
					SetServoStatus(eServoIdle);
					m_bIsReachedTarget = TRUE;
				}
				else
				{
					m_bIsReachedTarget = FALSE;
					SetServoStatus(eServoRunning);
				}
			}
			if (0 != (ausRawStatusWord & 0x0800))
				m_bIsSpeedZero = TRUE;
			else
				m_bIsSpeedZero = FALSE;

			if (0 != (ausRawStatusWord & 0x1000))
				m_bIsSpeedSaturated = TRUE;
			else
				m_bIsSpeedSaturated = FALSE;

		case CIA402_CYCLIC_VELOCITY:
		case CIA402_CYCLIC_POSITION:
		case CIA402_CYCLIC_TORQUE:
			/* Drive is in state operation. Enables and follows the target and setpoint values of the control device */
			/* todo: check whether the state machine is valid */
			if (0 != (ausRawStatusWord & 0x1000))
			{
				SetServoStatus(eServoRunning);
			}
			/* Drive does not follow the target value */
			else
			{
				SetServoStatus(eServoIdle);
			}
			break;
		case CIA402_PROFILE_TORQUE:
		case CIA402_INTERPOLATED_POSITION:
		default:
			break;
		}
	}
	
	if ((ausRawStatusWord & 0x4F) == 0x00)
		return CIA402_NOT_READY_TO_SWITCH_ON;
	else if ((ausRawStatusWord & 0x4F) == 0x08)
		return  CIA402_FAULT;
	else if ((ausRawStatusWord & 0x4F) == 0x40)
		return  CIA402_SWITCH_ON_DISABLED;
	else if ((ausRawStatusWord & 0x6F) == 0x27)
		return  CIA402_OPERATION_ENABLED;
	else if ((ausRawStatusWord & 0x6F) == 0x23)
		return  CIA402_SWITCH_ON_ENABLED;
	else if ((ausRawStatusWord & 0x6F) == 0x21)
		return  CIA402_READY_TO_SWITCH_ON;
	else if ((ausRawStatusWord & 0x6F) == 0x07)
		return  CIA402_QUICK_STOP_ACTIVE;
	else if ((ausRawStatusWord & 0x4F) == 0x0F)
		return  CIA402_FAULT_REACTION_ACTIVE;
	else
		return 0xFFFF;
}

void CSlaveCIA402Base::SetServoStatus(eSERVO_STATUS aeStatus)
{
	eSERVO_STATUS ePrevStatus = m_eServoStatus;
	eSERVO_STATUS eCurrStatus = aeStatus;
	m_eServoStatus = eCurrStatus;
	
	if (eCurrStatus != ePrevStatus)
	{
		switch (eCurrStatus)
		{
		case eServoFault:
			// DBG_LOG_WARN("(%s) Axis[%d:%d] CIA402_FAULT! Attempting Fault Reset Procedure", "CSlaveCIA402Base", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
			break;
		case eServoDisconnected:
			DBG_LOG_ERROR("(%s) Axis[%d:%d] Check EtherCAT Connection", "CSlaveCIA402Base", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
			break;
		case eServoOff:
		case eServoIdle:
		case eServoRunning:
		case eServoHoming:
		case eServoStopped:
		default:
			break;
		}

		if (NULL != m_pStateChange)
			m_pStateChange((PVOID)this, &eCurrStatus, NULL, NULL);

	}
}

void
CSlaveCIA402Base::ReadFromSlave()
{
	// if (FALSE == IsSlaveOp())
	// {
		// SetServoStatus(eServoDisconnected);
		// return;
	// }

	BOOL bExtendAnalysis = TRUE;
	m_stSlaveParams.stInPDOs.usRawStatusWord = CEcatSlaveBase::ReadPdoU16(m_stSlaveParams.GET_OFFSET(StatusWord));

	if (FALSE == IsSlaveOnline())
	{
		bExtendAnalysis = FALSE;
		SetServoStatus(eServoDisconnected);
	}

	// todo: move state machine change here
	m_stSlaveParams.stInPDOs.usStatusWord = AnalyzeStatusWord(m_stSlaveParams.stInPDOs.usRawStatusWord, bExtendAnalysis);
	if (CIA402_OPERATION_ENABLED == m_stSlaveParams.stInPDOs.usStatusWord)
		m_bIsServoOn = TRUE;
	else
	{
		m_bIsServoOn = FALSE;
	}
	
	m_stSlaveParams.stInPDOs.nActualPos = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(ActualPos));
	m_stSlaveParams.stInPDOs.nActualVel = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(ActualVel));
	m_stSlaveParams.stInPDOs.sActualTor = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(ActualTor));
	// m_stSlaveParams.stInPDOs.usErrorCode = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(ErrorCode));
	m_stSlaveParams.stInPDOs.btDriveModeDisplay = CEcatSlaveBase::ReadPdoS8(m_stSlaveParams.GET_OFFSET(DriveModeDisplay));
	// m_stSlaveParams.stInPDOs.dAdditionalPos = CEcatSlaveBase::ReadPdoDouble(m_stSlaveParams.GET_OFFSET(AdditionalPos));
	// m_stSlaveParams.stInPDOs.uMotorRatedCurrent = CEcatSlaveBase::ReadPdoU32(m_stSlaveParams.GET_OFFSET(MotorRatedCurrent));
	// printf("uMotorRatedCurrent: %d\n", m_stSlaveParams.stInPDOs.uMotorRatedCurrent);

	if (CIA402_FAULT == m_stSlaveParams.stInPDOs.usStatusWord)
	{
		if (m_bFault == FALSE)
		{
			DBG_LOG_ERROR("(%s) Axis[%d:%d] FAULT OCCURED ErrorCode:0x%04x", "CSlaveCIA402Base", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, m_stSlaveParams.stInPDOs.usErrorCode);
		}
		m_bFault = TRUE;
	}
	else if (CIA402_FAULT_REACTION_ACTIVE == m_stSlaveParams.stInPDOs.usStatusWord)
	{
	}
	else if (CIA402_SWITCH_ON_DISABLED == m_stSlaveParams.stInPDOs.usStatusWord)
	{
		if (m_bFault == TRUE)
		{
			/*DBG_LOG_ERROR("(%s) Axis[%d:%d] FAULT RECOVERED: %d", "CSlaveCIA402Base", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, m_stSlaveParams.stInPDOs.nActualPos);*/
			m_bFault = FALSE;
		}
	}

	/* update raw data structureand send to callback function  */ 
	m_stCurrRawData.usControlWord = GetControlWord();
	m_stCurrRawData.nTargetPos = GetTargetPos();
	m_stCurrRawData.nTargetVel = GetTargetVel();
	m_stCurrRawData.nTargetTor = (INT32)GetTargetTor();
	m_stCurrRawData.btDriveMode = GetDriveMode();

	m_stCurrRawData.usStatusWord = GetRawStatusWord();
	m_stCurrRawData.nActualPos = GetActualPos();
	m_stCurrRawData.nActualVel = GetActualVel();
	m_stCurrRawData.nActualTor = (INT32)GetActualTor();
	m_stCurrRawData.btActualDriveMode = GetActualDriveMode();

	if (NULL != m_pUpdateParam)
		m_pUpdateParam((PVOID)this, &m_stCurrRawData, &m_eServoStatus, NULL);
	
}

void CSlaveCIA402Base::RegisterCallbackStateChange(CALLBACK_FN afnCallback)
{
	if (NULL == m_pStateChange)
	{
		m_pStateChange = std::move(afnCallback);
		m_pStateChange((PVOID)this, &m_eServoStatus, NULL, NULL);
	}
}

void CSlaveCIA402Base::RegisterCallbackParamUpdate(CALLBACK_FN afnCallback)
{
	if (NULL == m_pUpdateParam)
	{
		m_pUpdateParam = std::move(afnCallback);
	}
}

void
CSlaveCIA402Base::WriteToSlave ()
{
	if (FALSE == IsSlaveOp())
	{
		// SetServoStatus(eServoDisconnected);
		// return;
	}
	unsigned short usStatusWord = m_stSlaveParams.stInPDOs.usStatusWord;
	
	if (TRUE == m_bIsSetServoOnOff && FALSE == m_bIsServoOn)
	{
		/* IgH EtherCAT Master 1.6.4 */
		// if (usStatusWord != m_usStatusWordPrev)
		{
			switch (usStatusWord)
			{
			case CIA402_SWITCH_ON_DISABLED:
				SetServoStatus(eServoOff);
				m_stSlaveParams.stOutPDOs.usControlWord = CIA402_SHUTDOWN;
				break;
			case CIA402_READY_TO_SWITCH_ON:
				m_stSlaveParams.stOutPDOs.usControlWord = CIA402_SWITCH_ON;
				break;
			case CIA402_SWITCH_ON_ENABLED:
				m_nStartPos = m_stSlaveParams.stInPDOs.nActualPos;
				m_nStartVel = m_stSlaveParams.stInPDOs.nActualVel;
				m_uStartTor = m_stSlaveParams.stInPDOs.sActualTor;
				
				m_stSlaveParams.stOutPDOs.nTargetPos = m_nStartPos;
				// m_stSlaveParams.stOutPDOs.nTargetVel = m_nStartVel;
				// m_stSlaveParams.stOutPDOs.sTargetTor = m_uStartTor;
				m_stSlaveParams.stOutPDOs.nTargetVel = 0.0;
				m_stSlaveParams.stOutPDOs.sTargetTor = 0.0;
				m_stSlaveParams.stOutPDOs.usControlWord = CIA402_ENABLE_OPERATION;
				break;
			case CIA402_FAULT:
				SetServoStatus(eServoFault);
				m_stSlaveParams.stOutPDOs.usControlWord = CIA402_FAULT_RESET;
				break;
			case CIA402_OPERATION_ENABLED:
				SetServoStatus(eServoIdle);
				break;
			case CIA402_QUICK_STOP_ACTIVE:
				m_stSlaveParams.stOutPDOs.usControlWord = CIA402_DISABLE_VOLTAGE;
				break;
				/* these are automatic transitions */
			case CIA402_FAULT_REACTION_ACTIVE:
				break;
			case CIA402_NOT_READY_TO_SWITCH_ON:
			default:
				break;
			}
		}
	}
	else if (FALSE == m_bIsSetServoOnOff)
	{
		m_stSlaveParams.stOutPDOs.usControlWord = CIA402_SHUTDOWN;
		SetServoStatus(eServoOff);
	}
	else if (TRUE == IsServoOn())
	{
		/* HOMING */
		if ((TRUE == m_bTriggerGoHome) && (CIA402_HOMING == GetActualDriveMode()))
		{
			if (TRUE == IsHalt())
				SetHalt(FALSE);
			else
			{
				SetNewSetPoint();
				m_bIsSetGoHome = TRUE;
				m_bTriggerGoHome = FALSE;
			}
		}
		if (CIA402_PROFILE_POSITION == GetActualDriveMode() && (TRUE == IsSetPointAck()))
		{
			m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x30);
		}
	}
	m_usStatusWordPrev = usStatusWord;


	/* write configured values to PDOS */
	CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(ControlWord), m_stSlaveParams.stOutPDOs.usControlWord, 16);
	CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos), m_stSlaveParams.stOutPDOs.nTargetPos, 32);
	CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetVel), m_stSlaveParams.stOutPDOs.nTargetVel, 32);
	CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetTor), m_stSlaveParams.stOutPDOs.sTargetTor, 16);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(ProfileVel), m_stSlaveParams.stOutPDOs.unProfileVel, 32);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(ProfileAcc), m_stSlaveParams.stOutPDOs.unProfileAcc, 32);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(ProfileDec), m_stSlaveParams.stOutPDOs.unProfileDec, 32);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(MaxProfileVel), m_stSlaveParams.stOutPDOs.unMaxProfileVel, 32);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(MaxProfileAcc), m_stSlaveParams.stOutPDOs.unMaxProfileAcc, 32);
	// CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(QuickStopDec), m_stSlaveParams.stOutPDOs.unQuickStopDec, 32);
	CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(DriveMode), m_stSlaveParams.stOutPDOs.btDriveMode, 8);
}

BOOL
CSlaveCIA402Base::RegisterPDO()
{
	/* Output PDOs */
	if (0 > (m_stSlaveParams.GET_OFFSET(ControlWord) = CEcatSlaveBase::RegisterPDOEntry(0x6040, 0x00, &m_stSlaveParams.GET_BITPOS(ControlWord), eOutput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos) = CEcatSlaveBase::RegisterPDOEntry(0x607A, 0x00, &m_stSlaveParams.GET_BITPOS(TargetPos), eOutput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(TargetVel) = CEcatSlaveBase::RegisterPDOEntry(0x60FF, 0x00, &m_stSlaveParams.GET_BITPOS(TargetVel), eOutput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(TargetTor) = CEcatSlaveBase::RegisterPDOEntry(0x6071, 0x00, &m_stSlaveParams.GET_BITPOS(TargetTor), eOutput)))
		return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(ProfileVel) = CEcatSlaveBase::RegisterPDOEntry(0x6081, 0x00, &m_stSlaveParams.GET_BITPOS(ProfileVel), eOutput)))
	// 	return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(ProfileAcc) = CEcatSlaveBase::RegisterPDOEntry(0x6083, 0x00, &m_stSlaveParams.GET_BITPOS(ProfileAcc), eOutput)))
	// 	return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(ProfileDec) = CEcatSlaveBase::RegisterPDOEntry(0x6084, 0x00, &m_stSlaveParams.GET_BITPOS(ProfileDec), eOutput)))
	// 	return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(QuickStopDec) = CEcatSlaveBase::RegisterPDOEntry(0x6085, 0x00, &m_stSlaveParams.GET_BITPOS(QuickStopDec), eOutput)))
	// 	return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(MaxProfileVel) = CEcatSlaveBase::RegisterPDOEntry(0x607F, 0x00, &m_stSlaveParams.GET_BITPOS(MaxProfileVel), eOutput)))
	// 	return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(MaxProfileAcc) = CEcatSlaveBase::RegisterPDOEntry(0x60C5, 0x00, &m_stSlaveParams.GET_BITPOS(MaxProfileAcc), eOutput)))
	// 	return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(DriveMode) = CEcatSlaveBase::RegisterPDOEntry(0x6060, 0x00, &m_stSlaveParams.GET_BITPOS(DriveMode), eOutput)))
		return FALSE;

	/* Input PDOs */
	if (0 > (m_stSlaveParams.GET_OFFSET(StatusWord) = CEcatSlaveBase::RegisterPDOEntry(0x6041, 0x00, &m_stSlaveParams.GET_BITPOS(StatusWord), eInput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(ActualPos) = CEcatSlaveBase::RegisterPDOEntry(0x6064, 0x00, &m_stSlaveParams.GET_BITPOS(ActualPos), eInput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(ActualVel) = CEcatSlaveBase::RegisterPDOEntry(0x606C, 0x00, &m_stSlaveParams.GET_BITPOS(ActualVel), eInput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(ActualTor) = CEcatSlaveBase::RegisterPDOEntry(0x6077, 0x00, &m_stSlaveParams.GET_BITPOS(ActualTor), eInput)))
		return FALSE;
	if (0 > (m_stSlaveParams.GET_OFFSET(DriveModeDisplay) = CEcatSlaveBase::RegisterPDOEntry(0x6061, 0x00, &m_stSlaveParams.GET_BITPOS(DriveModeDisplay), eInput)))
		return FALSE;
	// if (0 > (m_stSlaveParams.GET_OFFSET(ErrorCode) = CEcatSlaveBase::RegisterPDOEntry(0x603F, 0x00, &m_stSlaveParams.GET_BITPOS(ErrorCode), eInput)))
	// 	return FALSE;

	// if (0 > (m_stSlaveParams.GET_OFFSET(AdditionalPos) = CEcatSlaveBase::RegisterPDOEntry(0x2fe4, 0x02, &m_stSlaveParams.GET_BITPOS(AdditionalPos), eInput)))
	// 	return FALSE;

	// if (0 > (m_stSlaveParams.GET_OFFSET(MotorRatedCurrent) = CEcatSlaveBase::RegisterPDOEntry(0x6075, 0x00, &m_stSlaveParams.GET_BITPOS(MotorRatedCurrent), eInput)))
	// 	return FALSE;

	return TRUE;
}

BOOL
CSlaveCIA402Base::SetTargetPos(INT32 anTargetPos, BOOL abForced, BOOL abRelative)
{
	if ((CIA402_CYCLIC_POSITION != GetActualDriveMode()) && (CIA402_PROFILE_POSITION != GetActualDriveMode()))
		return FALSE;

	if (TRUE == abForced || TRUE == IsHalt())
	{
		SetNewSetPoint(TRUE);
		SetHalt(FALSE);
	}
	else
	{
		SetNewSetPoint(FALSE);
	}

	SetRelativePosMode(abRelative);
	SetTargetPosPDO(anTargetPos);

	return TRUE;
}

BOOL
CSlaveCIA402Base::SetTargetVel(INT32 anTargetVel, BOOL abForced)
{
	if ((CIA402_CYCLIC_VELOCITY != GetActualDriveMode()) && (CIA402_PROFILE_VELOCITY != GetActualDriveMode()))
		return FALSE;

	SetTargetVelPDO(anTargetVel);

	if (TRUE == abForced || TRUE == IsHalt())
	{
		SetNewSetPoint(TRUE);
		SetHalt(FALSE);
	}
	else
	{
		SetNewSetPoint(FALSE);
	}

	return TRUE;
}

/*
 * Attempt to set the CIA402 state machine to SwitchON if the current state is not SwitchOn.
 * Arguments	:	void
 *
 * Return		:	BOOL = TRUE if success, otherwise FALSE
 *
*/
BOOL
CSlaveCIA402Base::SetServoOff()
{
	if (FALSE == IsServoOn()) return TRUE;
	
	SetServoOnOff(FALSE);
	return TRUE;
}

/*
 * Attempt to set the CIA402 state machine to OP if the current state is not OP.
 * Otherwise, change the drive mode only.
 * Arguments	:	INT8 abtDriveMode = CIA402 operation mode 
 * 
 * Return		:	BOOL = TRUE if success, otherwise FALSE
 * 
*/
BOOL
CSlaveCIA402Base::SetServoOn(INT8 abtDriveMode)
{
	SetDriveMode((INT8)abtDriveMode);

	if (FALSE == IsServoOn())
		SetServoOnOff(TRUE);
	
	return TRUE;
}

BOOL
CSlaveCIA402Base::IsRelativePos()
{
	if (m_ePosMode == eAbsolute) return TRUE;

	return FALSE;
}

BOOL
CSlaveCIA402Base::SetEndlessMovement(BOOL abEndless)
{
	if (CIA402_PROFILE_POSITION != GetActualDriveMode()) return FALSE;

	if (TRUE == abEndless)
		m_stSlaveParams.stOutPDOs.usControlWord |= (UINT16)(0x8000);
	else
		m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x8000);

	m_bIsEndlessMovement = abEndless;

	return m_bIsEndlessMovement;
}

BOOL
CSlaveCIA402Base::SetRelativePosMode(BOOL abRelative)
{
	if (CIA402_PROFILE_POSITION != GetActualDriveMode()) return FALSE;

	m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x40);

	m_ePosMode = eAbsolute;
	
	if (TRUE == abRelative)
	{
		m_stSlaveParams.stOutPDOs.usControlWord |= (UINT16)(0x40);
		m_ePosMode = eRelative;
	}
	return TRUE;
}

BOOL
CSlaveCIA402Base::SetNewSetPoint(BOOL abForced)
{
	if (CIA402_PROFILE_POSITION == GetActualDriveMode() && TRUE == abForced)
		m_stSlaveParams.stOutPDOs.usControlWord |= (UINT16)(0x20);
	else
		m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x20);

	m_stSlaveParams.stOutPDOs.usControlWord |= (UINT16)(0x10);
	return TRUE;
}

/* Quick Stop*/
BOOL
CSlaveCIA402Base::SetEmgStop()
{
	m_stSlaveParams.stOutPDOs.usControlWord = CIA402_QUICK_STOP;
	return TRUE;
}

BOOL
CSlaveCIA402Base::SetHalt(BOOL abHalt)
{
	if (TRUE == abHalt)
		m_stSlaveParams.stOutPDOs.usControlWord |= (UINT16)(0x100);
	else
		m_stSlaveParams.stOutPDOs.usControlWord &= (UINT16)(~0x100); // Disable Halt

	m_bIsHalt = abHalt;

	return TRUE;
}

void
CSlaveCIA402Base::GoHome(	)
{
	if (FALSE == IsServoOn())
		SetServoOn(CIA402_HOMING);
	else
		SetDriveMode(CIA402_HOMING);

	m_bTriggerGoHome = TRUE;
}