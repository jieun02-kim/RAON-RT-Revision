/*****************************************************************************
*	Name: AxisNRMKCore.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of the CAxisNRMKCore child class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "AxisNRMKCore.h"
#include <algorithm>
#include <functional>
#include "posix_rt.h"

/* adEncRes should include the encoding type of the encoder (X1, X2, X4)
* For joint types other than revolute, the adOneTurnRef should be given explicitly
* Ex) Screw-type prismatic joint, one turn is equal to the lead of the screw, thus adOneTurnRef should be equal to the lead (pitch * number of starts) of the screw.
*/
CAxisNRMKCore::CAxisNRMKCore(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
	: CAxisCIA402(aeAxisType, adEncRes, adGearRatio, adGearRatio, abAbsoluteEncoder, abCCW, abEnabled, abConnected)
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

BOOL
CAxisNRMKCore::MoveTorque(double adTor)
{

	if (FALSE == IsMovable()) return FALSE;

	/* only allow this when in any of the torque-based operation mode */
	if ((CIA402_CYCLIC_TORQUE != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_TORQUE != m_cEcSlave.GetActualDriveMode()))
		return FALSE;

	// motor_torque = adTor / m_dGearRatio
	double dmotorcurrent = (adTor / m_dGearRatio / m_dTorqueConstant);

	//printf("%lf %lf %d\n", adTor, motor_current, (INT16)(motor_current * m_dCurrentRatio));

	m_cEcSlave.SetTargetTor((INT16)(dmotorcurrent * m_dCurrentRatio * m_nDirection)); // * m_nDirection

	return TRUE;
}

/* Callback function for servo status change */
void
CAxisNRMKCore::OnSlaveStatusChanged(PVOID apSlave, PVOID apStatus, PVOID apReserved1, PVOID apReserved2)
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
				SetHomePosition(nStartPos);
				// SetHomePosition(nStartPos - GetPositionBeforeExit());
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

INT32
CAxisNRMKCore::ConvertTor2Res(double adTor)
{
	// motor_torque = adTor / m_dGearRatio
	double dmotorcurrent = (adTor / m_dGearRatio / m_dTorqueConstant);
	INT32 res = (INT32)(dmotorcurrent * m_dCurrentRatio * m_nDirection); 
	return res;
}

double
CAxisNRMKCore::ConvertRes2Tor(INT32 adRes)
{
	double dmotorcurrent = (double)adRes / m_dCurrentRatio;
	double dtorque = dmotorcurrent * m_dGearRatio * m_dTorqueConstant * m_nDirection; 
	return dtorque;
}