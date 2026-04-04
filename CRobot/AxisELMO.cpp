/*****************************************************************************
*	Name: AxisEPOS4.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CAxisEPOS4 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "AxisELMO.h"
#include <algorithm>
#include <functional>
#include "posix_rt.h"

/* adEncRes should include the encoding type of the encoder (X1, X2, X4)
* For joint types other than revolute, the adOneTurnRef should be given explicitly
* Ex) Screw-type prismatic joint, one turn is equal to the lead of the screw, thus adOneTurnRef should be equal to the lead (pitch * number of starts) of the screw.
*/
CAxisELMO::CAxisELMO(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
	: CAxisCIA402(aeAxisType, adEncRes, adGearRatio, adTransRatio, abAbsoluteEncoder, abCCW, abEnabled, abConnected)
{
	
}

BOOL
CAxisELMO::MoveTorque(double adTor) // Nm
{
	if (FALSE == IsMovable()) return FALSE;

	/* only allow this when in any of the torque-based operation mode */
	if ((CIA402_CYCLIC_TORQUE != m_cEcSlave.GetActualDriveMode()) && (CIA402_PROFILE_TORQUE != m_cEcSlave.GetActualDriveMode()))
		return FALSE;

	// printf("adTor:%f ", adTor);
	// printf("tar torq: %d\n",(INT16)ConvertTor2Res(adTor));
	m_dTargetTorq = adTor;

	double dTor = adTor*1000.0; // Nm --> mNm

	if (!IsAllowableTorque(dTor)) {	
		DBG_LOG_WARN("Torque Limited");
	}

	if (IsAllowablePosition(GetCurrentPos())) {
		m_cEcSlave.SetTargetTor((INT16)ConvertTor2Res(dTor));
	}
	else {
		m_cEcSlave.SetTargetTor(0); // set target torque to 0 to avoid exceeding the position limit
		DBG_LOG_WARN("(%s) Axis[%d:%d] Move Torque - Exceeded Position Limit!", "CAxisELMO", m_usAxisAlias, m_usAxisPos);		
		return FALSE;
	}

	return TRUE;
}

double CAxisELMO::GetAdditionalPos()
{
	double dAdditionalPos = m_cEcSlave.GetAdditionalPos()*m_dOneTurnRef/m_dResolution;
	
	return dAdditionalPos;
}

double CAxisELMO::GetCurrentTor()
{
	double torq = m_pstAxisParams->dTor;

	return torq/1000.0; // Nm
}

/* Callback function for servo status change */
void
CAxisELMO::OnSlaveStatusChanged(PVOID apSlave, PVOID apStatus, PVOID apReserved1, PVOID apReserved2)
{
	CSlaveCIA402Base cEcatSlave = *(CSlaveCIA402Base*)apSlave;
	eSERVO_STATUS	eServoStatus = *(eSERVO_STATUS*)apStatus;
	
	switch (eServoStatus)
	{
	case eServoOff:
		//SetState(eAxisInit);
		// 
		break;
	case eServoIdle:
        m_bFirstHome = FALSE; 
		if (TRUE == m_bFirstHome)
		{
			INT32 nStartPos = cEcatSlave.GetActualPos();
            printf("nStartPos:%d\n", nStartPos);
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