/*****************************************************************************
*	Name: Robot.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CRobot class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Robot.h"

CRobot::CRobot()
{
	m_vAxis.clear();
	m_vKistarHand.clear();
	m_pcEcatMaster = NULL;
	m_pcConfigRobot = NULL;
	m_pcExtInterface = NULL;
	m_nTotalRTTasks = 0;
	m_bAging = FALSE;
	m_nAgingDur = 60;
	m_dAgingFreq = 1.;
}

CRobot::~CRobot()
{
}

BOOL	
CRobot::Init(BOOL abSim)
{
	m_bIsSim = abSim;

	if (FALSE == abSim)
	{
		if (FALSE == InitExtInterface())
		{
			DBG_LOG_WARN("(%s) Cannot Initialize or Disabled External Interface", "CRobot");
			// we do not need to close entire application
		}

		if (FALSE == InitEtherCAT())
		{
			DBG_LOG_ERROR("(%s) Cannot Init EtherCAT Connection", "CRobot");
			return FALSE;
		}

		if (FALSE == InitRTTasks())
		{
			DBG_LOG_ERROR("(%s) Cannot Init RT Tasks", "CRobot");
			return FALSE;
		}

	}
	DBG_LOG_INFO("(%s) Initialized Successfully!", "CRobot");
	DBG_LOG_INFO("(%s) Robot Name: %s", "CRobot", GetName().c_str());
	return TRUE;
}

BOOL
CRobot::InitExtInterface()
{
	BOOL bExtIfaceEnabled = FALSE;
	if (m_pcConfigRobot)
		bExtIfaceEnabled = m_pcConfigRobot->GetSystemConf().stExternalIface.bEnabled;

	if (TRUE == bExtIfaceEnabled)
	{
		m_pcExtInterface = new CExtInterface();
		m_pcExtInterface->RegisterCallbackAxisCmd(std::bind(&CRobot::OnRecvAxisCommand, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

		return m_pcExtInterface->Init(this);
	}
	return FALSE;
}

BOOL
CRobot::DeInit()
{
	StopTasks();
	/* we need delay of 2 seconds to ensure that all tasks are deleted before EtherCAT deinit */
	usleep(2000000);
	if (NULL != m_pcEcatMaster)
	{
		m_pcEcatMaster->DeInit();
	}

	// for (INT nCnt = 0; nCnt < GetTotalAxis(); nCnt++)
	// {
	// 	m_pcConfigRobot->WriteLastPosition(nCnt, m_vAxis[nCnt]->GetCurrentRawPos());
	// }

	if (NULL != m_pcConfigRobot)
	{
		delete m_pcConfigRobot;
		m_pcConfigRobot = NULL;
	}

	m_vAxis.clear();
	m_vTask.clear();
	m_vTaskFunctions.clear();
	m_vKistarHand.clear();

	SetState(eRobotDeInit);
	return TRUE;
}

INT
CRobot::GetTotalAxis()
{
	return (INT)m_vAxis.size();
}


BOOL
CRobot::InitRTTasks()
{
	VEC_TASK_CONF vTaskConf = m_pcConfigRobot->GetTaskList();
	INT32 nTotalRtTasks = (INT32)vTaskConf.size();
	
	if ((INT32)m_vTaskFunctions.size() != nTotalRtTasks)
		return FALSE;

	/* separate task creation from execution */
	for (int nCnt = 0; nCnt < nTotalRtTasks; nCnt++)
	{
		PTASK taskTemp;

		create_rt_task(&taskTemp, (const PCHAR)vTaskConf[nCnt].strName.c_str(), 0, vTaskConf[nCnt].nPriority);

		if (0 != vTaskConf[nCnt].nPeriod)
		{
			RTTIME tmStartDelay;
			if (0 == vTaskConf[nCnt].nStartDelay)
				tmStartDelay = SET_TM_NOW;
			else
				tmStartDelay = read_timer() + vTaskConf[nCnt].nStartDelay;

			set_task_period(&taskTemp, tmStartDelay, vTaskConf[nCnt].nPeriod);
		}
		m_vTask.push_back(taskTemp);
	}

	for (int nCnt = 0; nCnt < nTotalRtTasks; nCnt++)
	{
		if (TRUE == vTaskConf[nCnt].bEnabled)
		{
			start_task(&m_vTask[nCnt], m_vTaskFunctions[nCnt], this);
		}
			
	}
	return TRUE;
}


void
CRobot::EnableAgingTest(BOOL abAging)
{
	m_bAging = abAging;
}

void
CRobot::SetAgingParams(INT anDuration, double adFrequency)
{
	m_nAgingDur = anDuration;
	m_dAgingFreq = adFrequency;
}

void
CRobot::UpdateExtInterfaceData()
{
	if (m_pcExtInterface)
	{
		if (m_pcEcatMaster)
		{
			m_pcExtInterface->UpdateEcatMetadata((UINT16)m_pcEcatMaster->GetSlaveCnt(), m_pcEcatMaster->GetMasterState(), m_pcEcatMaster->GetSlaveState(), m_pcEcatMaster->GetDomainState());
		}
	}
}

void CRobot::OnRecvAxisCommand(PVOID apAxisCmd, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2)
{

	VEC_AXIS_CMD vAxisCmd = *(VEC_AXIS_CMD*)apAxisCmd;
	//m_qAxisCmd.push(vAxisCmd);
}

BOOL
CRobot::IsEcatEnabled()
{
	if (m_pcEcatMaster)
		return TRUE;

	return FALSE;
}

INT
CRobot::GetTotalEcatSlaves()
{
	if (TRUE == IsEcatEnabled())
		return m_pcEcatMaster->GetSlaveCnt();

	return 0;
}

BOOL
CRobot::AddAxis(CAxis* apcAxis)
{
	/* check if axis already exists */
	if (0 > GetTotalAxis())
	{
		for (int nCnt = 0; nCnt < GetTotalAxis(); nCnt++)
		{
			if (apcAxis->GetName() == m_vAxis[nCnt]->GetName())
			{
				return FALSE;
			}
		}
	}
	
	m_vAxis.push_back(apcAxis);
	
	if (m_pcExtInterface)
	{
		m_pcExtInterface->RegisterAxis(apcAxis);
	}
	
	return TRUE;
}

BOOL 
CRobot::AddTaskFunction(PTASKFCN apTaskFn)
{
	m_vTaskFunctions.push_back(apTaskFn);
	return TRUE;
}

BOOL CRobot::AddHand(CKistarHand* apHand)
{
	if (0 > GetTotalHands())
	{
		for (int nCnt = 0; nCnt < GetTotalHands(); nCnt++)
		{
			if (apHand->GetName() == m_vKistarHand[nCnt]->GetName())
			{
				return FALSE;
			}
		}
	}

	m_vKistarHand.push_back(apHand);
	
	return TRUE;
}
