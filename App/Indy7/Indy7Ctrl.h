/*****************************************************************************
*	Name: RobotUSurgery.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CRobotIndy7 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROBOT_EXAMPLE_APP__
#define __ROBOT_EXAMPLE_APP__

#include <list>
#include <queue>

#include "Robot.h"
#include "AxisNRMKCore.h"
#include "SensorNRMKEndTool.h"
#include "FullDynControllerRT.h"


typedef std::vector<double> VECDOUBLE;
typedef std::list<UINT64>   LISTULONG;
typedef std::list<INT32>	LISTINT;


typedef struct stDataLog
{
	LISTULONG	vecTimestamp;
	LISTINT		vecTarPos;
	LISTINT		vecActPos;
	LISTINT		vecActTor;

	stDataLog()
	{
		vecTimestamp.clear();
		vecTarPos.clear();
		vecActPos.clear();
		vecActTor.clear();
	}

}ST_DATALOG;

class CRobotIndy7 : public CRobot
{
public:
	CRobotIndy7(CConfigRobot* apcConfig = NULL);
	virtual ~CRobotIndy7();

public:
	virtual BOOL	Init					(BOOL abSim);
	virtual BOOL	DeInit					(	);
	void	WriteDataLog					(	);
	
	/* Controller */
	CControllerFullDynamicsRT* GetController() { return m_pController; }
    BOOL InitController(const TSTRING& astrURDFPath);
    BOOL EnableController(BOOL abEnable);
    BOOL SetControllerMode(CControllerFullDynamicsRT::eControlMode aeMode);
    BOOL SetControllerGains(const std::vector<double>& avKp, const std::vector<double>& avKd);
	BOOL IsMoving();

private:
	TSTRING		m_strDataLog;
	ST_DATALOG	m_stDataLog[32];

protected:
	virtual BOOL	InitEtherCAT			(	);
	virtual BOOL	InitConfig				(	);
	virtual void	DoAgingTest				(	);

	CAxisNRMKCore**			m_pEcatAxis;
	CSensorNRMKEndTool**	m_pEcatSensor;

	/* TEMPORARY */
	char	m_cKeyPress;
	void	DoInput						(	);
	void	DoHoming() {};

protected:
	friend void	proc_main_control(void*);
	friend void proc_ethercat_control(void*);
	friend void	proc_keyboard_control(void*);
	friend void	proc_logger(void*); 
	friend void proc_terminal_output(void*);

private:
	BOOL m_bEcatOP;
	UINT32	m_nEcatCycle;
	BOOL m_bEnableTriangleControl{false};

	/* Controller */
	CControllerFullDynamicsRT*	m_pController;
	BOOL m_bRTControllerEnabled;

	// control vectors 
    std::vector<double> m_vCurrentPos;
    std::vector<double> m_vCurrentVel;
    std::vector<double> m_vCurrentTor;
    std::vector<double> m_vOutputTorque;
}; 
#endif //__ROBOT_EXAMPLE_APP__
