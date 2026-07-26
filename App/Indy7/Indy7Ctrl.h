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
#include <atomic>
#include <ctime>

#include "Robot.h"
#include "AxisNRMKCore.h"
#include "SensorNRMKEndTool.h"
#include "FullDynControllerRT.h"
#include "DataRecorder.h"
#include "CalibCapture.h"
// [kv260-merge] VisualServo (ViSP AprilTag) removed from the control app:
// it opens the RealSense directly and cannot coexist with the perception
// pipeline. Goal poses come from ROS2 (/pick_target_base) instead.
// VisualServo.{h,cpp} remain in-tree (unbuilt) as closed-loop reference.


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

	enum eISOHWState { eISO_HW_IDLE=0, eISO_HW_TO_TARGET, eISO_HW_TO_CLEARANCE, eISO_HW_DONE };
	enum eRectState  { eRECT_IDLE=0, eRECT_TO_CORNER };
	
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
	friend void proc_terminal_output(void*);
	friend void proc_logger(void*);
	friend FILE* make_csv(CRobotIndy7*);

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

	//=====================================================
	//jieun

	// TCP Trajectory Logging
	LogRingBuffer            m_logBuffer;
	std::atomic<bool>        m_bLogTrigger{false};

	// ISO Hardware Test
	std::atomic<bool>               m_bIsoHWTrigger{false};
	eISOHWState                     m_eISOHWState{eISO_HW_IDLE};
	int                             m_nISOPointIdx{0};
	int                             m_nISOCycle{0};
	bool                            m_bISOCmdSent{false};
	double                          m_isoHWRecords[5][10][3];
	CControllerFullDynamicsRT::Pose m_isoTargets[5];
	CControllerFullDynamicsRT::Pose m_isoClearance;

	// Rectangle (Square) Motion — 'n' key
	std::atomic<bool>               m_bRectTrigger{false};
	eRectState                      m_eRectState{eRECT_IDLE};
	int                             m_nRectCornerIdx{0};
	int                             m_nRectWaitCount{0};
	CControllerFullDynamicsRT::Pose m_rectCorners[4];
	RigidBodyDynamics::Math::VectorNd m_rectJointTargets[4];

	CControllerFullDynamicsRT::Pose	m_Pose;
	void SaveISOHWResults();
	void SaveRobotPose();
	int  m_nPoseCapture{0};

	//=====================================================

public:
	//=====================================================



};

//=====================================================
// jieun — ViSP / Calibration
// Defined in Indy7Ctrl.cpp; visible to any TU that includes this header.
extern CalibCapture s_calibCapture;
//=====================================================

#endif //__ROBOT_EXAMPLE_APP__
