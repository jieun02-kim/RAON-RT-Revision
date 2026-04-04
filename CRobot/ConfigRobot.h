/*****************************************************************************
*	Name: ConfigRobot.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for CConfigRobot class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __CONFIG_ROBOT__
#define __CONFIG_ROBOT__

#include "Defines.h"
#include "ConfigParser.h"

#define DEFAULT_CONFIGURATION_FULLPATH ("ELMO.cfg")

typedef struct stConfigTask
{
	BOOL	bEnabled;
	TSTRING	strName;
	INT32	nPriority;
	INT32	nStartDelay;
	INT32	nPeriod;

	stConfigTask()
	{
		bEnabled = FALSE;
		strName = "";
		nPriority = 0;
		nStartDelay = 0;
		nPeriod = 0;
	}
}ST_CONFIG_TASK;

typedef struct stConfigEcatMaster
{
	BOOL	bDCEnabled;
	INT32	nNoOfSlaves;
	INT32 	nNoOfHands;
	INT32	nCycleTime;

	stConfigEcatMaster()
	{
		bDCEnabled = FALSE;
		nNoOfSlaves = 0;
		nCycleTime = 0;
	}

}ST_CONFIG_ECAT_MASTER;

typedef struct stConfigEcatSlave
{
	BOOL	bEnabled;
	BOOL	bConnected;
	TSTRING	strName;
	INT32	nPhyPos;
	UINT32	uVendorID;
	UINT32	uProductCode;
	BOOL	bDCSupported;
	UINT16	usDCActivateWord;
	INT32	nDCSyncShift;
	INT32   nSlaveType; // 0: Axis, 1: Sensor, 3: Terminal, 4: Junction Box, 5: Hand

    BOOL    bAutoServoOn;
    BOOL    bAbsEnc;
    BOOL    bCCW;
    INT32   nJointType;
    INT32   nDriveMode;
    double  dEncResolution;
    double  dGearRatio;
    double  dTransRatio;
    double  dTorqueConstant;
    double  dCurrentRatio;
    double  dOneTurnRef;

    INT32   nHomingMethod;
    double  dHomeSearchRef;
    INT32   nHomePositionOffset;

	double	dOperatingVel;
	double	dOperatingAcc;
	double	dOperatingDec;

	double	dPosLimitU;
	double	dPosLimitL;

	double	dVelLimitU;
	double	dVelLimitL;

	double	dAccLimitU;
	double	dAccLimitL;

	double	dDecLimitU;
	double	dDecLimitL;

	double dTorLimitU;
	double dTorLimitL;

	double	dJerkLimitU;
	double	dJerkLimitL;

	double dRatedTorque;
	double dRatedCurrent;

	INT32	nPosBeforeExit;

	stConfigEcatSlave()
	{
		bEnabled = FALSE;
		bConnected = FALSE;
		strName = "EtherCAT Slave";
		nPhyPos = 0;
		uVendorID = 0;
		uProductCode = 0;
		bDCSupported = FALSE;
		usDCActivateWord = 0x300;
		nDCSyncShift = 0;
		nSlaveType = 0;

        bAutoServoOn      = FALSE;
        bAbsEnc           = FALSE;
        bCCW              = TRUE;
        nJointType        = 0;
        nDriveMode        = 1;
        dEncResolution    = 1.;
        dGearRatio        = 1.;
        dTransRatio       = 1.;
        dTorqueConstant   = 1.;
        dCurrentRatio     = 1.;

        dOneTurnRef       = 1.;

		nHomingMethod = 0;
		dHomeSearchRef = 1.;
		nHomePositionOffset = 0;

		dOperatingVel = 0.;
		dOperatingAcc = 0.;
		dOperatingDec = 0.;

		dPosLimitL = 0.;
		dPosLimitU = 0.;

		dVelLimitL = 0.;
		dVelLimitU = 0.;

		dAccLimitL = 0.;
		dAccLimitU = 0.;

		dDecLimitL = 0.;
		dDecLimitU = 0.;
		
		dTorLimitL = 0.;
		dTorLimitU = 0.;
		
		dJerkLimitL = 0.;
		dJerkLimitU = 0.;

		dRatedTorque = 0.0;
		dRatedCurrent = 0.0;

		nPosBeforeExit = 0;
	}
}ST_CONFIG_ECAT_SLAVE;


typedef struct stHandConfigEcatSlave {
	BOOL	bEnabled;
	BOOL	bConnected;
	TSTRING	strName;
	INT32	nPhyPos;
	UINT32	uVendorID;
	UINT32	uProductCode;
	BOOL	bDCSupported;
	UINT16	usDCActivateWord;
	INT32	nDCSyncShift;
	INT32 	nNoAxes;

	double dPosLimitU[16];
	double dPosLimitL[16];
	
	stHandConfigEcatSlave()
	{
		bEnabled = FALSE;
		bConnected = FALSE;
		strName = "EtherCAT Hand";
		nPhyPos = 0;
		uVendorID = 0;
		uProductCode = 0;
		bDCSupported = FALSE;
		usDCActivateWord = 0x300;
		nDCSyncShift = 0;
		nNoAxes = 0;

		for (int i = 0; i < 16; i++)
		{
			dPosLimitU[i] = 0.;
			dPosLimitL[i] = 0.;
		}
	}

} ST_HAND_CONFIG_ECAT_SLAVE;




typedef struct stConfigExtInterface
{
	BOOL	bEnabled;
	INT32	nPort;

	stConfigExtInterface()
	{
		bEnabled = FALSE;
		nPort = 7420;
	}
}ST_CONFIG_EXT_IFACE;

typedef struct stConfigSystem
{
	TSTRING strName;
	BOOL	bSim;
	BOOL	bAging;
	INT32	nAgingDur;
	INT32	nTeleMode;
	double	dTeleScalingRatio;
	TSTRING strURDFPath;
	TSTRING strMojucoXmlPath;
	BOOL	bEnableControllerAtStartup; 
	INT32	nDefaultControllerMode; // 0: Gravity Compensation, 1: CTC, 2: Full Dynamics
	INT32	nRobotAxis; // Number of robot axes
	std::vector<double> vKp; // Kp gains for each axis
	std::vector<double> vKd; // Kd gains for each axis

	ST_CONFIG_EXT_IFACE stExternalIface;

	stConfigSystem()
	{
		strName = "ROBOT";
		strURDFPath = "TEST.urdf";
		strMojucoXmlPath = "TEST.xml";
		bSim = FALSE;
		bAging = FALSE;
		nTeleMode = 0;
		dTeleScalingRatio = 0.;
		bEnableControllerAtStartup = TRUE;
		nDefaultControllerMode = 0; //
		vKp.resize(6, 0.0); // Default 6 DOF
		vKd.resize(6, 0.0); // Default 6 DOF  
	};

}ST_CONFIG_SYSTEM;

typedef std::vector<ST_CONFIG_TASK>			VEC_TASK_CONF;
typedef std::vector<ST_CONFIG_ECAT_SLAVE>	VEC_ECAT_SLAVE_CONF;
typedef std::vector<ST_HAND_CONFIG_ECAT_SLAVE>	VEC_HAND_ECAT_SLAVE_CONF;

class CConfigRobot
{
public:
	CConfigRobot();
	virtual ~CConfigRobot();

public:
	void						SetConfigPath		(TSTRING astrPath)	{ m_strConfigPath = astrPath; }
	TSTRING						GetConfigPath		(	)				{ return m_strConfigPath; }
	BOOL						ReadConfiguration	(TSTRING astrPath = "DEFAULT");
	void						LoadDefaultConfig	(	);
	BOOL						UpdateConfiguration	(	);
	BOOL						CreateDefaultConfig	(	);

	VEC_TASK_CONF				GetTaskList			(	) { return m_vstTaskList; }
	VEC_ECAT_SLAVE_CONF			GetEcatSlaveList	(	) { return m_vstEcatSlaveList; }
	VEC_HAND_ECAT_SLAVE_CONF	GetHandEcatSlaveList(	) { return m_vstHandEcatSlaveList; }
	ST_CONFIG_ECAT_MASTER		GetEcatMasterConf	(	) { return m_stEcatMaster; }
	ST_CONFIG_SYSTEM			GetSystemConf		(	) { return m_stSystem; }
	ST_CONFIG_EXT_IFACE			GetExtIfaceConf		(	) { return m_stSystem.stExternalIface; }
	void						WriteLastPosition	(INT32, INT32);

private:
	BOOL	ReadRTTaskConfig	(	);
	BOOL	ReadEtherCATConfig	(	);
	BOOL	ReadSystemConfig	(	);

private:
	TSTRING					m_strConfigPath;
	ST_CONFIG_ECAT_MASTER	m_stEcatMaster;
	ST_CONFIG_SYSTEM		m_stSystem;
	ST_CONFIG_EXT_IFACE		m_stExtIface;
	VEC_TASK_CONF			m_vstTaskList;
	VEC_ECAT_SLAVE_CONF		m_vstEcatSlaveList;
	VEC_HAND_ECAT_SLAVE_CONF m_vstHandEcatSlaveList;

}; // CConfigRobot



#endif // __CONFIG_ROBOT__


