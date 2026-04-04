/*****************************************************************************
*	Name: Axis.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CAxis class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __AXIS__
#define __AXIS__

#include "Defines.h"


typedef enum
{
	eAxisHomingNotSet = 0,
	eAxisHomeStartPos,
	eAxisHomeSearch,
	eAxisHomeManual,
	eAxisHomeBuiltin,
} eHomingMethod;

typedef enum
{
	eAxisRS232 = 0,
	eAxisRS485,
	eAxisEthernet,
	eAxisEtherCAT,
	eAxisMuJoCo = 0x20,
	eAxisUnknownCommType = 0x30,
} eCommType;

typedef enum
{
	eAxisRevolute = 0,
	eAxisContinuous,
	eAxisPrismatic,
	eAxisPlanar,
	eAxisFixed,
	eAxisFloating,
	eAxisJointless,
	eAxisSensor,
}eAxisType;


typedef enum
{
	eAxisEmergency = 0x00,
	eAxisError,
	eAxisUnknownState,
	eAxisDeinit,
	eAxisDisconnected,
	eAxisInit,
	eAxisIdle,
	eAxisHoming,
	eAxisRunning,
	eAxisStopped,
} eAxisState;

typedef struct stAxisInfo
{
	eAxisState eState;
	eAxisType eJoint;
	eCommType eComm;
	eHomingMethod eHoming;

	stAxisInfo()
	{
		eState = eAxisUnknownState;
		eJoint = eAxisRevolute;
		eComm = eAxisEtherCAT;
		eHoming = eAxisHomingNotSet;
	}

}ST_AXIS_INFO;


typedef struct stAxisParams
{
	double dPos;
	double dVel;
	double dTor;

	stAxisParams()
	{
		dPos = 0.;
		dVel = 0.;
		dTor = 0.;
	}
}ST_AXIS_PARAMS;

typedef struct stAxisRawParams
{
	INT32 nPos;
	INT32 nVel;
	INT32 nTor;

	stAxisRawParams()
	{
		nPos = 0;
		nVel = 0;
		nTor = 0;
	}
}ST_AXIS_RAW_PARAMS;

typedef struct stLimits
{
	BOOL	bIsSet;
	double	dUpper;
	double	dLower;
	
	stLimits()
	{
		bIsSet = FALSE;
		dUpper = 0.;
		dLower = 0.;
	}
}ST_LIMITS;

typedef struct stAxisLimits
{
	BOOL bIsSet;
	ST_LIMITS stPos;
	ST_LIMITS stVel;
	ST_LIMITS stAcc;
	ST_LIMITS stDec;
	ST_LIMITS stJerk;
	ST_LIMITS stTorque;

	stAxisLimits()
	{
		bIsSet = FALSE;
	}

}ST_AXIS_LIMITS;


class CAxis
{

public:
	CAxis(eCommType, eAxisType, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE);
	virtual ~CAxis();

protected:
	virtual BOOL InitHW();
	virtual BOOL InitSW();
	virtual BOOL DeInitHW();
	virtual BOOL DeinitSW();

public:
	virtual	BOOL	Init					(	);
	virtual	BOOL	DeInit					(	);
	virtual	BOOL	IsEnabled				(	)				{ return m_bEnabled; }
	virtual void	SetEnabled				(BOOL abEnabled)	{ m_bEnabled = abEnabled; }
	virtual BOOL	IsAutoServoOn			(	) 				{ return m_bIsAutoServoOn; }
	virtual void	SetAutoServoOn			(BOOL abEnabled) 	{ m_bIsAutoServoOn = abEnabled; }

	virtual BOOL	ServoOn					(	);
	virtual BOOL	ServoOff				(	);
	virtual BOOL	DoHoming				(	);
	virtual BOOL	MoveHome				(BOOL abForced = FALSE);
	virtual BOOL	MovePosLimit			(BOOL abForced = FALSE);
	virtual BOOL	MoveNegLimit			(BOOL abForced = FALSE);
	virtual BOOL	MoveAxis				(double, BOOL abForce=FALSE, BOOL abAbs=TRUE);
	virtual BOOL	StopAxis				(	);
	virtual BOOL	EmgStopAxis				(	);
	virtual BOOL	MoveVelocity			(double);
	virtual BOOL	MoveTorque				(double);
	virtual BOOL	SetVelocity				(double);
	virtual BOOL	SetAcceleration			(double);
	virtual BOOL	SetDeceleration			(double);
	virtual BOOL	SetJerk					(double);
	virtual BOOL	SetTorque				(double);
	virtual BOOL	SetHomePosition			(INT32);
	virtual INT32	GetHomePosition			(	);
	virtual BOOL	IsHomeSet				(	) { return m_bHomeSet; }
	virtual void	ClearHomeSet			(	) { m_bHomeSet = FALSE; }
	virtual void	ClearHomeRefSet			(	) { m_bHomeRefSet = FALSE; }
	virtual void	SetPositionBeforeExit	(double adPos) { m_nPosBeforeExit = adPos; }
	virtual double	GetPositionBeforeExit	(	) { return m_nPosBeforeExit; }
	virtual BOOL	SetPositionLimits		(double, BOOL abIsRad = FALSE);
	virtual BOOL	SetPositionLimits		(double, double, BOOL abIsRad = FALSE);
	virtual BOOL	SetVelocityLimits		(double, double);
	virtual BOOL	SetVelocityLimits		(double);
	virtual BOOL	SetAccelerationLimits	(double);
	virtual BOOL	SetAccelerationLimits	(double, double);
	virtual BOOL	SetDecelerationLimits	(double);
	virtual BOOL	SetDecelerationLimits	(double, double);
	virtual BOOL	SetJerkLimits			(double);
	virtual BOOL	SetJerkLimits			(double, double);
	virtual BOOL	SetTorqueLimits			(double);
	virtual BOOL	SetTorqueLimits			(double, double);
	virtual BOOL	SetTorqueConstant		(double);

	virtual void	UpdateCurrentParams		(INT32 anPos, INT32 anVel, INT32 anTor);
	virtual void	UpdateStartRawParams	(INT32 anPos, INT32 anVel, INT32 anTor);
	virtual void	UpdateCurrentParams		(double adPos, double adVel, double adTor);
	virtual INT32	GetCurrentRawPos		(	);
	virtual INT32	GetCurrentRawVel		(	);
	virtual INT32	GetCurrentRawTor		(	);
	virtual	double	GetCurrentPos			(	);
	virtual	double	GetCurrentPosD			(	);
	virtual	double	GetCurrentVel			(	);
	virtual	double	GetCurrentTor			(	);

	virtual double GetEncResolution() const { return m_dResolution; }
	virtual double GetGearRatio() const { return m_dGearRatio; }
	virtual double GetTransRatio() const { return m_dTransRatio; }

	virtual void 	SetCurrentRatio(double aCurRatio)	{ m_dCurrentRatio = aCurRatio; }
	virtual double	GetTorqueConstant()  				{ return m_dTorqueConstant; }
	virtual double	GetCurrentRatio()					{ return m_dCurrentRatio; }


	virtual INT32	GetStartRawPos			(	);
	virtual INT32	GetStartRawVel			(	);
	virtual INT32	GetStartRawTor			(	);

	virtual double	GetMaxPos				(	);
	virtual double	GetMinPos				(	);

	virtual ST_AXIS_LIMITS		GetAxisLimits		(	);
	virtual	ST_AXIS_PARAMS		GetCurrentParams	(	);
	virtual	ST_AXIS_RAW_PARAMS	GetCurrentRawParams	(	);
	virtual void				SetState			(eAxisState aeState);
	eAxisState					GetState			(	) { return m_pstAxisInfo->eState; }

	virtual void				PrintAxisInfo		(	) { return;	};
	
	BOOL	SetOneTurnRef					(double);
	BOOL	SetHomeReference				(double);
	double	GetHomeReference				(	) { return m_dHomeReference; }
	void	SetID							(UINT16 ausID) { m_usAxisID = ausID; }
	void	SetAliasPos						(UINT16, UINT16);
	void	SetResolution					(double, double adGearRatio = 1., double adEncRatio = 1.);
	void	SetAbsoluteEncoder				(BOOL abAbsEnc) { m_bAbsEncoder = abAbsEnc; }
	BOOL	IsLimitConfigured				(	);
	BOOL	IsMovable						(	);
	BOOL	IsServoOn						(	);
	BOOL	IsAllowablePosition				(double);
	BOOL	IsAllowableVelocity				(double);
	BOOL	IsAllowableAcceleration			(double);
	BOOL	IsAllowableDeceleration			(double);
	BOOL	IsAllowableJerk					(double);
	BOOL	IsAllowableTorque				(double);
	BOOL	IsAbsoluteEncoder				(	) { return m_bAbsEncoder; }
	BOOL	IsHomeReferenceSet				(	) { return m_bHomeRefSet; }

	eCommType		GetCommType				(	) { return m_pstAxisInfo->eComm; }
	void			SetCommType				(eCommType aeComm) { m_pstAxisInfo->eComm = aeComm; }
	eAxisType		GetAxisType				(	) { return m_pstAxisInfo->eJoint; }
	void			SetAxisType				(eAxisType aeJoint) { m_pstAxisInfo->eJoint = aeJoint; }
	eHomingMethod	GetHomingMethod			(	) { return m_pstAxisInfo->eHoming; }
	void			SetHomingMethod			(eHomingMethod aeHoming) { m_pstAxisInfo->eHoming = aeHoming; }
	BOOL			IsActuator				();
	void			SetName					(TSTRING astrName) { m_strName = astrName; }
	TSTRING			GetName					(	) { return m_strName; }

	/* EtherCAT Specific */
	virtual INT32	GetTargetRawPos			(	) { return 0; }
	virtual INT32	GetTargetRawVel			(	) { return 0; }
	virtual INT32	GetTargetRawTor			(	) { return 0; }
	virtual INT32	GetVelocity				(	) { return 0; }
	virtual INT32	GetAcceleration			(	) { return 0; }
	virtual INT32	GetDeceleration			(	) { return 0; }
	virtual INT32	GetMaxVel				(	) { return 0; }
	virtual INT32	GetMaxAcc				(	) { return 0; }
	virtual INT32	GetQuickStopDec			(	) { return 0; }
	virtual UINT16	GetStatusWord			(	) { return 0; }
	virtual UINT16	GetControlWord			(	) { return 0; }
	virtual UINT8	GetDriveMode			(	) { return 0; }
	virtual BOOL	ChangeDriveMode			(INT8 abtDriveMode);

	virtual void	SetVendorInfo			(UINT32 auVendorID, UINT32 auProductCode) { return; };
	virtual void	SetDCInfo				(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime) { return; };


	virtual void	SetHomingFlag			(BOOL abFlag) { ; }
	BOOL			GetHomingFlag			(	) { return m_bIsHoming; }

	double 			GetRatedTorque() 		{ return m_dRatedTorque; }
	double 			GetRatedCurrent() 		{ return m_dRatedCurrent; }	
	void 	 		SetRatedTorque(double adRatedTorque) 	{ m_dRatedTorque = adRatedTorque; } // mNm
	void 	 		SetRatedCurrent(double adRatedCurrent) 	{ m_dRatedCurrent = adRatedCurrent; } // mA

	/* 20250714 raim.delgado: select between torque constant, or rated torque/current for conversion */
	virtual double  ConvertCur2Tor(double adCurrent, BOOL abUseTConstant=TRUE);
	virtual double  ConvertTor2Cur(double adTorque,  BOOL abUseTConstant=TRUE);		

	virtual double 	GetTargetTorq()	{ return m_dTargetTorq; }
	virtual double	GetTargetVel()	{return m_dTargetVel; }
	virtual double 	GetTargetPos() { return m_dTargetPos; }

	virtual	INT32	ConvertRadMM2Res		(double);
	virtual	double	ConvertRes2RadMM		(INT32);
	virtual INT32	ConvertTor2Res			(double);
	virtual double	ConvertRes2Tor			(INT32);

	BOOL			IsMoving();

private:
	BOOL	SetLimits						(ST_LIMITS&, double, double);
	BOOL	CheckLimits						(ST_LIMITS&, double);
	
protected:
	double		m_dResolution;
	double		m_dGearRatio;
	double		m_dTransRatio;
	double		m_dEncoderRatio;
	double		m_dOneTurnRef;
	double		m_dTorqueConstant;
	INT32		m_nPosBeforeExit;
    double 		m_dCurrentRatio;
	INT			m_nDirection;

	UINT16		m_usAxisAlias;
	UINT16		m_usAxisPos;
	UINT16		m_usAxisID;

	INT32		m_nHomePosition;
	BOOL		m_bFirstHome;
	BOOL		m_bAbsEncoder;
	BOOL		m_bIsHoming;

	double 		m_dTargetTorq;
	double 		m_dTargetPos;
	double		m_dTargetVel;

	double 		m_dRatedTorque;
	double 		m_dRatedCurrent;

	ST_AXIS_INFO*	m_pstAxisInfo;
	ST_AXIS_LIMITS* m_pstAxisLimits;
	ST_AXIS_PARAMS* m_pstAxisParams;
	ST_AXIS_RAW_PARAMS* m_pstAxisRawParams;
	ST_AXIS_RAW_PARAMS* m_pstAxisStartRawParams;

private:
	double		m_dHomeReference;
	BOOL		m_bHomeSet;
	BOOL		m_bHomeRefSet;
	BOOL		m_bEnabled;
	BOOL		m_bIsAutoServoOn;
	TSTRING		m_strName;
}; //CAxis

#endif //__AXIS__

