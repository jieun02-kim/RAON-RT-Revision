/*****************************************************************************
*	Name: AxisCIA402.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CAxisCIA402 class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __AXIS__CIA402__
#define __AXIS__CIA402__

#include "Axis.h"
#include "SlaveCIA402Base.h"
#include "EcatMasterBase.h"

class CAxisCIA402 : public CAxis
{
public:
	CAxisCIA402(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE, BOOL abConnected = TRUE);
	//CAxisCIA402(eAxisType aeAxisType, BOOL abConnected = TRUE, BOOL abEnabled = TRUE);
	virtual ~CAxisCIA402();

public:
	virtual	BOOL	Init				(CEcatMaster&, INT8 abtDriveMode = CIA402_PROFILE_POSITION); // we overload the Init method to accomodate registration of the EtherCAT Master
	virtual	BOOL	DeInit();
	virtual BOOL	DoHoming			(	);
	virtual BOOL	MoveAxis			(double, BOOL abForce=TRUE, BOOL abAbs=TRUE);
	virtual BOOL	MoveHome			(BOOL abForce = FALSE);
	virtual BOOL	StopAxis			(	);
	virtual BOOL	EmgStopAxis			(	);
	virtual BOOL	MoveVelocity		(double);
	virtual BOOL	MoveTorque			(double);
	virtual BOOL	ServoOn				(INT8 abtDriveMode = CIA402_PROFILE_POSITION);
	virtual BOOL	ChangeDriveMode		(INT8 abtDriveMode);
	virtual BOOL	ServoOff			(	);
	virtual BOOL	IsServoOn			(	);
	virtual BOOL	IsConnected			(	) { return m_bConnected; }
	
	virtual BOOL	SetVelocity			(double);
	virtual BOOL	SetAcceleration		(double);
	virtual BOOL	SetDeceleration		(double);

	/* todo:	1. add acquisition of Target Velocity and Torque
	*/
	virtual INT32	GetTargetRawPos		(	);
	virtual INT32	GetTargetRawVel		(	);
	virtual INT32	GetTargetRawTor		(	);
	virtual INT32	GetVelocity			(	);
	virtual INT32	GetAcceleration		(	);
	virtual INT32	GetDeceleration		(	);
	virtual INT32	GetMaxVel			(	);
	virtual INT32	GetMaxAcc			(	);
	virtual INT32	GetQuickStopDec		(	);
	virtual UINT8	GetDriveMode		(	);

	virtual double GetAdditionalPos();

	virtual UINT16	GetStatusWord		(	);
	virtual UINT16	GetControlWord		(	);
	
	
	virtual void	SetVendorInfo		(UINT32 auVendorID, UINT32 auProductCode);
	virtual void	SetDCInfo			(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime);
	virtual void	SetHomingFlag		(BOOL abFlag = TRUE);

	void 			SetTargetTorq		(int16_t target) { m_cEcSlave.SetTargetTor(target); }
	void			SetTargetPos		(int32_t target) { m_cEcSlave.SetTargetPos(target); }
	

protected:
	virtual void	OnSlaveStatusChanged	(PVOID, PVOID, PVOID, PVOID);
	virtual void	OnSlaveUpdateRawParam	(PVOID, PVOID, PVOID, PVOID);
	virtual void	SetState				(eAxisState aeState);

protected:
	CSlaveCIA402	m_cEcSlave;
	CEcatMaster*	m_pEcMaster;
	BOOL			m_bConnected;
	INT32			m_nPosBeforeDisconnect;

}; // CAxisCIA402


#endif // __AXIS__CIA402__