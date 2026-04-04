/*****************************************************************************
*	Name: AxisNRMKCore.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CAxisNRMKCore child class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __AXIS__NRMK_CORE__
#define __AXIS__NRMK_CORE__

#include "AxisCIA402.h"
#include "SlaveCIA402Base.h"
#include "EcatMasterBase.h"

class CAxisNRMKCore : public CAxisCIA402
{
public:
	CAxisNRMKCore(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE, BOOL abConnected = TRUE);

public:
	virtual BOOL	MoveTorque				(double);
	virtual INT32	ConvertTor2Res			(double);
	virtual double	ConvertRes2Tor			(INT32);

protected:
	virtual void	OnSlaveStatusChanged	(PVOID, PVOID, PVOID, PVOID);
	
}; // CAxisNRMKCore


#endif // __AXIS__NRMK_CORE__