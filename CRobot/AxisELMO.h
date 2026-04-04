/*****************************************************************************
*	Name: AxisEPOS4.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CAxisEPOS4 child class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef __AXIS__ELMO__
#define __AXIS__ELMO__

#include "AxisCIA402.h"

// #define RatedCurrent 		14710.0 // mA
// #define RatedTorque			1323.0 //mNm
// #define Current2Torque(x)  (x*RatedTorque/RatedCurrent) //A to Nm
// #define Torqrue2Current(x)  (x*RatedCurrent/RatedTorque) // Nm to A

class CAxisELMO : public CAxisCIA402
{
public:
	CAxisELMO(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE, BOOL abConnected = TRUE);
	//CAxisEPOS4(eAxisType aeAxisType, BOOL abConnected = TRUE, BOOL abEnabled = TRUE);

public:
    virtual BOOL MoveTorque(double) override;
    virtual double GetAdditionalPos() override;  
    virtual double GetCurrentTor() override;

protected:
	virtual void OnSlaveStatusChanged(PVOID apSlave, PVOID apStatus, PVOID apReserved1, PVOID apReserved2) override;

}; // 


#endif // 