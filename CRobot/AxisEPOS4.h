/*****************************************************************************
*	Name: AxisEPOS4.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CAxisEPOS4 child class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef __AXIS__EPOS4__
#define __AXIS__EPOS4__

#include "AxisCIA402.h"

class CAxisEPOS4 : public CAxisCIA402
{
public:
	CAxisEPOS4(eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder = TRUE, 
		BOOL abCCW = TRUE, BOOL abEnabled = TRUE, BOOL abConnected = TRUE) 
		: CAxisCIA402(aeAxisType, adEncRes, adGearRatio, adTransRatio, abAbsoluteEncoder, abCCW, abEnabled, abConnected)
		{
			
		}

}; // CAxisEPOS4


#endif // __AXIS__EPOS4__