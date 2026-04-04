/*****************************************************************************
*	Name: SlaveBeckhoffEK1100.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveBeckhoffEK1100 child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_SLAVE_BECKHOFF_CU1124_
#define _ECAT_SLAVE_BECKHOFF_CU1124_

#include "EcatSlaveBase.h"

#ifndef BECKHOFF_VENDOR_ID
#define BECKHOFF_VENDOR_ID         		0x00000002
#endif
#define BECKHOFF_CU1124 				0x04645432
// #define BECKHOFF_CU1124 				0x04685432  // 1층

class CSlaveBeckhoffCU1124 : public CEcatSlaveBase
{
	public:
		CSlaveBeckhoffCU1124()
		{
			CEcatSlaveBase::SetVendorID((UINT32)BECKHOFF_VENDOR_ID);
			CEcatSlaveBase::SetProductCode((UINT32)BECKHOFF_CU1124);
			CEcatSlaveBase::SetDeviceType(eTERMINAL);
			CEcatSlaveBase::SetDCSupported(FALSE);
		};
		~CSlaveBeckhoffCU1124(){};

}; //CSlaveBeckhoffCU1124
#endif // _ECAT_SLAVE_BECKHOFF_CU1124_