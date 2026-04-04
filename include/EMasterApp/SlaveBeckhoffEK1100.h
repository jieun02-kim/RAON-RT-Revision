/*****************************************************************************
*	Name: SlaveBeckhoffEK1100.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveBeckhoffEK1100 child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_SLAVE_BECKHOFF_EK1100_
#define _ECAT_SLAVE_BECKHOFF_EK1100_

#include "EcatSlaveBase.h"

#ifndef BECKHOFF_VENDOR_ID
#define BECKHOFF_VENDOR_ID         		0x00000002
#endif
#define BECKHOFF_EK1100 				0x044c2c52

class CSlaveBeckhoffEK1100 : public CEcatSlaveBase
{
	public:
		CSlaveBeckhoffEK1100()
		{
			CEcatSlaveBase::SetVendorID((UINT32)BECKHOFF_VENDOR_ID);
			CEcatSlaveBase::SetProductCode((UINT32)BECKHOFF_EK1100);
			CEcatSlaveBase::SetDeviceType(eTERMINAL);
			CEcatSlaveBase::SetDCSupported(FALSE);
		};
		~CSlaveBeckhoffEK1100(){};

}; //CSlaveBeckhoffEK1100
#endif // _ECAT_SLAVE_BECKHOFF_EK1100_