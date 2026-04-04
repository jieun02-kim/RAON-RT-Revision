/*****************************************************************************
*	Name: SlaveBeckhoffEL2024.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveBeckhoffEL2024 child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_SLAVE_BECKHOFF_EL2024_
#define _ECAT_SLAVE_BECKHOFF_EL2024_

#include "EcatSlaveBase.h"

#ifndef BECKHOFF_VENDOR_ID
#define BECKHOFF_VENDOR_ID         		0x00000002
#endif
#define BECKHOFF_EL2024 				0x07e83052

ec_pdo_entry_info_t beckhoff_el2024_pdo_entries[] = {
    {0x7000, 0x01, 1}, 
    {0x7010, 0x01, 1}, 
    {0x7020, 0x01, 1}, 
    {0x7030, 0x01, 1}, 
};

ec_pdo_info_t beckhoff_el2024_pdos[] = {
	{0x1600, 1, beckhoff_el2024_pdo_entries + 0}, /* Channel 1 */
	{0x1601, 1, beckhoff_el2024_pdo_entries + 1}, /* Channel 2 */
	{0x1602, 1, beckhoff_el2024_pdo_entries + 2}, /* Channel 3 */
	{0x1603, 1, beckhoff_el2024_pdo_entries + 3}, /* Channel 4 */
};

ec_sync_info_t beckhoff_el2024_syncs[] = {
    {0, EC_DIR_OUTPUT, 4, beckhoff_el2024_pdos + 0, EC_WD_ENABLE},
    {0xff}
};

class CSlaveBeckhoffEL2024 : public CEcatSlaveBase
{
public:
	CSlaveBeckhoffEL2024()
	{
		CEcatSlaveBase::SetVendorID((UINT32)BECKHOFF_VENDOR_ID);
		CEcatSlaveBase::SetProductCode((UINT32)BECKHOFF_EL2024);
		CEcatSlaveBase::SetDeviceType(eDIGITAL_OUT);
		CEcatSlaveBase::SetDCSupported(FALSE);
		CEcatSlaveBase::SetEcatPdoSync(beckhoff_el2024_syncs);

		m_stSlaveParams.stOutPDOs.unLedValue = 0;
	};
	~CSlaveBeckhoffEL2024(){};

private:
	BOOL RegisterPDO(   )  
	{
		BOOL bRet = TRUE;
		/* Output PDOs */
		if(0 > (m_stSlaveParams.GET_OFFSET(LedValue) = CEcatSlaveBase::RegisterPDOEntry(0x7000,0x01,&m_stSlaveParams.GET_BITPOS(LedValue), eOutput)))
			bRet = FALSE;

		return bRet;
	}
	struct BECKHOFF_EL2024_PDOS_OUT //Input 
	{
		UINT32 unLedValue;
	};
	struct BECKHOFF_EL2024_SLAVE_PARAMS
	{
		BECKHOFF_EL2024_PDOS_OUT    stOutPDOs;
		SET_OFFSET(LedValue); //e.g., UINT32 offILed;
		SET_BITPOS(LedValue); //e.g., UINT32 bitILed;
	};
	BECKHOFF_EL2024_SLAVE_PARAMS m_stSlaveParams;

public:
	inline void SetLedValue(UINT8 abtLedValue)
	{
		m_stSlaveParams.stOutPDOs.unLedValue = abtLedValue;
	}
	inline void WriteToSlave()
	{
		CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(LedValue), m_stSlaveParams.stOutPDOs.unLedValue, 8);
	}

}; //CSlaveBeckhoffEL2024
#endif // _ECAT_SLAVE_BECKHOFF_EL2024_