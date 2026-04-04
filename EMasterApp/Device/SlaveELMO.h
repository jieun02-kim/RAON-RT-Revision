/*****************************************************************************
*	Name: SlaveEPOS4.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveEPOS4 child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#pragma once
#ifndef _ECAT_SLAVE_ELMO_
#define _ECAT_SLAVE_ELMO_

#include "SlaveCIA402Base.h"

#ifndef ELMO_VENDOR_ID
// #define EPOS4_VENDOR_ID 		0x000000fb
#define ELMO_VENDOR_ID 		    0x0000009a
#endif

// #define EPOS4_MODULE24_1_5      0x60500000
#define ELMO_MODULE_PLATINUM    0x00100002
// #define ELMO_MODULE_PLATINUM    0x00030924

#define ELMO_ACTIVATE_WORD    0x300
#define ELMO_SYNC0_SHIFT      0

class CSlaveELMO : public CSlaveCIA402Base
{
public:
    CSlaveELMO()
    {
        CSlaveCIA402Base::SetVendorInfo((UINT32)ELMO_VENDOR_ID, (UINT32)ELMO_MODULE_PLATINUM);
        CSlaveCIA402Base::SetDCInfo(TRUE, (UINT16)ELMO_ACTIVATE_WORD, (INT32)ELMO_SYNC0_SHIFT);
    };
    ~CSlaveELMO() {};

}; //SlaveEPOS4
#endif // _ECAT_SLAVE_EPOS4_