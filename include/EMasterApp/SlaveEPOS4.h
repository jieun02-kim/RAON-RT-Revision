/*****************************************************************************
*	Name: SlaveEPOS4.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveEPOS4 child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#pragma once
#ifndef _ECAT_SLAVE_MECAPIONL7N_
#define _ECAT_SLAVE_MECAPIONL7N_

#include "SlaveCIA402Base.h"

/* this is for ELMO */
#ifndef EPOS4_VENDOR_ID
// #define EPOS4_VENDOR_ID 		0x000000fb
#define EPOS4_VENDOR_ID 		0x0000009a
#endif

// #define EPOS4_MODULE24_1_5      0x60500000
#define EPOS4_MODULE24_1_5      0x00100002
// #define EPOS4_MODULE24_1_5        0x00030924       //1층

#define EPOS4_ACTIVATE_WORD    0x300
#define EPOS4_SYNC0_SHIFT      0

class CSlaveEPOS4 : public CSlaveCIA402Base
{
public:
    CSlaveEPOS4()
    {
        CSlaveCIA402Base::SetVendorInfo((UINT32)EPOS4_VENDOR_ID, (UINT32)EPOS4_MODULE24_1_5);
        CSlaveCIA402Base::SetDCInfo(TRUE, (UINT16)EPOS4_ACTIVATE_WORD, (INT32)EPOS4_SYNC0_SHIFT);
    };
    ~CSlaveEPOS4() {};

}; //SlaveEPOS4
#endif // _ECAT_SLAVE_EPOS4_