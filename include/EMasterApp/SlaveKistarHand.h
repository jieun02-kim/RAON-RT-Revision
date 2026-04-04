/*****************************************************************************
*	Name: SlaveKistarHand.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveKistFT child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef _ECAT_SLAVE_KIST_HAND_

#include "EcatSlaveBase.h"

#ifndef KISTAR_HAND_VENDOR_ID
#define KISTAR_HAND_VENDOR_ID         		0x00000000
#endif
#define KISTAR_HAND_PRODUCT_CODE 				0x00000004
#define KIST_FT_ACTIVATE_WORD       0x0300
#define KIST_FT_SYNC0_SHIFT         125000

/* Master 0, Slave 2, "IFBOX4NEWIF"
 * Vendor ID:       0x00828845
 * Product code:    0x00009252
 * Revision number: 0x00000001
 */



class CSlaveKistarHand : public CEcatSlaveBase
{
public:
    CSlaveKistarHand();
	virtual ~CSlaveKistarHand();


public:

	void SetVendorInfo(UINT32 aunVendorID, UINT32 aunProductCode);
	void SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime);

    virtual void WriteToSlave();
    virtual void ReadFromSlave();
    

	void SetServoStatus(UINT16 usFlag) { m_stSlaveParams.stOutPDOs.usStatusOut1 = usFlag; }
	void SetControlMode(UINT16 usMode) { m_stSlaveParams.stOutPDOs.usStatusOut2 = usMode; }
	void SetTargetPos(int nAxis, INT16 dTarget) { m_stSlaveParams.stOutPDOs.nTargetPos[nAxis] = dTarget; }

    UINT16 GetServoStatus() { return m_stSlaveParams.stInPDOs.usStatusIn1; }
    UINT16 GetControlMode() { return m_stSlaveParams.stInPDOs.usStatusIn2; }
    INT16 GetPosition(int nAxis) { return m_stSlaveParams.stInPDOs.nPosition[nAxis]; }
    INT16 GetCurrent(int nAxis) { return m_stSlaveParams.stInPDOs.nCurrent[nAxis]; }
    INT16 GetKinesthetic(int nNo) { return m_stSlaveParams.stInPDOs.nKinesthetic[nNo]; }
    INT16 GetTactile(int nNo) { return m_stSlaveParams.stInPDOs.nTactile[nNo]; }
    INT16 GetAddInfoIn(int nNo) { return m_stSlaveParams.stInPDOs.nAddInfoIn[nNo]; }
    UINT16 GetStatusOut1() { return m_stSlaveParams.stOutPDOs.usStatusOut1; }
    UINT16 GetStatusOut2() { return m_stSlaveParams.stOutPDOs.usStatusOut2; }
    INT16 GetTargetPosition(int nAxis) { return m_stSlaveParams.stOutPDOs.nTargetPos[nAxis]; }
    UINT16 GetAddInfoOut(int nNo) { return m_stSlaveParams.stOutPDOs.usAddInfoOut[nNo]; }
    

private:


    typedef union 
    {
        uint8_t Byte[128];

        struct __attribute__((packed))
        {
            UINT16 usStatusIn1;
            UINT16 usStatusIn2;
            INT16 nPosition[16];
            INT16 nCurrent[16];
            INT16 nKinesthetic[12];
            INT16 nTactile[4];
            INT16 nAddInfoIn[14];        
        };

    } KISTAR_HAND_PDO_IN;
    
    typedef union 
    {
        uint8_t Byte[64];


        struct __attribute__((packed))
        {
            UINT16 usStatusOut1;
            UINT16 usStatusOut2;
            INT16 nTargetPos[16];
            UINT16 usAddInfoOut[14]; 
        };

    } KISTAR_HAND_PDO_OUT;

	#pragma pack(push, 1)
	typedef struct {
		UINT16 usWRMode;
		INT16 nPosGainP;
		INT16 nPosGainD;
		INT16 nTactileThresholdThumb;
		INT16 nTactileThresholdIndex;
		INT16 nTactileThresholdMiddle;
		INT16 nTactileThresholdRing;
		UINT8 reserved[12];
		UINT16 nInitOrgin;
	} ST_AddInfo;
	#pragma pack(pop)

    

    struct KISTAR_HAND_SLAVE_PARAMS
    {
        KISTAR_HAND_PDO_OUT stOutPDOs;
        KISTAR_HAND_PDO_IN  stInPDOs;

        SET_OFFSET(StatusOut1);
		SET_OFFSET(StatusOut2);

		SET_OFFSET(TargetPos1);
		SET_OFFSET(TargetPos2);
		SET_OFFSET(TargetPos3);
		SET_OFFSET(TargetPos4);

		SET_OFFSET(TargetPos5);
		SET_OFFSET(TargetPos6);
		SET_OFFSET(TargetPos7);
		SET_OFFSET(TargetPos8);

		SET_OFFSET(TargetPos9);
		SET_OFFSET(TargetPos10);
		SET_OFFSET(TargetPos11);
		SET_OFFSET(TargetPos12);

		SET_OFFSET(TargetPos13);
		SET_OFFSET(TargetPos14);
		SET_OFFSET(TargetPos15);
		SET_OFFSET(TargetPos16);

		SET_OFFSET(AddInfoOut1);
		SET_OFFSET(AddInfoOut2);
		SET_OFFSET(AddInfoOut3);
		SET_OFFSET(AddInfoOut4);
		SET_OFFSET(AddInfoOut5);
		SET_OFFSET(AddInfoOut6);
		SET_OFFSET(AddInfoOut7);
		SET_OFFSET(AddInfoOut8);
		SET_OFFSET(AddInfoOut9);
		SET_OFFSET(AddInfoOut10);
		SET_OFFSET(AddInfoOut11);
		SET_OFFSET(AddInfoOut12);
		SET_OFFSET(AddInfoOut13);
		SET_OFFSET(AddInfoOut14);	
		

		SET_OFFSET(StatusIn1);
		SET_OFFSET(StatusIn2);

		SET_OFFSET(Position1);
		SET_OFFSET(Position2);
		SET_OFFSET(Position3);
		SET_OFFSET(Position4);
		SET_OFFSET(Position5);
		SET_OFFSET(Position6);
		SET_OFFSET(Position7);
		SET_OFFSET(Position8);
		SET_OFFSET(Position9);
		SET_OFFSET(Position10);
		SET_OFFSET(Position11);
		SET_OFFSET(Position12);
		SET_OFFSET(Position13);
		SET_OFFSET(Position14);
		SET_OFFSET(Position15);
		SET_OFFSET(Position16);

		SET_OFFSET(Current1);
		SET_OFFSET(Current2);
		SET_OFFSET(Current3);
		SET_OFFSET(Current4);
		SET_OFFSET(Current5);
		SET_OFFSET(Current6);
		SET_OFFSET(Current7);
		SET_OFFSET(Current8);
		SET_OFFSET(Current9);
		SET_OFFSET(Current10);
		SET_OFFSET(Current11);
		SET_OFFSET(Current12);
		SET_OFFSET(Current13);
		SET_OFFSET(Current14);
		SET_OFFSET(Current15);
		SET_OFFSET(Current16);

		SET_OFFSET(Kinesthetic1);
		SET_OFFSET(Kinesthetic2);
		SET_OFFSET(Kinesthetic3);
		SET_OFFSET(Kinesthetic4);
		SET_OFFSET(Kinesthetic5);
		SET_OFFSET(Kinesthetic6);
		SET_OFFSET(Kinesthetic7);
		SET_OFFSET(Kinesthetic8);
		SET_OFFSET(Kinesthetic9);
		SET_OFFSET(Kinesthetic10);
		SET_OFFSET(Kinesthetic11);
		SET_OFFSET(Kinesthetic12);

		SET_OFFSET(Tactile1);
		SET_OFFSET(Tactile2);
		SET_OFFSET(Tactile3);
		SET_OFFSET(Tactile4);

		SET_OFFSET(AddInfoIn1);
		SET_OFFSET(AddInfoIn2);
		SET_OFFSET(AddInfoIn3);
		SET_OFFSET(AddInfoIn4);
		SET_OFFSET(AddInfoIn5);
		SET_OFFSET(AddInfoIn6);
		SET_OFFSET(AddInfoIn7);
		SET_OFFSET(AddInfoIn8);
		SET_OFFSET(AddInfoIn9);
		SET_OFFSET(AddInfoIn10);
		SET_OFFSET(AddInfoIn11);
		SET_OFFSET(AddInfoIn12);
		SET_OFFSET(AddInfoIn13);
		SET_OFFSET(AddInfoIn14);

		SET_BITPOS(StatusOut1);
		SET_BITPOS(StatusOut2);

		SET_BITPOS(TargetPos1);
		SET_BITPOS(TargetPos2);
		SET_BITPOS(TargetPos3);
		SET_BITPOS(TargetPos4);
		SET_BITPOS(TargetPos5);
		SET_BITPOS(TargetPos6);
		SET_BITPOS(TargetPos7);
		SET_BITPOS(TargetPos8);
		SET_BITPOS(TargetPos9);
		SET_BITPOS(TargetPos10);
		SET_BITPOS(TargetPos11);
		SET_BITPOS(TargetPos12);
		SET_BITPOS(TargetPos13);
		SET_BITPOS(TargetPos14);
		SET_BITPOS(TargetPos15);
		SET_BITPOS(TargetPos16);

		SET_BITPOS(AddInfoOut1);
		SET_BITPOS(AddInfoOut2);
		SET_BITPOS(AddInfoOut3);
		SET_BITPOS(AddInfoOut4);
		SET_BITPOS(AddInfoOut5);
		SET_BITPOS(AddInfoOut6);
		SET_BITPOS(AddInfoOut7);
		SET_BITPOS(AddInfoOut8);
		SET_BITPOS(AddInfoOut9);
		SET_BITPOS(AddInfoOut10);
		SET_BITPOS(AddInfoOut11);
		SET_BITPOS(AddInfoOut12);
		SET_BITPOS(AddInfoOut13);
		SET_BITPOS(AddInfoOut14);

		SET_BITPOS(StatusIn1);
		SET_BITPOS(StatusIn2);

		SET_BITPOS(Position1);
		SET_BITPOS(Position2);
		SET_BITPOS(Position3);
		SET_BITPOS(Position4);
		SET_BITPOS(Position5);
		SET_BITPOS(Position6);
		SET_BITPOS(Position7);
		SET_BITPOS(Position8);
		SET_BITPOS(Position9);
		SET_BITPOS(Position10);
		SET_BITPOS(Position11);
		SET_BITPOS(Position12);
		SET_BITPOS(Position13);
		SET_BITPOS(Position14);
		SET_BITPOS(Position15);
		SET_BITPOS(Position16);

		SET_BITPOS(Current1);
		SET_BITPOS(Current2);
		SET_BITPOS(Current3);
		SET_BITPOS(Current4);
		SET_BITPOS(Current5);
		SET_BITPOS(Current6);
		SET_BITPOS(Current7);
		SET_BITPOS(Current8);
		SET_BITPOS(Current9);
		SET_BITPOS(Current10);
		SET_BITPOS(Current11);
		SET_BITPOS(Current12);
		SET_BITPOS(Current13);
		SET_BITPOS(Current14);
		SET_BITPOS(Current15);
		SET_BITPOS(Current16);

		SET_BITPOS(Kinesthetic1);
		SET_BITPOS(Kinesthetic2);
		SET_BITPOS(Kinesthetic3);
		SET_BITPOS(Kinesthetic4);
		SET_BITPOS(Kinesthetic5);
		SET_BITPOS(Kinesthetic6);
		SET_BITPOS(Kinesthetic7);
		SET_BITPOS(Kinesthetic8);
		SET_BITPOS(Kinesthetic9);
		SET_BITPOS(Kinesthetic10);
		SET_BITPOS(Kinesthetic11);
		SET_BITPOS(Kinesthetic12);

		SET_BITPOS(Tactile1);
		SET_BITPOS(Tactile2);
		SET_BITPOS(Tactile3);
		SET_BITPOS(Tactile4);

		SET_BITPOS(AddInfoIn1);
		SET_BITPOS(AddInfoIn2);
		SET_BITPOS(AddInfoIn3);
		SET_BITPOS(AddInfoIn4);
		SET_BITPOS(AddInfoIn5);
		SET_BITPOS(AddInfoIn6);
		SET_BITPOS(AddInfoIn7);
		SET_BITPOS(AddInfoIn8);
		SET_BITPOS(AddInfoIn9);
		SET_BITPOS(AddInfoIn10);
		SET_BITPOS(AddInfoIn11);
		SET_BITPOS(AddInfoIn12);
		SET_BITPOS(AddInfoIn13);
		SET_BITPOS(AddInfoIn14);
	
        
    };


    

protected:
    
    ec_pdo_entry_info_t slave_kistar_hand_pdo_entries[100] = {
        {0x0007, 0x01, 16}, /* STATUS1 */
		{0x0007, 0x02, 16}, /* STATUS2 */

		{0x0007, 0x03, 16}, /* TARGET1 */
		{0x0007, 0x04, 16}, /* TARGET2 */
		{0x0007, 0x05, 16}, /* TARGET3 */
		{0x0007, 0x06, 16}, /* TARGET4 */
		{0x0007, 0x07, 16}, /* TARGET5 */
		{0x0007, 0x08, 16}, /* TARGET6 */
		{0x0007, 0x09, 16}, /* TARGET7 */
		{0x0007, 0x0a, 16}, /* TARGET8 */
		{0x0007, 0x0b, 16}, /* TARGET9 */
		{0x0007, 0x0c, 16}, /* TARGET10 */
		{0x0007, 0x0d, 16}, /* TARGET11 */
		{0x0007, 0x0e, 16}, /* TARGET12 */
		{0x0007, 0x0f, 16}, /* TARGET13 */
		{0x0007, 0x10, 16}, /* TARGET14 */
		{0x0007, 0x11, 16}, /* TARGET15 */
		{0x0007, 0x12, 16}, /* TARGET16 */
		{0x0007, 0x13, 16}, /* ADD1 */
		{0x0007, 0x14, 16}, /* ADD2 */
		{0x0007, 0x15, 16}, /* ADD3 */
		{0x0007, 0x16, 16}, /* ADD4 */
		{0x0007, 0x17, 16}, /* ADD5 */
		{0x0007, 0x18, 16}, /* ADD6 */
		{0x0007, 0x19, 16}, /* ADD7 */
		{0x0007, 0x1a, 16}, /* ADD8 */
		{0x0007, 0x1b, 16}, /* ADD9 */
		{0x0007, 0x1c, 16}, /* ADD10 */
		{0x0007, 0x1d, 16}, /* ADD11 */
		{0x0007, 0x1e, 16}, /* ADD12 */
		{0x0007, 0x1f, 16}, /* ADD13 */
		{0x0007, 0x20, 16}, /* ADD14 */


		// Input Data
		{0x0006, 0x01, 16}, /* STATUS1 */
		{0x0006, 0x02, 16}, /* STATUS2 */
		{0x0006, 0x03, 16}, /* Position1 */
		{0x0006, 0x04, 16}, /* Position2 */
		{0x0006, 0x05, 16}, /* Position3 */
		{0x0006, 0x06, 16}, /* Position4 */
		{0x0006, 0x07, 16}, /* Position5 */
		{0x0006, 0x08, 16}, /* Position6 */
		{0x0006, 0x09, 16}, /* Position7 */
		{0x0006, 0x0a, 16}, /* Position8 */
		{0x0006, 0x0b, 16}, /* Position9 */
		{0x0006, 0x0c, 16}, /* Position10 */
		{0x0006, 0x0d, 16}, /* Position11 */
		{0x0006, 0x0e, 16}, /* Position12 */
		{0x0006, 0x0f, 16}, /* Position13 */
		{0x0006, 0x10, 16}, /* Position14 */
		{0x0006, 0x11, 16}, /* Position15 */
		{0x0006, 0x12, 16}, /* Position16 */
		{0x0006, 0x13, 16}, /* Current1 */
		{0x0006, 0x14, 16}, /* Current2 */
		{0x0006, 0x15, 16}, /* Current3 */
		{0x0006, 0x16, 16}, /* Current4 */
		{0x0006, 0x17, 16}, /* Current5 */
		{0x0006, 0x18, 16}, /* Current6 */
		{0x0006, 0x19, 16}, /* Current7 */
		{0x0006, 0x1a, 16}, /* Current8 */
		{0x0006, 0x1b, 16}, /* Current9 */
		{0x0006, 0x1c, 16}, /* Current10 */
		{0x0006, 0x1d, 16}, /* Current11 */
		{0x0006, 0x1e, 16}, /* Current12 */
		{0x0006, 0x1f, 16}, /* Current13 */
		{0x0006, 0x20, 16}, /* Current14 */
		{0x0006, 0x21, 16}, /* Current15 */
		{0x0006, 0x22, 16}, /* Current16 */
		{0x0006, 0x23, 16}, /* Kinesthetic1 */
		{0x0006, 0x24, 16}, /* Kinesthetic2 */
		{0x0006, 0x25, 16}, /* Kinesthetic3 */
		{0x0006, 0x26, 16}, /* Kinesthetic4 */
		{0x0006, 0x27, 16}, /* Kinesthetic5 */
		{0x0006, 0x28, 16}, /* Kinesthetic6 */
		{0x0006, 0x29, 16}, /* Kinesthetic7 */
		{0x0006, 0x2a, 16}, /* Kinesthetic8 */
		{0x0006, 0x2b, 16}, /* Kinesthetic9 */
		{0x0006, 0x2c, 16}, /* Kinesthetic10 */
		{0x0006, 0x2d, 16}, /* Kinesthetic11 */
		{0x0006, 0x2e, 16}, /* Kinesthetic12 */
		{0x0006, 0x2f, 16}, /* Tactile1 */
		{0x0006, 0x30, 16}, /* Tactile2 */
		{0x0006, 0x31, 16}, /* Tactile3 */
		{0x0006, 0x32, 16}, /* Tactile4 */
		{0x0006, 0x33, 16}, /* ADD1 */
		{0x0006, 0x34, 16}, /* ADD2 */
		{0x0006, 0x35, 16}, /* ADD3 */
		{0x0006, 0x36, 16}, /* ADD4 */
		{0x0006, 0x37, 16}, /* ADD5 */
		{0x0006, 0x38, 16}, /* ADD6 */
		{0x0006, 0x39, 16}, /* ADD7 */
		{0x0006, 0x3a, 16}, /* ADD8 */
		{0x0006, 0x3b, 16}, /* ADD9 */
		{0x0006, 0x3c, 16}, /* ADD10 */
		{0x0006, 0x3d, 16}, /* ADD11 */
		{0x0006, 0x3e, 16}, /* ADD12 */
		{0x0006, 0x3f, 16}, /* ADD13 */
		{0x0006, 0x40, 16} /* ADD14 */
    
    };
    
    ec_pdo_info_t slave_kistar_hand_pdos[2] = {
        {0x1600, 32, slave_kistar_hand_pdo_entries + 0}, /* Outputs */
	    {0x1a00, 64, slave_kistar_hand_pdo_entries + 32} /* Inputs */
    };

    ec_sync_info_t kistar_hand_syncs[5];
    
        

    virtual BOOL RegisterPDO();
    virtual	void InitSyncs();


private:
    KISTAR_HAND_SLAVE_PARAMS m_stSlaveParams;



}; //CSlaveKistFT
#endif // _ECAT_SLAVE_KIST_FT_
