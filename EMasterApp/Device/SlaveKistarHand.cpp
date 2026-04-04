#include "SlaveKistarHand.h"

CSlaveKistarHand::CSlaveKistarHand()
{
    CEcatSlaveBase::SetVendorID((UINT32)KISTAR_HAND_VENDOR_ID);
    CEcatSlaveBase::SetProductCode((UINT32)KISTAR_HAND_PRODUCT_CODE);
    CEcatSlaveBase::SetDeviceType(eKistarHand);
    CEcatSlaveBase::SetDCSupported(FALSE);
}

CSlaveKistarHand::~CSlaveKistarHand()
{
    
}

void CSlaveKistarHand::SetVendorInfo(UINT32 aunVendorID, UINT32 aunProductCode)
{
    CEcatSlaveBase::SetVendorID(aunVendorID);
    CEcatSlaveBase::SetProductCode(aunProductCode);
}

void CSlaveKistarHand::SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime)
{
    CEcatSlaveBase::SetDCSupported(abDCSupported);
    CEcatSlaveBase::SetDCActivateWord(ausActivateWord);
    CEcatSlaveBase::SetDCShiftTime(anShiftTime);

    
}

void CSlaveKistarHand::InitSyncs()
{
    kistar_hand_syncs[0] = { 0, EC_DIR_OUTPUT, 1, slave_kistar_hand_pdos + 0, EC_WD_ENABLE };	
	kistar_hand_syncs[1] = { 1, EC_DIR_INPUT, 1, slave_kistar_hand_pdos + 1, EC_WD_ENABLE };
	kistar_hand_syncs[2] = { 0xff };
    CEcatSlaveBase::SetEcatPdoSync(kistar_hand_syncs);
}

BOOL CSlaveKistarHand::RegisterPDO()
{

    /* Output PDOS */
    if (0 > (m_stSlaveParams.GET_OFFSET(StatusOut1) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x01, &m_stSlaveParams.GET_BITPOS(StatusOut1), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(StatusOut2) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x02, &m_stSlaveParams.GET_BITPOS(StatusOut2), eOutput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos1) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x03, &m_stSlaveParams.GET_BITPOS(TargetPos1), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos2) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x04, &m_stSlaveParams.GET_BITPOS(TargetPos2), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos3) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x05, &m_stSlaveParams.GET_BITPOS(TargetPos3), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos4) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x06, &m_stSlaveParams.GET_BITPOS(TargetPos4), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos5) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x07, &m_stSlaveParams.GET_BITPOS(TargetPos5), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos6) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x08, &m_stSlaveParams.GET_BITPOS(TargetPos6), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos7) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x09, &m_stSlaveParams.GET_BITPOS(TargetPos7), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos8) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0A, &m_stSlaveParams.GET_BITPOS(TargetPos8), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos9) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0B, &m_stSlaveParams.GET_BITPOS(TargetPos9), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos10) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0C, &m_stSlaveParams.GET_BITPOS(TargetPos10), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos11) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0D, &m_stSlaveParams.GET_BITPOS(TargetPos11), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos12) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0E, &m_stSlaveParams.GET_BITPOS(TargetPos12), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos13) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x0F, &m_stSlaveParams.GET_BITPOS(TargetPos13), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos14) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x10, &m_stSlaveParams.GET_BITPOS(TargetPos14), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos15) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x11, &m_stSlaveParams.GET_BITPOS(TargetPos15), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(TargetPos16) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x12, &m_stSlaveParams.GET_BITPOS(TargetPos16), eOutput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut1) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x13, &m_stSlaveParams.GET_BITPOS(AddInfoOut1), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut2) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x14, &m_stSlaveParams.GET_BITPOS(AddInfoOut2), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut3) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x15, &m_stSlaveParams.GET_BITPOS(AddInfoOut3), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut4) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x16, &m_stSlaveParams.GET_BITPOS(AddInfoOut4), eOutput)))  
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut5) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x17, &m_stSlaveParams.GET_BITPOS(AddInfoOut5), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut6) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x18, &m_stSlaveParams.GET_BITPOS(AddInfoOut6), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut7) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x19, &m_stSlaveParams.GET_BITPOS(AddInfoOut7), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut8) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1A, &m_stSlaveParams.GET_BITPOS(AddInfoOut8), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut9) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1B, &m_stSlaveParams.GET_BITPOS(AddInfoOut9), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut10) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1C, &m_stSlaveParams.GET_BITPOS(AddInfoOut10), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut11) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1D, &m_stSlaveParams.GET_BITPOS(AddInfoOut11), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut12) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1E, &m_stSlaveParams.GET_BITPOS(AddInfoOut12), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut13) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x1F, &m_stSlaveParams.GET_BITPOS(AddInfoOut13), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoOut14) = CEcatSlaveBase::RegisterPDOEntry(0x0007, 0x20, &m_stSlaveParams.GET_BITPOS(AddInfoOut14), eOutput)))
        return FALSE;

    /* Input PDOS */
    if (0 > (m_stSlaveParams.GET_OFFSET(StatusIn1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x01, &m_stSlaveParams.GET_BITPOS(StatusIn1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(StatusIn2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x02, &m_stSlaveParams.GET_BITPOS(StatusIn2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x03, &m_stSlaveParams.GET_BITPOS(Position1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x04, &m_stSlaveParams.GET_BITPOS(Position2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position3) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x05, &m_stSlaveParams.GET_BITPOS(Position3), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position4) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x06, &m_stSlaveParams.GET_BITPOS(Position4), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position5) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x07, &m_stSlaveParams.GET_BITPOS(Position5), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position6) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x08, &m_stSlaveParams.GET_BITPOS(Position6), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position7) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x09, &m_stSlaveParams.GET_BITPOS(Position7), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position8) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0A, &m_stSlaveParams.GET_BITPOS(Position8), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position9) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0B, &m_stSlaveParams.GET_BITPOS(Position9), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position10) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0C, &m_stSlaveParams.GET_BITPOS(Position10), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position11) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0D, &m_stSlaveParams.GET_BITPOS(Position11), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position12) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0E, &m_stSlaveParams.GET_BITPOS(Position12), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position13) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0F, &m_stSlaveParams.GET_BITPOS(Position13), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position14) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x10, &m_stSlaveParams.GET_BITPOS(Position14), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position15) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x11, &m_stSlaveParams.GET_BITPOS(Position15), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Position16) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x12, &m_stSlaveParams.GET_BITPOS(Position16), eInput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(Current1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x13, &m_stSlaveParams.GET_BITPOS(Current1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x14, &m_stSlaveParams.GET_BITPOS(Current2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current3) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x15, &m_stSlaveParams.GET_BITPOS(Current3), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current4) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x16, &m_stSlaveParams.GET_BITPOS(Current4), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current5) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x17, &m_stSlaveParams.GET_BITPOS(Current5), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current6) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x18, &m_stSlaveParams.GET_BITPOS(Current6), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current7) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x19, &m_stSlaveParams.GET_BITPOS(Current7), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current8) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1A, &m_stSlaveParams.GET_BITPOS(Current8), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current9) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1B, &m_stSlaveParams.GET_BITPOS(Current9), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current10) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1C, &m_stSlaveParams.GET_BITPOS(Current10), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current11) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1D, &m_stSlaveParams.GET_BITPOS(Current11), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current12) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1E, &m_stSlaveParams.GET_BITPOS(Current12), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current13) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x1F, &m_stSlaveParams.GET_BITPOS(Current13), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current14) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x20, &m_stSlaveParams.GET_BITPOS(Current14), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current15) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x21, &m_stSlaveParams.GET_BITPOS(Current15), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Current16) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x22, &m_stSlaveParams.GET_BITPOS(Current16), eInput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x23, &m_stSlaveParams.GET_BITPOS(Kinesthetic1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x24, &m_stSlaveParams.GET_BITPOS(Kinesthetic2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic3) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x25, &m_stSlaveParams.GET_BITPOS(Kinesthetic3), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic4) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x26, &m_stSlaveParams.GET_BITPOS(Kinesthetic4), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic5) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x27, &m_stSlaveParams.GET_BITPOS(Kinesthetic5), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic6) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x28, &m_stSlaveParams.GET_BITPOS(Kinesthetic6), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic7) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x29, &m_stSlaveParams.GET_BITPOS(Kinesthetic7), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic8) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2A, &m_stSlaveParams.GET_BITPOS(Kinesthetic8), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic9) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2B, &m_stSlaveParams.GET_BITPOS(Kinesthetic9), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic10) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2C, &m_stSlaveParams.GET_BITPOS(Kinesthetic10), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic11) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2D, &m_stSlaveParams.GET_BITPOS(Kinesthetic11), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Kinesthetic12) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2E, &m_stSlaveParams.GET_BITPOS(Kinesthetic12), eInput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(Tactile1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x2F, &m_stSlaveParams.GET_BITPOS(Tactile1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Tactile2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x30, &m_stSlaveParams.GET_BITPOS(Tactile2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Tactile3) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x31, &m_stSlaveParams.GET_BITPOS(Tactile3), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(Tactile4) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x32, &m_stSlaveParams.GET_BITPOS(Tactile4), eInput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(StatusIn1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x33, &m_stSlaveParams.GET_BITPOS(StatusIn1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(StatusIn2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x34, &m_stSlaveParams.GET_BITPOS(StatusIn2), eInput)))
        return FALSE;

    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn1) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x35, &m_stSlaveParams.GET_BITPOS(AddInfoIn1), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn2) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x36, &m_stSlaveParams.GET_BITPOS(AddInfoIn2), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn3) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x37, &m_stSlaveParams.GET_BITPOS(AddInfoIn3), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn4) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x38, &m_stSlaveParams.GET_BITPOS(AddInfoIn4), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn5) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x39, &m_stSlaveParams.GET_BITPOS(AddInfoIn5), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn6) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3A, &m_stSlaveParams.GET_BITPOS(AddInfoIn6), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn7) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3B, &m_stSlaveParams.GET_BITPOS(AddInfoIn7), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn8) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3C, &m_stSlaveParams.GET_BITPOS(AddInfoIn8), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn9) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3D, &m_stSlaveParams.GET_BITPOS(AddInfoIn9), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn10) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3E, &m_stSlaveParams.GET_BITPOS(AddInfoIn10), eInput)))   
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn11) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x3F, &m_stSlaveParams.GET_BITPOS(AddInfoIn11), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn12) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x40, &m_stSlaveParams.GET_BITPOS(AddInfoIn12), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn13) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x41, &m_stSlaveParams.GET_BITPOS(AddInfoIn13), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(AddInfoIn14) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x42, &m_stSlaveParams.GET_BITPOS(AddInfoIn14), eInput)))
        return FALSE;

    

    return TRUE;
}

void CSlaveKistarHand::WriteToSlave()
{    

    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(StatusOut1), m_stSlaveParams.stOutPDOs.usStatusOut1, 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(StatusOut2), m_stSlaveParams.stOutPDOs.usStatusOut2, 16);

    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos1), m_stSlaveParams.stOutPDOs.nTargetPos[0], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos2), m_stSlaveParams.stOutPDOs.nTargetPos[1], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos3), m_stSlaveParams.stOutPDOs.nTargetPos[2], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos4), m_stSlaveParams.stOutPDOs.nTargetPos[3], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos5), m_stSlaveParams.stOutPDOs.nTargetPos[4], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos6), m_stSlaveParams.stOutPDOs.nTargetPos[5], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos7), m_stSlaveParams.stOutPDOs.nTargetPos[6], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos8), m_stSlaveParams.stOutPDOs.nTargetPos[7], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos9), m_stSlaveParams.stOutPDOs.nTargetPos[8], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos10), m_stSlaveParams.stOutPDOs.nTargetPos[9], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos11), m_stSlaveParams.stOutPDOs.nTargetPos[10], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos12), m_stSlaveParams.stOutPDOs.nTargetPos[11], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos13), m_stSlaveParams.stOutPDOs.nTargetPos[12], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos14), m_stSlaveParams.stOutPDOs.nTargetPos[13], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos15), m_stSlaveParams.stOutPDOs.nTargetPos[14], 16);
    CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(TargetPos16), m_stSlaveParams.stOutPDOs.nTargetPos[15], 16);

    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut1), m_stSlaveParams.stOutPDOs.usAddInfoOut[0], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut2), m_stSlaveParams.stOutPDOs.usAddInfoOut[1], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut3), m_stSlaveParams.stOutPDOs.usAddInfoOut[2], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut4), m_stSlaveParams.stOutPDOs.usAddInfoOut[3], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut5), m_stSlaveParams.stOutPDOs.usAddInfoOut[4], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut6), m_stSlaveParams.stOutPDOs.usAddInfoOut[5], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut7), m_stSlaveParams.stOutPDOs.usAddInfoOut[6], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut8), m_stSlaveParams.stOutPDOs.usAddInfoOut[7], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut9), m_stSlaveParams.stOutPDOs.usAddInfoOut[8], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut10), m_stSlaveParams.stOutPDOs.usAddInfoOut[9], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut11), m_stSlaveParams.stOutPDOs.usAddInfoOut[10], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut12), m_stSlaveParams.stOutPDOs.usAddInfoOut[11], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut13), m_stSlaveParams.stOutPDOs.usAddInfoOut[12], 16);
    CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(AddInfoOut14), m_stSlaveParams.stOutPDOs.usAddInfoOut[13], 16);

    
}

void CSlaveKistarHand::ReadFromSlave()
{
    m_stSlaveParams.stInPDOs.usStatusIn1 = CEcatSlaveBase::ReadPdoU16(m_stSlaveParams.GET_OFFSET(StatusIn1));
    m_stSlaveParams.stInPDOs.usStatusIn2 = CEcatSlaveBase::ReadPdoU16(m_stSlaveParams.GET_OFFSET(StatusIn2));

    m_stSlaveParams.stInPDOs.nPosition[0] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position1));
    m_stSlaveParams.stInPDOs.nPosition[1] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position2));
    m_stSlaveParams.stInPDOs.nPosition[2] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position3));
    m_stSlaveParams.stInPDOs.nPosition[3] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position4));
    m_stSlaveParams.stInPDOs.nPosition[4] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position5));
    m_stSlaveParams.stInPDOs.nPosition[5] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position6));
    m_stSlaveParams.stInPDOs.nPosition[6] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position7));
    m_stSlaveParams.stInPDOs.nPosition[7] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position8));
    m_stSlaveParams.stInPDOs.nPosition[8] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position9));
    m_stSlaveParams.stInPDOs.nPosition[9] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position10));
    m_stSlaveParams.stInPDOs.nPosition[10] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position11));
    m_stSlaveParams.stInPDOs.nPosition[11] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position12));
    m_stSlaveParams.stInPDOs.nPosition[12] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position13));
    m_stSlaveParams.stInPDOs.nPosition[13] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position14));
    m_stSlaveParams.stInPDOs.nPosition[14] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position15));
    m_stSlaveParams.stInPDOs.nPosition[15] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Position16));

    m_stSlaveParams.stInPDOs.nCurrent[0] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current1));
    m_stSlaveParams.stInPDOs.nCurrent[1] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current2));
    m_stSlaveParams.stInPDOs.nCurrent[2] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current3));
    m_stSlaveParams.stInPDOs.nCurrent[3] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current4));
    m_stSlaveParams.stInPDOs.nCurrent[4] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current5));
    m_stSlaveParams.stInPDOs.nCurrent[5] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current6));
    m_stSlaveParams.stInPDOs.nCurrent[6] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current7));
    m_stSlaveParams.stInPDOs.nCurrent[7] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current8));
    m_stSlaveParams.stInPDOs.nCurrent[8] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current9));
    m_stSlaveParams.stInPDOs.nCurrent[9] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current10));
    m_stSlaveParams.stInPDOs.nCurrent[10] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current11));
    m_stSlaveParams.stInPDOs.nCurrent[11] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current12));
    m_stSlaveParams.stInPDOs.nCurrent[12] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current13));
    m_stSlaveParams.stInPDOs.nCurrent[13] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current14));
    m_stSlaveParams.stInPDOs.nCurrent[14] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current15));
    m_stSlaveParams.stInPDOs.nCurrent[15] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Current16));

    m_stSlaveParams.stInPDOs.nKinesthetic[0] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic1));
    m_stSlaveParams.stInPDOs.nKinesthetic[1] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic2));
    m_stSlaveParams.stInPDOs.nKinesthetic[2] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic3));
    m_stSlaveParams.stInPDOs.nKinesthetic[3] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic4));
    m_stSlaveParams.stInPDOs.nKinesthetic[4] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic5));
    m_stSlaveParams.stInPDOs.nKinesthetic[5] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic6));
    m_stSlaveParams.stInPDOs.nKinesthetic[6] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic7));
    m_stSlaveParams.stInPDOs.nKinesthetic[7] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic8));
    m_stSlaveParams.stInPDOs.nKinesthetic[8] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic9));
    m_stSlaveParams.stInPDOs.nKinesthetic[9] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic10));
    m_stSlaveParams.stInPDOs.nKinesthetic[10] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic11));
    m_stSlaveParams.stInPDOs.nKinesthetic[11] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Kinesthetic12));

    m_stSlaveParams.stInPDOs.nTactile[0] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Tactile1));
    m_stSlaveParams.stInPDOs.nTactile[1] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Tactile2));
    m_stSlaveParams.stInPDOs.nTactile[2] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Tactile3));
    m_stSlaveParams.stInPDOs.nTactile[3] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(Tactile4));

    m_stSlaveParams.stInPDOs.nAddInfoIn[0] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn1));
    m_stSlaveParams.stInPDOs.nAddInfoIn[1] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn2));
    m_stSlaveParams.stInPDOs.nAddInfoIn[2] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn3));
    m_stSlaveParams.stInPDOs.nAddInfoIn[3] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn4));
    m_stSlaveParams.stInPDOs.nAddInfoIn[4] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn5));
    m_stSlaveParams.stInPDOs.nAddInfoIn[5] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn6));
    m_stSlaveParams.stInPDOs.nAddInfoIn[6] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn7));
    m_stSlaveParams.stInPDOs.nAddInfoIn[7] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn8));
    m_stSlaveParams.stInPDOs.nAddInfoIn[8] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn9));
    m_stSlaveParams.stInPDOs.nAddInfoIn[9] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn10));
    m_stSlaveParams.stInPDOs.nAddInfoIn[10] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn11));
    m_stSlaveParams.stInPDOs.nAddInfoIn[11] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn12));
    m_stSlaveParams.stInPDOs.nAddInfoIn[12] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn13));
    m_stSlaveParams.stInPDOs.nAddInfoIn[13] = CEcatSlaveBase::ReadPdoS16(m_stSlaveParams.GET_OFFSET(AddInfoIn14));
    
}