/*****************************************************************************
*	Name: SlaveNRMKEndTool.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CSlaveNrmkEndtool child class.
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef _ECAT_SLAVE_NRMK_ENDTOOL_
#define _ECAT_SLAVE_NRMK_ENDTOOL_

#include "EcatSlaveBase.h"

#ifndef NRMK_VENDOR_ID
#define NRMK_VENDOR_ID         		0x0000089a
#endif

#define NRMK_ENDTOOL 				0x10000007
#define NRMK_ENDTOOL_ACTIVATE_WORD  0x0300
#define NRMK_ENDTOOL_SYNC0_SHIFT    0

#define BLINK 	1
#define STEADY 	0

#define FT_START_DATA_OUTPUT	(INT32)0x0000000B
#define FT_STOP_DATA_OUTPUT		(INT32)0x0000000C
#define FT_BIAS_COMMAND			(INT32)0x00000011
#define FT_DATARATE_COMMAND		(INT32)0x0000000F
#define FT_FILTER_COMMAND		(INT32)0x00000118

#define FT_FORCE_DIVIDER		50.
#define FT_TORQUE_DIVIDER		2000.

class CSlaveNrmkEndtool : public CEcatSlaveBase
{
private:
    struct NRMK_ENDTOOL_OUT // Output from slave (input to master)
    {
        UINT8 ILed;
        UINT8 IGripper;
        INT32 FTConfigParam;
        UINT8 LEDMode;
        UINT8 LEDG;
        UINT8 LEDR;
        UINT8 LEDB;
    };
    
    struct NRMK_ENDTOOL_IN // Input to slave (output from master)
    {
        UINT8 IStatus;
        UINT8 IButton;
        INT16 FTRawFx;
        INT16 FTRawFy;
        INT16 FTRawFz;
        INT16 FTRawTx;
        INT16 FTRawTy;
        INT16 FTRawTz;
        UINT8 FTOverloadStatus;
        UINT8 FTErrorFlag;

        // Calculated values
        float Fx;
        float Fy;
        float Fz;
        float Tx;
        float Ty;
        float Tz;
    };
    
    struct NRMK_ENDTOOL_SLAVE_PARAMS
    {
        NRMK_ENDTOOL_OUT stOutPDOs;
        NRMK_ENDTOOL_IN stInPDOs;

        // PDO Offsets
        SET_OFFSET(ILed);
        SET_OFFSET(IGripper);
        SET_OFFSET(FTConfigParam);
        SET_OFFSET(LEDMode);
        SET_OFFSET(LEDG);
        SET_OFFSET(LEDR);
        SET_OFFSET(LEDB);
        SET_OFFSET(IStatus);
        SET_OFFSET(IButton);
        SET_OFFSET(FTRawFx);
        SET_OFFSET(FTRawFy);
        SET_OFFSET(FTRawFz);
        SET_OFFSET(FTRawTx);
        SET_OFFSET(FTRawTy);
        SET_OFFSET(FTRawTz);
        SET_OFFSET(FTOverloadStatus);
        SET_OFFSET(FTErrorFlag);

        // PDO Bit Positions
        SET_BITPOS(ILed);
        SET_BITPOS(IGripper);
        SET_BITPOS(FTConfigParam);
        SET_BITPOS(LEDMode);
        SET_BITPOS(LEDG);
        SET_BITPOS(LEDR);
        SET_BITPOS(LEDB);
        SET_BITPOS(IStatus);
        SET_BITPOS(IButton);
        SET_BITPOS(FTRawFx);
        SET_BITPOS(FTRawFy);
        SET_BITPOS(FTRawFz);
        SET_BITPOS(FTRawTx);
        SET_BITPOS(FTRawTy);
        SET_BITPOS(FTRawTz);
        SET_BITPOS(FTOverloadStatus);
        SET_BITPOS(FTErrorFlag);
    };

    // NRMK EndTool specific PDO configuration
    ec_pdo_entry_info_t m_nrmk_pdo_entries[17] = {
        // Output PDOs (to slave)
        {0x7000, 0x01, 8},  // ILed
        {0x7000, 0x02, 8},  // IGripper
        {0x7000, 0x03, 32}, // FTConfigParam
        {0x7000, 0x04, 8},  // LEDMode
        {0x7000, 0x05, 8},  // LEDG
        {0x7000, 0x06, 8},  // LEDR
        {0x7000, 0x07, 8},  // LEDB
        // Input PDOs (from slave)
        {0x6000, 0x01, 8},  // IStatus
        {0x6000, 0x02, 8},  // IButton
        {0x6000, 0x03, 16}, // FTRawFx
        {0x6000, 0x04, 16}, // FTRawFy
        {0x6000, 0x05, 16}, // FTRawFz
        {0x6000, 0x06, 16}, // FTRawTx
        {0x6000, 0x07, 16}, // FTRawTy
        {0x6000, 0x08, 16}, // FTRawTz
        {0x6000, 0x09, 8},  // FTOverloadStatus
        {0x6000, 0x0a, 8},  // FTErrorFlag
    };

    ec_pdo_info_t m_nrmk_pdos[2] = {
        {0x1600, 7, m_nrmk_pdo_entries + 0},  // Output PDO
        {0x1a00, 10, m_nrmk_pdo_entries + 7}, // Input PDO
    };

    ec_sync_info_t m_nrmk_syncs[5];

protected:
    void InitSyncs() override {
        m_nrmk_syncs[0] = {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
        m_nrmk_syncs[1] = {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
        m_nrmk_syncs[2] = {2, EC_DIR_OUTPUT, 1, m_nrmk_pdos + 0, EC_WD_ENABLE};
        m_nrmk_syncs[3] = {3, EC_DIR_INPUT, 1, m_nrmk_pdos + 1, EC_WD_DISABLE};
        m_nrmk_syncs[4] = {0xff};
        
        SetEcatPdoSync(m_nrmk_syncs);
    }

    BOOL RegisterPDO() override;

public:
    CSlaveNrmkEndtool();
    ~CSlaveNrmkEndtool();

    enum LED_MODE
    {
        NO_COLOR = 0,
        BLINK_RED,
        STEADY_RED,
        BLINK_GREEN,
        STEADY_GREEN,
        BLINK_BLUE,
        STEADY_BLUE,
        BLINK_YELLOW,
        STEADY_YELLOW,
    };

    enum FT_ERROR_CODE {
        NO_ERROR = 0x00,
        UNSUPPORTED_COMMAND = 0x01,
        OUT_OF_RANGE = 0x02,
        FAILED_PARAMETER = 0x03,
    };

    enum FT_STATE_MACHINE {
        FT_NONE = 0x00,
        FT_READY,
        FT_SET_DATARATE,
        FT_SET_FILTER,
        FT_SET_START,
        FT_SET_BIAS,
        FT_FAULT
    };

    // Override base class methods
	/* EtherCAT */
	virtual void WriteToSlave();
	virtual void ReadFromSlave();

    // Setters
    void SetILed(UINT8 value) { m_stSlaveParams.stOutPDOs.ILed = value; }
    void SetIGripper(UINT8 value) { m_stSlaveParams.stOutPDOs.IGripper = value; }
    void SetFTConfigParam(INT32 value) { m_stSlaveParams.stOutPDOs.FTConfigParam = value; }
    void SetLEDMode(UINT8 value) { m_stSlaveParams.stOutPDOs.LEDMode = value; }
    void SetLEDG(UINT8 value) { m_stSlaveParams.stOutPDOs.LEDG = value; }
    void SetLEDR(UINT8 value) { m_stSlaveParams.stOutPDOs.LEDR = value; }
    void SetLEDB(UINT8 value) { m_stSlaveParams.stOutPDOs.LEDB = value; }

    // LED convenience methods
    void LED_OFF() { SetLEDMode(NO_COLOR); }
    void LED_RED(int blink) { SetLEDMode(blink ? BLINK_RED : STEADY_RED); }
    void LED_GREEN(int blink) { SetLEDMode(blink ? BLINK_GREEN : STEADY_GREEN); }
    void LED_BLUE(int blink) { SetLEDMode(blink ? BLINK_BLUE : STEADY_BLUE); }
    void LED_YELLOW(int blink) { SetLEDMode(blink ? BLINK_YELLOW : STEADY_YELLOW); }

    // Getters
    UINT8 GetIStatus() const { return m_stSlaveParams.stInPDOs.IStatus; }
    UINT8 GetIButton() const { return m_stSlaveParams.stInPDOs.IButton; }
    INT16 GetFTRawFx() const { return m_stSlaveParams.stInPDOs.FTRawFx; }
    INT16 GetFTRawFy() const { return m_stSlaveParams.stInPDOs.FTRawFy; }
    INT16 GetFTRawFz() const { return m_stSlaveParams.stInPDOs.FTRawFz; }
    INT16 GetFTRawTx() const { return m_stSlaveParams.stInPDOs.FTRawTx; }
    INT16 GetFTRawTy() const { return m_stSlaveParams.stInPDOs.FTRawTy; }
    INT16 GetFTRawTz() const { return m_stSlaveParams.stInPDOs.FTRawTz; }
    UINT8 GetFTOverloadStatus() const { return m_stSlaveParams.stInPDOs.FTOverloadStatus; }
    UINT8 GetFTErrorFlag() const { return m_stSlaveParams.stInPDOs.FTErrorFlag; }

    // Calculated force/torque values
    float GetFx() const { return m_stSlaveParams.stInPDOs.Fx; }
    float GetFy() const { return m_stSlaveParams.stInPDOs.Fy; }
    float GetFz() const { return m_stSlaveParams.stInPDOs.Fz; }
    float GetTx() const { return m_stSlaveParams.stInPDOs.Tx; }
    float GetTy() const { return m_stSlaveParams.stInPDOs.Ty; }
    float GetTz() const { return m_stSlaveParams.stInPDOs.Tz; }

    // FT Sensor control
    void FTSensorOn(bool biasing = true, int output_rate = 1000, int lpf_cof = 100);
    void FTSensorOff();
    bool IsFTSensorOn() const { return m_bIsFTOn; }
    int GetFTOutputRate() const { return m_nFTOutputRate; }
    int GetFTLPFCof() const { return m_nFTLPFCof; }

private:
    NRMK_ENDTOOL_SLAVE_PARAMS m_stSlaveParams;

    // FT Sensor state
    bool m_bIsFTOn;
    bool m_bFTOn;
    bool m_bFTBiasing;
    int m_nFTOutputRate;
    int m_nFTLPFCof;
    FT_STATE_MACHINE m_eFTState;

    // Helper methods
    UINT8 GetFTDataRate(int rate);
    UINT8 GetFTLPFCof(int cof);
    INT32 GetFTDataRateCmd(int datarate);
    INT32 GetFTLPFCmd(int cof);
    INT32 GetFTBiasCmd(bool bias);
};

#endif // _ECAT_SLAVE_NRMK_ENDTOOL_