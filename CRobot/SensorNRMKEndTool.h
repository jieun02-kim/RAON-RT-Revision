/*****************************************************************************
*	Name: SensorNRMKEndTool.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CSensorNRMKEndTool class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#ifndef __SENSOR_NRMK_ENDTOOL__
#define __SENSOR_NRMK_ENDTOOL__

#include "SensorFT.h"
#include "SlaveNRMKEndTool.h"
#include "EcatMasterBase.h"

class CSensorNRMKEndTool : public CSensorFT
{
public:
    CSensorNRMKEndTool(BOOL abEnabled = TRUE);
    virtual ~CSensorNRMKEndTool();

    // Override base class methods
    virtual BOOL Init(){return TRUE;}  
    virtual BOOL Init(CEcatMaster& apEcmaster);
    virtual BOOL DeInit() override;
    virtual BOOL Start() override;
    virtual BOOL Stop() override;
    virtual BOOL Reset() override;
    virtual BOOL Calibrate(eSensorCalibration aeCalType = eSensorCalibrationZero) override;
    virtual BOOL ReadData() override;
    virtual BOOL IsDataReady() override;

    // Configuration overrides
    virtual BOOL SetSampleRate(UINT32 auRate) override;
    virtual BOOL SetFilterCutoff(UINT32 auCutoff) override;

    // NRMK-specific methods
    virtual BOOL InitSlave(CEcatMaster& apEcmaster);
    virtual void SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode) override;
    virtual void SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime) override;

    // LED control
    virtual void SetLED(CSlaveNrmkEndtool::LED_MODE aeLEDMode);
    virtual void LED_OFF();
    virtual void LED_RED(BOOL abBlink = FALSE);
    virtual void LED_GREEN(BOOL abBlink = FALSE);
    virtual void LED_BLUE(BOOL abBlink = FALSE);
    virtual void LED_YELLOW(BOOL abBlink = FALSE);

    // Gripper control
    virtual void SetGripper(UINT8 auValue);
    virtual UINT8 GetGripper() const;

    // Button/Status
    virtual BOOL IsButtonPressed() const;
    virtual UINT8 GetStatus() const;

    // FT Sensor specific
    virtual BOOL IsFTSensorOn() const;
    virtual UINT8 GetFTErrorFlag() const;
    virtual UINT8 GetFTOverloadStatus() const;

    // EtherCAT slave access
    virtual CSlaveNrmkEndtool* GetSlave() { return &m_cNrmkSlave; }

protected:
    // Override data processing
    virtual void ProcessRawData() override;
    virtual void UpdateRawData(INT16 anFx, INT16 anFy, INT16 anFz, INT16 anTx, INT16 anTy, INT16 anTz) override;

    // NRMK-specific processing
    virtual void HandleFTErrors();
    virtual void UpdateLEDStatus();

private:
    CSlaveNrmkEndtool m_cNrmkSlave;
    CEcatMaster* m_pEcMaster;
    
    // LED status tracking
    CSlaveNrmkEndtool::LED_MODE m_eLEDMode;
    BOOL m_bLEDBlink;
    
    // Internal state
    BOOL m_bSlaveInitialized;
    UINT32 m_uLastDataTime;
};

#endif //__SENSOR_NRMK_ENDTOOL__
