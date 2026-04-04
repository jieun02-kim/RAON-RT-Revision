/*****************************************************************************
*	Name: RobotEx.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implemenation of the CRobotIndy7 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Indy7Ctrl.h"
#include <unistd.h>
#include <cmath>    

#define CTRL_MODE_GRAV_COMP     0
#define CTRL_MODE_FULL_DYN      1
#define CTRL_MODE_CTC           2
#define CTRL_MODE_ADAPTIVE      3

void proc_main_control(void* apRobot);
void proc_ethercat_control(void* apRobot);
void proc_keyboard_control(void* apRobot);
void proc_terminal_output(void* apRobot);
void proc_logger(void* apRobot);


/****************************************************************************/
CRobotIndy7::CRobotIndy7(CConfigRobot* apConfig)
{
    m_cKeyPress = ' ';
    m_pEcatAxis = NULL;
    m_pEcatSensor = NULL;
    m_bStopTask = FALSE;
    m_strDataLog = "DataLog_";
    m_nEcatCycle = 0;
    m_bEcatOP = FALSE;
 
    // Initialize RT Controller
    m_pController = NULL;
    m_bRTControllerEnabled = FALSE;

    
    /* Initialize Configuration */
    if (apConfig == NULL)
    {
        m_pcConfigRobot = new CConfigRobot();
        m_pcConfigRobot->LoadDefaultConfig();
    }
    else
    {
        m_pcConfigRobot = apConfig;
        m_pcConfigRobot->ReadConfiguration();
    }
    /* Set Robot Name */
    m_strName = m_pcConfigRobot->GetSystemConf().strName;

    /* todo: 1. check return values, 2. include fucnction pointers in the loop without using void vectors */
    /* this is temporary */
    AddTaskFunction(&proc_main_control);
    AddTaskFunction(&proc_ethercat_control);
    AddTaskFunction(&proc_keyboard_control);
    AddTaskFunction(&proc_terminal_output);
    AddTaskFunction(&proc_logger);
}

CRobotIndy7::~CRobotIndy7()
{
    if (NULL != m_pcConfigRobot)
    {
        delete m_pcConfigRobot;
        m_pcConfigRobot = NULL;
    }

    if (m_pController != NULL)
    {
        delete m_pController;
        m_pController = NULL;
    }
}

BOOL 
CRobotIndy7::Init(BOOL abSim)
{
    /* Init Controller */
    if (FALSE == InitController(m_strName = m_pcConfigRobot->GetSystemConf().strURDFPath))
    {
        DBG_LOG_ERROR("(%s) Failed to initialize RT Controller", "CRobotIndy7");
        return FALSE;
    }

    if (!CRobot::Init(abSim))      
        return FALSE;

    return TRUE;
}

BOOL 
CRobotIndy7::DeInit()
{
    if (TRUE == CRobot::DeInit())
    {
        // if (NULL != m_pEcatAxis)
        // {
        //     delete m_pEcatAxis;
        //     m_pEcatAxis = NULL;
        // }
        delete[] m_pEcatAxis;
        m_pEcatAxis = nullptr;

        delete[] m_pEcatSensor;
        m_pEcatSensor = nullptr;    

        return TRUE;
    }
    
    return FALSE;
}

BOOL 
CRobotIndy7::InitController(const TSTRING& astrURDFPath)
{
    // Clean up existing controller
    if (m_pController != NULL)
    {
        delete m_pController;
        m_pController = NULL;
    }

    // const unsigned int dof = GetTotalAxis();
    const unsigned int dof = m_pcConfigRobot->GetSystemConf().nRobotAxis; // Get from system config
    if (dof == 0)
    {
        DBG_LOG_ERROR("(%s) Cannot initialize RT Controller: Invalid DOF = 0", "CRobotIndy7");
        return FALSE;
    }

    // Create RT Controller
    m_pController = new CControllerFullDynamicsRT(astrURDFPath, dof);
    if (m_pController == NULL)
    {
        DBG_LOG_ERROR("(%s) Failed to create RT controller", "CRobotIndy7");
        return FALSE;
    }

    // Configure RT Controller
    m_pController->EnableRTMode(TRUE);
    m_pController->SetDeadline(500000);    // 500µs deadline for 1kHz

    INT32 nDefaultControllerMode = m_pcConfigRobot->GetSystemConf().nDefaultControllerMode;

    switch(nDefaultControllerMode)
    {
        case CTRL_MODE_GRAV_COMP:
            m_pController->SetControlMode(CControllerFullDynamicsRT::eGravityCompensation);
            break;
        case CTRL_MODE_CTC:
            m_pController->SetControlMode(CControllerFullDynamicsRT::eComputedTorque);
            break;
        case CTRL_MODE_FULL_DYN:
            m_pController->SetControlMode(CControllerFullDynamicsRT::eFullDynamics);
            break;
        default:
            DBG_LOG_WARN("(%s) Invalid controller mode %d, defaulting to Gravity Compensation", 
                            "CRobotIndy7", nDefaultControllerMode);
            m_pController->SetControlMode(CControllerFullDynamicsRT::eGravityCompensation);
            break;
    }

    // Initialize RT Controller
    if (!m_pController->Init())
    {
        DBG_LOG_ERROR("(%s) Failed to initialize RT controller", "CRobotIndy7");
        delete m_pController;
        m_pController = NULL;
        return FALSE;
    }

    // Set controller gains 
    std::vector<double> Kp, Kd;
    if (dof == 6) // Indy7 specific gains
    {
       for (int nCnt = 0; nCnt < dof; nCnt++)
       {
           Kp.push_back(m_pcConfigRobot->GetSystemConf().vKp[nCnt]);
           Kd.push_back(m_pcConfigRobot->GetSystemConf().vKd[nCnt]);
       }
    }
    else // Generic gains for other DOF configurations
    {
        // Scale gains based on typical joint characteristics
        Kp.resize(dof);
        Kd.resize(dof);
        
        for (unsigned int i = 0; i < dof; ++i)
        {
            if (i < 2)      // Base and shoulder joints - high gains
            {
                Kp[i] = 800.0 - i * 200.0;  // 800, 600
                Kd[i] = 80.0 - i * 20.0;    // 80, 60
            }
            else if (i < 4) // Elbow and wrist - moderate gains
            {
                Kp[i] = 400.0 - (i-2) * 200.0;  // 400, 200
                Kd[i] = 40.0 - (i-2) * 20.0;    // 40, 20
            }
            else            // End-effector joints - lower gains
            {
                Kp[i] = 150.0 - (i-4) * 50.0;   // 150, 100
                Kd[i] = 15.0 - (i-4) * 5.0;     // 15, 10
            }
        }
    }
    m_pController->SetControlGains(Kp, Kd);
    
    // Pre-allocate RT control vectors
    m_vCurrentPos.resize(dof, 0.0);
    m_vCurrentVel.resize(dof, 0.0);
    m_vCurrentTor.resize(dof, 0.0);
    m_vOutputTorque.resize(dof, 0.0);

    DBG_LOG_INFO("(%s) RT Controller initialized successfully with DOF=%u", "CRobotIndy7", dof);
    DBG_LOG_INFO("(%s) Gains - Kp: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", "CRobotIndy7",
                 Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
    DBG_LOG_INFO("(%s) Gains - Kd: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", "CRobotIndy7",
                 Kd[0], Kd[1], Kd[2], Kd[3], Kd[4], Kd[5]);

    // Enable the controller if Init is successful
    if (TRUE == m_pcConfigRobot->GetSystemConf().bEnableControllerAtStartup)
        EnableController(TRUE);

    return TRUE;
}

BOOL 
CRobotIndy7::EnableController(BOOL abEnable)
{
    if (m_pController == NULL)
    {
        DBG_LOG_ERROR("(CRobotIndy7::EnableRTController) RT Controller not initialized");
        return FALSE;
    }

    m_pController->Enable(abEnable);
    m_bRTControllerEnabled = abEnable;

    DBG_LOG_INFO("(CRobotIndy7::EnableRTController) RT Controller %s", 
                 abEnable ? "ENABLED" : "DISABLED");
    return TRUE;
}

BOOL 
CRobotIndy7::SetControllerMode(CControllerFullDynamicsRT::eControlMode aeMode)
{
    if (m_pController == NULL)
    {
        DBG_LOG_ERROR("(CRobotIndy7::SetRTControllerMode) RT Controller not initialized");
        return FALSE;
    }

    m_pController->SetControlMode(aeMode);
    
    const char* mode_names[] = {"Gravity Compensation", "Full Dynamics", "Computed Torque", "Adaptive Control"};
    if (aeMode < 4)
    {
        DBG_LOG_INFO("(CRobotIndy7::SetRTControllerMode) Mode set to: %s", mode_names[aeMode]);
    }
    
    return TRUE;
}

BOOL 
CRobotIndy7::SetControllerGains(const std::vector<double>& avKp, const std::vector<double>& avKd)
{
    if (m_pController == NULL)
    {
        DBG_LOG_ERROR("(CRobotIndy7::SetRTControllerGains) RT Controller not initialized");
        return FALSE;
    }

    const unsigned int dof = GetTotalAxis();
    if (avKp.size() != dof || avKd.size() != dof)
    {
        DBG_LOG_ERROR("(CRobotIndy7::SetRTControllerGains) Invalid gain vector size. Expected: %u, Got: Kp=%zu, Kd=%zu", 
                     dof, avKp.size(), avKd.size());
        return FALSE;
    }

    m_pController->SetControlGains(avKp, avKd);
    DBG_LOG_INFO("(CRobotIndy7::SetRTControllerGains) Gains updated successfully");
    return TRUE;
}

BOOL
CRobotIndy7::InitEtherCAT()
{
    m_pcEcatMaster = new CEcatMaster();
    // 1) End-Effector 슬레이브를 가장 먼저 등록

    INT32 nTotalSlaves = m_pcConfigRobot->GetEcatMasterConf().nNoOfSlaves;
    VEC_ECAT_SLAVE_CONF vSlaveInfoList = m_pcConfigRobot->GetEcatSlaveList();
    
    m_pEcatAxis = new CAxisNRMKCore* [nTotalSlaves];
    m_pEcatSensor = new CSensorNRMKEndTool* [nTotalSlaves];        

    /* Read */
	for (int nCnt = 0; nCnt < nTotalSlaves; nCnt++)
	{
        ST_CONFIG_ECAT_SLAVE stSlaveInfo = vSlaveInfoList[nCnt];

        /* Determine Joint type 
        * Default is revolute
        */
        INT nSlaveType = stSlaveInfo.nSlaveType;

        if (nSlaveType == 0)
        {
            eAxisType ejointtemp;
            if (stSlaveInfo.nJointType >= eAxisRevolute && stSlaveInfo.nJointType <= eAxisJointless)
                ejointtemp = eAxisType(stSlaveInfo.nJointType);
            else
                ejointtemp = eAxisRevolute;

            DBG_LOG_WARN("(%s) Transmission Ratio %lf axis:%d", "CRobotIndy7", stSlaveInfo.dTransRatio, nCnt);

            m_pEcatAxis[nCnt] = new CAxisNRMKCore(ejointtemp, stSlaveInfo.dEncResolution, stSlaveInfo.dGearRatio, stSlaveInfo.dTransRatio, stSlaveInfo.bAbsEnc, stSlaveInfo.bCCW, stSlaveInfo.bEnabled, stSlaveInfo.bConnected);
            m_pEcatAxis[nCnt]->SetName(stSlaveInfo.strName);
            m_pEcatAxis[nCnt]->SetOneTurnRef(stSlaveInfo.dOneTurnRef);
            m_pEcatAxis[nCnt]->SetTorqueConstant(stSlaveInfo.dTorqueConstant);
            m_pEcatAxis[nCnt]->SetCurrentRatio(stSlaveInfo.dCurrentRatio);

            /* EtherCAT Configuration */
            m_pEcatAxis[nCnt]->SetVendorInfo(stSlaveInfo.uVendorID, stSlaveInfo.uProductCode);
            m_pEcatAxis[nCnt]->SetDCInfo(stSlaveInfo.bDCSupported, stSlaveInfo.usDCActivateWord, stSlaveInfo.nDCSyncShift);

            /* Set physical limits */
            m_pEcatAxis[nCnt]->SetVelocityLimits(stSlaveInfo.dVelLimitL, stSlaveInfo.dVelLimitU);
            m_pEcatAxis[nCnt]->SetAccelerationLimits(stSlaveInfo.dAccLimitL, stSlaveInfo.dAccLimitU);
            m_pEcatAxis[nCnt]->SetDecelerationLimits(stSlaveInfo.dDecLimitL, stSlaveInfo.dDecLimitU);
            m_pEcatAxis[nCnt]->SetJerkLimits(stSlaveInfo.dJerkLimitL, stSlaveInfo.dJerkLimitU);
            m_pEcatAxis[nCnt]->SetTorqueLimits(stSlaveInfo.dTorLimitL, stSlaveInfo.dJerkLimitU);
            m_pEcatAxis[nCnt]->SetPositionLimits(stSlaveInfo.dPosLimitL, stSlaveInfo.dPosLimitU);
            m_pEcatAxis[nCnt]->SetAutoServoOn(stSlaveInfo.bAutoServoOn);
            
            /* Set Position at startup, compensate position for incremental encoder */
            m_pEcatAxis[nCnt]->SetPositionBeforeExit(stSlaveInfo.nPosBeforeExit);

            /* determine homing method */
            eHomingMethod ehomingtemp;
            if (stSlaveInfo.nHomingMethod > (INT32)eAxisHomingNotSet && stSlaveInfo.nHomingMethod <= (INT32)eAxisHomeBuiltin)
                ehomingtemp = (eHomingMethod)stSlaveInfo.nHomingMethod;
            else
                ehomingtemp = eAxisHomeStartPos;

            m_pEcatAxis[nCnt]->SetHomingMethod(ehomingtemp);

            if (eAxisHomeSearch == ehomingtemp)
                m_pEcatAxis[nCnt]->SetHomeReference(stSlaveInfo.dHomeSearchRef);
               
            else if (eAxisHomeManual == ehomingtemp)
                m_pEcatAxis[nCnt]->SetHomePosition(stSlaveInfo.nHomePositionOffset);

            /* Set operational motion parameters
            * Limits should be configured first.
            */
            if (FALSE == m_pEcatAxis[nCnt]->SetVelocity(stSlaveInfo.dOperatingVel))
            {
                DBG_LOG_WARN("(%s) Cannot Set Velocity to %lf for Slave No. %d", "CRobotIndy7", stSlaveInfo.dOperatingVel ,nCnt);
            }

            if (FALSE == m_pEcatAxis[nCnt]->SetAcceleration(stSlaveInfo.dOperatingAcc))
            {
                DBG_LOG_WARN("(%s) Cannot Set Acceleration to %lf for Slave No. %d", "CRobotIndy7", stSlaveInfo.dOperatingAcc, nCnt);
            }

            if (FALSE == m_pEcatAxis[nCnt]->SetDeceleration(stSlaveInfo.dOperatingDec))
            {
                DBG_LOG_WARN("(%s) Cannot Set Deceleration to %lf for Slave No. %d", "CRobotIndy7", stSlaveInfo.dOperatingDec, nCnt);
            }

            if (FALSE == m_pEcatAxis[nCnt]->Init(*m_pcEcatMaster, (INT8)stSlaveInfo.nDriveMode))
            {
                DBG_LOG_ERROR("(%s) Cannot Initialize Slave No. %d", "CRobotIndy7", nCnt);
                return FALSE;
            }
            AddAxis(m_pEcatAxis[nCnt]);
        }
        else if (nSlaveType == 1) // Sensor
        {
            int nPos = nTotalSlaves - 1 - nCnt; // 역순으로 센서 생성
            m_pEcatSensor[nPos] = new CSensorNRMKEndTool(stSlaveInfo.bEnabled);
            m_pEcatSensor[nPos]->SetName(stSlaveInfo.strName);
            m_pEcatSensor[nPos]->SetVendorInfo(stSlaveInfo.uVendorID, stSlaveInfo.uProductCode);
            m_pEcatSensor[nPos]->SetDCInfo(stSlaveInfo.bDCSupported, stSlaveInfo.usDCActivateWord, stSlaveInfo.nDCSyncShift);
            if (FALSE == m_pEcatSensor[nPos]->Init(*m_pcEcatMaster))
            {
                DBG_LOG_ERROR("(%s) Cannot Initialize Sensor No. %d", "CRobotIndy7", nCnt);
                return FALSE;
            }
        }
        else
        {
            DBG_LOG_ERROR("(%s) Unknown Slave Type %d for Slave No. %d", "CRobotIndy7", nSlaveType, nCnt);
            return FALSE;
        }
	}
    
    ST_CONFIG_ECAT_MASTER stEcatMaster = m_pcConfigRobot->GetEcatMasterConf();
    m_nEcatCycle = stEcatMaster.nCycleTime;
    /* todo: parameterize EtherCAT cycle time (task period) */
    if (FALSE == m_pcEcatMaster->Init(stEcatMaster.nCycleTime, stEcatMaster.bDCEnabled))
    {
        DBG_LOG_ERROR("(%s) Cannot Initialize EtherCAT Master!", "CRobotIndy7");
        return FALSE;
    }
	return TRUE;
}

BOOL
CRobotIndy7::InitConfig()
{
    return TRUE;
}

void
CRobotIndy7::DoAgingTest()
{
   
}

void
CRobotIndy7::DoInput()
{

    switch (m_cKeyPress)
    {
        case 'y':
        case 'Y':
            m_pEcatSensor[0]->LED_GREEN(TRUE);
            break;
        case 'u':
        case 'U':
            m_pEcatSensor[0]->LED_RED(TRUE);
            break;
        case 'o':
        case 'O':
            m_pEcatSensor[0]->LED_OFF();
        case 'h':
        case 'H':
            break;
        case 'e':
        case 'E':
            for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalAxis(); nMotorCnt++)
            {
                m_pEcatAxis[nMotorCnt]->EmgStopAxis();
            }
            break;
        case 'x':
        case 'X':
            DBG_LOG_INFO(">>> STOP AXIS!");
            for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalAxis(); nMotorCnt++)
            {
                m_pEcatAxis[nMotorCnt]->StopAxis();
                // m_pEcatAxis[nMotorCnt]->ChangeDriveMode(CIA402_PROFILE_POSITION);
            }
            break;
        case 't':
        case 'T':
            DBG_LOG_INFO(">>> Change Drive Mode to CST!");
            for (int n = 0; n < GetTotalAxis() ; ++n)
            {
                m_pEcatAxis[n]->ChangeDriveMode(CIA402_CYCLIC_TORQUE);
            }
            DBG_LOG_WARN("(CRobotIndy7) All EPOS4 → CST mode");
            break;
        case 'r':
        case 'R':
            DBG_LOG_INFO(">>> RT Controller: %s", m_bRTControllerEnabled ? "DISABLED" : "ENABLED");
            EnableController(!m_bRTControllerEnabled);
            break;
        case 'g':
        case 'G':
            DBG_LOG_INFO(">>> RT Controller: Gravity Compensation Mode");
            SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
            break;
        case 'f':
        case 'F':
            DBG_LOG_INFO(">>> RT Controller: Full Dynamics Mode");
            SetControllerMode(CControllerFullDynamicsRT::eFullDynamics);
            break;
        case 'c':
        case 'C':
            DBG_LOG_INFO(">>> RT Controller: Computed Torque Mode");
            SetControllerMode(CControllerFullDynamicsRT::eComputedTorque);
            break;
        case 'i':
        case 'I':
            m_pEcatSensor[0]->LED_RED(TRUE);
            DBG_LOG_INFO(">>> RT Controller: Computed Inverse Kinematics Mode");
            SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics);
            break;
        default:
            break;
    }

    m_cKeyPress = ' ';
    return;
}

void
CRobotIndy7::WriteDataLog()
{
    for (int nCnt = 0; nCnt < 8; nCnt++)
    {
        TSTRING strAxisNo;
        sprintf(&strAxisNo[0], "Axis%d", nCnt);
        TSTRING fileName =  m_strDataLog + strAxisNo.c_str() + ".csv";
        FILE* pfFileTiming = fopen(fileName.c_str(), "w");
        double dTimeDuration = 0.;

        LISTINT::iterator itTarPos = m_stDataLog[nCnt].vecTarPos.begin();
        LISTINT::iterator itActPos = m_stDataLog[nCnt].vecActPos.begin();
        LISTINT::iterator itActTor = m_stDataLog[nCnt].vecActTor.begin();
        LISTULONG::iterator itTimeStamp = m_stDataLog[nCnt].vecTimestamp.begin();
    
        if ((int)m_stDataLog[nCnt].vecActPos.size() == (int)m_stDataLog[nCnt].vecTarPos.size())
        {
        
            for (; itTarPos != m_stDataLog[nCnt].vecTarPos.end(); itTarPos++, itActPos++, itActTor++, itTimeStamp++)
            {
                dTimeDuration += 0.001;
                fprintf(pfFileTiming, "%lf %ld %d %d %d\n", dTimeDuration, *itTimeStamp, *itTarPos, *itActPos, *itActTor);
            }
        }
        else
        {
            DBG_LOG_WARN("[%s] Cannot write datalog for Axis %d, ActPos:%d, TarPos:%d, ActTor:%d, TS:%d", "CRobotIndy7", nCnt, m_stDataLog[nCnt].vecActPos.size(),
                m_stDataLog[nCnt].vecTarPos.size(), m_stDataLog[nCnt].vecActTor.size(), m_stDataLog[nCnt].vecTimestamp.size());
            
            continue;
        }
        fclose(pfFileTiming);
    }
}

BOOL
CRobotIndy7::IsMoving()
{
    // Check if any axis is moving
    for (unsigned int i = 0; i < GetTotalAxis(); ++i)
    {
        if (m_pEcatAxis[i]->IsMoving())
            return TRUE;
    }
    return FALSE;   
}

void proc_main_control(void* apRobot)
{
    auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
    DBG_LOG_INFO("(proc_main_control) Main Control Task Started!");

    const unsigned int udof = pRobot->GetTotalAxis();
    if (udof == 0) 
    {
        DBG_LOG_ERROR("(proc_main_control) Invalid DOF = 0");
        return;
    }

     // Check if RT Controller is available
    CControllerFullDynamicsRT* pRTController = pRobot->GetController();
    BOOL bUseRTController = (pRTController != NULL && pRTController->IsEnabled());
    if (bUseRTController)
        DBG_LOG_INFO("(proc_main_control) Using RT Controller");
    else 
        DBG_LOG_INFO("(proc_main_control) RT Controller not available - using fallback");

    RTTIME tPrev = read_timer();
    uint64_t max_calc_ns = 0, avg_calc_ns = 0, samp = 0;
    long cycle = 0;


    while (!pRobot->CheckStopTask())
    {
        wait_next_period(nullptr);
        if (!pRobot->m_bEcatOP) continue;
        
        pRobot->DoInput();

        // Read current joint states
        for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
        {   

            auto ax = static_cast<CAxisNRMKCore*>(pRobot->m_pEcatAxis[nCnt]);
            
            if (bUseRTController) {
                // Update RT controller vectors
                pRobot->m_vCurrentPos[nCnt] = ax->GetCurrentPos();
                pRobot->m_vCurrentVel[nCnt] = ax->GetCurrentVel();
                pRobot->m_vCurrentTor[nCnt] = ax->GetCurrentTor();
            }
        }

        // Compute control torques
        if (bUseRTController)
        {

            // Use RT Controller - it handles all RT optimizations internally
            if (pRTController->Update(pRobot->m_vCurrentPos, pRobot->m_vCurrentVel, 
                                     pRobot->m_vCurrentTor, pRobot->m_vOutputTorque))
            {
                // Apply RT controller output
                for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
                {
                    auto* ax = pRobot->m_pEcatAxis[nCnt];
                    ax->MoveTorque(pRobot->m_vOutputTorque[nCnt]);
                }
      
            }
            else
            {
                // RT Controller failed, apply zero torque for safety
                DBG_LOG_ERROR("(proc_main_control) RT Controller update failed - applying zero torque");
                for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
                {
                    pRobot->m_pEcatAxis[nCnt]->MoveTorque(0.0);
                }
            }
        }
        else
        {
            // Fallback: Apply zero torque if no controller available
            for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
            {
                pRobot->m_pEcatAxis[nCnt]->MoveTorque(0.0);
            }
        }

        /* Turn On LED depending on control mode */
        BOOL bBlink = pRobot->IsMoving();
        switch(pRTController->GetControlMode())
        {
            case CControllerFullDynamicsRT::eGravityCompensation:
                pRobot->m_pEcatSensor[0]->LED_GREEN(bBlink);
                break;
            case CControllerFullDynamicsRT::eFullDynamics:
                pRobot->m_pEcatSensor[0]->LED_BLUE(bBlink);
                break;
            case CControllerFullDynamicsRT::eComputedTorque:

                pRobot->m_pEcatSensor[0]->LED_YELLOW(bBlink);
                break;
            case CControllerFullDynamicsRT::eAdaptiveControl:
            default:
                pRobot->m_pEcatSensor[0]->LED_RED(FALSE);
                break;
        }

        // if (++cycle % 1000 == 0) 
        // {
        //     RTTIME now = read_timer();
        //     if (bUseRTController) 
        //     {
        //         auto perf = pRTController->GetPerformance();
        //         DBG_LOG_TRACE("[RT-Controller] period=%lld µs max_calc=%llu µs avg_calc=%llu µs violations=%llu",
        //                      (long long)((now - tPrev) / 1000),
        //                      (unsigned long long)(perf.max_compute_time_ns / 1000),
        //                      (unsigned long long)(perf.avg_compute_time_ns / 1000),
        //                      (unsigned long long)perf.rt_violations);
        //         pRTController->ResetPerformance();
        //     } 
        //     else 
        //     {
        //         DBG_LOG_TRACE("[Fallback] period=%lld µs max_calc=%llu µs avg_calc=%llu µs",
        //                      (long long)((now - tPrev) / 1000),
        //                      (unsigned long long)(max_calc_ns / 1000),
        //                      (unsigned long long)(avg_calc_ns / 1000));
        //     }
        //     max_calc_ns = avg_calc_ns = samp = 0;
        //     tPrev = now;
        // }
    }
    DBG_LOG_WARN("[proc_main_control] TASK ENDED!");
}

void
proc_ethercat_control(void* apRobot)
{
    CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
    int nCnt = 0;

    RTTIME tmCurrent = 0, tmPrev = 0, tmEcat = 0;
    RTTIME tmSend = 0;
    RTTIME tmPeriod = 0, tmResp = 0;
    RTTIME tmMaxPeriod = 0, tmMaxResp = 0;

    DBG_LOG_INFO("(%s) EtherCAT Control Task Started!", "proc_ethercat_control");

    int nCleanUp = 0;

    while (TRUE)
    {
        wait_next_period(NULL);
        tmCurrent = read_timer();

        pRobot->m_bEcatOP = FALSE;

        pRobot->m_pcEcatMaster->ReadSlaves();
        pRobot->UpdateExtInterfaceData();
        
        if (TRUE == pRobot->m_pcEcatMaster->IsAllSlavesOp() && TRUE == pRobot->m_pcEcatMaster->IsMasterOp())
        {
            pRobot->m_bEcatOP = TRUE;
        }
        tmSend = read_timer();
        pRobot->m_pcEcatMaster->WriteSlaves(tmSend);
        tmEcat = read_timer();

        if (3000 < nCnt)
        {
            tmPeriod = tmCurrent - tmPrev;
            tmResp = tmEcat - tmCurrent;

            if (tmPeriod > tmMaxPeriod) tmMaxPeriod = tmPeriod;
            if (tmResp > tmMaxResp) tmMaxResp = tmResp;

            if (!(nCnt % 1000)) // print every second
            {
                // DBG_LOG_TRACE("Task Period:%d.%06d ms Task Ecat Resp:%d.%03d us", tmPeriod / 1000000, tmPeriod % 1000000, tmResp / 1000, tmResp % 1000);
                // DBG_LOG_TRACE("Max Period:%d.%06d ms Max Ecat Resp:%d.%03d us", tmMaxPeriod / 1000000, tmMaxPeriod % 1000000, tmMaxResp / 1000, tmMaxResp % 1000);
                // DBG_LOG_TRACE("Master State: %02x, Slave State: %02x, DomainStateIn: %02x, DomainStateOut: %02x, No. Of. Slaves: %d", pRobot->m_pcEcatMaster->GetMasterState(), pRobot->m_pcEcatMaster->GetSlaveState(), pRobot->m_pcEcatMaster->GetDomainState(), pRobot->m_pcEcatMaster->GetDomainState(eOutput), pRobot->m_pcEcatMaster->GetRespSlaveNo());
                // DBG_LOG_NOTHING("\n");
            }
        }
        tmPrev = tmCurrent;
        nCnt++;
        
        if (pRobot->CheckStopTask() == TRUE)
        {
            nCleanUp++;
            for (int nMotorCnt = 0; nMotorCnt < (int)pRobot->GetTotalAxis(); nMotorCnt++)
            {
                pRobot->m_pEcatAxis[nMotorCnt]->ServoOff();
            }
            if (nCleanUp > 1000)
                break;
        }
    }
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_ethercat_control");
}

void
proc_keyboard_control(void* apRobot)
{
	CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
    CControllerFullDynamicsRT* pRTController = pRobot->GetController();

    DBG_LOG_INFO("(%s) Keyboard Input Task Started!", "proc_keyboard_control");
    char cKeyPress = ' ';

    while (TRUE)
    {
        cKeyPress = (char)getche();
        pRobot->m_cKeyPress = cKeyPress;

        if ('q' == cKeyPress)
        {
            pRobot->StopTasks();
            break;
        }

        if ((cKeyPress == 'i' || cKeyPress == 'I')&& pRTController != NULL)
        {
            //pRTController->SetControlMode(CControllerFullDynamicsRT::eInverseKinematics);
            pRTController->m_bIkTrigger = TRUE;

        }
    }
    
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_keyboard_control");
}

void
proc_terminal_output(void* apRobot)
{
	CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
    CControllerFullDynamicsRT* pRTController = pRobot->GetController();
    (void)pRobot; // temporary: just to avoid warning

    DBG_LOG_INFO("(%s) Terminal Output Task Started!", "proc_terminal_output");

    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);

        // /* TODO: Last slave is Sensor, we must do this only for Axis */
        // for (int nMotCnt = 0; nMotCnt < pRobot->GetTotalAxis(); nMotCnt++)
        // {
        //     DBG_LOG_TRACE("[SLAVE %d]", nMotCnt);
        //     DBG_LOG_TRACE("TargetPos: %d, RawPos:%d", pRobot->m_pEcatAxis[nMotCnt]->GetTargetRawPos(), pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawPos());
        //     DBG_LOG_TRACE("RawVel:%d RawTor: %d",pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawVel(), pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawTor());
                                
        //     DBG_LOG_TRACE("ControlWord %04x, StatusWord %04x", pRobot->m_pEcatAxis[nMotCnt]->GetControlWord(), pRobot->m_pEcatAxis[nMotCnt]->GetStatusWord());
        //     DBG_LOG_TRACE("CurrentPos: %lf", pRobot->m_pEcatAxis[nMotCnt]->GetCurrentPosD());
        //     DBG_LOG_TRACE("HomePosition: %d StartPos:%d", pRobot->m_pEcatAxis[nMotCnt]->GetHomePosition(), pRobot->m_pEcatAxis[nMotCnt]->GetStartRawPos());
        //     DBG_LOG_TRACE("IsServoOn: %d\n", pRobot->m_pEcatAxis[nMotCnt]->IsServoOn());
        //     DBG_LOG_TRACE("DriveMode: %s\n", GetCIA402DriveMode(pRobot->m_pEcatAxis[nMotCnt]->GetDriveMode()).c_str());
        // }


                //==================================================================================
        // jieun
        // pRTController->ComputeTcpFK();
        // printf("X: %f, Y: %f, Z: %f\n", 
        // pRTController->tcpPose.m_position[0], 
        // pRTController->tcpPose.m_position[1], 
        // pRTController->tcpPose.m_position[2]);
        // printf("0: %f, 1: %f, 2: %f\n", 
        // pRTController->tcpPose.m_rotation(0,0), 
        // pRTController->tcpPose.m_rotation(0,1), 
        // pRTController->tcpPose.m_rotation(0,2));
        // printf("0: %f, 1: %f, 2: %f\n",
        // pRTController->tcpPose.m_rotation(1,0), 
        // pRTController->tcpPose.m_rotation(1,1), 
        // pRTController->tcpPose.m_rotation(1,2));
        // printf("0: %f, 1: %f, 2: %f\n",
        // pRTController->tcpPose.m_rotation(2,0), 
        // pRTController->tcpPose.m_rotation(2,1), 
        // pRTController->tcpPose.m_rotation(2,2));

        

        //==================================================================================          

        DBG_LOG_NOTHING("\n");
    }
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_terminal_output");
}

void
proc_logger(void* apRobot)
{

    return;
}


