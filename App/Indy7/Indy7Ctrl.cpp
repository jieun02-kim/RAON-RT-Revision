/*****************************************************************************
*	Name: RobotEx.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implemenation of the CRobotIndy7 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "CalibCapture.h"
#include <unistd.h>
#include <cmath>
#include <sys/stat.h>

CalibCapture s_calibCapture("/home/ubuntu/RAON-RT-Revision/App/CalibUtils/kv260");

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

    // [kv260-merge] Deterministic ready seed: bootstrap ONE posture from the
    // measured workspace box and verify its (r-gate-clamped) corners, so every
    // approach solves from the same seed no matter where the operator parked
    // the arm. Non-fatal: on failure approaches fall back to live-posture
    // seeding (the pre-2026-07-27 behavior).
    {
        using RigidBodyDynamics::Math::Vector3d;
        const double* pdBox  = CROS2PickBridge::DEF_BOX;
        const double  dRMax  = CROS2PickBridge::DEF_RMAX_XY;
        Vector3d vCenter(0.5 * (pdBox[0] + pdBox[1]),
                         0.5 * (pdBox[2] + pdBox[3]),
                         0.5 * (pdBox[4] + pdBox[5]));
        std::vector<Vector3d> vProbes;
        for (int ix = 0; ix < 2; ix++)
            for (int iy = 0; iy < 2; iy++)
                for (int iz = 0; iz < 2; iz++)
                {
                    double dX = pdBox[ix], dY = pdBox[2 + iy];
                    const double dR = std::sqrt(dX * dX + dY * dY);
                    if (dR > dRMax) { dX *= dRMax / dR; dY *= dRMax / dR; }
                    vProbes.push_back(Vector3d(dX, dY, pdBox[4 + iz]));
                }
        // A persisted operator posture (a previous session's recorded HOME)
        // anchors the bootstrap to a branch a human actually demonstrated —
        // the synthetic ladder alone once produced a contorted q_ready
        // (2026-07-27 field, J4 2.61 / J5 1.65 rad).
        RigidBodyDynamics::Math::VectorNd vqFileSeed;
        bool bFileSeed = false;
        {
            FILE* pSeedFile = fopen(CROS2PickBridge::ReadySeedPath(), "r");
            if (pSeedFile != NULL)
            {
                double adQ[8];
                int    nRead = 0;
                while (nRead < 8 && fscanf(pSeedFile, "%lf", &adQ[nRead]) == 1)
                    nRead++;
                fclose(pSeedFile);
                const int nDof = (int)m_pcConfigRobot->GetSystemConf().nRobotAxis;
                if (nRead >= nDof && nDof > 0)
                {
                    vqFileSeed.resize(nDof);
                    for (int i = 0; i < nDof; i++) vqFileSeed[i] = adQ[i];
                    bFileSeed = true;
                    DBG_LOG_INFO("[SEED] persisted operator posture found (%s)",
                                 CROS2PickBridge::ReadySeedPath());
                }
            }
        }
        m_pController->ComputeReadySeed(vProbes, vCenter,
                                        bFileSeed ? &vqFileSeed : NULL);
    }

    // [kv260-merge] Gate 2b — perception-pipeline bridge. Non-fatal: the
    // robot must stay usable standalone (no ROS2 pipeline running).
    m_pPickBridge = new CROS2PickBridge();
    if (FALSE == m_pPickBridge->Init())
    {
        DBG_LOG_WARN("(%s) ROS2 pick bridge unavailable — running without pipeline link", "CRobotIndy7");
        delete m_pPickBridge;
        m_pPickBridge = nullptr;
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
        delete[] m_pEcatAxis;
        m_pEcatAxis = nullptr;

        delete[] m_pEcatSensor;
        m_pEcatSensor = nullptr;

        // [kv260-merge] bridge teardown (joins its non-RT threads)
        if (m_pPickBridge != nullptr)
        {
            m_pPickBridge->DeInit();
            delete m_pPickBridge;
            m_pPickBridge = nullptr;
        }

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

            /* [kv260-merge] guide §14.2 #6: Init() never auto-selects the DC
             * reference clock — pin slave 0 explicitly. No-op while the cfg
             * leaves DC_SUPPORT off (author's proven SM-sync setup); becomes
             * active the moment DC_SUPPORT=1 is enabled per-axis. */
            if (nCnt == 0 && stSlaveInfo.bDCSupported)
            {
                if (FALSE == m_pEcatAxis[0]->GetEcatSlave().SetAsDCRef())
                    DBG_LOG_WARN("(%s) SetAsDCRef(slave 0) failed", "CRobotIndy7");
                else
                    DBG_LOG_INFO("(%s) Slave 0 pinned as DC reference clock", "CRobotIndy7");
            }
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
            break;   // [kv260-merge] was falling through into 'h'
        // [kv260-merge] Phase 4 — operator-gated servo arm/disarm (there was
        // NO keyboard servo-on path; the author ran AUTO_SERVO_ON=1 instead).
        // Interlock: CST drives enable at zero torque, so the controller must
        // already be enabled ('r') and in grav-comp before the brakes release.
        case 'h':
        case 'H':
            if (!m_bRTControllerEnabled ||
                m_pController->GetControlMode() !=
                    CControllerFullDynamicsRT::eGravityCompensation)
            {
                printf("[ARM] refused — press 'r' (controller) then 'g' (grav-comp) first\n");
                break;
            }
            DBG_LOG_WARN(">>> SERVO ON (all axes, CST) — brakes release, grav-comp holds");
            for (int nCnt = 0; nCnt < (int)GetTotalAxis(); nCnt++)
                m_pEcatAxis[nCnt]->ServoOn(CIA402_CYCLIC_TORQUE);
            break;
        case 'j':
        case 'J':
            DBG_LOG_WARN(">>> SERVO OFF (all axes) — brakes engage");
            for (int nCnt = 0; nCnt < (int)GetTotalAxis(); nCnt++)
                m_pEcatAxis[nCnt]->ServoOff();
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
        case 'l':
        case 'L':
            DBG_LOG_INFO(">>> Logging triggered: collecting TCP trajectory");
            m_bLogTrigger = TRUE;
            break;
        case 'i':
        case 'I':
            m_pEcatSensor[0]->LED_RED(TRUE);
            DBG_LOG_INFO(">>> RT Controller: Jacobian IK Mode");
            SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics);
            m_pController->m_bIkTrigger = TRUE;
            break;
        case 'm':
        case 'M':
            m_pEcatSensor[0]->LED_RED(TRUE);
            DBG_LOG_INFO(">>> RT Controller: RBDL IK (6DOF) Mode");
            if (m_pController->SetTargetPose(m_Pose))
            {
                SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
                m_bLogTrigger = TRUE;
            }
            else
                DBG_LOG_ERROR(">>> SetTargetPose FAILED — press 's' first to set target pose");
            break;
        case 'n':
        case 'N':
            if (m_bRectTrigger.load()) {
                m_bRectTrigger = false;
                m_eRectState   = eRECT_IDLE;
                m_bLogTrigger  = FALSE;
                SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
                DBG_LOG_INFO(">>> Rectangle Mode STOP");
            } else {
                // gravity comp 먼저: RT 루프가 m_traj를 소비 중이면 중단
                SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);

                // n 시작 순간 현재 TCP 위치를 중심으로 사각형 계산
                m_pController->GetCurrentPose(m_Pose);
                const double d = 0.25;
                const double offsets[4][2] = {{+d,+d},{-d,+d},{-d,-d},{+d,-d}};
                for (int i = 0; i < 4; ++i) {
                    m_rectCorners[i].m_position[0] = m_Pose.m_position[0];
                    m_rectCorners[i].m_position[1] = m_Pose.m_position[1] + offsets[i][0];
                    m_rectCorners[i].m_position[2] = m_Pose.m_position[2] + offsets[i][1];
                    m_rectCorners[i].m_rotation    = m_Pose.m_rotation;
                }

                // 4개 꼭짓점 IK 미리 계산 (위치만, 키보드 스레드 non-RT에서 안전)
                bool all_ok = true;
                for (int i = 0; i < 4; ++i) {
                    if (!m_pController->SetTargetPosePositionOnly(m_rectCorners[i])) {
                        DBG_LOG_ERROR(">>> [RECT] IK failed for corner %d — abort", i+1);
                        all_ok = false;
                        break;
                    }
                    m_rectJointTargets[i] = m_pController->GetTrajGoal();
                }
                if (!all_ok) break;

                // 첫 번째 꼭짓점으로 1.5s 궤적 시작
                m_nRectCornerIdx = 0;
                m_nRectWaitCount = 0;
                m_eRectState     = eRECT_TO_CORNER;
                m_pEcatSensor[0]->LED_RED(TRUE);
                m_pController->StartJointTrajectory(m_rectJointTargets[0], 2.0);
                SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
                m_bRectTrigger = true;
                DBG_LOG_INFO(">>> Rectangle Mode START: Center X=%.4f Y=%.4f Z=%.4f",
                    m_Pose.m_position[0], m_Pose.m_position[1], m_Pose.m_position[2]);
            }
            break;


        case 'q':
        case 'Q':
        {
            DBG_LOG_INFO(">>> ISO Cube Hardware Test Start");
            const CControllerFullDynamicsRT::Pose& cur = m_pController->GetTcpPose();
            const double cx = cur.m_position[0] + 0.02;
            const double cy = cur.m_position[1];
            const double cz = cur.m_position[2];
            const double a = 0.100;  // ISO 9283: 0.8*(L/2), L=0.25m
            const double offsets[5][3] = {{0,0,0},{a,a,-a},{-a,a,-a},{-a,-a,a},{a,-a,a}};
            for (int i = 0; i < 5; ++i) {
                m_isoTargets[i].m_position[0] = cx + offsets[i][0];
                m_isoTargets[i].m_position[1] = cy + offsets[i][1];
                m_isoTargets[i].m_position[2] = cz + offsets[i][2];
                m_isoTargets[i].m_rotation    = cur.m_rotation;
            }
            m_isoClearance.m_position[0] = cx;
            m_isoClearance.m_position[1] = cy;
            m_isoClearance.m_position[2] = cz + 0.08;
            m_isoClearance.m_rotation    = cur.m_rotation;
            DBG_LOG_INFO("[ISO-HW] Center: X=%.4f Y=%.4f Z=%.4f", cx, cy, cz);

            SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
            m_nISOPointIdx  = 0;
            m_nISOCycle     = 0;
            m_bISOCmdSent   = false;
            m_eISOHWState   = eISO_HW_TO_TARGET;
            m_bIsoHWTrigger = true;
            break;
        }
        case 'd':
        case 'D':
            DBG_LOG_INFO(">>> Log Distance Error");
            m_pController->LogDistanceError(m_Pose);
            break;
        case 's':
        case 'S':
            m_pController->GetCurrentPose(m_Pose);
            SaveRobotPose();
            s_calibCapture.Capture(m_Pose);
            printf("[TCP Pose]\n");
            printf("  t = [%.6f, %.6f, %.6f]\n",
                   m_Pose.m_position[0], m_Pose.m_position[1], m_Pose.m_position[2]);
            printf("  R = [%.6f, %.6f, %.6f]\n"
                   "      [%.6f, %.6f, %.6f]\n"
                   "      [%.6f, %.6f, %.6f]\n",
                   m_Pose.m_rotation(0,0), m_Pose.m_rotation(0,1), m_Pose.m_rotation(0,2),
                   m_Pose.m_rotation(1,0), m_Pose.m_rotation(1,1), m_Pose.m_rotation(1,2),
                   m_Pose.m_rotation(2,0), m_Pose.m_rotation(2,1), m_Pose.m_rotation(2,2));
            break;
        case 'a':
        case 'A':
        {
            DBG_LOG_INFO(">>> RT Controller: SetTargetPose (A key)");
            CControllerFullDynamicsRT::Pose ref_pose;
            ref_pose.m_position[0] = -0.1806;
            ref_pose.m_position[1] = -0.1808;
            ref_pose.m_position[2] = 0.9307;
            ref_pose.m_rotation_rpy[0] = 0.0872;
            ref_pose.m_rotation_rpy[1] = -0.0860;
            ref_pose.m_rotation_rpy[2] = 1.5670;
            ref_pose.m_rotation = CControllerFullDynamicsRT::RPYToRot(
                ref_pose.m_rotation_rpy[0],
                ref_pose.m_rotation_rpy[1],
                ref_pose.m_rotation_rpy[2]);

            if (m_pController->SetTargetPose(ref_pose))
            {
                SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
                m_bLogTrigger = TRUE;
                StartRefine(ref_pose);   // [kv260-merge] close the CTC droop
            }
            else
                DBG_LOG_ERROR(">>> SetTargetPose FAILED — IK did not converge");

            break;
        }

        //==============================================================
        // [kv260-merge] Gate 2b — pipeline pick flow ('p' → digit → 'v').
        // Handlers only flip bridge atomics; all ROS2 work runs on the
        // bridge's own non-RT threads.
        case 'p':
        case 'P':
            if (m_pPickBridge != nullptr)
                m_pPickBridge->RequestMenu();
            else
                printf("[PickBridge] not available (init failed at startup)\n");
            break;
        case 'v':
        case 'V':
            if (m_pPickBridge != nullptr)
                m_pPickBridge->RequestApproach();
            else
                printf("[PickBridge] not available (init failed at startup)\n");
            break;
        case 'k':
        case 'K':
            if (m_pController != NULL)
            {
                const bool bEn = !m_pController->m_bStickyEnable.load();
                m_pController->m_bStickyEnable = bEn;
                DBG_LOG_INFO("[STICKY] grav-comp hold %s (dead-band %.0f mrad)",
                             bEn ? "ON" : "OFF — pure float",
                             CControllerFullDynamicsRT::HOLD_DB_RAD * 1e3);
            }
            break;
        case 'b':
        case 'B':
        {
            // [kv260-merge] no user HOME recorded → fall back to the computed
            // q_ready (same posture every approach solves from)
            const bool bUseReady = (!m_bHomeSet && m_pController != NULL &&
                                    m_pController->HasReadySeed());
            if (!m_bHomeSet && !bUseReady)
            {
                DBG_LOG_WARN("[HOME] no home recorded — 'p' menu, last entry");
                break;
            }
            if (!m_bRTControllerEnabled || m_pController == NULL ||
                !m_pController->IsEnabled())
            {
                DBG_LOG_WARN("[HOME] controller not enabled ('r' first)");
                break;
            }
            {
                bool bServoAll = true;
                for (unsigned int i = 0; i < GetTotalAxis(); i++)
                    if (!m_pEcatAxis[i]->IsServoOn()) { bServoAll = false; break; }
                if (!bServoAll)
                {
                    DBG_LOG_WARN("[HOME] servo OFF — press 'h' first");
                    break;
                }
            }
            // pure joint-space return: no IK, so no branch risk — just cap
            // the quintic's peak speed like E13 and go.
            m_bRefineActive  = false;
            m_eApproachState = eAPPROACH_IDLE;
            const RigidBodyDynamics::Math::VectorNd& vqTarget =
                bUseReady ? m_pController->GetReadyQ() : m_vHomeQ;
            double dDqMax = 0.0;
            for (size_t i = 0; i < m_vCurrentPos.size() &&
                               i < (size_t)vqTarget.size(); i++)
            {
                const double dDq = std::fabs(vqTarget[i] - m_vCurrentPos[i]);
                if (dDq > dDqMax) dDqMax = dDq;
            }
            const double dT = CControllerFullDynamicsRT::ScaledTrajTime(
                dDqMax, APPROACH_DURATION_S);
            SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
            m_pController->StartJointTrajectory(vqTarget, dT);
            SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
            DBG_LOG_INFO("[HOME] returning%s, T=%.1f s (dq_max %.2f rad)",
                         bUseReady ? " to computed q_ready (no user HOME)" : "",
                         dT, dDqMax);
            break;
        }
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            if (m_pPickBridge != nullptr)
                m_pPickBridge->FeedDigit(m_cKeyPress);
            break;
        //==============================================================

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

// [kv260-merge] arm the refinement SM (see header). Rotation is irrelevant:
// refinement passes use position-only IK.
void
CRobotIndy7::StartRefine(const CControllerFullDynamicsRT::Pose& astDesired)
{
    m_vRefineDesired = astDesired.m_position;
    m_stRefineCmd    = astDesired;   // bias accumulates on top of m_position;
                                     // m_rotation rides along for the IK
                                     // orientation constraint (E13)
    m_nRefineIter = 0;
    m_nRefineSettleCnt = 0;
    m_bRefineActive = true;
}

// [kv260-merge] One ready-seed approach attempt (rationale in the header).
// Called from the main-control SM only. Returns TRUE when a motion started
// (direct goal leg or the staging leg); FALSE means nothing moved.
BOOL
CRobotIndy7::TryReadyApproach(const CROS2PickBridge::Goal& astGoal,
                              BOOL abAllowStage)
{
    CControllerFullDynamicsRT* pC = m_pController;

    CControllerFullDynamicsRT::Pose stTarget;
    pC->GetCurrentPose(stTarget);            // R rides along for refine/logs
    stTarget.m_position[0] = astGoal.dX;
    stTarget.m_position[1] = astGoal.dY;
    stTarget.m_position[2] = astGoal.dZ;

    RigidBodyDynamics::Math::VectorNd vqSol;
    double dErrM = 0.0, dDqMax = 0.0;
    int    nAxis = -1;
    if (!pC->SolveReadyIK(stTarget.m_position, vqSol, dErrM, dDqMax, nAxis))
    {
        DBG_LOG_ERROR("[APPROACH] no reachable solution for (%.3f, %.3f, %.3f)"
                      " even from q_ready — move the object closer",
                      astGoal.dX, astGoal.dY, astGoal.dZ);
        return FALSE;
    }

    if (dDqMax <= CControllerFullDynamicsRT::IK_DQ_MAX_RAD)
    {
        const double dT = CControllerFullDynamicsRT::ScaledTrajTime(
            dDqMax, APPROACH_DURATION_S);
        // grav-comp bracket: the IK6dof consumer must not see a half-built
        // trajectory (same pattern as 'b')
        SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
        pC->StartJointTrajectory(vqSol, dT);
        SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
        m_eApproachState = eAPPROACH_MOVING;
        StartRefine(stTarget);
        DBG_LOG_INFO("[APPROACH] %s → (%.3f, %.3f, %.3f)  T=%.1f s  "
                     "[ready-seed direct, dq %.2f rad J%d]  "
                     "[std mm %.1f/%.1f/%.1f n=%d]",
                     astGoal.szClass, astGoal.dX, astGoal.dY, astGoal.dZ, dT,
                     dDqMax, nAxis,
                     astGoal.dStdMM[0], astGoal.dStdMM[1], astGoal.dStdMM[2],
                     astGoal.nSamples);
        return TRUE;
    }

    if (!abAllowStage)
    {
        DBG_LOG_ERROR("[APPROACH] solution still %.2f rad away (J%d) after "
                      "staging — holding at q_ready", dDqMax, nAxis);
        return FALSE;
    }

    // stage: 'b'-class joint move to q_ready, goal leg fires on settle
    const RigidBodyDynamics::Math::VectorNd& vqReady = pC->GetReadyQ();
    double dDqHome = 0.0;
    for (size_t i = 0; i < m_vCurrentPos.size() &&
                       i < (size_t)vqReady.size(); i++)
    {
        const double dDq = std::fabs(vqReady[i] - m_vCurrentPos[i]);
        if (dDq > dDqHome) dDqHome = dDq;
    }
    const double dT = CControllerFullDynamicsRT::ScaledTrajTime(
        dDqHome, APPROACH_DURATION_S);
    m_stStagedGoal    = astGoal;
    m_nStageSettleCnt = 0;
    SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
    pC->StartJointTrajectory(vqReady, dT);
    SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
    m_eApproachState = eAPPROACH_STAGING;
    DBG_LOG_INFO("[APPROACH] %s: solution %.2f rad away (J%d) — staging via "
                 "q_ready first (T=%.1f s), goal leg follows",
                 astGoal.szClass, dDqMax, nAxis, dT);
    return TRUE;
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
    // [kv260-merge] availability only — the enable state is re-read EVERY
    // cycle below. The upstream one-shot snapshot froze IsEnabled() at task
    // start, which silently killed the runtime 'r' key whenever
    // ENABLE_CONTROLLER_AT_STARTUP=0 (our safe bring-up cfg).
    const BOOL bUseRTController = (pRTController != NULL);
    DBG_LOG_INFO("(proc_main_control) RT Controller %s (enable at runtime: 'r')",
                 bUseRTController ? "available" : "NOT available - fallback");

    RTTIME tPrev = read_timer();
    uint64_t max_calc_ns = 0, avg_calc_ns = 0, samp = 0;
    long cycle = 0;


    while (!pRobot->CheckStopTask())
    {
        wait_next_period(nullptr);
        if (!pRobot->m_bEcatOP) continue;
        
        pRobot->DoInput();

        // [kv260-merge] live enable state — makes 'r' actually work at runtime
        const BOOL bCtrlOn = bUseRTController && pRTController->IsEnabled();

        // Read current joint states
        for (int nCnt = 0; nCnt < (int)udof; ++nCnt)
        {
            auto ax = static_cast<CAxisNRMKCore*>(pRobot->m_pEcatAxis[nCnt]);

            if (bCtrlOn) {
                // Update RT controller vectors
                pRobot->m_vCurrentPos[nCnt] = ax->GetCurrentPos();
                pRobot->m_vCurrentVel[nCnt] = ax->GetCurrentVel();
                pRobot->m_vCurrentTor[nCnt] = ax->GetCurrentTor();
            }
        }

        // Compute control torques (Update 먼저 → tcpPose 갱신)
        if (bCtrlOn)
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

        // [kv260-merge] ViSP visual-servo goal injection removed here.
        // The ROS2 bridge (/pick_target_base) will provide goal poses instead;
        // its safety-gated injection lands with the bridge wiring.

        //======================================================================
        // TCP Trajectory Logging
        if (bCtrlOn && pRobot->m_bLogTrigger.load(std::memory_order_acquire))
        {
            ST_LOG_ENTRY entry{};
            entry.timestamp_ns = (uint64_t)read_timer();
            pRTController->ComputeTcpFK();
            const CControllerFullDynamicsRT::Pose& tcp = pRTController->GetTcpPose();
            entry.tcp_x = tcp.m_position[0];
            entry.tcp_y = tcp.m_position[1];
            entry.tcp_z = tcp.m_position[2];
            const auto& R    = tcp.m_rotation;
            entry.tcp_roll   = atan2(R(2,1), R(2,2));
            entry.tcp_pitch  = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
            entry.tcp_yaw    = atan2(R(1,0), R(0,0));
            const auto& goal = pRTController->goal_tcpPose;
            entry.goal_x     = goal.m_position[0];
            entry.goal_y     = goal.m_position[1];
            entry.goal_z     = goal.m_position[2];
            const auto& Rg   = goal.m_rotation;
            entry.goal_roll  = atan2(Rg(2,1), Rg(2,2));
            entry.goal_pitch = atan2(-Rg(2,0), sqrt(Rg(2,1)*Rg(2,1) + Rg(2,2)*Rg(2,2)));
            entry.goal_yaw   = atan2(Rg(1,0), Rg(0,0));
            for (int i = 0; i < 6; ++i) {
                entry.q[i]     = pRobot->m_vCurrentPos[i];
                entry.qd[i]    = pRobot->m_vCurrentVel[i];
                entry.tau[i]   = pRobot->m_vOutputTorque[i];
                entry.q_ref[i] = pRTController->GetQRef()[i];
            }
            entry.ctrl_mode = (uint8_t)pRTController->GetControlMode();
            pRobot->m_logBuffer.push(entry);
        }
        //======================================================================
        // ISO Hardware Test state machine
        if (pRobot->m_bIsoHWTrigger.load()) {
            switch (pRobot->m_eISOHWState) {
                case CRobotIndy7::eISO_HW_TO_TARGET:
                    if (!pRobot->m_bISOCmdSent) {
                        pRTController->goal_tcpPose = pRobot->m_isoTargets[pRobot->m_nISOPointIdx];
                        pRTController->m_bIkTrigger = true;
                        pRobot->m_bISOCmdSent = true;
                        DBG_LOG_INFO("[ISO-HW] Cycle %d/10  P%d  moving...",
                            pRobot->m_nISOCycle+1, pRobot->m_nISOPointIdx+1);
                    } else if (pRTController->m_bVerifyDone) {
                        const auto& tcp = pRTController->GetTcpPose();
                        pRobot->m_isoHWRecords[pRobot->m_nISOPointIdx][pRobot->m_nISOCycle][0] = tcp.m_position[0];
                        pRobot->m_isoHWRecords[pRobot->m_nISOPointIdx][pRobot->m_nISOCycle][1] = tcp.m_position[1];
                        pRobot->m_isoHWRecords[pRobot->m_nISOPointIdx][pRobot->m_nISOCycle][2] = tcp.m_position[2];
                        pRobot->m_bISOCmdSent = false;
                        pRobot->m_eISOHWState = CRobotIndy7::eISO_HW_TO_CLEARANCE;
                    }
                    break;

                case CRobotIndy7::eISO_HW_TO_CLEARANCE:
                    if (!pRobot->m_bISOCmdSent) {
                        pRTController->goal_tcpPose = pRobot->m_isoClearance;
                        pRTController->m_bIkTrigger = true;
                        pRobot->m_bISOCmdSent = true;
                    } else if (pRTController->m_bVerifyDone) {
                        pRobot->m_bISOCmdSent = false;
                        pRobot->m_nISOPointIdx++;
                        if (pRobot->m_nISOPointIdx >= 5) {
                            pRobot->m_nISOPointIdx = 0;
                            pRobot->m_nISOCycle++;
                            if (pRobot->m_nISOCycle >= 10)
                                pRobot->m_eISOHWState = CRobotIndy7::eISO_HW_DONE;
                            else
                                pRobot->m_eISOHWState = CRobotIndy7::eISO_HW_TO_TARGET;
                        } else {
                            pRobot->m_eISOHWState = CRobotIndy7::eISO_HW_TO_TARGET;
                        }
                    }
                    break;

                case CRobotIndy7::eISO_HW_DONE:
                    pRobot->m_bIsoHWTrigger = false;
                    pRobot->SaveISOHWResults();
                    pRobot->SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
                    pRobot->m_eISOHWState = CRobotIndy7::eISO_HW_IDLE;
                    break;

                default: break;
            }
        }
        //======================================================================
        // Rectangle Motion state machine — 'n' key (time-based, 2.0s per corner)
        if (pRobot->m_bRectTrigger.load() &&
            pRTController->GetControlMode() == CControllerFullDynamicsRT::eInverseKinematics_6dof) {
            pRobot->m_nRectWaitCount++;
            if (pRobot->m_nRectWaitCount == 1230) {  // 1.23s 시점에 로깅 시작
                pRobot->m_bLogTrigger = TRUE;
                DBG_LOG_INFO("[RECT] Logging triggered at count 1230");
            }
            if (pRobot->m_nRectWaitCount >= 2000) {  // 2.0s at 1ms period
                pRobot->m_nRectWaitCount = 0;
                pRobot->m_nRectCornerIdx = (pRobot->m_nRectCornerIdx + 1) % 4;
                pRTController->StartJointTrajectory(
                    pRobot->m_rectJointTargets[pRobot->m_nRectCornerIdx], 2.0);
                DBG_LOG_INFO("[RECT] → Corner %d  Y=%.4f Z=%.4f",
                    pRobot->m_nRectCornerIdx + 1,
                    pRobot->m_rectCorners[pRobot->m_nRectCornerIdx].m_position[1],
                    pRobot->m_rectCorners[pRobot->m_nRectCornerIdx].m_position[2]);
            }
        }
        //======================================================================
        // [kv260-merge] HOME record — menu entry N+1 asked for a snapshot of
        // the CURRENT joints (works in any mode; measured q is always live).
        if (pRobot->m_pPickBridge != nullptr &&
            pRobot->m_pPickBridge->PopHomeRecord())
        {
            const size_t uDof = pRobot->m_vCurrentPos.size();
            pRobot->m_vHomeQ.resize(uDof);
            for (size_t i = 0; i < uDof; i++)
                pRobot->m_vHomeQ[i] = pRobot->m_vCurrentPos[i];
            pRobot->m_bHomeSet = true;
            DBG_LOG_INFO("[HOME] recorded q = [%.2f %.2f %.2f %.2f %.2f %.2f] rad"
                         " — 'b' returns here",
                         pRobot->m_vHomeQ[0], pRobot->m_vHomeQ[1],
                         pRobot->m_vHomeQ[2], pRobot->m_vHomeQ[3],
                         pRobot->m_vHomeQ[4], pRobot->m_vHomeQ[5]);
            // [kv260-merge] the recorded posture IS the best IK anchor — a
            // branch demonstrated by a human beats any synthetic bootstrap.
            // Re-base q_ready now and persist for the next boot (file IO is
            // on the bridge worker; here only a wait-free mailbox fill).
            if (pRobot->m_pController != NULL)
                pRobot->m_pController->SetReadyAnchor(pRobot->m_vHomeQ);
            {
                double adQ[8];
                int    nN = (int)uDof < 8 ? (int)uDof : 8;
                for (int i = 0; i < nN; i++) adQ[i] = pRobot->m_vHomeQ[i];
                pRobot->m_pPickBridge->PersistReadySeed(adQ, nN);
            }
        }
        // [kv260-merge] amortized anchor-coverage verification: one IK solve
        // per cycle for a few cycles right after a HOME record (same per-cycle
        // cost as the field-proven 'v' solve; a 9-solve burst would stall the
        // loop). No-op while idle.
        if (pRobot->m_pController != NULL)
            pRobot->m_pController->TickAnchorVerify();
        //======================================================================
        // [kv260-merge] Vision approach — consume a gated goal from the ROS2
        // bridge ('p' → digit → 'v'). Bridge already ran the N-frame std gate
        // and the workspace-box check; here we mirror the proven 'n'-key
        // sequence: position-only IK → quintic joint trajectory → IK6dof+CTC.
        if (pRobot->m_pPickBridge != nullptr && bCtrlOn)
        {
            if (pRobot->m_eApproachState == CRobotIndy7::eAPPROACH_MOVING)
            {
                // refinement runs between the first trajectory and "done"
                if (pRTController->IsTrajectoryRefDone() && !pRobot->m_bRefineActive)
                {
                    pRobot->m_eApproachState = CRobotIndy7::eAPPROACH_IDLE;
                    const auto& tcp = pRTController->GetTcpPose();
                    DBG_LOG_INFO("[APPROACH] done — holding at (%.3f, %.3f, %.3f); 'g'=grav-comp",
                                 tcp.m_position[0], tcp.m_position[1], tcp.m_position[2]);
                }
            }
            else if (pRobot->m_eApproachState == CRobotIndy7::eAPPROACH_STAGING)
            {
                // [kv260-merge] staged approach leg 2 — fire ONLY if the run
                // is still exactly as we left it (IK6dof + servo on). Any
                // operator intervention cancels the stashed goal instead of
                // deferring it (a deferred motion is the j→v surprise again).
                bool bServoAll = true;
                for (unsigned int i = 0; i < pRobot->GetTotalAxis(); i++)
                    if (!pRobot->m_pEcatAxis[i]->IsServoOn()) { bServoAll = false; break; }
                if (pRTController->GetControlMode() !=
                        CControllerFullDynamicsRT::eInverseKinematics_6dof || !bServoAll)
                {
                    pRobot->m_eApproachState = CRobotIndy7::eAPPROACH_IDLE;
                    DBG_LOG_WARN("[APPROACH] staging cancelled (mode change / "
                                 "servo off) — goal dropped");
                }
                else if (!pRTController->IsTrajectoryRefDone())
                {
                    pRobot->m_nStageSettleCnt = 0;
                }
                else if ([&]{   // wait until the arm has actually stopped
                    double dVelSq = 0.0;
                    for (int nCnt = 0; nCnt < (int)udof; nCnt++)
                        dVelSq += pRobot->m_vCurrentVel[nCnt] * pRobot->m_vCurrentVel[nCnt];
                    if (dVelSq > CRobotIndy7::STAGE_VEL_EPS * CRobotIndy7::STAGE_VEL_EPS)
                    {
                        pRobot->m_nStageSettleCnt = 0;
                        return false;
                    }
                    return ++pRobot->m_nStageSettleCnt >= CRobotIndy7::STAGE_SETTLE_CYC;
                }())
                {
                    // arrived at q_ready — the goal leg is near by init
                    // verification, so no further staging is allowed
                    pRobot->m_nStageSettleCnt = 0;
                    pRobot->m_eApproachState  = CRobotIndy7::eAPPROACH_IDLE;
                    if (!pRobot->TryReadyApproach(pRobot->m_stStagedGoal, FALSE))
                        DBG_LOG_WARN("[APPROACH] staged goal leg did not start"
                                     " — holding at q_ready; 'g'=grav-comp");
                }
            }
            else if (!pRobot->m_bIsoHWTrigger.load() && !pRobot->m_bRectTrigger.load() &&
                     pRobot->m_bRTControllerEnabled)
            {
                CROS2PickBridge::Goal stGoal;
                if (pRobot->m_pPickBridge->PopGoal(stGoal))
                {
                    // Brakes engaged? The whole approach+refine would then run
                    // against a frozen arm and read as a huge "droop"
                    // (operator hit this with j->v on 2026-07-27). Discard —
                    // deferring the goal would fire surprise motion at 'h'.
                    bool bServo = true;
                    for (unsigned int i = 0; i < pRobot->GetTotalAxis(); i++)
                        if (!pRobot->m_pEcatAxis[i]->IsServoOn()) { bServo = false; break; }
                    if (!bServo)
                    {
                        DBG_LOG_ERROR("[APPROACH] goal discarded — servo OFF. "
                                      "Press 'h' (servo on) first, then 'v' again.");
                    }
                    else if (pRTController->HasReadySeed())
                    {
                        // [kv260-merge] deterministic path: solve from
                        // q_ready, then direct or staged (TryReadyApproach).
                        // If nothing started, the arm floats in grav-comp.
                        pRobot->SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
                        (void)pRobot->TryReadyApproach(stGoal, TRUE);
                    }
                    else
                    {
                    // legacy live-posture seeding (ready seed unavailable)
                    pRobot->SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
                    pRobot->m_pController->GetCurrentPose(pRobot->m_Pose);
                    CControllerFullDynamicsRT::Pose stTarget = pRobot->m_Pose;  // keep current R
                    stTarget.m_position[0] = stGoal.dX;
                    stTarget.m_position[1] = stGoal.dY;
                    stTarget.m_position[2] = stGoal.dZ;
                    // E13: SetTargetPosePositionOnly now starts the (possibly
                    // T-stretched) trajectory itself — re-issuing
                    // StartJointTrajectory here would clobber the scaled T.
                    pRTController->SetTrajectoryDuration(CRobotIndy7::APPROACH_DURATION_S);
                    if (pRTController->SetTargetPosePositionOnly(stTarget))
                    {
                        pRobot->SetControllerMode(CControllerFullDynamicsRT::eInverseKinematics_6dof);
                        pRobot->m_eApproachState = CRobotIndy7::eAPPROACH_MOVING;
                        pRobot->StartRefine(stTarget);
                        DBG_LOG_INFO("[APPROACH] %s → (%.3f, %.3f, %.3f)  T=%.1f s  [std mm %.1f/%.1f/%.1f n=%d]",
                                     stGoal.szClass, stGoal.dX, stGoal.dY, stGoal.dZ,
                                     pRTController->GetTrajT(),
                                     stGoal.dStdMM[0], stGoal.dStdMM[1], stGoal.dStdMM[2],
                                     stGoal.nSamples);
                    }
                    else
                    {
                        DBG_LOG_ERROR("[APPROACH] IK failed for (%.3f, %.3f, %.3f) — staying in grav-comp",
                                      stGoal.dX, stGoal.dY, stGoal.dZ);
                    }
                    }  // servo-on legacy path
                }
            }
        }
        //======================================================================
        // [kv260-merge] Position-refinement state machine (see Indy7Ctrl.h).
        // Waits REFINE_SETTLE_CYC after each trajectory, measures the FK
        // error vs the desired position, folds it into the commanded target
        // as an accumulated bias and re-runs IK — until <REFINE_TOL_M or
        // REFINE_MAX_ITER. A mode change ('g'/'j'/E-stop paths) aborts it.
        if (pRobot->m_bRefineActive)
        {
            if (!bCtrlOn || pRTController->GetControlMode() !=
                                CControllerFullDynamicsRT::eInverseKinematics_6dof)
            {
                pRobot->m_bRefineActive = false;
                DBG_LOG_WARN("[REFINE] aborted (controller/mode change)");
            }
            else if (!pRTController->IsTrajectoryRefDone())
            {
                pRobot->m_nRefineSettleCnt = 0;
            }
            else if ([&]{   // measure only once the arm has actually stopped
                double dVelSq = 0.0;
                for (int nCnt = 0; nCnt < (int)udof; nCnt++)
                    dVelSq += pRobot->m_vCurrentVel[nCnt] * pRobot->m_vCurrentVel[nCnt];
                if (dVelSq > CRobotIndy7::REFINE_VEL_EPS * CRobotIndy7::REFINE_VEL_EPS)
                {
                    pRobot->m_nRefineSettleCnt = 0;
                    return false;
                }
                return ++pRobot->m_nRefineSettleCnt >= CRobotIndy7::REFINE_SETTLE_CYC;
            }())
            {
                pRobot->m_nRefineSettleCnt = 0;
                pRTController->ComputeTcpFK();
                const RigidBodyDynamics::Math::Vector3d vErr =
                    pRobot->m_vRefineDesired - pRTController->GetTcpPose().m_position;
                const double dErrMM = vErr.norm() * 1e3;

                if (vErr.norm() <= CRobotIndy7::REFINE_TOL_M)
                {
                    pRobot->m_bRefineActive = false;
                    DBG_LOG_INFO("[REFINE] done — err %.1f mm after %d pass(es)",
                                 dErrMM, pRobot->m_nRefineIter);
                }
                else if (pRobot->m_nRefineIter >= CRobotIndy7::REFINE_MAX_ITER)
                {
                    pRobot->m_bRefineActive = false;
                    DBG_LOG_WARN("[REFINE] stop at max iters — residual %.1f mm", dErrMM);
                }
                else
                {
                    pRobot->m_stRefineCmd.m_position += CRobotIndy7::REFINE_DAMPING * vErr;   // damped bias
                    // E13: trajectory (incl. any T stretch) starts inside.
                    // Ori weight 0: refine biases are small LOCAL moves — the
                    // soft keep-R trade stalls on large biases (regression).
                    pRTController->SetTrajectoryDuration(CRobotIndy7::REFINE_TRAJ_T_S);
                    if (pRTController->SetTargetPosePositionOnly(pRobot->m_stRefineCmd, 0.0))
                    {
                        pRobot->m_nRefineIter++;
                        DBG_LOG_INFO("[REFINE] pass %d — err %.1f mm, re-targeting",
                                     pRobot->m_nRefineIter, dErrMM);
                    }
                    else
                    {
                        pRobot->m_bRefineActive = false;
                        DBG_LOG_ERROR("[REFINE] IK failed on bias target — stop");
                    }
                }
            }
        }
        //======================================================================

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
                pRobot->m_pEcatSensor[0]->LED_RED(bBlink);
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

        // 화살표 키 등 이스케이프 시퀀스(\033[X) 무시
        if (cKeyPress == '\033') {
            getche(); // '['
            getche(); // 방향 문자 (A/B/C/D)
            continue;
        }

        // [kv260-merge] newline never carries a command; without this a piped
        // "p\n" overwrites m_cKeyPress with '\n' before the 1 kHz consumer
        // samples it (real-terminal Enter would clobber a pending key too).
        if (cKeyPress == '\n' || cKeyPress == '\r')
            continue;

        if ('q' == cKeyPress)
        {
            pRobot->StopTasks();
            break;
        }

        if ((cKeyPress == 'z' || cKeyPress == 'Z') && pRTController != NULL)
        {
            DBG_LOG_INFO(">>> ISO Cube IK Validation (software only)");
            if (!pRTController->RunISOCubeIKValidation())
                DBG_LOG_ERROR(">>> ISO Cube IK Validation failed");
            continue;
        }

        pRobot->m_cKeyPress = cKeyPress;

        // if ((cKeyPress == 'i' || cKeyPress == 'I')&& pRTController != NULL)
        // {
        //     //pRTController->SetControlMode(CControllerFullDynamicsRT::eInverseKinematics);
        //     pRTController->m_bIkTrigger = TRUE;

        // }
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

        // DBG_LOG_NOTHING("\n");
    }
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_terminal_output");
}

FILE* make_csv(CRobotIndy7* pRobot);

void
proc_logger(void* apRobot)
{
    CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
    ST_LOG_ENTRY entry;
    int nCount = 0;

    DBG_LOG_INFO("(proc_logger) Task Started!");

    while (!pRobot->CheckStopTask())
    {
        // 트리거 대기
        if (!pRobot->m_bLogTrigger.load(std::memory_order_acquire)) {
            usleep(10000);
            continue;
        }

        nCount = 0;

        DBG_LOG_INFO("(proc_logger) Trigger received. Flushing old buffer...");
        while (pRobot->m_logBuffer.pop(entry)) {}

        DBG_LOG_INFO("(proc_logger) Collecting 24 seconds of data...");
        sleep(24);

        pRobot->m_bLogTrigger.store(false, std::memory_order_release);

        FILE* fp = make_csv(pRobot);
        if (!fp) continue;

        fprintf(fp,
            "timestamp_ns,"
            "q0,q1,q2,q3,q4,q5,"
            "q_ref0,q_ref1,q_ref2,q_ref3,q_ref4,q_ref5,"
            "qd0,qd1,qd2,qd3,qd4,qd5,"
            "tau0,tau1,tau2,tau3,tau4,tau5,"
            "tcp_x,tcp_y,tcp_z,"
            "tcp_roll,tcp_pitch,tcp_yaw,"
            "goal_x,goal_y,goal_z,"
            "goal_roll,goal_pitch,goal_yaw,"
            "ctrl_mode\n");

        while (pRobot->m_logBuffer.pop(entry)) {
            fprintf(fp,
                "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%d\n",
                (unsigned long long)entry.timestamp_ns,
                entry.q[0],entry.q[1],entry.q[2],entry.q[3],entry.q[4],entry.q[5],
                entry.q_ref[0],entry.q_ref[1],entry.q_ref[2],entry.q_ref[3],entry.q_ref[4],entry.q_ref[5],
                entry.qd[0],entry.qd[1],entry.qd[2],entry.qd[3],entry.qd[4],entry.qd[5],
                entry.tau[0],entry.tau[1],entry.tau[2],entry.tau[3],entry.tau[4],entry.tau[5],
                entry.tcp_x, entry.tcp_y, entry.tcp_z,
                entry.tcp_roll, entry.tcp_pitch, entry.tcp_yaw,
                entry.goal_x, entry.goal_y, entry.goal_z,
                entry.goal_roll, entry.goal_pitch, entry.goal_yaw,
                (int)entry.ctrl_mode);
            nCount++;
        }
        fclose(fp);
        DBG_LOG_INFO("(proc_logger) %d entries saved", nCount);
    }

    DBG_LOG_WARN("[proc_logger] TASK ENDED!");
}

FILE*
make_csv(CRobotIndy7* pRobot)
{
    // [kv260-merge] was the author's absolute home path — every log write failed
    mkdir("/home/ubuntu/RAON-RT-Revision/App/Indy7/rt_log_results", 0777);
    char szFilename[256];
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    snprintf(szFilename, sizeof(szFilename),
        "/home/ubuntu/RAON-RT-Revision/App/Indy7/rt_log_results/DataLog_%04d%02d%02d_%02d%02d%02d.csv",
        t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec);
    FILE* fp = fopen(szFilename, "w");
    if (!fp)
        DBG_LOG_ERROR("make_csv: Cannot open %s", szFilename);
    else
        DBG_LOG_INFO("(proc_logger) Saving to %s", szFilename);
    return fp;
}



void
CRobotIndy7::SaveRobotPose()
{
    static const char* kDir  = "/home/ubuntu/RAON-RT-Revision/App/CalibUtils/kv260";
    static const char* kPath = "/home/ubuntu/RAON-RT-Revision/App/CalibUtils/kv260/robot_poses.csv";

    mkdir(kDir, 0777);
    m_nPoseCapture++;

    FILE* fp;
    if (m_nPoseCapture == 1)
    {
        fp = fopen(kPath, "w");
        if (!fp) { DBG_LOG_ERROR("SaveRobotPose: Cannot open %s", kPath); m_nPoseCapture--; return; }
        fprintf(fp, "capture,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22\n");
    }
    else
    {
        fp = fopen(kPath, "a");
        if (!fp) { DBG_LOG_ERROR("SaveRobotPose: Cannot open %s", kPath); m_nPoseCapture--; return; }
    }

    const auto& p = m_Pose.m_position;
    const auto& R = m_Pose.m_rotation;
    fprintf(fp, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        m_nPoseCapture,
        p[0], p[1], p[2],
        R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2));
    fclose(fp);

    DBG_LOG_INFO("[SaveRobotPose] #%d saved: t=[%.4f, %.4f, %.4f]",
                 m_nPoseCapture, p[0], p[1], p[2]);
    printf("[robot_poses] #%d saved → %s\n", m_nPoseCapture, kPath);
}

void
CRobotIndy7::SaveISOHWResults()
{
    mkdir("/home/raimlab/RAON-RT/App/Indy7/iso_csv", 0777);
    mkdir("/home/raimlab/RAON-RT/App/Indy7/iso_csv/hardware", 0777);

    char szFilename[256];
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    snprintf(szFilename, sizeof(szFilename),
        "/home/raimlab/RAON-RT/App/Indy7/iso_csv/hardware/iso_hw_%04d%02d%02d_%02d%02d%02d.csv",
        t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    FILE* fp = fopen(szFilename, "w");
    if (!fp) { DBG_LOG_ERROR("SaveISOHWResults: Cannot open %s", szFilename); return; }

    // Raw records
    fprintf(fp, "point,cycle,x,y,z\n");
    for (int pt = 0; pt < 5; ++pt)
        for (int cy = 0; cy < 10; ++cy)
            fprintf(fp, "%d,%d,%.6f,%.6f,%.6f\n", pt+1, cy+1,
                m_isoHWRecords[pt][cy][0], m_isoHWRecords[pt][cy][1], m_isoHWRecords[pt][cy][2]);

    // AP / RP
    fprintf(fp, "\npoint,cmd_x,cmd_y,cmd_z,mean_x,mean_y,mean_z,AP_mm,RP_mm\n");
    for (int pt = 0; pt < 5; ++pt) {
        double mean[3] = {0,0,0};
        for (int cy = 0; cy < 10; ++cy) {
            mean[0] += m_isoHWRecords[pt][cy][0];
            mean[1] += m_isoHWRecords[pt][cy][1];
            mean[2] += m_isoHWRecords[pt][cy][2];
        }
        mean[0] /= 30.0; mean[1] /= 30.0; mean[2] /= 30.0;

        double cx = m_isoTargets[pt].m_position[0];
        double cy = m_isoTargets[pt].m_position[1];
        double cz = m_isoTargets[pt].m_position[2];
        double AP = sqrt((mean[0]-cx)*(mean[0]-cx) + (mean[1]-cy)*(mean[1]-cy) + (mean[2]-cz)*(mean[2]-cz)) * 1000.0;

        double d_sum = 0, d_sq_sum = 0;
        for (int cy = 0; cy < 10; ++cy) {
            double dx = m_isoHWRecords[pt][cy][0] - mean[0];
            double dy = m_isoHWRecords[pt][cy][1] - mean[1];
            double dz = m_isoHWRecords[pt][cy][2] - mean[2];
            double d  = sqrt(dx*dx + dy*dy + dz*dz);
            d_sum    += d;
            d_sq_sum += d*d;
        }
        double d_mean = d_sum / 10.0;
        double d_std  = sqrt(d_sq_sum/30.0 - d_mean*d_mean);
        double RP = (d_mean + 3.0*d_std) * 1000.0;

        fprintf(fp, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f\n",
            pt+1, cx, cy, cz, mean[0], mean[1], mean[2], AP, RP);
        DBG_LOG_INFO("[ISO-HW] P%d  AP=%.4f mm  RP=%.4f mm", pt+1, AP, RP);
    }
    fclose(fp);
    DBG_LOG_INFO("[ISO-HW] Saved: %s", szFilename);
}



