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
    m_pSimRobot = NULL;
    m_pCurrentAxes = NULL;
    m_bSimulationMode = FALSE;
    m_bStopTask = FALSE;
    m_strDataLog = "DataLog_";
    m_nEcatCycle = 0;
    m_bEcatOP = FALSE;

#ifdef MUJOCO_ENABLED
    m_pMjModel = nullptr;
    m_pMjData = nullptr;
    // Visualization is now handled by global manager
#endif

    /* ROS2 */
    m_bROS2Enabled = FALSE;
    m_pROS2Executor = nullptr;
 
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
    DeInitROS2Interface();

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
    
    // Cleanup simulation
    if (m_pSimRobot != NULL)
    {
        delete m_pSimRobot;
        m_pSimRobot = NULL;
    }
    
#ifdef MUJOCO_ENABLED
    // Cleanup handled by global visualization manager
    if (m_pMjData) mj_deleteData(m_pMjData);
    if (m_pMjModel) mj_deleteModel(m_pMjModel);
#endif
}

// // Overlay/keyboard viewer switch (default: enabled)
// #include <thread>
// static bool g_EnableMuJoCoOverlay = true;

// // Forward declaration for overlay viewer launcher
// void StartMuJoCoOverlayViewer(const char* modelFile, mjModel* m, mjData* d);

// // Minimal stub implementation to resolve linker error
// #ifdef MUJOCO_ENABLED
// #include <cstdio>
// void StartMuJoCoOverlayViewer(const char* modelFile, mjModel* m, mjData* d) {
//     (void)modelFile;
//     (void)m;
//     (void)d;
//     printf("[MuJoCo Overlay] Overlay/keyboard viewer stub called.\n");
//     // TODO: Implement overlay/keyboard logic here
// }
// #endif

BOOL 
CRobotIndy7::Init(BOOL abSim)
{
    m_bSimulationMode = abSim;

    /* Init Controller */
    if (FALSE == InitController(m_strName = m_pcConfigRobot->GetSystemConf().strURDFPath))
    {
        DBG_LOG_ERROR("(%s) Failed to initialize RT Controller", "CRobotIndy7");
        return FALSE;
    }
    DBG_LOG_INFO("(%s) RT Controller initialized successfully", "CRobotIndy7");

    InitROS2Interface();
    DBG_LOG_INFO("(%s) ROS2 Interface initialized", "CRobotIndy7");

    // Unified initialization for both hardware and simulation
    DBG_LOG_INFO("(%s) Initializing in %s mode", "CRobotIndy7", m_bSimulationMode ? "SIMULATION" : "HARDWARE");
    if (!InitEtherCAT())
    {
        DBG_LOG_ERROR("(%s) Failed to initialize EtherCAT/Simulation", "CRobotIndy7");
        return FALSE;
    }
    DBG_LOG_INFO("(%s) EtherCAT/Simulation initialized successfully", "CRobotIndy7");
    
    if (FALSE == InitRTTasks())
    {
        DBG_LOG_ERROR("(%s) Cannot Init RT Tasks", "CRobot");
        return FALSE;
    }
    DBG_LOG_INFO("(%s) RT Tasks initialized successfully", "CRobotIndy7");

// #ifdef MUJOCO_ENABLED
//     // Start overlay/keyboard viewer in a non-RT thread if enabled and in simulation mode
//     if (m_bSimulationMode && g_EnableMuJoCoOverlay && m_pMjModel && m_pMjData) {
//         std::string modelFile = m_pcConfigRobot->GetSystemConf().strMojucoXmlPath;
//         std::thread overlayThread(StartMuJoCoOverlayViewer, modelFile.c_str(), m_pMjModel, m_pMjData);
//         overlayThread.detach();
//         DBG_LOG_INFO("(%s) MuJoCo overlay/keyboard viewer started in background thread", "CRobotIndy7");
//     }
// #endif

    return TRUE;
}

BOOL 
CRobotIndy7::DeInit()
{

    if (m_bSimulationMode)
    {
        // Cleanup simulation
        if (m_pSimRobot)
        {
            m_pSimRobot->Shutdown();
        }
        StopTasks();
        if (m_pCurrentAxes)
        {
            delete[] m_pCurrentAxes;
            m_pCurrentAxes = nullptr;
        }
        
        return TRUE;
    }
    else
    {
        // Cleanup hardware
        if (TRUE == CRobot::DeInit())
        {
            delete[] m_pEcatAxis;
            m_pEcatAxis = nullptr;

            delete[] m_pEcatSensor;
            m_pEcatSensor = nullptr;    

            return TRUE;
        }
    }
    // StopTasks();
    
    return FALSE;
}

BOOL
CRobotIndy7::InitROS2Interface()
{
if (m_bROS2Enabled) {
        DBG_LOG_WARN("(%s) ROS2 Interface already initialized", "CRobotIndy7");
        return TRUE;
    }

    try {
        // Create ROS2 executor
        m_pROS2Executor = new CROS2Executor();
        if (!m_pROS2Executor) 
        {
            DBG_LOG_ERROR("(%s) Failed to create ROS2 executor", "CRobotIndy7");
            return FALSE;
        }

        // Create state publisher
        m_pROS2StatePub = std::make_shared<CROS2IndyStatePub>("indy_state_pub", "indy_state");
        
        // Create service servers
        m_pROS2GetGainsSrv = std::make_shared<CROS2GetGainsSrv>("get_gains_srv", "get_gains");
        m_pROS2SetGainsSrv = std::make_shared<CROS2SetGainsSrv>("set_gains_srv", "set_gains");
        m_pROS2SetPosSrv = std::make_shared<CROS2SetPosSrv>("set_pos_srv", "set_pos");
        m_pROS2SetAllPosSrv = std::make_shared<CROS2SetAllPosSrv>("set_all_pos_srv", "set_all_pos");
        m_pROS2SetControlModeSrv = std::make_shared<CROS2SetControlModeSrv>("set_control_mode_srv", "set_control_mode");
        m_pROS2GetControlModeSrv = std::make_shared<CROS2GetControlModeSrv>("get_control_mode_srv", "get_control_mode");

        // Set up service callbacks
        m_pROS2GetGainsSrv->SetCallback([this]() -> IndyControlGains {
            return GetCurrentControlGains();
        });

        m_pROS2SetGainsSrv->SetCallback([this](const IndyControlGains& gains) -> bool {
            return SetControlGains(gains);
        });

        m_pROS2SetPosSrv->SetCallback([this](uint32_t joint_index, double position) -> bool {
            return SetJointPosition(joint_index, position);
        });

        m_pROS2SetAllPosSrv->SetCallback([this](const std::vector<double>& positions) -> bool {
            return SetAllJointPositions(positions);
        });

        m_pROS2SetControlModeSrv->SetCallback([this](uint8_t control_mode) -> bool {
            return SetControlMode(control_mode);
        });
        
        m_pROS2GetControlModeSrv->SetCallback([this]() -> uint8_t {
            return GetControlMode();
        });

        // Add nodes to executor
        m_pROS2Executor->AddNode(m_pROS2StatePub.get());
        m_pROS2Executor->AddNode(m_pROS2GetGainsSrv.get());
        m_pROS2Executor->AddNode(m_pROS2SetGainsSrv.get());
        m_pROS2Executor->AddNode(m_pROS2SetPosSrv.get());
        m_pROS2Executor->AddNode(m_pROS2SetAllPosSrv.get());
        m_pROS2Executor->AddNode(m_pROS2SetControlModeSrv.get());
        m_pROS2Executor->AddNode(m_pROS2GetControlModeSrv.get());

        // Initialize executor
        if (!m_pROS2Executor->Init()) {
            DBG_LOG_ERROR("(%s) Failed to initialize ROS2 executor", "CRobotIndy7");
            return FALSE;
        }

        m_bROS2Enabled = TRUE;
        DBG_LOG_INFO("(%s) ROS2 Interface initialized successfully", "CRobotIndy7");
        return TRUE;
    }
    catch (const std::exception& e) {
        DBG_LOG_ERROR("(%s) Exception in InitROS2Interface: %s", "CRobotIndy7", e.what());
        return FALSE;
    }
}

BOOL CRobotIndy7::DeInitROS2Interface()
{
    if (!m_bROS2Enabled) {
        return TRUE;
    }

    try {
        // Clean up executor
        if (m_pROS2Executor) {
            m_pROS2Executor->DeInit();
            delete m_pROS2Executor;
            m_pROS2Executor = nullptr;
        }

        // Reset shared pointers
        m_pROS2StatePub.reset();
        m_pROS2GetGainsSrv.reset();
        m_pROS2SetGainsSrv.reset();
        m_pROS2SetPosSrv.reset();
        m_pROS2SetAllPosSrv.reset();

        m_bROS2Enabled = FALSE;
        DBG_LOG_INFO("(%s) ROS2 Interface deinitialized", "CRobotIndy7");
        return TRUE;
    }
    catch (const std::exception& e) {
        DBG_LOG_ERROR("(%s) Exception in DeInitROS2Interface: %s", "CRobotIndy7", e.what());
        return FALSE;
    }
}

void 
CRobotIndy7::PublishRobotState()
{
    if (!m_bROS2Enabled || !m_pROS2StatePub) 
    {
        return;
    }
    IndyRobotState state = GetCurrentRobotState();
    m_pROS2StatePub->PublishState(state);
}

IndyRobotState CRobotIndy7::GetCurrentRobotState()
{
    IndyRobotState state;
    
    // Get current timestamp
    state.timestamp = read_timer();

    const unsigned int nDof = GetTotalAxis();
    
    for (unsigned int i = 0; i < nDof; ++i) 
    {
        if (i < nDof) {
            // RT-safe: Direct access to pre-allocated vectors
            if (i < m_vCurrentPos.size()) 
            {
                // send deg to ROS2
                state.positions[i] = ConvertRad2Deg(m_vCurrentPos[i]);
            } else 
            {
                state.positions[i] = 0.0;
            }
            
            if (i < m_vCurrentVel.size()) {
                state.velocities[i] = m_vCurrentVel[i];
            } else {
                state.velocities[i] = 0.0;
            }
            
            if (i < m_vCurrentTor.size()) {
                state.torques[i] = m_vCurrentTor[i];
            } else {
                state.torques[i] = 0.0;
            }
            
            // RT-safe: Direct axis access (no dynamic allocation)
            if (m_bSimulationMode)
            {
                // In simulation mode, no faults
                state.faults[i] = 1;  // 1 means no fault
            }
            else if (m_pEcatAxis && m_pEcatAxis[i]) 
            {
                eAxisState axstatus = m_pEcatAxis[i]->GetState();
                uint8_t isFault = 1;
                if (axstatus < eAxisInit)
                {
                    isFault = 2;
                }
                state.faults[i] =  isFault;
            } else {
                state.faults[i] = 0;
            }
        } else {
            // Fill remaining elements with zeros
            state.positions[i] = 0.0;
            state.velocities[i] = 0.0;
            state.torques[i] = 0.0;
            state.faults[i] = 0;
        }
    }
    return state;
}

IndyControlGains 
CRobotIndy7::GetCurrentControlGains()
{
    IndyControlGains gains;
    
    if (m_pController) {
        const unsigned int dof = m_pController->GetDOF();
        gains.kp.resize(6, 0.0);
        gains.kd.resize(6, 0.0);
        
        for (unsigned int i = 0; i < std::min(dof, 6u); ++i) {
            double kp, kd;
            m_pController->GetControlGain(i, &kp, &kd);
            gains.kp[i] = kp;
            gains.kd[i] = kd;
        }
    }
    
    return gains;
}

BOOL 
CRobotIndy7::SetControlMode(uint8_t control_mode)
{
    if (!m_pController) {
        DBG_LOG_ERROR("(%s) RT Controller not initialized", "CRobotIndy7");
        return FALSE;
    }
    
    CControllerFullDynamicsRT::eControlMode mode;
    
    switch(control_mode)
    {
        case 0:
            mode = CControllerFullDynamicsRT::eGravityCompensation;
            break;
        case 1:
            mode = CControllerFullDynamicsRT::eFullDynamics;
            break;
        case 2:
            mode = CControllerFullDynamicsRT::eComputedTorque;
            break;
        case 3:
            mode = CControllerFullDynamicsRT::eAdaptiveControl;
            break;
        default:
            DBG_LOG_ERROR("(%s) Invalid control mode: %d", "CRobotIndy7", control_mode);
            return FALSE;
    }
    /* set current position as the reference position */
    for (int i=0; i < GetTotalAxis(); i++)
    {
        m_pController->SetReferencePos(i, m_pEcatAxis[i]->GetCurrentPos());
    }
    return SetControllerMode(mode);
}

uint8_t 
CRobotIndy7::GetControlMode()
{
    if (!m_pController) {
        DBG_LOG_ERROR("(%s) RT Controller not initialized", "CRobotIndy7");
        return 0;
    }
    
    CControllerFullDynamicsRT::eControlMode mode = m_pController->GetControlMode();
    
    switch(mode)
    {
        case CControllerFullDynamicsRT::eGravityCompensation:
            return 0;
        case CControllerFullDynamicsRT::eFullDynamics:
            return 1;
        case CControllerFullDynamicsRT::eComputedTorque:
            return 2;
        case CControllerFullDynamicsRT::eAdaptiveControl:
            return 3;
        default:
            return 0;
    }
}

BOOL 
CRobotIndy7::SetControlGains(const IndyControlGains& gains)
{
    if (!m_pController) {
        return FALSE;
    }
    
    if (gains.kp.size() != 6 || gains.kd.size() != 6) {
        return FALSE;
    }
    
    return SetControllerGains(gains.kp, gains.kd);
}

BOOL 
CRobotIndy7::SetJointPosition(uint32_t joint_index, double position)
{
    if (!m_pController || joint_index >= GetTotalAxis()) {
        return FALSE;
    }
    // convert degree to radians
    return m_pController->SetReferencePos(joint_index, ConvertDeg2Rad(position));
}

BOOL 
CRobotIndy7::SetAllJointPositions(const std::vector<double>& positions)
{
    if (!m_pController || positions.size() != 6) {
        return FALSE;
    }
    // convert degree to radians
    for (size_t i = 0; i < positions.size() && i < GetTotalAxis(); ++i) {
        if (!m_pController->SetReferencePos(i, ConvertDeg2Rad(positions[i]))) {
            return FALSE;
        }
    }
    
    return TRUE;
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
    INT32 nTotalSlaves = m_pcConfigRobot->GetEcatMasterConf().nNoOfSlaves;
    VEC_ECAT_SLAVE_CONF vSlaveInfoList = m_pcConfigRobot->GetEcatSlaveList();

    // Axis and sensor arrays must be sized for both modes
    // Use CAxis* for both simulation and hardware for pointer compatibility
    m_pEcatAxis = new CAxis*[nTotalSlaves];
    m_pEcatSensor = new CSensorNRMKEndTool*[nTotalSlaves];

    if (m_bSimulationMode)
    {
#ifdef MUJOCO_ENABLED
        // --- SIMULATION FULL INITIALIZATION ---
        // 1. Load MuJoCo model
        TSTRING modelFile = m_pcConfigRobot->GetSystemConf().strMojucoXmlPath;
        char error[1000] = "Could not load model";
        DBG_LOG_INFO("(%s) Loading MuJoCo model from: %s", "CRobotIndy7", modelFile.c_str());
        m_pMjModel = mj_loadXML(modelFile.c_str(), 0, error, 1000);
        if (!m_pMjModel)
        {
            DBG_LOG_ERROR("(%s) MuJoCo load error: %s", "CRobotIndy7", error);
            return FALSE;
        }
        m_pMjData = mj_makeData(m_pMjModel);
        if (!m_pMjData)
        {
            DBG_LOG_ERROR("(%s) Failed to create MuJoCo data", "CRobotIndy7");
            mj_deleteModel(m_pMjModel);
            return FALSE;
        }
        // 2. Axis and sensor abstraction (config-driven, single set of axis objects)
        // Build config-driven axes as unique_ptrs for MuJoCo robot
        std::vector<std::unique_ptr<CAxisRTMuJoCo>> mujocoAxes;
        for (int nCnt = 0; nCnt < nTotalSlaves; nCnt++)
        {
            ST_CONFIG_ECAT_SLAVE stSlaveInfo = vSlaveInfoList[nCnt];
            INT nSlaveType = stSlaveInfo.nSlaveType;
            if (nSlaveType == 0) // Axis
            {
                eAxisType ejointtemp;
                if (stSlaveInfo.nJointType >= eAxisRevolute && stSlaveInfo.nJointType <= eAxisJointless)
                    ejointtemp = eAxisType(stSlaveInfo.nJointType);
                else
                    ejointtemp = eAxisRevolute;
                auto axisPtr = std::make_unique<CAxisRTMuJoCo>(
                    ejointtemp,
                    nCnt, // mjJointId
                    stSlaveInfo.dEncResolution,
                    stSlaveInfo.dGearRatio,
                    stSlaveInfo.dTransRatio,
                    stSlaveInfo.bAbsEnc,
                    stSlaveInfo.bCCW,
                    stSlaveInfo.bEnabled
                );
                axisPtr->SetName(stSlaveInfo.strName);
                axisPtr->SetPositionLimits(stSlaveInfo.dPosLimitL, stSlaveInfo.dPosLimitU);
                axisPtr->SetVelocityLimits(stSlaveInfo.dVelLimitL, stSlaveInfo.dVelLimitU);
                axisPtr->SetAccelerationLimits(stSlaveInfo.dAccLimitL, stSlaveInfo.dAccLimitU);

                axisPtr->SetDecelerationLimits(stSlaveInfo.dDecLimitL, stSlaveInfo.dDecLimitU);
                axisPtr->SetJerkLimits(stSlaveInfo.dJerkLimitL, stSlaveInfo.dJerkLimitU);
                axisPtr->SetTorqueLimits(stSlaveInfo.dTorLimitL, stSlaveInfo.dTorLimitU);
                if (!axisPtr->Init())
                {
                    DBG_LOG_ERROR("(%s) Cannot Initialize Sim Axis No. %d", "CRobotIndy7", nCnt);
                    return FALSE;
                }
                AddAxis(axisPtr.get());
                m_pEcatAxis[nCnt] = axisPtr.get(); // maintain raw pointer for compatibility
                mujocoAxes.push_back(std::move(axisPtr));
            }
            else if (nSlaveType == 1) // Sensor
            {
                int nPos = nTotalSlaves - 1 - nCnt;
                m_pEcatSensor[nPos] = new CSensorNRMKEndTool(stSlaveInfo.bEnabled);
                m_pEcatSensor[nPos]->SetName(stSlaveInfo.strName);
                m_pEcatSensor[nPos]->Init();
            }
        }
        // 3. Create simulation robot controller and inject axes
        m_pSimRobot = new CRobotRTMuJoCo();
        m_pSimRobot->SetAxes(std::move(mujocoAxes));
        if (!m_pSimRobot->InitRT())
        {
            DBG_LOG_ERROR("(%s) Failed to initialize simulation RT controller", "CRobotIndy7");
            return FALSE;
        }
        if (!m_pSimRobot->InitMuJoCo(m_pMjModel, m_pMjData))
        {
            DBG_LOG_ERROR("(%s) Failed to initialize MuJoCo interface", "CRobotIndy7");
            return FALSE;
        }
        // 4. Create axis interface pointers for compatibility (optional, for legacy code)
        int nJoints = GetTotalAxis();
        m_pCurrentAxes = new CAxis*[nJoints];
        for (int i = 0; i < nJoints; ++i)
        {
            m_pCurrentAxes[i] = m_pEcatAxis[i];
        }
        // 5. Initialize global visualization manager
        if (!MuJoCoViz::Initialize(modelFile.c_str())) {
            DBG_LOG_WARN("(%s) Failed to initialize MuJoCo visualization (continuing without visualization)", "CRobotIndy7");
        } else {
            if (!MuJoCoViz::Start()) {
                DBG_LOG_WARN("(%s) Failed to start MuJoCo visualization thread", "CRobotIndy7");
            }
        }
        m_nEcatCycle = 1000; // Default sim cycle
        DBG_LOG_INFO("(%s) Simulation initialized successfully with %d joints", "CRobotIndy7", nJoints);
        return TRUE;
#else
        DBG_LOG_ERROR("(%s) Simulation requested but MuJoCo not available (compile with -DMUJOCO_ENABLED)", "CRobotIndy7");
        return FALSE;
#endif
    }
    else
    {
        // --- HARDWARE AXIS ABSTRACTION (original logic) ---
        m_pcEcatMaster = new CEcatMaster();
        for (int nCnt = 0; nCnt < nTotalSlaves; nCnt++)
        {
            ST_CONFIG_ECAT_SLAVE stSlaveInfo = vSlaveInfoList[nCnt];
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
                m_pEcatAxis[nCnt]->SetTorqueLimits(stSlaveInfo.dTorLimitL, stSlaveInfo.dTorLimitU);
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

                if (FALSE == static_cast<CAxisNRMKCore*>(m_pEcatAxis[nCnt])->Init(*m_pcEcatMaster, (INT8)stSlaveInfo.nDriveMode))
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
}

BOOL
CRobotIndy7::InitSimulation()
{
    // All simulation initialization is now handled in InitEtherCAT().
    DBG_LOG_INFO("(%s) InitSimulation() is now a stub. All simulation setup is performed in InitEtherCAT().", "CRobotIndy7");
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
    if (m_bSimulationMode)
    {
        // Check if simulation robot is moving
        for (unsigned int i = 0; i < GetTotalAxis(); ++i)
        {
            if (std::abs(m_pSimRobot->GetJointVelocity(i)) > 1e-6)
                return TRUE;
        }
        return FALSE;
    }
    else
    {
        // Check if any hardware axis is moving
        for (unsigned int i = 0; i < GetTotalAxis(); ++i)
        {
            if (m_pEcatAxis[i]->IsMoving())
                return TRUE;
        }
        return FALSE;
    }
}

void 
proc_main_control(void* apRobot)
{
    auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
    DBG_LOG_INFO("(proc_main_control) Dynamics/Kinematics Control Task Started! Mode: %s", 
                 pRobot->m_bSimulationMode ? "SIMULATION" : "HARDWARE");

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
        // For simulation, always proceed; for hardware, check EtherCAT status
        if (!pRobot->m_bSimulationMode && !pRobot->m_bEcatOP) continue;
        
        pRobot->DoInput();

        // Read current joint states (unified interface for both modes)
        if (pRobot->m_bSimulationMode)
        {
            // Simulation: Get state from simulation robot
            for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
            {   
                pRobot->m_vCurrentPos[nCnt] = pRobot->m_pSimRobot->GetJointPosition(nCnt);
                // pRobot->m_vCurrentPos[nCnt] = 0.0; // MuJoCo does not provide direct position, use simulation state
                pRobot->m_vCurrentVel[nCnt] = pRobot->m_pSimRobot->GetJointVelocity(nCnt);
                // pRobot->m_vCurrentVel[nCnt] = 0.0;
                pRobot->m_vCurrentTor[nCnt] = pRobot->m_pSimRobot->GetJointTorque(nCnt);

                // printf("Joint %d: Pos=%.2f, Vel=%.2f, Tor=%.2f\n", nCnt, 
                //        pRobot->m_vCurrentPos[nCnt], 
                //        pRobot->m_vCurrentVel[nCnt], 
                //        pRobot->m_vCurrentTor[nCnt]);

            }
        }
        else
        {
            // Hardware: Read from EtherCAT axes
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
        }

        // Compute control torques using dynamics/kinematics controller
        // This is the high-level control logic (same for both hardware and simulation)
        if (bUseRTController)
        {
            // Use RT Controller for dynamics/kinematics computation
            if (pRTController->Update(pRobot->m_vCurrentPos, pRobot->m_vCurrentVel, 
                                     pRobot->m_vCurrentTor, pRobot->m_vOutputTorque))
            {
                // Send computed torques to actuators 
                // NOTE: The actual communication (EtherCAT/MuJoCo) is handled by proc_ethercat_control
                if (pRobot->m_bSimulationMode)
                {
                    // Simulation: Send target torques to simulation robot
                    for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
                    {
                        pRobot->m_pEcatAxis[nCnt]->MoveTorque(pRobot->m_vOutputTorque[nCnt]);
                    }
                }
                else
                {
                    // Hardware: Send torques to EtherCAT axes
                    for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
                    {
                        auto* ax = pRobot->m_pEcatAxis[nCnt];
                        ax->MoveTorque(pRobot->m_vOutputTorque[nCnt]);
                    }
                }
            }
            else
            {
                // RT Controller failed, apply zero torque for safety
                DBG_LOG_ERROR("(proc_main_control) RT Controller update failed - applying zero torque");
                for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
                {
                    if (pRobot->m_bSimulationMode)
                    {
                        pRobot->m_pSimRobot->SetJointTorqueCommand(nCnt, 0.0);
                    }
                    else
                    {
                        pRobot->m_pEcatAxis[nCnt]->MoveTorque(0.0);
                    }
                }
            }
        }
        else
        {
            // Fallback: Apply zero torque if no controller available
            for (int nCnt = 0; nCnt < (int)udof; ++nCnt) 
            {
                if (pRobot->m_bSimulationMode)
                {
                    pRobot->m_pSimRobot->SetJointTorqueCommand(nCnt, 0.0);
                }
                else
                {
                    pRobot->m_pEcatAxis[nCnt]->MoveTorque(0.0);
                }
            }
        }
        
        // Publish robot state to ROS2 (unified interface)
        pRobot->PublishRobotState();

        /* LED Status Control (hardware mode only) */
        if (!pRobot->m_bSimulationMode && pRTController)
        {
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
        }
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

    DBG_LOG_INFO("(%s) EtherCAT/Communication Control Task Started! Mode: %s", 
                 "proc_ethercat_control", 
                 pRobot->m_bSimulationMode ? "SIMULATION" : "HARDWARE");

    int nCleanUp = 0;

    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);
        tmCurrent = read_timer();

        if (!pRobot->m_bSimulationMode)
        {
            // === HARDWARE MODE: EtherCAT Communication ===
            pRobot->m_bEcatOP = FALSE;

            // Read from EtherCAT slaves (sensors, encoders, etc.)
            pRobot->m_pcEcatMaster->ReadSlaves();
            pRobot->UpdateExtInterfaceData();
            
            if (TRUE == pRobot->m_pcEcatMaster->IsAllSlavesOp() && TRUE == pRobot->m_pcEcatMaster->IsMasterOp())
            {
                pRobot->m_bEcatOP = TRUE;
            }
            
            tmSend = read_timer();
            // Write to EtherCAT slaves (motor commands, outputs, etc.)
            pRobot->m_pcEcatMaster->WriteSlaves(tmSend);
            tmEcat = read_timer();
        }
        else
        {
            // === SIMULATION MODE: MuJoCo Model Updates ===
            pRobot->m_bEcatOP = TRUE;  // Always operational in simulation
            
            tmSend = read_timer();
            
            // Send target values to MuJoCo via queue-based IPC
            if (pRobot->m_pSimRobot)
            {
                // Step the RT motor simulation (updates internal m_dSimTime)
                pRobot->m_pSimRobot->StepRTSimulation(0.001);  // 1ms timestep
                
                // Update MuJoCo model with current target values
                pRobot->m_pSimRobot->UpdateMuJoCoFromRT();
                
                // Send physics commands to 1kHz physics thread via queue (RT-safe)
                const int numJoints = pRobot->GetTotalAxis();
                for (int i = 0; i < numJoints; ++i) {
                    // Send current joint torque commands to physics thread
                    double torque = pRobot->m_pSimRobot->GetJointTorque(i);
                    // printf("Sending torque command for joint %d: %lf\n", i, torque);

                    MuJoCoViz::SendJointTorqueCommand(i, torque, tmCurrent * 1e-9);
                }
                
                // Send step command to trigger physics update at 1kHz
                MuJoCoViz::SendPhysicsStepCommand(tmCurrent * 1e-9);
                
                // Update RT state from physics feedback (lock-free read)
                if (numJoints > 0) 
                {
                    double positions[6], velocities[6], torques[6];
                    for (int i = 0; i < numJoints && i < 6; ++i) {
                        if (MuJoCoViz::GetJointFeedback(i, positions[i], velocities[i], torques[i])) {
                            pRobot->m_pEcatAxis[i]->UpdateCurrentParams(positions[i], velocities[i], torques[i]);
                            // pRobot->m_vCurrentVel[i] = velocities[i];
                            // pRobot->m_vCurrentTor[i] = torques[i];
                        }
                    }
                    
                    // Update visualization state (separate from physics)
                    MuJoCoViz::UpdateRobotState(numJoints, positions, velocities, torques, tmCurrent * 1e-9);
                }
            }
            
            tmEcat = read_timer();
        }

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
            if (!pRobot->m_bSimulationMode)
            {
                for (int nMotorCnt = 0; nMotorCnt < (int)pRobot->GetTotalAxis(); nMotorCnt++)
                {
                    pRobot->m_pEcatAxis[nMotorCnt]->ServoOff();
                }
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
    }
    
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_keyboard_control");
}

void
proc_terminal_output(void* apRobot)
{
    CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
    (void)pRobot; // temporary: just to avoid warning

    DBG_LOG_INFO("(%s) Terminal Output Task Started!", "proc_terminal_output");

    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);

        /* TODO: Last slave is Sensor, we must do this only for Axis */
        for (int nMotCnt = 0; nMotCnt < pRobot->GetTotalAxis(); nMotCnt++)
        {
            DBG_LOG_TRACE("[SLAVE %d]", nMotCnt);
            DBG_LOG_TRACE("TargetPos: %d, RawPos:%d", pRobot->m_pEcatAxis[nMotCnt]->GetTargetRawPos(), pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawPos());
            DBG_LOG_TRACE("RawVel:%d RawTor: %d",pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawVel(), pRobot->m_pEcatAxis[nMotCnt]->GetCurrentRawTor());
                                
            DBG_LOG_TRACE("ControlWord %04x, StatusWord %04x", pRobot->m_pEcatAxis[nMotCnt]->GetControlWord(), pRobot->m_pEcatAxis[nMotCnt]->GetStatusWord());
            DBG_LOG_TRACE("CurrentPos: %lf", pRobot->m_pEcatAxis[nMotCnt]->GetCurrentPosD());
            DBG_LOG_TRACE("HomePosition: %d StartPos:%d", pRobot->m_pEcatAxis[nMotCnt]->GetHomePosition(), pRobot->m_pEcatAxis[nMotCnt]->GetStartRawPos());
            DBG_LOG_TRACE("IsServoOn: %d\n", pRobot->m_pEcatAxis[nMotCnt]->IsServoOn());
            DBG_LOG_TRACE("DriveMode: %s\n", GetCIA402DriveMode(pRobot->m_pEcatAxis[nMotCnt]->GetDriveMode()).c_str());
        }
        DBG_LOG_NOTHING("\n");
    }
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_terminal_output");
}

void
proc_logger(void* apRobot)
{

    return;
}


