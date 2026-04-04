/*****************************************************************************
*	Name: MuJoCoVisualization.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Separate MuJoCo visualization manager implementation
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "MuJoCoVisualization.h"
#include "../Global/Defines.h"  // For DBG_LOG macros
#include <unistd.h>  // For usleep
#include <chrono>    // For high-resolution timing
#include <algorithm> // For std::min

#ifdef MUJOCO_ENABLED
#include <GL/gl.h>
#include <GL/glu.h>
#endif

CMuJoCoVisualizationManager::CMuJoCoVisualizationManager()
    : m_bStopVisualization(false)
    , m_bVisualizationActive(false)
    , m_bInitialized(false)
    , m_physicsTime(0.0)
    , m_physicsStepCount(0)
{
#ifdef MUJOCO_ENABLED
    m_pMjModel = nullptr;
    m_pMjData = nullptr;
    m_numJoints = 0;
    
    // Initialize physics feedback arrays
    for (int i = 0; i < 8; ++i) {
        m_jointPositions[i] = 0.0;
        m_jointVelocities[i] = 0.0;
        m_jointTorques[i] = 0.0;
    }
#endif
    // Initialize RT-POSIX task structure
    memset(&m_visualizationTask, 0, sizeof(POSIX_TASK));
}

CMuJoCoVisualizationManager::~CMuJoCoVisualizationManager()
{
    Shutdown();
}

CMuJoCoVisualizationManager& CMuJoCoVisualizationManager::GetInstance()
{
    static CMuJoCoVisualizationManager instance;
    return instance;
}

bool CMuJoCoVisualizationManager::Initialize(const char* modelFile)
{
#ifdef MUJOCO_ENABLED
    if (m_bInitialized.load()) {
        DBG_LOG_WARN("MuJoCo visualization already initialized");
        return true;
    }
    
    // Load MuJoCo model
    char error[1000] = "Could not load model";
    m_pMjModel = mj_loadXML(modelFile, 0, error, 1000);
    if (!m_pMjModel) {
        DBG_LOG_ERROR("MuJoCo visualization: Failed to load model %s: %s", modelFile, error);
        return false;
    }
    
    m_pMjData = mj_makeData(m_pMjModel);
    if (!m_pMjData) {
        DBG_LOG_ERROR("MuJoCo visualization: Failed to create mjData");
        mj_deleteModel(m_pMjModel);
        m_pMjModel = nullptr;
        return false;
    }
    
    // Initialize MuJoCo state
    mj_resetData(m_pMjModel, m_pMjData);
    
    m_bInitialized = true;
    DBG_LOG_INFO("MuJoCo visualization initialized with model: %s", modelFile);
    return true;
    
#else
    DBG_LOG_ERROR("MuJoCo visualization: Compiled without MuJoCo support");
    return false;
#endif
}

bool CMuJoCoVisualizationManager::StartVisualization()
{
#ifdef MUJOCO_ENABLED
    if (!m_bInitialized.load()) {
        DBG_LOG_ERROR("MuJoCo visualization: Not initialized");
        return false;
    }
    
    if (m_bVisualizationActive.load()) {
        DBG_LOG_WARN("MuJoCo visualization: Already running");
        return true;
    }
    
    m_bStopVisualization = false;
    
    // Create and start non-RT task using RT-POSIX (better scheduling control)
    int result = spawn_nrt_task(&m_visualizationTask, "MuJoCoViz", 
                                 DEFAULT_STKSIZE, VisualizationTaskFunc, this);
    
    if (result != 0) {
        DBG_LOG_ERROR("MuJoCo visualization: Failed to spawn NRT task: %d", result);
        return false;
    }
    
    // Wait a bit for thread to start
    usleep(10000);  // 10ms
    
    DBG_LOG_INFO("MuJoCo visualization thread started");
    return true;
    
#else
    return false;
#endif
}

void CMuJoCoVisualizationManager::StopVisualization()
{
    if (m_bVisualizationActive.load()) {
        m_bStopVisualization = true;
        
        // Delete the RT-POSIX task (will wait for completion)
        int result = delete_task(&m_visualizationTask);
        if (result != 0) {
            DBG_LOG_WARN("MuJoCo visualization: Task deletion returned: %d", result);
        }
        
        m_bVisualizationActive = false;
        DBG_LOG_INFO("MuJoCo visualization task stopped");
    }
}

void CMuJoCoVisualizationManager::Shutdown()
{
    StopVisualization();
    
#ifdef MUJOCO_ENABLED
    if (m_pMjData) {
        mj_deleteData(m_pMjData);
        m_pMjData = nullptr;
    }
    
    if (m_pMjModel) {
        mj_deleteModel(m_pMjModel);
        m_pMjModel = nullptr;
    }
#endif
    
    m_bInitialized = false;
    DBG_LOG_INFO("MuJoCo visualization shutdown complete");
}

// RT-safe physics command methods
bool CMuJoCoVisualizationManager::SendJointTorqueCommand(int joint_index, double torque, double timestamp)
{
    if (!m_bVisualizationActive.load()) {
        return false;  // No physics running
    }
    
    return m_physicsQueue.Push(PhysicsCommand::SET_JOINT_TORQUE, joint_index, torque, timestamp);
}

bool CMuJoCoVisualizationManager::SendJointPositionCommand(int joint_index, double position, double timestamp)
{
    if (!m_bVisualizationActive.load()) {
        return false;  // No physics running
    }
    
    return m_physicsQueue.Push(PhysicsCommand::SET_JOINT_POSITION, joint_index, position, timestamp);
}

bool CMuJoCoVisualizationManager::SendJointVelocityCommand(int joint_index, double velocity, double timestamp)
{
    if (!m_bVisualizationActive.load()) {
        return false;  // No physics running
    }
    
    return m_physicsQueue.Push(PhysicsCommand::SET_JOINT_VELOCITY, joint_index, velocity, timestamp);
}

bool CMuJoCoVisualizationManager::SendPhysicsStepCommand(double timestamp)
{
    if (!m_bVisualizationActive.load()) {
        return false;  // No physics running
    }
    
    return m_physicsQueue.PushStepCommand(timestamp);
}

bool CMuJoCoVisualizationManager::GetJointFeedback(int joint_index, double& position, double& velocity, double& torque) const
{
#ifdef MUJOCO_ENABLED
    if (joint_index < 0 || joint_index >= m_numJoints.load()) {
        return false;
    }
    
    position = m_jointPositions[joint_index].load();
    velocity = m_jointVelocities[joint_index].load();
    torque = m_jointTorques[joint_index].load();
    return true;
#else
    return false;
#endif
}

void CMuJoCoVisualizationManager::UpdateRobotState(int numJoints, const double* positions, const double* velocities, const double* torques, double timestamp)
{
    // This is called from RT threads - must be lock-free and fast
    if (!m_bVisualizationActive.load()) {
        return;  // No visualization running, skip update
    }
    
    m_visualizationData.UpdateFromRT(numJoints, positions, velocities, torques, timestamp);
}

void CMuJoCoVisualizationManager::VisualizationTaskFunc(void* arg)
{
    CMuJoCoVisualizationManager* manager = static_cast<CMuJoCoVisualizationManager*>(arg);
    manager->RunVisualization();
}

void CMuJoCoVisualizationManager::RunVisualization()
{
#ifdef MUJOCO_ENABLED
    // Initialize GLFW
    if (!glfwInit()) {
        DBG_LOG_ERROR("MuJoCo visualization: Failed to initialize GLFW");
        return;
    }
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(
        m_settings.window_width, 
        m_settings.window_height, 
        m_settings.window_title, 
        nullptr, nullptr
    );
    
    if (!window) {
        glfwTerminate();
        DBG_LOG_ERROR("MuJoCo visualization: Failed to create window");
        return;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync
    
    // Initialize MuJoCo visualization structures
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m_pMjModel, &scn, 2000);
    mjr_makeContext(m_pMjModel, &con, mjFONTSCALE_150);
    
    // Setup camera
    cam.azimuth = m_settings.camera_azimuth;
    cam.elevation = m_settings.camera_elevation;
    cam.distance = m_settings.camera_distance;
    cam.lookat[0] = m_settings.camera_lookat[0];
    cam.lookat[1] = m_settings.camera_lookat[1];
    cam.lookat[2] = m_settings.camera_lookat[2];
    
    // Initialize MuJoCo physics state
    mj_resetData(m_pMjModel, m_pMjData);
    m_numJoints = std::min(m_pMjModel->njnt, 8);
    
    m_bVisualizationActive = true;
    DBG_LOG_INFO("MuJoCo visualization: Rendering + Physics loop started (1kHz physics, %d Hz visualization)", 
                 m_settings.visualization_fps);
    
    // Timing variables for dual-rate processing
    const double physics_dt = 1.0 / m_settings.physics_hz;      // 1ms for 1kHz physics
    const double vis_dt = 1.0 / m_settings.visualization_fps;   // ~16.67ms for 60Hz vis
    
    double last_physics_time = 0.0;
    double last_vis_time = 0.0;
    double current_time = 0.0;
    
    uint64_t physics_steps = 0;
    uint64_t vis_frames = 0;
    
    // Performance monitoring
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Main dual-rate loop: 1kHz physics + 60Hz visualization
    while (!m_bStopVisualization.load() && !glfwWindowShouldClose(window)) 
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        current_time = std::chrono::duration<double>(loop_start - start_time).count();
        
        // === 1kHz PHYSICS PROCESSING ===
        if (current_time - last_physics_time >= physics_dt) {
            ProcessPhysicsCommands();  // Process RT commands
            StepMuJoCoPhysics(physics_dt);  // Step MuJoCo physics
            UpdateVisualizationFromPhysics();  // Update feedback state
            
            last_physics_time = current_time;
            physics_steps++;
            m_physicsStepCount = physics_steps;
            m_physicsTime = current_time;
        }
        
        // === 60Hz VISUALIZATION RENDERING ===
        if (current_time - last_vis_time >= vis_dt) {
            // Get framebuffer size
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            
            // Update scene and render (uses current MuJoCo state)
            mjv_updateScene(m_pMjModel, m_pMjData, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            
            // Swap buffers
            glfwSwapBuffers(window);
            
            // Process events
            glfwPollEvents();
            
            last_vis_time = current_time;
            vis_frames++;
            
            // Performance logging every 5 seconds
            if (vis_frames % (m_settings.visualization_fps * 5) == 0) {
                double avg_physics_hz = physics_steps / current_time;
                double avg_vis_hz = vis_frames / current_time;
                DBG_LOG_INFO("MuJoCo performance: %.1f Hz physics, %.1f Hz visualization, %lu commands processed", 
                           avg_physics_hz, avg_vis_hz, m_physicsQueue.Size());
            }
        }
        
        // Small sleep to prevent 100% CPU usage
        usleep(100);  // 0.1ms sleep
    }
    
    // Cleanup
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();
    
    m_bVisualizationActive = false;
    DBG_LOG_INFO("MuJoCo visualization: Thread exiting after %lu physics steps and %lu frames", 
                 physics_steps, vis_frames);
    
#endif // MUJOCO_ENABLED
}

void CMuJoCoVisualizationManager::ProcessPhysicsCommands()
{
#ifdef MUJOCO_ENABLED
    PhysicsCommand cmd;
    int commands_processed = 0;
    const int max_commands_per_cycle = 50;  // Limit to prevent overwhelming single cycle
    
    while (commands_processed < max_commands_per_cycle && m_physicsQueue.Pop(cmd)) {
        switch (cmd.GetType()) {
            case PhysicsCommand::SET_JOINT_TORQUE:
                {
                    int idx = cmd.GetJointIndex();
                    if (idx >= 0 && idx < m_pMjModel->nu) {
                        m_pMjData->ctrl[idx] = cmd.GetValue();
                    }
                }
                break;
                
            case PhysicsCommand::SET_JOINT_POSITION:
                {
                    int idx = cmd.GetJointIndex();
                    if (idx >= 0 && idx < m_pMjModel->njnt) {
                        m_pMjData->qpos[idx] = cmd.GetValue();
                    }
                }
                break;
                
            case PhysicsCommand::SET_JOINT_VELOCITY:
                {
                    int idx = cmd.GetJointIndex();
                    if (idx >= 0 && idx < m_pMjModel->njnt) {
                        m_pMjData->qvel[idx] = cmd.GetValue();
                    }
                }
                break;
                
            case PhysicsCommand::STEP_PHYSICS:
                // This will be handled by StepMuJoCoPhysics
                break;
        }
        commands_processed++;
    }
#endif
}

void CMuJoCoVisualizationManager::StepMuJoCoPhysics(double timestep)
{
#ifdef MUJOCO_ENABLED
    // Step MuJoCo physics simulation
    mj_step(m_pMjModel, m_pMjData);
    
    // Update simulation time
    m_pMjData->time += timestep;
#endif
}

void CMuJoCoVisualizationManager::UpdateVisualizationFromPhysics()
{
#ifdef MUJOCO_ENABLED
    // Update atomic feedback variables for RT-safe reading
    int num_joints = m_numJoints.load();
    for (int i = 0; i < num_joints; ++i) {
        if (i < m_pMjModel->njnt) {
            m_jointPositions[i] = m_pMjData->qpos[i];
            m_jointVelocities[i] = m_pMjData->qvel[i];
        }
        if (i < m_pMjModel->nu) {
            m_jointTorques[i] = m_pMjData->qfrc_applied[i];
        }
    }
#endif
}

// Global convenience functions implementation
namespace MuJoCoViz {
    
    bool Initialize(const char* modelFile) {
        return CMuJoCoVisualizationManager::GetInstance().Initialize(modelFile);
    }
    
    bool Start() {
        return CMuJoCoVisualizationManager::GetInstance().StartVisualization();
    }
    
    void Stop() {
        CMuJoCoVisualizationManager::GetInstance().StopVisualization();
    }
    
    void Shutdown() {
        CMuJoCoVisualizationManager::GetInstance().Shutdown();
    }
    
    void UpdateRobotState(int numJoints, const double* positions, const double* velocities, const double* torques, double timestamp) {
        CMuJoCoVisualizationManager::GetInstance().UpdateRobotState(numJoints, positions, velocities, torques, timestamp);
    }
    
    bool SendJointTorqueCommand(int joint_index, double torque, double timestamp) {
        return CMuJoCoVisualizationManager::GetInstance().SendJointTorqueCommand(joint_index, torque, timestamp);
    }
    
    bool SendJointPositionCommand(int joint_index, double position, double timestamp) {
        return CMuJoCoVisualizationManager::GetInstance().SendJointPositionCommand(joint_index, position, timestamp);
    }
    
    bool SendJointVelocityCommand(int joint_index, double velocity, double timestamp) {
        return CMuJoCoVisualizationManager::GetInstance().SendJointVelocityCommand(joint_index, velocity, timestamp);
    }
    
    bool SendPhysicsStepCommand(double timestamp) {
        return CMuJoCoVisualizationManager::GetInstance().SendPhysicsStepCommand(timestamp);
    }
    
    bool GetJointFeedback(int joint_index, double& position, double& velocity, double& torque) {
        return CMuJoCoVisualizationManager::GetInstance().GetJointFeedback(joint_index, position, velocity, torque);
    }
    
    bool IsActive() {
        return CMuJoCoVisualizationManager::GetInstance().IsVisualizationActive();
    }
    
}
