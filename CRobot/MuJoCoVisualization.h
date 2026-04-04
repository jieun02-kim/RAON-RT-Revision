/*****************************************************************************
*	Name: MuJoCoVisualization.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Separate MuJoCo visualization manager (RT-safe)
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __MUJOCO_VISUALIZATION_H__
#define __MUJOCO_VISUALIZATION_H__

#include <atomic>
#include <vector>
#include <array>

// Use RT-POSIX instead of standard pthread  
#include <posix_rt.h> 

#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#endif

// Lock-free physics command structure for RT → Physics communication
struct PhysicsCommand 
{
    enum Type {
        SET_JOINT_TORQUE = 0,
        SET_JOINT_POSITION,
        SET_JOINT_VELOCITY,
        STEP_PHYSICS
    };
    
    std::atomic<Type> type;
    std::atomic<int> joint_index;
    std::atomic<double> value;
    std::atomic<double> timestamp;
    std::atomic<bool> valid;
    
    PhysicsCommand() : type(STEP_PHYSICS), joint_index(-1), value(0.0), timestamp(0.0), valid(false) {}
    
    // Custom copy constructor that loads values
    PhysicsCommand(const PhysicsCommand& other) 
        : type(other.type.load())
        , joint_index(other.joint_index.load())
        , value(other.value.load())
        , timestamp(other.timestamp.load())
        , valid(other.valid.load()) 
    {}
    
    // Custom assignment operator that loads/stores values
    PhysicsCommand& operator=(const PhysicsCommand& other) {
        if (this != &other) {
            type.store(other.type.load());
            joint_index.store(other.joint_index.load());
            value.store(other.value.load());
            timestamp.store(other.timestamp.load());
            valid.store(other.valid.load());
        }
        return *this;
    }
    
    void Set(Type cmd_type, int idx, double val, double time) {
        type = cmd_type;
        joint_index = idx;
        value = val;
        timestamp = time;
        valid = true;
    }
    
    void SetStepCommand(double time) {
        type = STEP_PHYSICS;
        joint_index = -1;
        value = 0.0;
        timestamp = time;
        valid = true;
    }
    
    bool IsValid() const { return valid.load(); }
    void Consume() { valid = false; }
    
    // Helper methods for reading values
    Type GetType() const { return type.load(); }
    int GetJointIndex() const { return joint_index.load(); }
    double GetValue() const { return value.load(); }
    double GetTimestamp() const { return timestamp.load(); }
};

// Lock-free circular buffer for physics commands
template<size_t SIZE>
class LockFreePhysicsQueue 
{
private:
    std::array<PhysicsCommand, SIZE> m_buffer;
    std::atomic<size_t> m_writeIndex{0};
    std::atomic<size_t> m_readIndex{0};
    
public:
    // RT-safe: Push command from control thread
    bool Push(PhysicsCommand::Type type, int joint_idx, double value, double timestamp) {
        size_t currentWrite = m_writeIndex.load();
        size_t nextWrite = (currentWrite + 1) % SIZE;
        
        if (nextWrite == m_readIndex.load()) {
            return false; // Queue full
        }
        
        m_buffer[currentWrite].Set(type, joint_idx, value, timestamp);
        m_writeIndex.store(nextWrite);
        return true;
    }
    
    // RT-safe: Push step command
    bool PushStepCommand(double timestamp) {
        size_t currentWrite = m_writeIndex.load();
        size_t nextWrite = (currentWrite + 1) % SIZE;
        
        if (nextWrite == m_readIndex.load()) {
            return false; // Queue full
        }
        
        m_buffer[currentWrite].SetStepCommand(timestamp);
        m_writeIndex.store(nextWrite);
        return true;
    }
    
    // Non-RT: Pop command from physics thread
    bool Pop(PhysicsCommand& cmd) {
        size_t currentRead = m_readIndex.load();
        
        if (currentRead == m_writeIndex.load()) {
            return false; // Queue empty
        }
        
        if (!m_buffer[currentRead].IsValid()) {
            return false; // Invalid command
        }
        
        cmd = m_buffer[currentRead];
        m_buffer[currentRead].Consume();
        m_readIndex.store((currentRead + 1) % SIZE);
        return true;
    }
    
    bool IsEmpty() const {
        return m_readIndex.load() == m_writeIndex.load();
    }
    
    size_t Size() const {
        size_t write = m_writeIndex.load();
        size_t read = m_readIndex.load();
        return write >= read ? write - read : SIZE - read + write;
    }
};

// Lock-free state structure for RT → Visualization communication
struct RTVisualizationData 
{
    std::atomic<double> joint_positions[8];  // Support up to 8 joints
    std::atomic<double> joint_velocities[8];
    std::atomic<double> joint_torques[8];
    std::atomic<double> timestamp;
    std::atomic<bool> data_updated;
    std::atomic<int> num_joints;
    
    RTVisualizationData() : timestamp(0), data_updated(false), num_joints(0) 
    {
        for (int i = 0; i < 8; ++i) {
            joint_positions[i] = 0.0;
            joint_velocities[i] = 0.0;
            joint_torques[i] = 0.0;
        }
    }
    
    // RT-safe update from control thread
    void UpdateFromRT(int numJoints, const double* positions, const double* velocities, const double* torques, double time)
    {
        num_joints = numJoints;
        timestamp = time;
        
        for (int i = 0; i < numJoints && i < 8; ++i) {
            joint_positions[i] = positions[i];
            joint_velocities[i] = velocities[i];
            joint_torques[i] = torques[i];
        }
        
        data_updated = true;  // Signal new data available
    }
    
    // Non-RT read from visualization thread
    bool ReadForVisualization(int& numJoints, double* positions, double* velocities, double* torques, double& time)
    {
        if (!data_updated.load()) {
            return false;  // No new data
        }
        
        numJoints = num_joints.load();
        time = timestamp.load();
        
        for (int i = 0; i < numJoints && i < 8; ++i) {
            positions[i] = joint_positions[i].load();
            velocities[i] = joint_velocities[i].load();
            torques[i] = joint_torques[i].load();
        }
        
        data_updated = false;  // Mark as consumed
        return true;
    }
};

// Global visualization manager with integrated 1kHz MuJoCo physics
class CMuJoCoVisualizationManager 
{
public:
    static CMuJoCoVisualizationManager& GetInstance();
    
    // Initialize with MuJoCo model (called once during startup)
    bool Initialize(const char* modelFile);
    void Shutdown();
    
    // Start/stop visualization + physics thread
    bool StartVisualization();
    void StopVisualization();
    
    // RT-safe interface for control threads - now includes physics commands
    void UpdateRobotState(int numJoints, const double* positions, const double* velocities, const double* torques, double timestamp);
    
    // RT-safe physics commands (sent to 1kHz physics loop)
    bool SendJointTorqueCommand(int joint_index, double torque, double timestamp);
    bool SendJointPositionCommand(int joint_index, double position, double timestamp);
    bool SendJointVelocityCommand(int joint_index, double velocity, double timestamp);
    bool SendPhysicsStepCommand(double timestamp);
    
    // Check if visualization is running
    bool IsVisualizationActive() const { return m_bVisualizationActive.load(); }
    
    // Get physics feedback (RT-safe read)
    bool GetJointFeedback(int joint_index, double& position, double& velocity, double& torque) const;
    
private:
    CMuJoCoVisualizationManager();
    ~CMuJoCoVisualizationManager();
    
    // Prevent copying
    CMuJoCoVisualizationManager(const CMuJoCoVisualizationManager&) = delete;
    CMuJoCoVisualizationManager& operator=(const CMuJoCoVisualizationManager&) = delete;
    
    // Static visualization task function (RT-POSIX signature)
    static void VisualizationTaskFunc(void* arg);
    
    // Instance method called by static task function
    void RunVisualization();
    
    // Physics processing methods
    void ProcessPhysicsCommands();
    void StepMuJoCoPhysics(double timestep);
    void UpdateVisualizationFromPhysics();
    
#ifdef MUJOCO_ENABLED
    mjModel* m_pMjModel;
    mjData* m_pMjData;
    
    // Physics state feedback (atomic for RT-safe reading)
    mutable std::atomic<double> m_jointPositions[8];
    mutable std::atomic<double> m_jointVelocities[8];
    mutable std::atomic<double> m_jointTorques[8];
    mutable std::atomic<int> m_numJoints;
#endif
    
    // Thread management using RT-POSIX
    POSIX_TASK m_visualizationTask;
    std::atomic<bool> m_bStopVisualization;
    std::atomic<bool> m_bVisualizationActive;
    std::atomic<bool> m_bInitialized;
    
    // Lock-free data exchange
    RTVisualizationData m_visualizationData;
    
    // Lock-free physics command queue (1000 commands buffer)
    LockFreePhysicsQueue<1000> m_physicsQueue;
    
    // Physics timing
    std::atomic<double> m_physicsTime;
    std::atomic<uint64_t> m_physicsStepCount;
    
    // Visualization settings
    struct VisualizationSettings {
        int window_width = 1200;
        int window_height = 900;
        const char* window_title = "RAON-RT MuJoCo Simulation (1kHz Physics)";
        double camera_azimuth = 90.0;
        double camera_elevation = -20.0;
        double camera_distance = 2.0;
        double camera_lookat[3] = {0.0, 0.0, 1.0};
        int visualization_fps = 60;     // Visualization rendering rate
        int physics_hz = 1000;          // Physics simulation rate (1kHz)
    } m_settings;
};

// Global convenience functions for RT threads
namespace MuJoCoViz {
    // Initialize visualization (call once during startup)
    bool Initialize(const char* modelFile);
    
    // Start/stop visualization + physics
    bool Start();
    void Stop();
    void Shutdown();
    
    // RT-safe state update (call from control threads)
    void UpdateRobotState(int numJoints, const double* positions, const double* velocities, const double* torques, double timestamp);
    
    // RT-safe physics commands (replaces direct mj_step calls)
    bool SendJointTorqueCommand(int joint_index, double torque, double timestamp);
    bool SendJointPositionCommand(int joint_index, double position, double timestamp);
    bool SendJointVelocityCommand(int joint_index, double velocity, double timestamp);
    bool SendPhysicsStepCommand(double timestamp);
    
    // RT-safe physics feedback
    bool GetJointFeedback(int joint_index, double& position, double& velocity, double& torque);
    
    // Check status
    bool IsActive();
}

#endif // __MUJOCO_VISUALIZATION_H__
