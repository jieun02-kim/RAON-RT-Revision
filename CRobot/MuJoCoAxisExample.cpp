/*****************************************************************************
*	Name: MuJoCoAxisExample.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Example usage of CAxisMuJoCo with ROS2 integration
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "AxisMuJoCo.h"
#include <vector>
#include <memory>

// Example configuration structure similar to your INDY7.cfg
struct MuJoCoAxisConfig {
	int mjJointId;
	bool enabled;
	bool connected;
	std::string name;
	double encoderResolution;
	double gearRatio;
	double transmissionRatio;
	double torqueConstant;
	double currentRatio;
	double operatingVelocity;
	double operatingAcceleration;
	double positionLimitUpper;
	double positionLimitLower;
	double velocityLimitUpper;
	double torqueLimitUpper;
};

class MuJoCoRobotController 
{
public:
	MuJoCoRobotController() 
	{
		// Initialize 6 axes for INDY7 robot (similar to your INDY7.cfg)
		InitializeAxes();
	}
	
	~MuJoCoRobotController() 
	{
		Shutdown();
	}
	
	bool Initialize(mjModel* model, mjData* data)
	{
		m_pMjModel = model;
		m_pMjData = data;
		
		// Initialize all axes
		for (auto& axis : m_axes)
		{
			if (!axis->Init(model, data))
			{
				// Handle initialization error
				return false;
			}
			
			// Auto servo on if configured
			if (axis->IsEnabled())
			{
				axis->ServoOn();
			}
		}
		
		m_bInitialized = true;
		return true;
	}
	
	void Update()
	{
		if (!m_bInitialized) return;
		
		// Update all axes from MuJoCo simulation
		for (auto& axis : m_axes)
		{
			axis->UpdateFromMuJoCo();
		}
		
		// Apply control commands to MuJoCo
		for (auto& axis : m_axes)
		{
			axis->ApplyToMuJoCo();
		}
	}
	
	// Command interface (can be called from ROS2 callbacks)
	bool MoveJoint(int jointId, double position)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->MoveAxis(position);
		}
		return false;
	}
	
	bool MoveJointVelocity(int jointId, double velocity)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->MoveVelocity(velocity);
		}
		return false;
	}
	
	bool MoveJointTorque(int jointId, double torque)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->MoveTorque(torque);
		}
		return false;
	}
	
	// State queries (for ROS2 publishing)
	double GetJointPosition(int jointId)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->GetCurrentPos();
		}
		return 0.0;
	}
	
	double GetJointVelocity(int jointId)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->GetCurrentVel();
		}
		return 0.0;
	}
	
	double GetJointTorque(int jointId)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->GetCurrentTor();
		}
		return 0.0;
	}
	
	bool IsJointReady(int jointId)
	{
		if (jointId >= 0 && jointId < m_axes.size())
		{
			return m_axes[jointId]->IsServoOn();
		}
		return false;
	}
	
	void Shutdown()
	{
		for (auto& axis : m_axes)
		{
			axis->ServoOff();
			axis->DeInit();
		}
		m_bInitialized = false;
	}

private:
	void InitializeAxes()
	{
		// Configuration based on your INDY7.cfg
		std::vector<MuJoCoAxisConfig> configs = {
			// Joint 0
			{0, true, true, "AXIS 0", 65536, 121, 1.0, 0.0884, 48.0, 130, 10000, 35.0, -35.0, 150, 117.73},
			// Joint 1  
			{1, true, true, "AXIS 1", 65536, 121, 1.0, 0.0884, 48.0, 130, 10000, 35.0, -35.0, 150, 117.73},
			// Joint 2
			{2, true, true, "AXIS 2", 65536, 121, 1.0, 0.087, 96.0, 125, 10000, 35.0, -35.0, 150, 47.5},
			// Joint 3
			{3, true, true, "AXIS 3", 65536, 101, 1.0, 0.058, 96.0, 155, 10000, 35.0, -35.0, 178, 21.0},
			// Joint 4
			{4, true, true, "AXIS 4", 65536, 101, 1.0, 0.058, 96.0, 155, 10000, 35.0, -35.0, 178, 21.0},
			// Joint 5
			{5, true, true, "AXIS 5", 65536, 101, 1.0, 0.058, 96.0, 155, 10000, 35.0, -35.0, 178, 21.0}
		};
		
		for (const auto& config : configs)
		{
			auto axis = std::make_unique<CAxisMuJoCo>(
				eAxisRevolute,
				config.mjJointId,
				config.encoderResolution,
				config.gearRatio,
				config.transmissionRatio,
				TRUE,  // absolute encoder
				TRUE,  // CCW
				config.enabled
			);
			
			// Configure axis parameters
			axis->SetName(config.name);
			axis->SetTorqueConstant(config.torqueConstant);
			axis->SetCurrentRatio(config.currentRatio);
			
			// Set limits (convert degrees to radians)
			axis->SetPositionLimits(config.positionLimitLower, config.positionLimitUpper, false); // degrees
			axis->SetVelocityLimits(config.velocityLimitUpper);
			axis->SetTorqueLimits(config.torqueLimitUpper);
			
			// Set motion parameters
			axis->SetVelocity(config.operatingVelocity);
			axis->SetAcceleration(config.operatingAcceleration);
			
			// Set homing method
			axis->SetHomingMethod(eAxisHomeStartPos);
			
			m_axes.push_back(std::move(axis));
		}
	}

private:
	std::vector<std::unique_ptr<CAxisMuJoCo>> m_axes;
	mjModel* m_pMjModel = nullptr;
	mjData* m_pMjData = nullptr;
	bool m_bInitialized = false;
};

// Example of how this could be integrated with ROS2
/*
class MuJoCoROS2Node : public rclcpp::Node 
{
public:
	MuJoCoROS2Node() : Node("mujoco_robot_node")
	{
		// Initialize MuJoCo
		// ... load model, initialize data ...
		
		// Initialize robot controller
		m_robotController = std::make_unique<MuJoCoRobotController>();
		m_robotController->Initialize(mjModel, mjData);
		
		// Create ROS2 interfaces
		m_jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(
			"joint_states", 10);
			
		m_jointCommandSubscriber = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_commands", 10,
			std::bind(&MuJoCoROS2Node::jointCommandCallback, this, std::placeholders::_1));
		
		// Create timer for simulation update
		m_timer = this->create_wall_timer(
			std::chrono::milliseconds(1),  // 1kHz update rate
			std::bind(&MuJoCoROS2Node::updateCallback, this));
	}

private:
	void updateCallback()
	{
		// Step MuJoCo simulation
		mj_step(mjModel, mjData);
		
		// Update robot controller
		m_robotController->Update();
		
		// Publish joint states
		publishJointStates();
	}
	
	void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		// Process joint commands and send to robot controller
		for (size_t i = 0; i < msg->position.size(); ++i)
		{
			m_robotController->MoveJoint(i, msg->position[i]);
		}
	}
	
	void publishJointStates()
	{
		auto msg = sensor_msgs::msg::JointState();
		msg.header.stamp = this->now();
		
		for (int i = 0; i < 6; ++i)  // 6 joints for INDY7
		{
			msg.name.push_back("joint_" + std::to_string(i));
			msg.position.push_back(m_robotController->GetJointPosition(i));
			msg.velocity.push_back(m_robotController->GetJointVelocity(i));
			msg.effort.push_back(m_robotController->GetJointTorque(i));
		}
		
		m_jointStatePublisher->publish(msg);
	}

private:
	std::unique_ptr<MuJoCoRobotController> m_robotController;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatePublisher;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointCommandSubscriber;
	rclcpp::TimerBase::SharedPtr m_timer;
};
*/
