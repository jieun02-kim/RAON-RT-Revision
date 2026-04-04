/*****************************************************************************
*	Name: ROS2Node.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header of the communication interface based on ROS2 nodes
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROS2_EXECUTOR__
#define __ROS2_EXECUTOR__

#include "Defines.h"
#include <sys/stat.h>
#include <sys/types.h>

#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "rclcpp/executor.hpp"
// #include "rclcpp/executors.hpp"

#include "rclcpp/rclcpp/rclcpp.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp/executor.hpp"
#include "rclcpp/rclcpp/executors.hpp"

#include "ROS2Common.h"
#include "ROS2Node.h"

/* SuperMicroSurgery Robot */
// #include "ROS2PrismIface.h"

// #include "ROS2PointCloud.h"
// #include "ROS2AxisInfo.h"
// #include "ROS2Imu.h"

// Include the Indy Interface
#include "ROS2IndyIface.h"  // Add this line

using namespace std::chrono_literals;
using namespace rclcpp;

class CROS2Executor
{
public:
    CROS2Executor();
    CROS2Executor(int domain_id);
    virtual ~CROS2Executor();

    BOOL Init();
    BOOL Stop();
    BOOL DeInit();
    BOOL AddNode(CROS2Node*);
    BOOL IsInit() { return m_bInit; }

private:
    ROS2_SPIN_THREAD    CreateSpinThread(CROS2Node*, BOOL abMulti=TRUE);
    
private:
    VEC_SPIN_THREAD     m_vecSpinThread;
    VEC_PROS_NODE       m_vecROS2Nodes;
    BOOL                m_bInit;
    rclcpp::executors::StaticSingleThreadedExecutor* m_pExecutor;
};


#endif