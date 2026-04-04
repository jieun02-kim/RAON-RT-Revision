/*****************************************************************************
*	Name: ROS2Common.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Common Header for the defintion of ROS2 wrappers
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROS2_COMMONS__
#define __ROS2_COMMONS__

#include "Defines.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

#include "indy_iface/msg/joint_info.hpp"
#include "indy_iface/srv/get_gains.hpp"
#include "indy_iface/srv/set_gains.hpp"
#include "indy_iface/srv/set_pos.hpp"
#include "indy_iface/srv/set_all_pos.hpp"

// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

// #include "axis_info/msg/axis_cmd.hpp"
// #include "axis_info/msg/axis_state.hpp"

#include "std_msgs/msg/byte_multi_array.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#define	 HISTORY_DEPTH (1)

// Add these helper functions after the #define
template<typename T, size_t N>
inline void vectorToArray(const std::vector<T>& vec, std::array<T, N>& arr) {
    size_t copySize = std::min(vec.size(), N);
    std::copy(vec.begin(), vec.begin() + copySize, arr.begin());
    if (copySize < N) {
        std::fill(arr.begin() + copySize, arr.end(), T{});
    }
}

template<typename T, size_t N>
inline void arrayToVector(const std::array<T, N>& arr, std::vector<T>& vec) {
    vec.assign(arr.begin(), arr.end());
}

using namespace std::chrono_literals;
using namespace rclcpp;


class CROS2Node;
class CROS2StrPub;
class CROS2StrSub;

/* Indy7 */
class CROS2IndyStatePub;
class CROS2GetGainsSrv;
class CROS2SetGainsSrv;
class CROS2SetPosSrv;
class CROS2SetAllPosSrv;
class CROS2GetGainsCli;
class CROS2SetGainsCli;
class CROS2SetPosCli;
class CROS2SetAllPosCli;


/* SuperMicroSurgery Robot */
// class CROS2Usr2CtrlSub;
// class CROS2JointInfoPub;
// class CROS2UserInfoPub;

// class CROS2ImuPub;
// class CROS2ImuSub;

// class CROS2PointCloudPub;
// class CROS2PointCloudSub;

// class CROS2JointInfoPub;
// class CROS2JointInfoSub;

// class CROS2AxisInfoCmdPub;
// class CROS2AxisInfoCmdSub;

// class CROS2AxisInfoStatePub;
// class CROS2AxisInfoStateSub;

typedef TimerBase::SharedPtr ROS2_TIMER;
typedef Publisher<std_msgs::msg::String>::SharedPtr ROS2_STR_PUB;
typedef Subscription<std_msgs::msg::String>::SharedPtr ROS2_STR_SUB;


typedef Publisher<indy_iface::msg::JointInfo>::SharedPtr ROS2_INDY_STATE_PUB;
typedef Service<indy_iface::srv::GetGains>::SharedPtr ROS2_GET_GAINS_SRV;
typedef Service<indy_iface::srv::SetGains>::SharedPtr ROS2_SET_GAINS_SRV;
typedef Service<indy_iface::srv::SetPos>::SharedPtr ROS2_SET_POS_SRV;
typedef Service<indy_iface::srv::SetAllPos>::SharedPtr ROS2_SET_ALL_POS_SRV;
typedef Client<indy_iface::srv::GetGains>::SharedPtr ROS2_GET_GAINS_CLI;
typedef Client<indy_iface::srv::SetGains>::SharedPtr ROS2_SET_GAINS_CLI;
typedef Client<indy_iface::srv::SetPos>::SharedPtr ROS2_SET_POS_CLI;
typedef Client<indy_iface::srv::SetAllPos>::SharedPtr ROS2_SET_ALL_POS_CLI;

// typedef Publisher<sensor_msgs::msg::Imu>::SharedPtr ROS2_IMU_PUB;
// typedef Subscription<sensor_msgs::msg::Imu>::SharedPtr ROS2_IMU_SUB;

// typedef Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ROS2_POINTCLOUD_PUB;
// typedef Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ROS2_POINTCLOUD_SUB;

// typedef Publisher<sensor_msgs::msg::JointState>::SharedPtr ROS2_JOINTINFO_PUB;
// typedef Subscription<sensor_msgs::msg::JointState>::SharedPtr ROS2_JOINTINFO_SUB;

// typedef Publisher<axis_info::msg::AxisCmd>::SharedPtr ROS2_AXISINFO_CMD_PUB;
// typedef Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr ROS2_AXISINFO_CMD_SUB;

// typedef Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr ROS2_AXISINFO_STATE_PUB;
// typedef Subscription<axis_info::msg::AxisState>::SharedPtr ROS2_AXISINFO_STATE_SUB;


typedef executors::SingleThreadedExecutor   ROS2_EXECUTOR_SINGLE;
typedef executors::MultiThreadedExecutor    ROS2_EXECUTOR_MULTI;
typedef Executor ROS2_EXECUTOR, *PROS2_EXECUTOR;

typedef std::vector<CROS2Node*> VEC_PROS_NODE;
typedef std::shared_ptr<std::thread> ROS2_SPIN_THREAD;
typedef std::vector<std::shared_ptr<std::thread>> VEC_SPIN_THREAD;


/* todo: make these with template later */
typedef std::shared_ptr<CROS2StrPub> SPtrPublisher;
typedef std::shared_ptr<CROS2StrSub> SPtrSubscriber;
// typedef std::shared_ptr<CROS2Usr2CtrlSub> SPtrUsr2CtrlSub;
// typedef std::shared_ptr<CROS2JointInfoPub> SPtrJointInfoPub;
// typedef std::shared_ptr<CROS2UserInfoPub> SPtrUserInfoPub;

// typedef std::shared_ptr<CROS2ImuPub> SPtrImuPub;
// typedef std::shared_ptr<CROS2ImuSub> SPtrImuSub;

// typedef std::shared_ptr<CROS2PointCloudPub> SPtrPointCloudPub;
// typedef std::shared_ptr<CROS2PointCloudSub> SPtrPointCloudSub;

// typedef std::shared_ptr<CROS2JointInfoPub> SPtrJointInfoPub;
// typedef std::shared_ptr<CROS2JointInfoSub> SPtrJointInfoSub;

// typedef std::shared_ptr<CROS2AxisInfoCmdPub> SPtrAxisInfoCmdPub;
// typedef std::shared_ptr<CROS2AxisInfoCmdSub> SPtrAxisInfoCmdSub;

// typedef std::shared_ptr<CROS2AxisInfoStatePub> SPtrAxisInfoStatePub;
// typedef std::shared_ptr<CROS2AxisInfoStateSub> SPtrAxisInfoStateSub;

typedef enum
{
	eSTR_PUB = 0,
	eSTR_SUB,
	eUSR2CTRL_SUB,
	eUSR2CTRL_PUB,
    eJOINTINFO_SUB,
    eJOINTINFO_PUB,
	eUSERINFO_SUB,
    eUSERINFO_PUB,
	ePOINTCLOUD_SUB,
	ePOINTCLOUD_PUB,
	eAXISINFO_CMD_SUB,
	eAXISINFO_CMD_PUB,
	eAXISINFO_STATE_PUB,
	eAXISINFO_STATE_SUB,
	eIMU_PUB,
	eIMU_SUB,
	eINDY_STATE_PUB,
    eGET_GAINS_SRV,
    eSET_GAINS_SRV,
    eSET_POS_SRV,
    eSET_ALL_POS_SRV,
    eGET_GAINS_CLI,
    eSET_GAINS_CLI,
    eSET_POS_CLI,
    eSET_ALL_POS_CLI,
    eSET_CONTROL_MODE_SRV,
    eGET_CONTROL_MODE_SRV,
    eSET_CONTROL_MODE_CLI,
    eGET_CONTROL_MODE_CLI,
} eNodeType;


#endif