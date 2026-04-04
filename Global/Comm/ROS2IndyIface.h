#ifndef __ROS2_INDY_IFACE__
#define __ROS2_INDY_IFACE__

#include <unistd.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

#include "ROS2Node.h"

// Indy interface message and service includes
#include "indy_iface/msg/joint_info.hpp"
#include "indy_iface/srv/get_gains.hpp"
#include "indy_iface/srv/set_gains.hpp"
#include "indy_iface/srv/set_pos.hpp"
#include "indy_iface/srv/set_all_pos.hpp"
#include "indy_iface/srv/set_control_mode.hpp"
#include "indy_iface/srv/get_control_mode.hpp"

using namespace std::chrono_literals;
using namespace rclcpp;

// Robot state data structure
struct IndyRobotState
{
    std::vector<double> positions;      // Joint positions [rad]
    std::vector<double> velocities;     // Joint velocities [rad/s]
    std::vector<double> torques;        // Joint torques [Nm]
    std::vector<uint8_t> faults;        // Fault status for each joint
    uint64_t timestamp;                 // Timestamp in nanoseconds
    
    IndyRobotState() : positions(6, 0.0), velocities(6, 0.0), torques(6, 0.0), faults(6, 0), timestamp(0) {}
};

// Control gains structure
struct IndyControlGains
{
    std::vector<double> kp;             // Proportional gains
    std::vector<double> kd;             // Derivative gains
    
    IndyControlGains() : kp(6, 0.0), kd(6, 0.0) {}
};

// Message type definitions
typedef indy_iface::msg::JointInfo INDY_IFACE_STATE;

// Service type definitions
typedef indy_iface::srv::GetGains INDY_IFACE_GET_GAINS;
typedef indy_iface::srv::SetGains INDY_IFACE_SET_GAINS;
typedef indy_iface::srv::SetPos INDY_IFACE_SET_POS;
typedef indy_iface::srv::SetAllPos INDY_IFACE_SET_ALL_POS;
typedef indy_iface::srv::SetControlMode INDY_IFACE_SET_CONTROL_MODE;
typedef indy_iface::srv::GetControlMode INDY_IFACE_GET_CONTROL_MODE;

// Publisher/Subscriber type definitions
typedef Publisher<indy_iface::msg::JointInfo>::SharedPtr ROS2_INDY_STATE_PUB;

// Service type definitions
typedef Service<indy_iface::srv::GetGains>::SharedPtr ROS2_GET_GAINS_SRV;
typedef Service<indy_iface::srv::SetGains>::SharedPtr ROS2_SET_GAINS_SRV;
typedef Service<indy_iface::srv::SetPos>::SharedPtr ROS2_SET_POS_SRV;
typedef Service<indy_iface::srv::SetAllPos>::SharedPtr ROS2_SET_ALL_POS_SRV;


// Client type definitions
typedef Client<indy_iface::srv::GetGains>::SharedPtr ROS2_GET_GAINS_CLI;
typedef Client<indy_iface::srv::SetGains>::SharedPtr ROS2_SET_GAINS_CLI;
typedef Client<indy_iface::srv::SetPos>::SharedPtr ROS2_SET_POS_CLI;
typedef Client<indy_iface::srv::SetAllPos>::SharedPtr ROS2_SET_ALL_POS_CLI;

// Callback function types
typedef std::function<IndyControlGains()> GetGainsCallback;
typedef std::function<bool(const IndyControlGains&)> SetGainsCallback;
typedef std::function<bool(uint32_t, double)> SetPosCallback;
typedef std::function<bool(const std::vector<double>&)> SetAllPosCallback;

typedef std::function<bool(uint8_t)> SetControlModeCallback;
typedef std::function<uint8_t()> GetControlModeCallback;

/* Publisher for Indy State */
class CROS2IndyStatePub : public CROS2Node
{
public:
    CROS2IndyStatePub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2IndyStatePub() {};

    void SetMessage(INDY_IFACE_STATE astMsg);
    void PublishState(const IndyRobotState& state);

protected:
    ROS2_INDY_STATE_PUB m_cPublisher;

private:
    void PublishMessage(INDY_IFACE_STATE astMsg);
    INDY_IFACE_STATE m_stMessage;
};

/* Service Server for GetGains */
class CROS2GetGainsSrv : public CROS2Node
{
public:
    CROS2GetGainsSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2GetGainsSrv() {};

    void SetCallback(GetGainsCallback callback) { m_callback = callback; }

protected:
    void HandleGetGains(const std::shared_ptr<INDY_IFACE_GET_GAINS::Request> request,
                       std::shared_ptr<INDY_IFACE_GET_GAINS::Response> response);

private:
    ROS2_GET_GAINS_SRV m_cService;
    GetGainsCallback m_callback;
};

/* Service Server for SetGains */
class CROS2SetGainsSrv : public CROS2Node
{
public:
    CROS2SetGainsSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetGainsSrv() {};

    void SetCallback(SetGainsCallback callback) { m_callback = callback; }

protected:
    void HandleSetGains(const std::shared_ptr<INDY_IFACE_SET_GAINS::Request> request,
                       std::shared_ptr<INDY_IFACE_SET_GAINS::Response> response);

private:
    ROS2_SET_GAINS_SRV m_cService;
    SetGainsCallback m_callback;
};

/* Service Server for SetPos */
class CROS2SetPosSrv : public CROS2Node
{
public:
    CROS2SetPosSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetPosSrv() {};

    void SetCallback(SetPosCallback callback) { m_callback = callback; }

protected:
    void HandleSetPos(const std::shared_ptr<INDY_IFACE_SET_POS::Request> request,
                     std::shared_ptr<INDY_IFACE_SET_POS::Response> response);

private:
    ROS2_SET_POS_SRV m_cService;
    SetPosCallback m_callback;
};

/* Service Server for SetAllPos */
class CROS2SetAllPosSrv : public CROS2Node
{
public:
    CROS2SetAllPosSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetAllPosSrv() {};

    void SetCallback(SetAllPosCallback callback) { m_callback = callback; }

protected:
    void HandleSetAllPos(const std::shared_ptr<INDY_IFACE_SET_ALL_POS::Request> request,
                        std::shared_ptr<INDY_IFACE_SET_ALL_POS::Response> response);

private:
    ROS2_SET_ALL_POS_SRV m_cService;
    SetAllPosCallback m_callback;
};

/* Service Client Classes remain the same */
class CROS2GetGainsCli : public CROS2Node
{
public:
    CROS2GetGainsCli(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2GetGainsCli() {};

    bool CallGetGains(IndyControlGains& gains);

private:
    ROS2_GET_GAINS_CLI m_cClient;
};

class CROS2SetGainsCli : public CROS2Node
{
public:
    CROS2SetGainsCli(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetGainsCli() {};

    bool CallSetGains(const IndyControlGains& gains);

private:
    ROS2_SET_GAINS_CLI m_cClient;
};

class CROS2SetPosCli : public CROS2Node
{
public:
    CROS2SetPosCli(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetPosCli() {};

    bool CallSetPos(uint32_t auJointIndex, double adPosition);

private:
    ROS2_SET_POS_CLI m_cClient;
};

class CROS2SetAllPosCli : public CROS2Node
{
public:
    CROS2SetAllPosCli(TSTRING astrNodeName, TSTRING astrServiceName);
    virtual ~CROS2SetAllPosCli() {};

    bool CallSetAllPos(const std::vector<double>& avPositions);

private:
    ROS2_SET_ALL_POS_CLI m_cClient;
};

// Add these service server classes
class CROS2SetControlModeSrv : public CROS2Node
{
private:
    rclcpp::Service<INDY_IFACE_SET_CONTROL_MODE>::SharedPtr m_cService;
    SetControlModeCallback m_callback;

public:
    CROS2SetControlModeSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    ~CROS2SetControlModeSrv() {}

    void SetCallback(SetControlModeCallback callback) { m_callback = callback; }
    void HandleSetControlMode(const std::shared_ptr<INDY_IFACE_SET_CONTROL_MODE::Request> request,
                              std::shared_ptr<INDY_IFACE_SET_CONTROL_MODE::Response> response);
};

class CROS2GetControlModeSrv : public CROS2Node
{
private:
    rclcpp::Service<INDY_IFACE_GET_CONTROL_MODE>::SharedPtr m_cService;
    GetControlModeCallback m_callback;

public:
    CROS2GetControlModeSrv(TSTRING astrNodeName, TSTRING astrServiceName);
    ~CROS2GetControlModeSrv() {}

    void SetCallback(GetControlModeCallback callback) { m_callback = callback; }
    void HandleGetControlMode(const std::shared_ptr<INDY_IFACE_GET_CONTROL_MODE::Request> request,
                              std::shared_ptr<INDY_IFACE_GET_CONTROL_MODE::Response> response);
};

// Add these service client classes
class CROS2SetControlModeCli : public CROS2Node
{
private:
    rclcpp::Client<INDY_IFACE_SET_CONTROL_MODE>::SharedPtr m_cClient;

public:
    CROS2SetControlModeCli(TSTRING astrNodeName, TSTRING astrServiceName);
    ~CROS2SetControlModeCli() {}

    bool CallSetControlMode(uint8_t control_mode);
};

class CROS2GetControlModeCli : public CROS2Node
{
private:
    rclcpp::Client<INDY_IFACE_GET_CONTROL_MODE>::SharedPtr m_cClient;

public:
    CROS2GetControlModeCli(TSTRING astrNodeName, TSTRING astrServiceName);
    ~CROS2GetControlModeCli() {}

    bool CallGetControlMode(uint8_t& control_mode);
};

#endif // __ROS2_INDY_IFACE__