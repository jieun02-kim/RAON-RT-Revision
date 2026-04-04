/*****************************************************************************
*	Name: ROS2IndyIface.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of the ROS2 Indy interface wrapper
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#include "ROS2IndyIface.h"

using namespace rclcpp;

/****************************************************************************/
/* Publisher for Indy State */
/****************************************************************************/
CROS2IndyStatePub::CROS2IndyStatePub(TSTRING astrNodeName, TSTRING astrTopicName) 
    : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    m_cPublisher = create_publisher<indy_iface::msg::JointInfo>(GetTopicName(), GetHistDepth());
    m_eNodeType = eINDY_STATE_PUB;
    SetPublisher(TRUE);
}

void CROS2IndyStatePub::SetMessage(INDY_IFACE_STATE astMsg)
{
    m_stMessage = astMsg;
    PublishMessage(m_stMessage);
}

void CROS2IndyStatePub::PublishMessage(INDY_IFACE_STATE astMsg)
{
    m_cPublisher->publish(astMsg);
}

void CROS2IndyStatePub::PublishState(const IndyRobotState& state)
{
    INDY_IFACE_STATE msg;
    
    vectorToArray(state.faults, msg.is_fault);
    vectorToArray(state.positions, msg.actual_pos);
    vectorToArray(state.velocities, msg.actual_vel);
    vectorToArray(state.torques, msg.actual_tor);
    SetMessage(msg);
}

/****************************************************************************/
/* Service Server for GetGains */
/****************************************************************************/
CROS2GetGainsSrv::CROS2GetGainsSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::GetGains>(
        GetTopicName(), 
        std::bind(&CROS2GetGainsSrv::HandleGetGains, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eGET_GAINS_SRV;
    SetPublisher(FALSE);
}

void CROS2GetGainsSrv::HandleGetGains(const std::shared_ptr<INDY_IFACE_GET_GAINS::Request> request,
                                      std::shared_ptr<INDY_IFACE_GET_GAINS::Response> response)
{
    (void)request;  // Unused parameter
    
    if (!m_callback)
    {
        response->success = false;
        return;
    }
    
    try
    {
        IndyControlGains gains = m_callback();
        vectorToArray(gains.kp, response->kp);
        vectorToArray(gains.kd, response->kd);
        response->success = true;
    }
    catch (const std::exception& e)
    {
        response->success = false;
    }
}

/****************************************************************************/
/* Service Server for SetGains */
/****************************************************************************/
CROS2SetGainsSrv::CROS2SetGainsSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::SetGains>(
        GetTopicName(), 
        std::bind(&CROS2SetGainsSrv::HandleSetGains, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eSET_GAINS_SRV;
    SetPublisher(FALSE);
}

void CROS2SetGainsSrv::HandleSetGains(const std::shared_ptr<INDY_IFACE_SET_GAINS::Request> request,
                                      std::shared_ptr<INDY_IFACE_SET_GAINS::Response> response)
{
    if (!m_callback)
    {
        response->success = false;
        return;
    }
    
    if (request->kp.size() != 6 || request->kd.size() != 6)
    {
        response->success = false;
        return;
    }
    
    try
    {
        IndyControlGains gains;
        arrayToVector(request->kp, gains.kp);
        arrayToVector(request->kd, gains.kd);
        if (m_callback(gains))
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
    catch (const std::exception& e)
    {
        response->success = false;
    }
}

/****************************************************************************/
/* Service Server for SetPos */
/****************************************************************************/
CROS2SetPosSrv::CROS2SetPosSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::SetPos>(
        GetTopicName(), 
        std::bind(&CROS2SetPosSrv::HandleSetPos, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eSET_POS_SRV;
    SetPublisher(FALSE);
}

void CROS2SetPosSrv::HandleSetPos(const std::shared_ptr<INDY_IFACE_SET_POS::Request> request,
                                  std::shared_ptr<INDY_IFACE_SET_POS::Response> response)
{
    if (!m_callback)
    {
        response->success = false;
        return;
    }
    
    if (request->joint_index >= 6)
    {
        response->success = false;
        return;
    }
    
    try
    {
        if (m_callback(request->joint_index, request->position))
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
    catch (const std::exception& e)
    {
        response->success = false;
    }
}

/****************************************************************************/
/* Service Server for SetAllPos */
/****************************************************************************/
CROS2SetAllPosSrv::CROS2SetAllPosSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::SetAllPos>(
        GetTopicName(), 
        std::bind(&CROS2SetAllPosSrv::HandleSetAllPos, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eSET_ALL_POS_SRV;
    SetPublisher(FALSE);
}

void CROS2SetAllPosSrv::HandleSetAllPos(const std::shared_ptr<INDY_IFACE_SET_ALL_POS::Request> request,
                                        std::shared_ptr<INDY_IFACE_SET_ALL_POS::Response> response)
{
    if (!m_callback)
    {
        response->success = false;
        return;
    }
    
    if (request->positions.size() != 6)
    {
        response->success = false;
        return;
    }
    
    try
    {
        std::vector<double> positions;
        arrayToVector(request->positions, positions);

        if (m_callback(positions))
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
    catch (const std::exception& e)
    {
        response->success = false;
    }
}

/****************************************************************************/
/* Service Client implementations remain similar but use the new structures */
/****************************************************************************/
CROS2GetGainsCli::CROS2GetGainsCli(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cClient = create_client<indy_iface::srv::GetGains>(GetTopicName());
    m_eNodeType = eGET_GAINS_CLI;
    SetPublisher(FALSE);
}

bool CROS2GetGainsCli::CallGetGains(IndyControlGains& gains)
{
    if (!m_cClient->wait_for_service(std::chrono::seconds(1)))
    {
        return false;
    }
    
    auto request = std::make_shared<indy_iface::srv::GetGains::Request>();
    auto future = m_cClient->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        if (response->success)
        {
            arrayToVector(response->kp, gains.kp);
            arrayToVector(response->kd, gains.kd);
            return true;
        }
    }
    
    return false;
}

/****************************************************************************/
/* Service Client for SetPos */
/****************************************************************************/
CROS2SetPosCli::CROS2SetPosCli(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cClient = create_client<indy_iface::srv::SetPos>(GetTopicName());
    m_eNodeType = eSET_POS_CLI;
    SetPublisher(FALSE);
}

bool CROS2SetPosCli::CallSetPos(uint32_t auJointIndex, double adPosition)
{
    if (!m_cClient->wait_for_service(std::chrono::seconds(1)))
    {
        return false;
    }
    
    auto request = std::make_shared<indy_iface::srv::SetPos::Request>();
    request->joint_index = auJointIndex;
    request->position = adPosition;
    
    auto future = m_cClient->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        return response->success;
    }
    
    return false;
}

/****************************************************************************/
/* Service Client for SetAllPos */
/****************************************************************************/
CROS2SetAllPosCli::CROS2SetAllPosCli(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cClient = create_client<indy_iface::srv::SetAllPos>(GetTopicName());
    m_eNodeType = eSET_ALL_POS_CLI;
    SetPublisher(FALSE);
}

bool CROS2SetAllPosCli::CallSetAllPos(const std::vector<double>& avPositions)
{
    if (!m_cClient->wait_for_service(std::chrono::seconds(1)))
    {
        return false;
    }
    
    auto request = std::make_shared<indy_iface::srv::SetAllPos::Request>();
    vectorToArray(avPositions, request->positions);
    
    auto future = m_cClient->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        return response->success;
    }
    
    return false;
}

/****************************************************************************/
/* Service Server for SetControlMode */
/****************************************************************************/
CROS2SetControlModeSrv::CROS2SetControlModeSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::SetControlMode>(
        GetTopicName(), 
        std::bind(&CROS2SetControlModeSrv::HandleSetControlMode, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eSET_CONTROL_MODE_SRV;
    SetPublisher(FALSE);
}

void CROS2SetControlModeSrv::HandleSetControlMode(const std::shared_ptr<INDY_IFACE_SET_CONTROL_MODE::Request> request,
                                                  std::shared_ptr<INDY_IFACE_SET_CONTROL_MODE::Response> response)
{
    if (!m_callback)
    {
        response->success = false;
        return;
    }
    
    if (request->control_mode > 3)
    {
        response->success = false;
        return;
    }
    
    try
    {
        response->success = m_callback(request->control_mode);
    }
    catch (const std::exception& e)
    {
        response->success = false;
    }
}

/****************************************************************************/
/* Service Server for GetControlMode */
/****************************************************************************/
CROS2GetControlModeSrv::CROS2GetControlModeSrv(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cService = create_service<indy_iface::srv::GetControlMode>(
        GetTopicName(), 
        std::bind(&CROS2GetControlModeSrv::HandleGetControlMode, this, std::placeholders::_1, std::placeholders::_2)
    );
    m_eNodeType = eGET_CONTROL_MODE_SRV;
    SetPublisher(FALSE);
}

void CROS2GetControlModeSrv::HandleGetControlMode(const std::shared_ptr<INDY_IFACE_GET_CONTROL_MODE::Request> request,
                                                  std::shared_ptr<INDY_IFACE_GET_CONTROL_MODE::Response> response)
{
    (void)request;  // Unused parameter
    
    if (!m_callback)
    {
        response->success = false;
        response->control_mode = 0;
        return;
    }
    
    try
    {
        response->control_mode = m_callback();
        response->success = true;
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->control_mode = 0;
    }
}

/****************************************************************************/
/* Service Client for SetControlMode */
/****************************************************************************/
CROS2SetControlModeCli::CROS2SetControlModeCli(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cClient = create_client<indy_iface::srv::SetControlMode>(GetTopicName());
    m_eNodeType = eSET_CONTROL_MODE_CLI;
    SetPublisher(FALSE);
}

bool CROS2SetControlModeCli::CallSetControlMode(uint8_t control_mode)
{
    if (!m_cClient->wait_for_service(std::chrono::seconds(1)))
    {
        return false;
    }
    
    auto request = std::make_shared<indy_iface::srv::SetControlMode::Request>();
    request->control_mode = control_mode;
    
    auto future = m_cClient->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        return response->success;
    }
    
    return false;
}

/****************************************************************************/
/* Service Client for GetControlMode */
/****************************************************************************/
CROS2GetControlModeCli::CROS2GetControlModeCli(TSTRING astrNodeName, TSTRING astrServiceName)
    : CROS2Node(astrNodeName, astrServiceName, HISTORY_DEPTH)
{
    m_cClient = create_client<indy_iface::srv::GetControlMode>(GetTopicName());
    m_eNodeType = eGET_CONTROL_MODE_CLI;
    SetPublisher(FALSE);
}

bool CROS2GetControlModeCli::CallGetControlMode(uint8_t& control_mode)
{
    if (!m_cClient->wait_for_service(std::chrono::seconds(1)))
    {
        return false;
    }
    
    auto request = std::make_shared<indy_iface::srv::GetControlMode::Request>();
    auto future = m_cClient->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        if (response->success)
        {
            control_mode = response->control_mode;
            return true;
        }
    }
    
    return false;
}