/*****************************************************************************
*	Name: ROS2Node.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header of the communication interface based on ROS2 nodes
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROS2_NODE__
#define __ROS2_NODE__

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

using namespace std::chrono_literals;
using namespace rclcpp;

class CROS2Node : public Node
{
public:
    CROS2Node(TSTRING astrNodeName, TSTRING astrTopicName, INT anHistDepth);
    virtual ~CROS2Node() {};
    
    TSTRING GetTopicName() { return m_strTopicName; }
    TSTRING GetNodeName() { return m_strNodeName; }
    INT     GetHistDepth()  {return m_nHistDepth;}
    BOOL    IsPublisher() { return m_bIsPublisher; }
    BOOL    IsPeriodic() { return m_bIsPeriodic; }
    virtual void SetPeriod(INT64 anPeriodInMs);
    eNodeType GetNodeType() {return m_eNodeType;}
    std::shared_ptr<rclcpp::Node> GetSharedPtr() { 
        return shared_from_this(); 
    }
protected:
    void        SetPublisher(BOOL abPub) { m_bIsPublisher = abPub; }
    TSTRING     m_strNodeName;
    TSTRING     m_strTopicName;
    INT64       m_nPeriod;
    INT         m_nHistDepth;
    eNodeType   m_eNodeType;


private:
    BOOL    m_bIsPublisher;
    BOOL    m_bIsPeriodic;

protected:
    virtual void proc_timer_callback();

};

class CROS2StrPub : public CROS2Node
{
public:
    CROS2StrPub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2StrPub() {};
    virtual void SetPeriod(INT64 anPeriodInMs);
    
    void SetMessage(TSTRING astrMsg, BOOL abForce = FALSE);
    SPtrPublisher GetSharedPtr() { return *m_pSharedPtr; }

protected:
    virtual void proc_timer_callback();

protected:
    ROS2_TIMER      m_cTimer;
    ROS2_STR_PUB    m_cPublisher;

private:
    void            PublishMessage(TSTRING astrMsg);
    TSTRING         m_strMessage;
    SPtrPublisher*  m_pSharedPtr;

};

class CROS2StrSub : public CROS2Node
{
public:
    CROS2StrSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2StrSub() {};
    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrSubscriber GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const std_msgs::msg::String::SharedPtr msg) const;

protected:
    ROS2_STR_SUB        m_cSubscriber;
    GLOBAL_CALLBACK_FN	m_pCallbackSub;

private:
    SPtrSubscriber*      m_pSharedPtr;
};

#endif // __ROS2_NODE__
