/*****************************************************************************
*	Name: ConfigParser.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header of the communication interface based on ROS2 nodes
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROS2_PRISM_IFACE__
#define __ROS2_PRISM_IFACE__

#include "Defines.h"
#include <sys/stat.h>
#include <sys/types.h>

#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

#include "ROS2Node.h"

#include "prism_iface/msg/usr2_ctrl.hpp"
#include "prism_iface/msg/joint_info.hpp"
#include "prism_iface/msg/user_info.hpp"


using namespace std::chrono_literals;
using namespace rclcpp;

typedef prism_iface::msg::Usr2Ctrl  PRISM_IFACE_USR2CTRL;
typedef prism_iface::msg::JointInfo PRISM_IFACE_JOINTINFO;
typedef prism_iface::msg::UserInfo  PRISM_IFACE_USERINFO;

typedef Subscription<prism_iface::msg::Usr2Ctrl>::SharedPtr ROS2_USR2CTRL_SUB;
typedef Publisher<prism_iface::msg::JointInfo>::SharedPtr   ROS2_JOINTINFO_PUB;
typedef Publisher<prism_iface::msg::UserInfo>::SharedPtr   ROS2_USERINFO_PUB;

/* todo: make these with template later */
class CROS2Usr2CtrlSub : public CROS2Node
{

public:
    CROS2Usr2CtrlSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2Usr2CtrlSub() {};

    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrUsr2CtrlSub GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const prism_iface::msg::Usr2Ctrl & msg) const;

protected:
    ROS2_USR2CTRL_SUB       m_cSubscriber;
    GLOBAL_CALLBACK_FN      m_pCallbackSub;

private:
    SPtrUsr2CtrlSub*      m_pSharedPtr;

};

class CROS2JointInfoPub : public CROS2Node
{
public:
    CROS2JointInfoPub(TSTRING astrNodeName, TSTRING astrTopicName);
    SPtrJointInfoPub GetSharedPtr() { return *m_pSharedPtr; }

    void SetMessage(PRISM_IFACE_JOINTINFO astMsg);
    virtual ~CROS2JointInfoPub() {};

protected:
    ROS2_JOINTINFO_PUB      m_cPublisher;

private:
    void                    PublishMessage(PRISM_IFACE_JOINTINFO astMsg);
    SPtrJointInfoPub*       m_pSharedPtr;
    PRISM_IFACE_JOINTINFO   m_stMessage;

};

class CROS2UserInfoPub : public CROS2Node
{
public:
    CROS2UserInfoPub(TSTRING astrNodeName, TSTRING astrTopicName);
    SPtrUserInfoPub GetSharedPtr() { return *m_pSharedPtr; }

    void SetMessage(PRISM_IFACE_USERINFO astMsg);
    virtual ~CROS2UserInfoPub() {};

protected:
    ROS2_USERINFO_PUB       m_cPublisher;

private:
    void                    PublishMessage(PRISM_IFACE_USERINFO astMsg);
    SPtrUserInfoPub*        m_pSharedPtr;
    PRISM_IFACE_USERINFO    m_stMessage;

};

#endif // __ROS2_PRISM_IFACE__