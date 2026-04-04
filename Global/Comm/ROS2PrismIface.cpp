/*****************************************************************************
*	Name: ConfigParser.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the ROS2 node wrapper
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ROS2PrismIface.h"

using namespace rclcpp;

CROS2Usr2CtrlSub::CROS2Usr2CtrlSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
	m_cSubscriber = create_subscription<prism_iface::msg::Usr2Ctrl>(GetTopicName(), GetHistDepth(), std::bind(&CROS2Usr2CtrlSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrUsr2CtrlSub(this);
	m_pCallbackSub = NULL;
    m_eNodeType = eUSR2CTRL_SUB;
	SetPublisher(FALSE);
}

void
CROS2Usr2CtrlSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2Usr2CtrlSub::proc_sub_callback(const prism_iface::msg::Usr2Ctrl& msg) const
{
	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard a message");
}

CROS2JointInfoPub::CROS2JointInfoPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    m_cPublisher = create_publisher<prism_iface::msg::JointInfo>(GetTopicName(), GetHistDepth());
    m_pSharedPtr = new SPtrJointInfoPub(this);
    m_eNodeType = eJOINTINFO_PUB;
    SetPublisher(TRUE);
}
    
void 
CROS2JointInfoPub::SetMessage(PRISM_IFACE_JOINTINFO astMsg)
{
	m_stMessage = astMsg;
    PublishMessage(m_stMessage);
}  

void
CROS2JointInfoPub::PublishMessage(PRISM_IFACE_JOINTINFO astMsg)
{
	m_cPublisher->publish(astMsg);
}

CROS2UserInfoPub::CROS2UserInfoPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    m_cPublisher = create_publisher<prism_iface::msg::UserInfo>(GetTopicName(), GetHistDepth());
    m_pSharedPtr = new SPtrUserInfoPub(this);
    m_eNodeType = eUSERINFO_PUB;
    SetPublisher(TRUE);
}
    
void 
CROS2UserInfoPub::SetMessage(PRISM_IFACE_USERINFO astMsg)
{
	m_stMessage = astMsg;
    PublishMessage(m_stMessage);
}  

void
CROS2UserInfoPub::PublishMessage(PRISM_IFACE_USERINFO astMsg)
{
	m_cPublisher->publish(astMsg);
}