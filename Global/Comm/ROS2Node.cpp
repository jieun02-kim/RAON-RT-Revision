/*****************************************************************************
*	Name: ROS2Node.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the ROS2 node wrapper
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ROS2Node.h"

using namespace rclcpp;


/* ROS NODE */
CROS2Node::CROS2Node(TSTRING astrNodeName, TSTRING astrTopicName, INT anHistDepth) : Node(astrNodeName)
{
	m_strNodeName = astrNodeName;
	m_strTopicName = astrTopicName;
	m_nHistDepth = anHistDepth;
}

void
CROS2Node::SetPeriod(INT64 anPeriodInMs)
{
	m_nPeriod = anPeriodInMs;
	m_bIsPeriodic = TRUE;
}

void
CROS2Node::proc_timer_callback()
{
	
}


/* ROS String Publisher */
CROS2StrPub::CROS2StrPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
	m_nPeriod = 0;
	m_cPublisher = create_publisher<std_msgs::msg::String>(GetTopicName(), GetHistDepth());
	m_pSharedPtr = new SPtrPublisher(this);
	
	m_eNodeType = eSTR_PUB;
	SetPublisher(TRUE);
}

void
CROS2StrPub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2StrPub::proc_timer_callback, this));
}

void 
CROS2StrPub::SetMessage(TSTRING astrMsg, BOOL abForce)
{
	m_strMessage = astrMsg;

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(m_strMessage);
	}
}

void
CROS2StrPub::proc_timer_callback()
{
	PublishMessage(m_strMessage);
}

void
CROS2StrPub::PublishMessage(TSTRING astrMsg)
{
	auto message = std_msgs::msg::String();
	message.data = TSTRING(astrMsg);
	//RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
	m_cPublisher->publish(message);
}

/* ROS String Subscriber */
CROS2StrSub::CROS2StrSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
	m_cSubscriber = create_subscription<std_msgs::msg::String>(GetTopicName(), GetHistDepth(), std::bind(&CROS2StrSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrSubscriber(this);
	m_pCallbackSub = NULL;
	m_eNodeType = eSTR_SUB;
	SetPublisher(FALSE);
}

void
CROS2StrSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2StrSub::proc_sub_callback(const std_msgs::msg::String::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg->data), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}