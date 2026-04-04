#include "ROS2Imu.h"



CROS2ImuPub::CROS2ImuPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	
	m_nPeriod = 0;
	m_cPublisher = create_publisher<sensor_msgs::msg::Imu>(GetTopicName(), qos_profile);
	m_pSharedPtr = new SPtrImuPub(this);
	
	m_eNodeType = eIMU_PUB;
	SetPublisher(TRUE);
}

void
CROS2ImuPub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2ImuPub::proc_timer_callback, this));
}

void 
CROS2ImuPub::SetMessage(const sensor_msgs::msg::Imu::SharedPtr msg, BOOL abForce)
{
    if (msg) {
        imu_msg_ = *msg;
    }	

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(std::make_shared<sensor_msgs::msg::Imu>(imu_msg_));
	}
}

void
CROS2ImuPub::proc_timer_callback()
{
	PublishMessage(std::make_shared<sensor_msgs::msg::Imu>(imu_msg_));
}

void
CROS2ImuPub::PublishMessage(sensor_msgs::msg::Imu::SharedPtr msg)
{
	if (!m_cPublisher) {
		return;
	}
	auto compressed_msg = std::make_shared<sensor_msgs::msg::Imu>();
	// message.data = TSTRING(astrMsg);
	// //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
	m_cPublisher->publish(*msg);
}


CROS2ImuSub::CROS2ImuSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    rclcpp::QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_cSubscriber = create_subscription<sensor_msgs::msg::Imu>(GetTopicName(), qos_profile, std::bind(&CROS2ImuSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrImuSub(this);
	m_pCallbackSub = NULL;
	m_eNodeType = eIMU_SUB;
	SetPublisher(FALSE);
}

void
CROS2ImuSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2ImuSub::proc_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}

