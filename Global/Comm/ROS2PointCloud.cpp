#include "ROS2PointCloud.h"



CROS2PointCloudPub::CROS2PointCloudPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    
    QoS qos_profile = rclcpp::QoS(1).best_effort().durability_volatile();
	m_nPeriod = 0;
	m_cPublisher = create_publisher<sensor_msgs::msg::PointCloud2>(GetTopicName(), qos_profile);
	m_pSharedPtr = new SPtrPointCloudPub(this);
	
	m_eNodeType = ePOINTCLOUD_PUB;
	SetPublisher(TRUE);
}

void
CROS2PointCloudPub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2PointCloudPub::proc_timer_callback, this));
}

void 
CROS2PointCloudPub::SetMessage(const sensor_msgs::msg::PointCloud2::SharedPtr msg, BOOL abForce)
{
    if (msg) {
        point_cloud_msg_ = *msg;
    }	

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(std::make_shared<sensor_msgs::msg::PointCloud2>(point_cloud_msg_));
	}
}

void
CROS2PointCloudPub::proc_timer_callback()
{
	PublishMessage(std::make_shared<sensor_msgs::msg::PointCloud2>(point_cloud_msg_));
}

void
CROS2PointCloudPub::PublishMessage(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	if (!m_cPublisher) {
		return;
	}
	auto compressed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	// message.data = TSTRING(astrMsg);
	
	m_cPublisher->publish(*msg);
	printf("Publishing: '%zu'", compressed_msg->data.size());

}


CROS2PointCloudSub::CROS2PointCloudSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH){
    rclcpp::QoS qos_profile = rclcpp::QoS(1).best_effort().durability_volatile();
	m_cSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>(GetTopicName(), qos_profile, std::bind(&CROS2PointCloudSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrPointCloudSub(this);
	m_pCallbackSub = NULL;
	m_eNodeType = ePOINTCLOUD_SUB;
	SetPublisher(FALSE);
}

void
CROS2PointCloudSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2PointCloudSub::proc_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}

