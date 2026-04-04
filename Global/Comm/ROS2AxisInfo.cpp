#include "ROS2AxisInfo.h"



CROS2JointInfoPub::CROS2JointInfoPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{    
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_nPeriod = 0;
	m_cPublisher = create_publisher<sensor_msgs::msg::JointState>(GetTopicName(), qos_profile);
	m_pSharedPtr = new SPtrJointInfoPub(this);
	
	m_eNodeType = eJOINTINFO_PUB;
	SetPublisher(TRUE);
    
}

void
CROS2JointInfoPub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2JointInfoPub::proc_timer_callback, this));
}

void 
CROS2JointInfoPub::SetMessage(const sensor_msgs::msg::JointState::SharedPtr msg, BOOL abForce)
{
    if (msg) {
        m_joint_info_msg = *msg;
    }	

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(std::make_shared<sensor_msgs::msg::JointState>(m_joint_info_msg));
	}
}

void
CROS2JointInfoPub::proc_timer_callback()
{
	PublishMessage(std::make_shared<sensor_msgs::msg::JointState>(m_joint_info_msg));
}

void
CROS2JointInfoPub::PublishMessage(sensor_msgs::msg::JointState::SharedPtr msg)
{
	if (!m_cPublisher) {
		return;
	}
	auto joint_info_msg = std::make_shared<sensor_msgs::msg::JointState>();
	// message.data = TSTRING(astrMsg);
	// //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
	m_cPublisher->publish(*msg);
}


CROS2JointInfoSub::CROS2JointInfoSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_cSubscriber = create_subscription<sensor_msgs::msg::JointState>(GetTopicName(), qos_profile, std::bind(&CROS2JointInfoSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrJointInfoSub(this);
	m_pCallbackSub = NULL;
	m_eNodeType = eJOINTINFO_SUB;
	SetPublisher(FALSE);
}

void
CROS2JointInfoSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2JointInfoSub::proc_sub_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}


CROS2AxisInfoCmdPub::CROS2AxisInfoCmdPub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{    
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_nPeriod = 0;
	m_cPublisher = create_publisher<axis_info::msg::AxisCmd>(GetTopicName(), qos_profile);
	m_pSharedPtr = new SPtrAxisInfoCmdPub(this);
	
	m_eNodeType = eAXISINFO_CMD_PUB;
	SetPublisher(TRUE);
    
}

void
CROS2AxisInfoCmdPub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2AxisInfoCmdPub::proc_timer_callback, this));
}

void 
CROS2AxisInfoCmdPub::SetMessage(const axis_info::msg::AxisCmd::SharedPtr msg, BOOL abForce)
{
    if (msg) {
        m_axis_info_cmd_msg = *msg;
    }	

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(std::make_shared<axis_info::msg::AxisCmd>(m_axis_info_cmd_msg));
	}
}

void
CROS2AxisInfoCmdPub::proc_timer_callback()
{
	PublishMessage(std::make_shared<axis_info::msg::AxisCmd>(m_axis_info_cmd_msg));
}

void
CROS2AxisInfoCmdPub::PublishMessage(axis_info::msg::AxisCmd::SharedPtr msg)
{
	if (!m_cPublisher) {
		return;
	}
	auto axis_info_cmd_msg = std::make_shared<axis_info::msg::AxisCmd>();
	// message.data = TSTRING(astrMsg);
	// //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
	m_cPublisher->publish(*msg);
}


CROS2AxisInfoCmdSub::CROS2AxisInfoCmdSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
	
	m_cSubscriber = create_subscription<std_msgs::msg::ByteMultiArray>(GetTopicName(), qos_profile, std::bind(&CROS2AxisInfoCmdSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrAxisInfoCmdSub(this);
	m_pCallbackSub = NULL;
	m_eNodeType = eAXISINFO_CMD_SUB;
	SetPublisher(FALSE);
}

void
CROS2AxisInfoCmdSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2AxisInfoCmdSub::proc_sub_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}


CROS2AxisInfoStatePub::CROS2AxisInfoStatePub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{    
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_nPeriod = 0;
	m_cPublisher = create_publisher<std_msgs::msg::ByteMultiArray>(GetTopicName(), qos_profile);
	m_pSharedPtr = new SPtrAxisInfoStatePub(this);
	
	m_eNodeType = eAXISINFO_STATE_PUB;
	SetPublisher(TRUE);
    
}

void
CROS2AxisInfoStatePub::SetPeriod(INT64 anPeriodInMs)
{
	CROS2Node::SetPeriod(anPeriodInMs);

	std::chrono::milliseconds chronoPeriod = std::chrono::milliseconds(m_nPeriod);
	m_cTimer = create_wall_timer(chronoPeriod, std::bind(&CROS2AxisInfoStatePub::proc_timer_callback, this));
}

void 
CROS2AxisInfoStatePub::SetMessage(const axis_info::msg::AxisState::SharedPtr msg, BOOL abForce)
{
    if (msg) {
        m_axis_info_state_msg = *msg;
    }	

	if (FALSE == IsPeriodic() || TRUE == abForce)
	{
		PublishMessage(std::make_shared<axis_info::msg::AxisState>(m_axis_info_state_msg));
	}
}

void
CROS2AxisInfoStatePub::proc_timer_callback()
{
	PublishMessage(std::make_shared<axis_info::msg::AxisState>(m_axis_info_state_msg));
}

void
CROS2AxisInfoStatePub::PublishMessage(axis_info::msg::AxisState::SharedPtr msg)
{
	if (!m_cPublisher) {
		return;
	}
	// auto axis_info_state_msg = std::make_shared<axis_info::msg::AxisState>();
	// message.data = TSTRING(astrMsg);
	// //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());

	rclcpp::Serialization<axis_info::msg::AxisState> serializer;
	rclcpp::SerializedMessage serialized_msg;
	serializer.serialize_message(msg.get(), &serialized_msg);

	size_t original_size = serialized_msg.size();
	size_t max_compressed_size = ZSTD_compressBound(original_size);
	std::vector<uint8_t> compressed_data(max_compressed_size);

	size_t compressed_size = ZSTD_compress(compressed_data.data(), max_compressed_size, serialized_msg.get_rcl_serialized_message().buffer, original_size, 5);
	if (ZSTD_isError(compressed_size)) {
		RCLCPP_ERROR(get_logger(), "Failed to compress message: %s", ZSTD_getErrorName(compressed_size));
		return;
	}
	std_msgs::msg::ByteMultiArray compressed_msg;
	compressed_msg.data.resize(compressed_size);
	std::memcpy(compressed_msg.data.data(), compressed_data.data(), compressed_size);
	m_cPublisher->publish(compressed_msg);
	
	// RCLCPP_ERROR(get_logger(), "Axis State Published");
	// m_cPublisher->publish(*msg);
}


CROS2AxisInfoStateSub::CROS2AxisInfoStateSub(TSTRING astrNodeName, TSTRING astrTopicName) : CROS2Node(astrNodeName, astrTopicName, HISTORY_DEPTH)
{
    QoS qos_profile = rclcpp::QoS(1).best_effort();
	m_cSubscriber = create_subscription<axis_info::msg::AxisState>(GetTopicName(), qos_profile, std::bind(&CROS2AxisInfoStateSub::proc_sub_callback, this, std::placeholders::_1));
	m_pSharedPtr = new SPtrAxisInfoStateSub(this);
	m_pCallbackSub = NULL;
	m_eNodeType = eAXISINFO_STATE_SUB;
	SetPublisher(FALSE);
}

void
CROS2AxisInfoStateSub::RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackSub)
	{
		m_pCallbackSub = std::move(afnCallback);
	}
}

void
CROS2AxisInfoStateSub::proc_sub_callback(const axis_info::msg::AxisState::SharedPtr msg) const
{

	if (NULL != m_pCallbackSub)
	{
		m_pCallbackSub((PVOID)(&msg), NULL, NULL, NULL);
	}
	// RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}