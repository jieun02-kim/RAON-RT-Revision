/*****************************************************************************
*	Name: ROS2Executor.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the ROS2 node wrapper
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ROS2Executor.h"

// using namespace rclcpp;

CROS2Executor::CROS2Executor()
{
	if (!rclcpp::ok()) {
		rclcpp::init(0, NULL);
	}

	m_bInit = FALSE;
	m_vecROS2Nodes.clear();
	m_vecSpinThread.clear();

    m_pExecutor = new rclcpp::executors::StaticSingleThreadedExecutor();
}


CROS2Executor::CROS2Executor(int domain_id)
{
	if (!rclcpp::ok()) {
		rclcpp::InitOptions options;
		options.set_domain_id(domain_id);
		rclcpp::init(0, NULL);
	}

	m_bInit = FALSE;
	m_vecROS2Nodes.clear();
	m_vecSpinThread.clear();

    m_pExecutor = new rclcpp::executors::StaticSingleThreadedExecutor();
}

CROS2Executor::~CROS2Executor()
{
    if (m_pExecutor) {
        delete m_pExecutor;
        m_pExecutor = nullptr;
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

BOOL
CROS2Executor::AddNode(CROS2Node* apNode)
{
	for (int nCnt = 0; nCnt < (int)m_vecROS2Nodes.size(); nCnt++)
	{
		if (m_vecROS2Nodes[nCnt] == apNode)
			return FALSE;
	}

	/* only add node if not in list */
	m_vecROS2Nodes.push_back(apNode);
	return TRUE;
}

BOOL
CROS2Executor::Stop()
{
	return TRUE;
}

BOOL
CROS2Executor::Init()
{
	if (TRUE == m_vecROS2Nodes.empty())
	{
		m_bInit = FALSE;
		return FALSE;
	}

	if (TRUE == IsInit())
	{
		DBG_LOG_WARN("(%s) ROS2 Executor is already Initialized!", "CROS2Executor");
		return FALSE;
	}

	for (int nCnt = 0; nCnt < (int)m_vecROS2Nodes.size(); nCnt++)
	{
		m_vecSpinThread.push_back(CreateSpinThread(m_vecROS2Nodes[nCnt]));
	}

	m_bInit = TRUE;

	return TRUE;
}

ROS2_SPIN_THREAD
CROS2Executor::CreateSpinThread(CROS2Node* apNode, BOOL abMulti)
{
	return std::make_shared<std::thread>([apNode]()
		{
			auto node_ptr = apNode->GetSharedPtr();
			if (!node_ptr) return;

			rclcpp::executors::SingleThreadedExecutor executor;
			executor.add_node(node_ptr);
			executor.spin();
		}
	);
}

BOOL
CROS2Executor::DeInit()
{
	if (FALSE == IsInit())
		return FALSE;

	/* shutdown rclcpp FIRST so spin() returns, then join */
	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}

	m_vecROS2Nodes.clear();

	if (FALSE == m_vecSpinThread.empty())
	{
		for (int nCnt = 0; nCnt < (int)m_vecSpinThread.size(); nCnt++)
		{
			if (m_vecSpinThread[nCnt]->joinable())
			{
				m_vecSpinThread[nCnt]->join();
			}
		}
	}
	m_vecSpinThread.clear();

	return TRUE;
}
