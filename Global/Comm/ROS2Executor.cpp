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
	rclcpp::shutdown();
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
			rclcpp::executors::StaticSingleThreadedExecutor executor;

            eNodeType etype = apNode->GetNodeType();

            CROS2StrPub* pPub;
            CROS2StrSub* pSub;
            // CROS2Usr2CtrlSub* pSub1;
            // CROS2JointInfoPub* pPub1;
			// CROS2UserInfoPub* pPub2;

			// CROS2ImuPub* pPubImu;
			// CROS2ImuSub* pSubImu;

			// CROS2PointCloudPub* pPubPC;
			// CROS2PointCloudSub* pSubPC;

			// CROS2JointInfoPub* pPubJI;
			// CROS2JointInfoSub* pSubJI;

			// CROS2AxisInfoCmdPub* pPubAIC;
			// CROS2AxisInfoCmdSub* pSubAIC;

			// CROS2AxisInfoStatePub* pPubAIS;
			// CROS2AxisInfoStateSub* pSubAIS;

			// Add Indy interface declarations
            CROS2IndyStatePub* pPubIndyState;
            CROS2GetGainsSrv* pSrvGetGains;
            CROS2SetGainsSrv* pSrvSetGains;
            CROS2SetPosSrv* pSrvSetPos;
            CROS2SetAllPosSrv* pSrvSetAllPos;
            CROS2GetGainsCli* pCliGetGains;
            CROS2SetGainsCli* pCliSetGains;
            CROS2SetPosCli* pCliSetPos;
            CROS2SetAllPosCli* pCliSetAllPos;
			CROS2SetControlModeSrv* pSrvSetControlMode;
            CROS2GetControlModeSrv* pSrvGetControlMode;
            CROS2SetControlModeCli* pCliSetControlMode;
            CROS2GetControlModeCli* pCliGetControlMode;

            switch(etype)
            {
            // case eSTR_PUB:
            //         pPub = (CROS2StrPub*)apNode;
			// 	    executor.add_node(pPub->GetSharedPtr());
            //         break;
            // case eSTR_SUB:
            //         pSub = (CROS2StrSub*)apNode;
			// 	    executor.add_node(pSub->GetSharedPtr());
            //         break;
            // case eUSR2CTRL_SUB:
            //         // pSub1 = (CROS2Usr2CtrlSub*)apNode;
            //         // executor.add_node(pSub1->GetSharedPtr());
            //         break;
            // case eJOINTINFO_PUB:
            //         pPubJI = (CROS2JointInfoPub*)apNode;
            //         executor.add_node(pPubJI->GetSharedPtr());
            //         break;
			// case eUSERINFO_PUB:
			// 		// pPub2 = (CROS2UserInfoPub*)apNode;
            //         // executor.add_node(pPub2->GetSharedPtr());
			// 		break;

			// case ePOINTCLOUD_SUB:
			// 		pSubPC = (CROS2PointCloudSub*)apNode;
			// 		executor.add_node(pSubPC->GetSharedPtr());
			// 		break;
			// case ePOINTCLOUD_PUB:
			// 		pPubPC = (CROS2PointCloudPub*)apNode;
			// 		executor.add_node(pPubPC->GetSharedPtr());
			// 		break;
            // case eJOINTINFO_SUB:
			// 		pSubJI = (CROS2JointInfoSub*)apNode;
			// 		executor.add_node(pSubJI->GetSharedPtr());
			// 		break;

			// case eAXISINFO_CMD_PUB:
			// 		pPubAIC = (CROS2AxisInfoCmdPub*)apNode;
			// 		executor.add_node(pPubAIC->GetSharedPtr());
			// 		break;
					
			// case eAXISINFO_CMD_SUB:
			// 		pSubAIC = (CROS2AxisInfoCmdSub*)apNode;
			// 		executor.add_node(pSubAIC->GetSharedPtr());
			// 		break;

			// case eAXISINFO_STATE_PUB:
			// 		pPubAIS = (CROS2AxisInfoStatePub*)apNode;
			// 		executor.add_node(pPubAIS->GetSharedPtr());
			// 		break;
			// case eAXISINFO_STATE_SUB:
			// 		pSubAIS = (CROS2AxisInfoStateSub*)apNode;
			// 		executor.add_node(pSubAIS->GetSharedPtr());
			// 		break;

			// case eIMU_PUB:
			// 		pPubImu = (CROS2ImuPub*)apNode;
			// 		executor.add_node(pPubImu->GetSharedPtr());
			// 		break;
			// case eIMU_SUB:
			// 		pSubImu = (CROS2ImuSub*)apNode;
			// 		executor.add_node(pSubImu->GetSharedPtr());
			// 		break;

					// Add Indy interface cases
            case eINDY_STATE_PUB:
                    pPubIndyState = (CROS2IndyStatePub*)apNode;
                    executor.add_node(pPubIndyState->GetSharedPtr());
                    break;
            case eGET_GAINS_SRV:
                    pSrvGetGains = (CROS2GetGainsSrv*)apNode;
                    executor.add_node(pSrvGetGains->GetSharedPtr());
                    break;
            case eSET_GAINS_SRV:
                    pSrvSetGains = (CROS2SetGainsSrv*)apNode;
                    executor.add_node(pSrvSetGains->GetSharedPtr());
                    break;
            case eSET_POS_SRV:
                    pSrvSetPos = (CROS2SetPosSrv*)apNode;
                    executor.add_node(pSrvSetPos->GetSharedPtr());
                    break;
            case eSET_ALL_POS_SRV:
                    pSrvSetAllPos = (CROS2SetAllPosSrv*)apNode;
                    executor.add_node(pSrvSetAllPos->GetSharedPtr());
                    break;
            case eGET_GAINS_CLI:
                    pCliGetGains = (CROS2GetGainsCli*)apNode;
                    executor.add_node(pCliGetGains->GetSharedPtr());
                    break;
            case eSET_GAINS_CLI:
                    pCliSetGains = (CROS2SetGainsCli*)apNode;
                    executor.add_node(pCliSetGains->GetSharedPtr());
                    break;
            case eSET_POS_CLI:
                    pCliSetPos = (CROS2SetPosCli*)apNode;
                    executor.add_node(pCliSetPos->GetSharedPtr());
                    break;
            case eSET_ALL_POS_CLI:
                    pCliSetAllPos = (CROS2SetAllPosCli*)apNode;
                    executor.add_node(pCliSetAllPos->GetSharedPtr());
                    break;
					case eSET_CONTROL_MODE_SRV:
					pSrvSetControlMode = (CROS2SetControlModeSrv*)apNode;
					executor.add_node(pSrvSetControlMode->GetSharedPtr());
					break;
			case eGET_CONTROL_MODE_SRV:
					pSrvGetControlMode = (CROS2GetControlModeSrv*)apNode;
					executor.add_node(pSrvGetControlMode->GetSharedPtr());
					break;
			case eSET_CONTROL_MODE_CLI:
					pCliSetControlMode = (CROS2SetControlModeCli*)apNode;
					executor.add_node(pCliSetControlMode->GetSharedPtr());
					break;
			case eGET_CONTROL_MODE_CLI:
					pCliGetControlMode = (CROS2GetControlModeCli*)apNode;
					executor.add_node(pCliGetControlMode->GetSharedPtr());
					break;
			case eUSERINFO_SUB:
            case eUSR2CTRL_PUB:
            default:
                break;
            }
			executor.spin();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

            switch(etype)
            {
            case eSTR_PUB:
                    pPub = (CROS2StrPub*)apNode;
				    executor.remove_node(pPub->GetSharedPtr());
                    break;
            case eSTR_SUB:
                    pSub = (CROS2StrSub*)apNode;
				    executor.remove_node(pSub->GetSharedPtr());
                    break;
            // case eUSR2CTRL_SUB:
            //         // pSub1 = (CROS2Usr2CtrlSub*)apNode;
            //         // executor.remove_node(pSub1->GetSharedPtr());
            //         break;
            // case eJOINTINFO_PUB:
            //         pPubJI = (CROS2JointInfoPub*)apNode;
            //         executor.remove_node(pPubJI->GetSharedPtr());
            //         break;
			// case eJOINTINFO_SUB:
            //         pSubJI = (CROS2JointInfoSub*)apNode;
            //         executor.remove_node(pSubJI->GetSharedPtr());
            //         break;
			// case eUSERINFO_PUB:
			// 		// pPub2 = (CROS2UserInfoPub*)apNode;
            //         // executor.remove_node(pPub2->GetSharedPtr());
			// 		break;

			// case ePOINTCLOUD_PUB:
			// 		pPubPC = (CROS2PointCloudPub*)apNode;
			// 		executor.remove_node(pPubPC->GetSharedPtr());
			// 		break;
			// case ePOINTCLOUD_SUB:
			// 		pSubPC = (CROS2PointCloudSub*)apNode;
			// 		executor.remove_node(pSubPC->GetSharedPtr());
			// 		break;

			// case eAXISINFO_CMD_PUB:
			// 		pPubAIC = (CROS2AxisInfoCmdPub*)apNode;
			// 		executor.remove_node(pPubAIC->GetSharedPtr());
			// 		break;
			// case eAXISINFO_CMD_SUB:
			// 		pSubAIC = (CROS2AxisInfoCmdSub*)apNode;
			// 		executor.remove_node(pSubAIC->GetSharedPtr());
			// 		break;
			// case eAXISINFO_STATE_PUB:
			// 		pPubAIS = (CROS2AxisInfoStatePub*)apNode;
			// 		executor.remove_node(pPubAIS->GetSharedPtr());
			// 		break;
			// case eAXISINFO_STATE_SUB:	
			// 		pSubAIS = (CROS2AxisInfoStateSub*)apNode;
			// 		executor.remove_node(pSubAIS->GetSharedPtr());
			// 		break;

			// case eIMU_PUB:
			// 		pPubImu = (CROS2ImuPub*)apNode;
			// 		executor.remove_node(pPubImu->GetSharedPtr());
			// 		break;
			// case eIMU_SUB:
			// 		pSubImu = (CROS2ImuSub*)apNode;
			// 		executor.remove_node(pSubImu->GetSharedPtr());
			// 		break;
			case eINDY_STATE_PUB:
					pPubIndyState = (CROS2IndyStatePub*)apNode;
					executor.remove_node(pPubIndyState->GetSharedPtr());
					break;
			case eGET_GAINS_SRV:
					pSrvGetGains = (CROS2GetGainsSrv*)apNode;
					executor.remove_node(pSrvGetGains->GetSharedPtr());
					break;
			case eSET_GAINS_SRV:
					pSrvSetGains = (CROS2SetGainsSrv*)apNode;
					executor.remove_node(pSrvSetGains->GetSharedPtr());
					break;
			case eSET_POS_SRV:
					pSrvSetPos = (CROS2SetPosSrv*)apNode;
					executor.remove_node(pSrvSetPos->GetSharedPtr());
					break;
			case eSET_ALL_POS_SRV:
					pSrvSetAllPos = (CROS2SetAllPosSrv*)apNode;
					executor.remove_node(pSrvSetAllPos->GetSharedPtr());
					break;
			case eGET_GAINS_CLI:
					pCliGetGains = (CROS2GetGainsCli*)apNode;
					executor.remove_node(pCliGetGains->GetSharedPtr());
					break;
			case eSET_GAINS_CLI:
					pCliSetGains = (CROS2SetGainsCli*)apNode;
					executor.remove_node(pCliSetGains->GetSharedPtr());
					break;
			case eSET_POS_CLI:
					pCliSetPos = (CROS2SetPosCli*)apNode;
					executor.remove_node(pCliSetPos->GetSharedPtr());
					break;
			case eSET_ALL_POS_CLI:
					pCliSetAllPos = (CROS2SetAllPosCli*)apNode;
					executor.remove_node(pCliSetAllPos->GetSharedPtr());
					case eSET_CONTROL_MODE_SRV:
					pSrvSetControlMode = (CROS2SetControlModeSrv*)apNode;
					executor.remove_node(pSrvSetControlMode->GetSharedPtr());
					break;
			case eGET_CONTROL_MODE_SRV:
					pSrvGetControlMode = (CROS2GetControlModeSrv*)apNode;
					executor.remove_node(pSrvGetControlMode->GetSharedPtr());
					break;
			case eSET_CONTROL_MODE_CLI:
					pCliSetControlMode = (CROS2SetControlModeCli*)apNode;
					executor.remove_node(pCliSetControlMode->GetSharedPtr());
					break;
			case eGET_CONTROL_MODE_CLI:
					pCliGetControlMode = (CROS2GetControlModeCli*)apNode;
					executor.remove_node(pCliGetControlMode->GetSharedPtr());
					break;			
			case eUSERINFO_SUB:
            case eUSR2CTRL_PUB:
            default:
                break;
            }
		}
	);
}

BOOL
CROS2Executor::DeInit()
{
	if (FALSE == IsInit())
		return FALSE;

	m_vecROS2Nodes.clear();

	Stop();

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
	
	rclcpp::shutdown();
	return TRUE;
}