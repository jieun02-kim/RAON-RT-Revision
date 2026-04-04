#ifndef __ROS2_AXIS_INFO__
#define __ROS2_AXIS_INFO__

#include <zlib.h>
#include <vector>
#include <memory>
#include <iostream>
#include <zstd.h>

#include "ROS2Node.h"




class CROS2JointInfoPub : public CROS2Node
{
public:
    CROS2JointInfoPub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2JointInfoPub() {};
    virtual void SetPeriod(INT64 anPeriodInMs);
    
    void SetMessage(const sensor_msgs::msg::JointState::SharedPtr msg, BOOL abForce = FALSE);
    SPtrJointInfoPub GetSharedPtr() { return *m_pSharedPtr; }

    sensor_msgs::msg::JointState m_joint_info_msg;

protected:
    virtual void proc_timer_callback();

protected:
    ROS2_TIMER              m_cTimer;
    ROS2_JOINTINFO_PUB      m_cPublisher;

private:
    void            PublishMessage(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    SPtrJointInfoPub*  m_pSharedPtr;

};

class CROS2JointInfoSub : public CROS2Node
{
public:
    CROS2JointInfoSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2JointInfoSub() {};
    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrJointInfoSub GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const;

protected:
    ROS2_JOINTINFO_SUB      m_cSubscriber;
    GLOBAL_CALLBACK_FN      m_pCallbackSub;

private:
    SPtrJointInfoSub*      m_pSharedPtr;
};


class CROS2AxisInfoCmdPub : public CROS2Node
{
public:
    CROS2AxisInfoCmdPub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2AxisInfoCmdPub() {};
    virtual void SetPeriod(INT64 anPeriodInMs);
    
    void SetMessage(const axis_info::msg::AxisCmd::SharedPtr msg, BOOL abForce = FALSE);
    SPtrAxisInfoCmdPub GetSharedPtr() { return *m_pSharedPtr; }

    axis_info::msg::AxisCmd m_axis_info_cmd_msg;

protected:
    virtual void proc_timer_callback();

protected:
    ROS2_TIMER              m_cTimer;
    ROS2_AXISINFO_CMD_PUB      m_cPublisher;

private:
    void            PublishMessage(const axis_info::msg::AxisCmd::SharedPtr msg);
    
    SPtrAxisInfoCmdPub*  m_pSharedPtr;

};

class CROS2AxisInfoCmdSub : public CROS2Node
{
public:
    CROS2AxisInfoCmdSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2AxisInfoCmdSub() {};
    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrAxisInfoCmdSub GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) const;

protected:
    ROS2_AXISINFO_CMD_SUB      m_cSubscriber;
    GLOBAL_CALLBACK_FN      m_pCallbackSub;

private:
    SPtrAxisInfoCmdSub*      m_pSharedPtr;
};

class CROS2AxisInfoStatePub : public CROS2Node
{
public:
    CROS2AxisInfoStatePub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2AxisInfoStatePub() {};
    virtual void SetPeriod(INT64 anPeriodInMs);
    
    void SetMessage(const axis_info::msg::AxisState::SharedPtr msg, BOOL abForce = FALSE);
    SPtrAxisInfoStatePub GetSharedPtr() { return *m_pSharedPtr; }

    axis_info::msg::AxisState m_axis_info_state_msg;
    

protected:
    virtual void proc_timer_callback();

protected:
    ROS2_TIMER                   m_cTimer;
    ROS2_AXISINFO_STATE_PUB      m_cPublisher;

private:
    void            PublishMessage(const axis_info::msg::AxisState::SharedPtr msg);
    
    SPtrAxisInfoStatePub*  m_pSharedPtr;

};

class CROS2AxisInfoStateSub : public CROS2Node
{
public:
    CROS2AxisInfoStateSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2AxisInfoStateSub() {};
    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrAxisInfoStateSub GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const axis_info::msg::AxisState::SharedPtr msg) const;

protected:
    ROS2_AXISINFO_STATE_SUB      m_cSubscriber;
    GLOBAL_CALLBACK_FN                  m_pCallbackSub;

private:
    SPtrAxisInfoStateSub*      m_pSharedPtr;
};

#endif //__ROS2_AXIS_INFO__