#ifndef __ROS2_POINT_CLOUD__
#define __ROS2_POINT_CLOUD__

#include <zlib.h>
#include <vector>
#include <memory>
#include <iostream>
#include <lz4.h>
#include <zstd.h>
#include "ROS2Node.h"


class CROS2PointCloudPub : public CROS2Node
{
public:
    CROS2PointCloudPub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2PointCloudPub() {};
    virtual void SetPeriod(INT64 anPeriodInMs);
    
    void SetMessage(const sensor_msgs::msg::PointCloud2::SharedPtr msg, BOOL abForce = FALSE);
    SPtrPointCloudPub GetSharedPtr() { return *m_pSharedPtr; }

    sensor_msgs::msg::PointCloud2 point_cloud_msg_;

protected:
    virtual void proc_timer_callback();

protected:
    ROS2_TIMER              m_cTimer;
    ROS2_POINTCLOUD_PUB     m_cPublisher;

private:
    void            PublishMessage(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    SPtrPointCloudPub*  m_pSharedPtr;

};

class CROS2PointCloudSub : public CROS2Node
{
public:
    CROS2PointCloudSub(TSTRING astrNodeName, TSTRING astrTopicName);
    virtual ~CROS2PointCloudSub() {};
    virtual void RegisterCallbackSubscriber(GLOBAL_CALLBACK_FN afnCallback);
    SPtrPointCloudSub GetSharedPtr() { return *m_pSharedPtr; }

protected:
    void proc_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

protected:
    ROS2_POINTCLOUD_SUB     m_cSubscriber;
    GLOBAL_CALLBACK_FN      m_pCallbackSub;

private:
    SPtrPointCloudSub*      m_pSharedPtr;
};

#endif //__ROS2_POINT_CLOUD__