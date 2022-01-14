/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input, double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR&  cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2()); //ros第二代点云数据结构
   //实现一个CloudData::CLOUD_PTR到sensor_msgs::PointCloud2Ptr的转换
   //PCL到ROS中的pcl::PointCloud 点云数据格式转换
  
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);


    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    //发出来的是带有动态指针的点云
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization