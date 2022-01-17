/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class OdomPublisher {
  public:
    OdometryPublisherC(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdomPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel);

    bool HasSubscribers();

  private:
    void PublishDataC(
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data, 
      ros::Time time
    );

  private:
    ros::NodeHandle nh_;
    //ros::Publisher publisher_;
    ros::Publisher laser_odom;
    ros::Publisher _pubC214_vicon_pose,_pubC214_visual_pose;


    VelocityData velocity_data_;
    //nav_msgs::Odometry odometry_;
    nav_msgs::Odometry laser_odometry;
    geometry_msgs::TransformStamped C214_vicon_pose;
    nav_msgs::Odometry C214_visual_pose;

};
}
#endif