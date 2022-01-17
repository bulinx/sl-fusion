/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/odom_publisher.hpp"

namespace lidar_localization {
OdomPublisher::OdomPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    :nh_(nh) {

    //publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    laser_odom = nh_.advertise<nav_msgs::Odometry> (topic_name, buff_size);
    _pubC214_vicon_pose = nh_.advertise<geometry_msgs::TransformStamped> ("/C214_vicon_pose", 100);
    _pubC214_visual_pose = nh_.advertise<nav_msgs::Odometry> ("/C214_visual_pose", 100);
    laser_odometry.header.frame_id = base_frame_id;
    laser_odometry.child_frame_id = child_frame_id;
    C214_visual_pose.header.frame_id = "world";
   // C214_visual_pose.header.child_frame_id  = child_frame_id;
    C214_vicon_pose.header.frame_id = "world";
  //  C214_visual_pose.header.child_frame_id  = child_frame_id;




}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data_, ros_time);
}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix
) {
    PublishData(transform_matrix, velocity_data_, ros::Time::now());
}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data, 
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data, ros_time);
}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data
) {
    PublishData(transform_matrix, velocity_data, ros::Time::now());
}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const Eigen::Vector3f& vel, 
    double time
) {
    ros::Time ros_time(time);

    velocity_data_.linear_velocity.x = vel.x();
    velocity_data_.linear_velocity.y = vel.y();
    velocity_data_.linear_velocity.z = vel.z();
    
    PublishData(transform_matrix, velocity_data_, ros_time);

    velocity_data_.linear_velocity.x = velocity_data_.linear_velocity.y = velocity_data_.linear_velocity.z = 0.0;
}

void OdomPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const Eigen::Vector3f& vel
) {
    velocity_data_.linear_velocity.x = vel.x();
    velocity_data_.linear_velocity.y = vel.y();
    velocity_data_.linear_velocity.z = vel.z();

    PublishData(transform_matrix, velocity_data_, ros::Time::now());

    velocity_data_.linear_velocity.x = velocity_data_.linear_velocity.y = velocity_data_.linear_velocity.z = 0.0;
}

//单独把最新里程计的拿出来，做坐标系转换到world下面
void OdomPublisher::PublishDataC(
    const Eigen::Matrix4f& transform_matrix,
    const VelocityData &velocity_data,  
    ros::Time time
) {
   // geometry_msgs::TransformStamped C214_vicon_pose;
    optimized_odometry.header.stamp = time;
    C214_visual_pose.header.stamp = time;
   // C214_vicon_pose.header = odometry_->header;


    // set the pose
   laser_odometry.pose.pose.position.x = transform_matrix(0,3);
    laser_odometry.pose.pose.position.y = transform_matrix(1,3);
    laser_odometry.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    laser_odometry.pose.pose.orientation.x = q.x();
    laser_odometry.pose.pose.orientation.y = q.y();
    laser_odometry.pose.pose.orientation.z = q.z();
   laser_odometry.pose.pose.orientation.w = q.w();

    // set the twist:
    // a. linear:
    laser_odometry.twist.twist.linear.x = velocity_data.linear_velocity.x;
   laser_odometry.twist.twist.linear.y = velocity_data.linear_velocity.y;
    laser_odometry.twist.twist.linear.z = velocity_data.linear_velocity.z;
    // b. angular:
   laser_odometry.twist.twist.angular.x = velocity_data.angular_velocity.x;
    laser_odometry.twist.twist.angular.y = velocity_data.angular_velocity.y;
    laser_odometry.twist.twist.angular.z = velocity_data.angular_velocity.z;
    laser_odom.publish(laser_odometry);



    C214_vicon_pose.transform.rotation.x = q.x();
  	C214_vicon_pose.transform.rotation.y = q.y();
  	C214_vicon_pose.transform.rotation.z = q.z();
  	C214_vicon_pose.transform.rotation.w = q.w();
  	C214_vicon_pose.transform.translation.x = transform_matrix(0,3);
  	C214_vicon_pose.transform.translation.y = transform_matrix(1,3);
  	C214_vicon_pose.transform.translation.z = transform_matrix(2,3);

    C214_visual_pose.pose.pose.orientation.x = q.x();    //y      dz x
  	C214_visual_pose.pose.pose.orientation.y = q.y();    //z         y
  	C214_visual_pose.pose.pose.orientation.z = q.z();    //x        z
  	C214_visual_pose.pose.pose.orientation.w = q.w();    //w        w
  	C214_visual_pose.pose.pose.position.x = transform_matrix(0,3);
  	C214_visual_pose.pose.pose.position.y = transform_matrix(1,3);
  	C214_visual_pose.pose.pose.position.z = transform_matrix(2,3);

    
    _pubC214_vicon_pose.publish(C214_vicon_pose);
    _pubC214_visual_pose.publish(C214_visual_pose);


}

bool OdomPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}