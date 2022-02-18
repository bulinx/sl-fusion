/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    :nh_(nh) {

    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
   // optimized_odom = nh_.advertise<nav_msgs::Odometry> ("/ccccccccccccc", 100);
   _pubC214_vicon_pose = nh_.advertise<geometry_msgs::TransformStamped> ("/C214_vicon_pose", 100);
   _pubC214_visual_pose = nh_.advertise<nav_msgs::Odometry> ("/C214_visual_pose", 100);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
  //  optimized_odometry.header.frame_id =  "/map";
   // optimized_odometry.child_frame_id = "/lidar";
    C214_visual_pose.header.frame_id = "world";
   // C214_visual_pose.header.child_frame_id  = child_frame_id;
    C214_vicon_pose.header.frame_id = "world";
  //  C214_visual_pose.header.child_frame_id  = child_frame_id;




}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix,
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data_, ros_time);
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix
) {
    PublishData(transform_matrix, velocity_data_, ros::Time::now());
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix,
    const VelocityData &velocity_data, 
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data, ros_time);
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data
) {
    PublishData(transform_matrix, velocity_data, ros::Time::now());
}

void OdometryPublisher::Publish(
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

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const Eigen::Vector3f& vel
) {
    velocity_data_.linear_velocity.x = vel.x();
    velocity_data_.linear_velocity.y = vel.y();
    velocity_data_.linear_velocity.z = vel.z();

    PublishData(transform_matrix, velocity_data_, ros::Time::now());

    velocity_data_.linear_velocity.x = velocity_data_.linear_velocity.y = velocity_data_.linear_velocity.z = 0.0;
}
//每个发布的里程计话题都从这儿走
void OdometryPublisher::PublishData(
    //４×４的变换矩阵，包括平移和旋转（一种位姿的表示方式）// 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    const Eigen::Matrix4f& transform_matrix,
    const VelocityData &velocity_data,  //速度数据
    ros::Time time
) {

    odometry_.header.stamp = time;
   // optimized_odometry.header.stamp = time;
   // C214_visual_pose.header.stamp = time;
    // set the pose
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    // set the twist:
    // a. linear:
    odometry_.twist.twist.linear.x = velocity_data.linear_velocity.x;
    odometry_.twist.twist.linear.y = velocity_data.linear_velocity.y;
    odometry_.twist.twist.linear.z = velocity_data.linear_velocity.z;
    // b. angular:
    odometry_.twist.twist.angular.x = velocity_data.angular_velocity.x;
    odometry_.twist.twist.angular.y = velocity_data.angular_velocity.y;
    odometry_.twist.twist.angular.z = velocity_data.angular_velocity.z;
    publisher_.publish(odometry_);

}

//单独把最新里程计的拿出来，坐标系转换到world下面
void OdometryPublisher::PublishData2(
    //const ros::Time& stamp,
    const Eigen::Matrix4f& laser_pose,//４×４的变换矩阵，包括平移和旋转（一种位姿的表示方式） 
       // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    const VelocityData &velocity_data,  //速度数据
    ros::Time time
) {
    C214_visual_pose.header.stamp = time;
    /*
     // set the pose
    optimized_odometry.pose.pose.position.x = laser_odometry_(0,3);
    optimized_odometry.pose.pose.position.y = laser_odometry_(1,3);
    optimized_odometry.pose.pose.position.z = laser_odometry_(2,3);

    Eigen::Quaternionf q;
    q = laser_odometry_.block<3,3>(0,0);
    optimized_odometry.pose.pose.orientation.x = q.x();
    optimized_odometry.pose.pose.orientation.y = q.y();
    optimized_odometry.pose.pose.orientation.z = q.z();
    optimized_odometry.pose.pose.orientation.w = q.w();

    // set the twist:
    // a. linear:
    optimized_odometry.twist.twist.linear.x = velocity_data.linear_velocity.x;
    optimized_odometry.twist.twist.linear.y = velocity_data.linear_velocity.y;
    optimized_odometry.twist.twist.linear.z = velocity_data.linear_velocity.z;
    // b. angular:
    optimized_odometry.twist.twist.angular.x = velocity_data.angular_velocity.x;
    optimized_odometry.twist.twist.angular.y = velocity_data.angular_velocity.y;
    optimized_odometry.twist.twist.angular.z = velocity_data.angular_velocity.z;
    optimized_odom.publish(optimized_odometry);
*/

    Eigen::Quaternionf q;
    q = laser_pose.block<3,3>(0,0);
    C214_vicon_pose.transform.rotation.x = q.x();
  	C214_vicon_pose.transform.rotation.y = q.y();
  	C214_vicon_pose.transform.rotation.z = q.z();
  	C214_vicon_pose.transform.rotation.w = q.w();
  	C214_vicon_pose.transform.translation.x = laser_pose(0,3);
  	C214_vicon_pose.transform.translation.y = laser_pose(1,3);
  	C214_vicon_pose.transform.translation.z = laser_pose(2,3);

    C214_visual_pose.pose.pose.orientation.x = q.x();    //y      dz x
  	C214_visual_pose.pose.pose.orientation.y = q.y();    //z         y
  	C214_visual_pose.pose.pose.orientation.z = q.z();    //x        z
  	C214_visual_pose.pose.pose.orientation.w = q.w();    //w        w
  	C214_visual_pose.pose.pose.position.x = laser_pose(0,3);
  	C214_visual_pose.pose.pose.position.y = laser_pose(1,3);
  	C214_visual_pose.pose.pose.position.z = laser_pose(2,3);
       // set the twist:
    // a. linear:
   	C214_visual_pose.twist.twist.linear.x = velocity_data.linear_velocity.x;
   	C214_visual_pose.twist.twist.linear.y = velocity_data.linear_velocity.y;
   	C214_visual_pose.twist.twist.linear.z = velocity_data.linear_velocity.z;
    // b. angular:
    C214_visual_pose.twist.twist.angular.x = velocity_data.angular_velocity.x;
    C214_visual_pose.twist.twist.angular.y = velocity_data.angular_velocity.y;
    C214_visual_pose.twist.twist.angular.z = velocity_data.angular_velocity.z;

    _pubC214_vicon_pose.publish(C214_vicon_pose);
    _pubC214_visual_pose.publish(C214_visual_pose);

}



bool OdometryPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}