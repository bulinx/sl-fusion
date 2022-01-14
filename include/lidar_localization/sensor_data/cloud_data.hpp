/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<pcl::PointXYZ>;
    using CLOUD_PTR = CLOUD::Ptr;
    //    using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    //new: pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud< pcl::PointXYZ>)


  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif