#ifndef ROS_RSLIDAR_POINTCLOUD_POINT_TYPE_H_
#define ROS_RSLIDAR_POINTCLOUD_POINT_TYPE_H_

#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZITR
{
  PCL_ADD_POINT4D;                    // preferred way of adding a XYZ+padding
  uint8_t intensity;                  // point intensity 
  double timestamp;                   // point timestamp 
  uint16_t ring;                       // laser ring number  
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZITR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (uint16_t, ring, ring))

#endif  // ROS_RSLIDAR_POINTCLOUD_POINT_TYPE_H_
