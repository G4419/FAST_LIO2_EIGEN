#ifndef PROCESS_H
#define PROCESS_H

#include <sensor_msgs/PointCloud2.h>
#include "fast_lio2_eigen/type/base_type.h"
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver/CustomMsg.h>

namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time,time)
    (std::uint16_t, ring, ring)
)

namespace ROSNoetic{
    class Process{
        public:
            Process(){};
            ~Process(){};
            int lidar_type;
            double blind;
            void aviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg, IESKFSlam::PointCloud &cloud){
                cloud.cloud_ptr->clear();
                //进行雷达数据的处理
                for(uint i=0; i<msg->point_num; i++)
                {
                    //检查雷达点有效性以及屏蔽blind
                    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00 &&
                        msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                        msg->points[i].z * msg->points[i].z > blind * blind)
                    {
                        IESKFSlam::Point p;
                        p.x = msg->points[i].x;
                        p.y = msg->points[i].y;
                        p.z = msg->points[i].z;
                        p.intensity = msg->points[i].reflectivity;
                        p.curvature = msg->points[i].offset_time;
                        cloud.cloud_ptr->push_back(p);
                    }
                }
                cloud.time_stamp.fromNsec(msg->header.stamp.toNSec());
            }
            bool velodyneHandler(const sensor_msgs::PointCloud2 &msg, IESKFSlam::PointCloud &cloud){
                pcl::PointCloud<velodyne_ros::Point> rs_cloud;
                pcl::fromROSMsg(msg,rs_cloud);
                cloud.cloud_ptr->clear();
                double end_time = msg.header.stamp.toSec();
                double start_time = end_time + rs_cloud[0].time;
                for (auto &&p : rs_cloud)
                {
                    double point_time = p.time +end_time;
                    IESKFSlam::Point point;
                    point.x = p.x;
                    point.y = p.y;
                    point.z = p.z;
                    point.intensity = p.intensity;
                    point.curvature = (point_time - start_time)*1e9;
                    cloud.cloud_ptr->push_back(point);
                
                }
                cloud.time_stamp.fromSec(start_time);
            return true;
        }
    };
};

#endif