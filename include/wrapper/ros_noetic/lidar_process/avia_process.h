#ifndef AVIA_PROCESS_H
#define AVIA_PROCESS_H
#include "common_lidar_process_interface.h"

// namespace avia_ros{
//     struct EIGEN_ALIGN16 Point{
//         PCL_ADD_POINT4D;
//         float reflectivity;
//         std::uint32_t offset_time;
//         std::uint8_t line;
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//     };
// }

// POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
//     (float, x, x)
//     (float, y, y)
//     (float, z, z)
//     (float, reflectivity, reflectivity)
//     (std::uint32_t, offset_time, offset_time)
//     (std::uint8_t, line, line))

namespace ROSNoetic{
    class AVIAProcess : public CommonLidarProcessInterface{
        public:
            AVIAProcess(){};
            ~AVIAProcess(){};
            bool process(const livox_ros_driver::CustomMsg::ConstPtr &msg, IESKFSlam::PointCloud &cloud){
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
                        p.offset_time = msg->points[i].offset_time;
                        p.ring = msg->points[i].line;
                        cloud.cloud_ptr->push_back(p);
                    }
                }
                cloud.time_stamp.fromNsec(msg->header.stamp.toNSec());
                return true;
            }
    };
}
#endif