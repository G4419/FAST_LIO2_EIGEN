#ifndef IESKF_FRONTEND_WRAPPER_NOETIC_H
#define IESKF_FRONTEND_WRAPPER_NOETIC_H
#include "ieskf_slam/modules/frontend/frontend.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "lidar_process/avia_process.h"
namespace ROSNoetic{
    enum LIDAR_TYPE{
        AVIA = 0,
        VELO = 1,
    };
    class IESKFFrontEndWrapper
    {
    private:
        IESKFSlam::FrontEnd::Ptr frontend_ptr;
        ros::Subscriber imu_sub;
        ros::Subscriber cloud_sub;
        ros::Publisher path_pub;
        ros::Publisher cur_cloud_pub;
        ros::Publisher local_map_pub;

        std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

        IESKFSlam::PCLPointCloud cur_cloud;
        void cloudMsgsCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void imuMsgsCallback(const sensor_msgs::ImuPtr &msg);
        
        void run();
        void publishMsg();

    public:
        
        ~IESKFFrontEndWrapper();
        //构造函数
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
    };
}

#endif