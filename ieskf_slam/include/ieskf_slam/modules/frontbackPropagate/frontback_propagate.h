#ifndef FRONTBACK_PROPAGATE_H
#define FRONTBACK_PROPAGATE_H

#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/measure_group.h"
// #include <fstream>
namespace IESKFSlam{
    class FrontBackPropagate{
        private:
            struct ImuPose6D
            {
                double time;
                Eigen::Quaterniond rotation;
                Eigen::Vector3d position;
                Eigen::Vector3d velocity;
                Eigen::Vector3d angle_velocity;
                Eigen::Vector3d acceleration;
                ImuPose6D(double time_ = 0, Eigen::Vector3d acc_ = Eigen::Vector3d::Zero(),
                            Eigen::Vector3d av_ = Eigen::Vector3d::Zero(), 
                            Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
                            Eigen::Vector3d p_ = Eigen::Vector3d::Zero(),
                            Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity()){
                    time = time_;
                    rotation = q_;
                    velocity = v_;
                    position = p_;
                    angle_velocity = av_;
                    acceleration = acc_;
                }
            };
            Eigen::Vector3d last_accelaration;
            Eigen::Vector3d last_angle_velocity;
            // std::fstream debug_file;
            
        public:
            FrontBackPropagate();
            ~FrontBackPropagate();
            IMU last_imu;
            double last_lidar_end_time_;
            double imu_scale;
            void propagate(measure_group &mg, IESKF::Ptr ieskf_ptr);
    };
}

#endif