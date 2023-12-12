#ifndef IESKF_h
#define IESKF_h

#include <Eigen/Dense>
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/math/SO3.h"
namespace IESKFSlam{
    class IESKF : private ModuleBase{
        public:
            using Ptr = std::shared_ptr<IESKF>;
            //状态量
            struct State18
            {
                Eigen::Quaterniond rotation;
                Eigen::Vector3d position;
                Eigen::Vector3d velocity;
                Eigen::Vector3d b_g;
                Eigen::Vector3d b_a;
                Eigen::Vector3d grivity;
                State18(){
                    rotation = Eigen::Quaterniond::Identity();
                    position = Eigen::Vector3d::Zero();
                    velocity = Eigen::Vector3d::Zero();
                    b_g = Eigen::Vector3d::Zero();
                    b_a = Eigen::Vector3d::Zero();
                    grivity = Eigen::Vector3d::Zero();
                }
            };
            class calZHInterface{
                public:
                    virtual bool calculate(const State18 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) = 0;
            };
            //声明计算Z和H的指针
            std::shared_ptr<calZHInterface> cal_ZH_ptr;
        private:
            State18 X;
            //前向传播的协方差矩阵
            Eigen::Matrix<double, 18, 18> P;
            //噪声矩阵，包括imu的零偏和测量噪声
            Eigen::Matrix<double, 12, 12> Q;
            //最大更新次数
            int max_iter = 10;
        public:
            IESKF(const std::string &config_path, const std::string &prefix);
            void predict(IMU imu, double dt);
            bool update();
            ~IESKF();
            const State18& getX();
            void setX(const State18 &x_in);
            Eigen::Matrix<double, 18, 1> getErrorState(const State18 &s1, const State18 &s2);
            
        
    };

    
}
#endif