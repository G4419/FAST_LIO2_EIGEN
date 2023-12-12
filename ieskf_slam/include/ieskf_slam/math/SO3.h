#ifndef SO3_H
#define SO3_H

#include <Eigen/Dense>

namespace IESKFSlam{
    //反对称矩阵
    static inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &so3){
        Eigen::Matrix3d ans;
        ans.setZero();
        ans(0,1) = -so3(2);
        ans(0,2) = so3(1);
        ans(1,0) = so3(2);
        ans(1,2) = -so3(0);
        ans(2,0) = -so3(1);
        ans(2,1) = so3(0);
        return ans;
    }
    //指数映射
    static Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3){
        double theta = so3.norm();
        if(theta <= 1e-7) return Eigen::Matrix3d::Identity();
        Eigen::Matrix3d skew_sym = skewSymmetric(so3) / theta;
        //罗德里格斯公式有两种形式
        return Eigen::Matrix3d::Identity() + (1-cos(theta)) * (skew_sym * skew_sym )
            + sin(theta) * skew_sym;
    }
    //对数映射
    static Eigen::Vector3d SO3Log(const Eigen::Matrix3d &SO3){
        double theta = (SO3.trace()>3-1e6)?0:acos((SO3.trace()-1)/2);
        Eigen::Vector3d so3(SO3(2,1)-SO3(1,2),SO3(0,2)-SO3(2,0),SO3(1,0)-SO3(0,1)); 
        return fabs(theta)<0.001?(0.5*so3):(0.5*theta/sin(theta)*so3);
    }
    //参考fastlio所引用的论文，A就是这个形式，并且其是一个对称矩阵。
    static Eigen::Matrix3d  A_T(const Eigen::Vector3d& v){
        Eigen::Matrix3d res;
        double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        double norm = std::sqrt(squaredNorm);
        if(norm <1e-11){
            res = Eigen::Matrix3d::Identity();
        }
        else{
            res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) + (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
        }
        return res;
    }
    
};

#endif