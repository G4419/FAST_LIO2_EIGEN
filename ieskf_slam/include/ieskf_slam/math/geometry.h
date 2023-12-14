#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <Eigen/Dense>
#include <vector>

namespace IESKFSlam{
    template<typename pointTypeT>
    static bool planarCheck(const std::vector<pointTypeT> &points, Eigen::Vector4d &pabcd, double threhold){
        Eigen::Vector3d normal;
        Eigen::MatrixXd A;
        Eigen::VectorXd B;
        int point_size = points.size();
        A.resize(point_size, 3);
        B.resize(point_size);
        B.setOnes();
        B = -B;
        for(size_t i = 0; i < point_size; i++){
            A(i, 0) = points[i].x;
            A(i, 1) = points[i].y;
            A(i, 2) = points[i].z;
        }
        normal = A.colPivHouseholderQr().solve(B);

        for(size_t i = 0; i < point_size; i++){
            if(fabs(normal(0) * A(i, 0) + normal(1) * A(i, 1) + normal(2) * A(i, 2) + 1.0f) > threhold){
                // std::cout << "planar false" << std::endl;
                return false;
            } 
        }
        double norm = normal.norm();
        normal.normalize();
        pabcd(0) = normal(0);
        pabcd(1) = normal(1);
        pabcd(2) = normal(2);
        pabcd(3) = 1 / norm;

        return true;
    }


    template<typename PointType, typename T>
    static PointType transformPoint(PointType point, const Eigen::Quaternion<T> &q, const Eigen::Matrix<T, 3, 1> &t){
        Eigen::Matrix<T, 3, 1> ep = {point.x, point.y, point.z};
        ep = q*ep+t;
        point.x = ep.x();
        point.y = ep.y();
        point.z = ep.z();
        return point;
    }
}

#endif