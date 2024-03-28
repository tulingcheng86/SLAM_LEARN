#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

int main(int argc, char** argv) {
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    Sophus::SO3d SO3_R(R); // Sophus::SO(3)可以直接从旋转矩阵构造
    // 旋转向量（沿Z轴旋转90度）
    Eigen::Vector3d rot_vector(0, 0, M_PI / 2);
    // 将旋转向量转换为四元数
    //Eigen::Quaterniond q_from_vector = Eigen::AngleAxisd(rot_vector.norm(), rot_vector.normalized());
    // 使用四元数构造SO3d对象
    //Sophus::SO3d SO3_v(q_from_vector);
    Eigen::Quaterniond q(R); // 或者四元数
    Sophus::SO3d SO3_q(q);
    // 上述表达方式都是等价的
    //cout << "SO(3) from matrix: " << SO3_R << endl;
    //cout << "SO(3) from vector: " << SO3_v << endl;
    //cout << "SO(3) from quaternion: " << SO3_q << endl;

    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    //cout << "so3 = " << so3.transpose() << endl;
    //cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
    //cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); // 假设更新量为这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    //cout << "SO3 updated = " << SO3_updated << endl;

    //cout << "************我是分割线*************" << endl;
    // 对SE(3)操作大同小异
    Eigen::Vector3d t(1, 0, 0); // 沿X轴平移1
    Sophus::SE3d SE3_Rt(R, t); // 从R,t构造SE(3)
    Sophus::SE3d SE3_qt(q, t); // 从q,t构造SE(3)
    //cout << "SE3 from R,t= " << SE3_Rt.matrix() << endl;
    //cout << "SE3 from q,t= " << SE3_qt.matrix() << endl;

    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    //cout << "se3 = " << se3.transpose() << endl;
    //cout << "se3 hat = " << endl << Sophus::SE3d::hat(se3) << endl;
    //cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // 最后，演示一下更新
    Vector6d update_se3; // 更新量
    update_se3.setZero();
    //update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    //cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}
