#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

// 四元数转欧拉角
Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond& q) {
    // YPR - ZYX
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation
// sequence
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d& rpy) {
    // YPR - ZYX
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation
// sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
/**
 * @brief 将四元数转换为欧拉角
 *
 * @param q 输入的四元数
 * @return Eigen::Vector3d 返回包含欧拉角的向量
 *
 * 该函数通过计算四元数的欧拉角表示，其中欧拉角顺序通常为yaw(pitch), pitch(yaw),
 * roll(pitch)。
 * 公式来源于四元数到欧拉角的转换公式，具体细节可参考相关文献或资料。
 */
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) {
    // 从四元数对象中提取出四元数的实部和虚部成分
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    // 计算欧拉角中的yaw(y)成分
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                   1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    // 计算欧拉角中的pitch(p)成分
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    // 计算欧拉角中的roll(r)成分
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                   1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

//旋转矩阵转欧拉角
Eigen::Vector3d rotation_to_euler(Eigen::Matrix3d dcm) {
    Eigen::Vector3d euler_angle;

    double phi_val   = atan2(dcm(2, 1), dcm(2, 2));
    double theta_val = asin(-dcm(2, 0));
    double psi_val   = atan2(dcm(1, 0), dcm(0, 0));
    double pi        = M_PI;

    if (fabs(theta_val - pi / 2.0) < 1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(dcm(1, 2), dcm(0, 2));

    } else if (fabs(theta_val + pi / 2.0) < 1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }

    euler_angle(0) = phi_val;
    euler_angle(1) = theta_val;
    euler_angle(2) = psi_val;

    return euler_angle;
}

// constrain_function
float constrain_function(float data, float Max) {
    if (abs(data) > Max) {
        return (data > 0) ? Max : -Max;
    } else {
        return data;
    }
}

// constrain_function2
/**
 * @brief 限制输入数据在指定范围内
 *
 * 此函数用于将给定的数据限制在最小值和最大值之间。如果数据超出这个范围，
 * 则将其返回为范围的极限值。这在需要确保数据在特定范围内有效时非常有用。
 *
 * @param data 输入的数据，将被限制在最小值和最大值之间
 * @param Min 最小限制值
 * @param Max 最大限制值
 * @return float 限制后的数据值
 */
float constrain_function2(float data, float Min, float Max) {
    if (data > Max) {
        return Max;
    } else if (data < Min) {
        return Min;
    } else {
        return data;
    }
}

// sign_function
float sign_function(float data) {
    if (data > 0) {
        return 1.0;
    } else if (data < 0) {
        return -1.0;
    } else if (data == 0) {
        return 0.0;
    }
}

// min function
float min(float data1, float data2) {
    if (data1 >= data2) {
        return data2;
    } else {
        return data1;
    }
}

#endif
