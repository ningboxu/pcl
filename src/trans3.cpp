#include <Eigen/Dense>
#include <Eigen/Geometry>  // For Quaternion
#include <iostream>

// 欧拉角转换为四元数 (ZYX顺序)
Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f& euler_angles)
{
    Eigen::AngleAxisf rollAngle(euler_angles(0),
                                Eigen::Vector3f::UnitX());  // 绕x轴
    Eigen::AngleAxisf pitchAngle(euler_angles(1),
                                 Eigen::Vector3f::UnitY());  // 绕y轴
    Eigen::AngleAxisf yawAngle(euler_angles(2),
                               Eigen::Vector3f::UnitZ());  // 绕z轴
    return yawAngle * pitchAngle * rollAngle;  // ZYX顺序的四元数
}

void computeCameraToBaseTransform(
    const Eigen::Vector3f& translation_TC,  // 工具到相机的平移
    const Eigen::Vector3f& euler_TC,        // 工具到相机的欧拉角 (ZYX)
    const Eigen::Vector3f& translation_BT,  // 基坐标系到工具的平移
    const Eigen::Quaternionf& quat_BT,      // 基坐标系到工具的四元数
    Eigen::Vector3f& translation_CB,  // 输出：相机到基坐标系的平移
    Eigen::Quaternionf& quat_CB)  // 输出：相机到基坐标系的四元数
{
    // 1. 工具到相机的旋转四元数
    Eigen::Quaternionf quat_TC = eulerToQuaternion(euler_TC);

    // 2. 工具到相机的变换矩阵 T_TC
    Eigen::Matrix4f T_TC   = Eigen::Matrix4f::Identity();
    T_TC.block<3, 3>(0, 0) = quat_TC.toRotationMatrix();  // 旋转部分
    T_TC.block<3, 1>(0, 3) = translation_TC;              // 平移部分

    // 3. 基坐标系到工具的逆变换矩阵 T_TB
    Eigen::Matrix4f T_TB = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R_BT = quat_BT.toRotationMatrix();
    Eigen::Matrix3f R_TB = R_BT.transpose();  // 旋转矩阵取逆
    Eigen::Vector3f t_BT = translation_BT;
    Eigen::Vector3f t_TB = -R_TB * t_BT;  // 平移部分取逆

    T_TB.block<3, 3>(0, 0) = R_TB;  // 旋转部分
    T_TB.block<3, 1>(0, 3) = t_TB;  // 平移部分

    // 4. 相机到基坐标系的变换 T_CB = T_TB * T_TC
    Eigen::Matrix4f T_CB = T_TB * T_TC;

    // 5. 提取相机到基坐标系的平移和旋转
    translation_CB                     = T_CB.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation_CB_matrix = T_CB.block<3, 3>(0, 0);
    quat_CB                            = Eigen::Quaternionf(rotation_CB_matrix);

    // 打印结果
    std::cout << "相机到基坐标系的平移: " << translation_CB.transpose()
              << std::endl;
    std::cout << "相机到基坐标系的旋转 (四元数): "
              << quat_CB.coeffs().transpose() << std::endl;
}

int main()
{
    // 工具到相机的变换 (平移和欧拉角，ZYX顺序)
    Eigen::Vector3f translation_TC(46.307365116896307, 83.64781988216231,
                                   -270.34468528536888);  // 工具到相机的平移
    Eigen::Vector3f euler_TC(0.050761172951085926, 0.0047434909777201821,
                             -1.5778546245587957);  // 工具到相机的欧拉角

    // 基坐标系到工具的变换 (平移和四元数)
    Eigen::Vector3f translation_BT(2502, -76.18,
                                   996.38);  // 基坐标系到工具的平移
    Eigen::Quaternionf quat_BT(0.75678, -0.07721, 0.02453,
                               0.64863);  // 基坐标系到工具的四元数

    // 输出：相机到基坐标系的平移和旋转
    Eigen::Vector3f translation_CB;
    Eigen::Quaternionf quat_CB;

    // 计算相机到基坐标系的变换
    computeCameraToBaseTransform(translation_TC, euler_TC, translation_BT,
                                 quat_BT, translation_CB, quat_CB);

    return 0;
}
