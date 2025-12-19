#include <Eigen/Dense>
#include <Eigen/Geometry>  // For Quaternion
#include <iostream>

// 将欧拉角（ZYX顺序）转换为四元数
Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quaternion(rotation_matrix);
    return quaternion;
}

int main()
{
    // 相机到机械臂工具的欧拉角（ZYX顺序）
    double roll  = 0.050761172951085926;
    double pitch = 0.0047434909777201821;
    double yaw   = -1.5778546245587957;

    // 欧拉角转四元数
    Eigen::Quaterniond q_camera_to_tool = eulerToQuaternion(roll, pitch, yaw);

    // 相机到机械臂工具的平移
    Eigen::Vector3d t_camera_to_tool(46.307365116896307, 83.64781988216231,
                                     -270.34468528536888);

    // 构建相机到机械臂工具的变换矩阵
    Eigen::Matrix4d T_camera_to_tool = Eigen::Matrix4d::Identity();
    T_camera_to_tool.block<3, 3>(0, 0) =
        q_camera_to_tool.toRotationMatrix();                // 旋转部分
    T_camera_to_tool.block<3, 1>(0, 3) = t_camera_to_tool;  // 平移部分

    // 机械臂工具到基坐标系的平移
    Eigen::Vector3d t_tool_to_base(2502, -76.18, 996.38);

    // 机械臂工具到基坐标系的四元数
    Eigen::Quaterniond q_tool_to_base(0.75678, -0.07721, 0.02453, 0.64863);

    // 构建机械臂工具到基坐标系的变换矩阵
    Eigen::Matrix4d T_tool_to_base = Eigen::Matrix4d::Identity();
    T_tool_to_base.block<3, 3>(0, 0) =
        q_tool_to_base.toRotationMatrix();              // 旋转部分
    T_tool_to_base.block<3, 1>(0, 3) = t_tool_to_base;  // 平移部分

    // 相机到基坐标系的变换矩阵 = 机械臂工具到基坐标系的变换矩阵 *
    // 相机到机械臂工具的变换矩阵
    Eigen::Matrix4d T_camera_to_base = T_tool_to_base * T_camera_to_tool;

    // 提取相机相对于基坐标系的平移和旋转（四元数）
    Eigen::Vector3d t_camera_to_base = T_camera_to_base.block<3, 1>(0, 3);
    Eigen::Matrix3d R_camera_to_base = T_camera_to_base.block<3, 3>(0, 0);
    Eigen::Quaterniond q_camera_to_base(R_camera_to_base);

    // 输出结果
    std::cout << "相机到机械臂工具的平移: " << t_camera_to_tool.transpose()
              << std::endl;
    std::cout << "相机到机械臂工具的四元数 (w, x, y, z): "
              << q_camera_to_tool.w() << ", " << q_camera_to_tool.x() << ", "
              << q_camera_to_tool.y() << ", " << q_camera_to_tool.z()
              << std::endl;

    std::cout << "\n机械臂工具到基坐标系的平移: " << t_tool_to_base.transpose()
              << std::endl;
    std::cout << "机械臂工具到基坐标系的四元数 (w, x, y, z): "
              << q_tool_to_base.w() << ", " << q_tool_to_base.x() << ", "
              << q_tool_to_base.y() << ", " << q_tool_to_base.z() << std::endl;

    std::cout << "\n相机到基坐标系的平移: " << t_camera_to_base.transpose()
              << std::endl;
    std::cout << "相机到基坐标系的四元数 (w, x, y, z): " << q_camera_to_base.w()
              << ", " << q_camera_to_base.x() << ", " << q_camera_to_base.y()
              << ", " << q_camera_to_base.z() << std::endl;

    return 0;
}
