#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>

// 函数声明（你之前给出的裁剪函数）
pcl::PointCloud<pcl::PointXYZ>::Ptr cropPointCloudWithCube(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_area,
    const Eigen::Vector3f& corner1, const Eigen::Vector3f& corner2,
    const Eigen::Vector3f& y_norm, const Eigen::Vector3f& x_norm,
    const Eigen::Vector3f& z_norm);

int main()
{
    // 创建一个随机点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 生成1000个随机点
    for (int i = 0; i < 1000; ++i)
    {
        pcl::PointXYZ point;
        point.x = rand() % 10;  // x 坐标
        point.y = rand() % 10;  // y 坐标
        point.z = rand() % 10;  // z 坐标
        cloud->points.push_back(point);
    }
    cloud->width  = cloud->points.size();
    cloud->height = 1;

    // 立方体裁剪参数
    Eigen::Vector3f corner1(2.0f, 2.0f, 2.0f);  // 立方体的一个对角点
    Eigen::Vector3f corner2(7.0f, 7.0f, 7.0f);  // 立方体的另一个对角点

    // 方向向量（假设立方体与坐标轴对齐）
    Eigen::Vector3f x_norm(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f y_norm(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f z_norm(0.0f, 0.0f, 1.0f);

    // 调用裁剪函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped =
        cropPointCloudWithCube(cloud, corner1, corner2, y_norm, x_norm, z_norm);

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");

    // 添加原始点云
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,
        "original cloud");

    // 添加裁剪后的点云
    viewer.addPointCloud<pcl::PointXYZ>(cloud_cropped, "cropped cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
        "cropped cloud");

    // 可视化裁剪的立方体
    viewer.addCube(corner1[0], corner2[0], corner1[1], corner2[1], corner1[2],
                   corner2[2], 0.0, 1.0, 0.0, "cube");

    // 开始可视化
    viewer.spin();

    return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropPointCloudWithCube(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_area,
    const Eigen::Vector3f& corner1, const Eigen::Vector3f& corner2,
    const Eigen::Vector3f& y_norm, const Eigen::Vector3f& x_norm,
    const Eigen::Vector3f& z_norm)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 计算立方体的最小和最大边界
    Eigen::Vector3f min_corner = corner1.cwiseMin(corner2);
    Eigen::Vector3f max_corner = corner1.cwiseMax(corner2);

    // 遍历输入点云并根据立方体边界裁剪
    for (const auto& point : cloud_in_area->points)
    {
        Eigen::Vector3f point_eigen(point.x, point.y, point.z);

        // 判断点是否在立方体的范围内
        bool in_x_range = (point_eigen.dot(x_norm) >= min_corner.dot(x_norm)) &&
                          (point_eigen.dot(x_norm) <= max_corner.dot(x_norm));
        bool in_y_range = (point_eigen.dot(y_norm) >= min_corner.dot(y_norm)) &&
                          (point_eigen.dot(y_norm) <= max_corner.dot(y_norm));
        bool in_z_range = (point_eigen.dot(z_norm) >= min_corner.dot(z_norm)) &&
                          (point_eigen.dot(z_norm) <= max_corner.dot(z_norm));

        if (in_x_range && in_y_range && in_z_range)
        {
            cloud_out->points.push_back(point);  // 将点云点添加到输出点云
        }
    }

    cloud_out->width  = cloud_out->points.size();
    cloud_out->height = 1;  // 点云是一个单行的点集
    return cloud_out;
}
