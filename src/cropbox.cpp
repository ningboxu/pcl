#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include <omp.h>  // OpenMP并行化

// 优化后的裁剪函数
pcl::PointCloud<pcl::PointXYZ>::Ptr cropCloudByCube(
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

    // 计算方向向量上的最小和最大投影值
    float min_x_proj = min_corner.dot(x_norm);
    float max_x_proj = max_corner.dot(x_norm);
    float min_y_proj = min_corner.dot(y_norm);
    float max_y_proj = max_corner.dot(y_norm);
    float min_z_proj = min_corner.dot(z_norm);
    float max_z_proj = max_corner.dot(z_norm);

// 并行处理每个点（OpenMP并行化）
#pragma omp parallel for
    for (int i = 0; i < cloud_in_area->points.size(); ++i)
    {
        const auto& point = cloud_in_area->points[i];
        Eigen::Vector3f point_eigen(point.x, point.y, point.z);

        // 计算点在每个方向上的投影
        float x_proj = point_eigen.dot(x_norm);
        float y_proj = point_eigen.dot(y_norm);
        float z_proj = point_eigen.dot(z_norm);

        // 使用AABB过滤，判断点是否在裁剪区域内
        if (x_proj >= min_x_proj && x_proj <= max_x_proj &&
            y_proj >= min_y_proj && y_proj <= max_y_proj &&
            z_proj >= min_z_proj && z_proj <= max_z_proj)
        {
#pragma omp critical
            cloud_out->points.push_back(
                point);  // 并行下需要用critical保护共享资源
        }
    }

    cloud_out->width  = cloud_out->points.size();
    cloud_out->height = 1;
    return cloud_out;
}

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
        cropCloudByCube(cloud, corner1, corner2, y_norm, x_norm, z_norm);

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
