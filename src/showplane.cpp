#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

// 保存点云的函数
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name)
{
    std::string save_path = name + ".pcd";
    if (pointcloud->size())
    {
        pointcloud->width  = pointcloud->size();
        pointcloud->height = 1;
        pcl::io::savePCDFileASCII(save_path, *pointcloud);
        std::cout << "Saved point cloud to " << save_path << std::endl;
    }
    else
    {
        std::cerr << "WARNING: " << name << " " << pointcloud->size()
                  << std::endl;
    }
}

// 封装的可视化平面的函数
void VisualizeFittedPlane(const pcl::ModelCoefficients::Ptr& coefficients,
                          const std::string& name, float x_range, float y_range,
                          float resolution)
{
    // 生成用于保存平面的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 提取平面系数：ax + by + cz + d = 0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 确保 c 不为 0，否则无法计算 z 值
    if (c == 0)
    {
        std::cerr << "Invalid plane coefficients, c cannot be zero!"
                  << std::endl;
        return;
    }

    // 遍历 x 和 y 的范围，生成平面的点
    for (float x = -x_range / 2; x <= x_range / 2; x += resolution)
    {
        for (float y = -y_range / 2; y <= y_range / 2; y += resolution)
        {
            // 根据平面方程 ax + by + cz + d = 0 计算 z
            float z = (-d - a * x - b * y) / c;
            plane_cloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }

    // 保存生成的平面点云
    SavePointCloud(plane_cloud, name);
}

int main()
{
    // 创建并初始化平面系数（这里假设平面方程是 0.1x + 0.2y + 0.3z + 0.4 = 0）
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = 0.1;  // a
    coefficients->values[1] = 0.2;  // b
    coefficients->values[2] = 0.3;  // c
    coefficients->values[3] = 0.4;  // d

    // 调用函数可视化平面
    VisualizeFittedPlane(coefficients, "fitted_plane", 1.0, 1.0, 0.01);

    return 0;
}
