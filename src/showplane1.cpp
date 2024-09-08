#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>

// 生成密集平面点云并保存
void ShowPlane(const pcl::PointXYZ& centroid_point,
               const pcl::ModelCoefficients::Ptr& coefficients,
               std::string file_name, float plane_size = 1.0f,
               int resolution = 100)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    // 获取平面方程 ax + by + cz + d = 0 中的 a, b, c
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // // 确定平面上的两个方向向量
    // Eigen::Vector3f normal(a, b, c);
    // Eigen::Vector3f v1, v2;

    // // 创建平面上的两个正交向量
    // if (std::fabs(normal[0]) > std::fabs(normal[1]))
    //     v1 = Eigen::Vector3f(-normal[2], 0, normal[0]).normalized();
    // else
    //     v1 = Eigen::Vector3f(0, -normal[2], normal[1]).normalized();

    // v2 = normal.cross(v1).normalized();

    // 确定平面上的两个方向向量
    Eigen::Vector3f normal(a, b, c);
    Eigen::Vector3f v1, v2;

    // 创建平面上的两个正交向量，确保与法向量垂直
    // 选择任意不平行于法向量的向量
    if (std::fabs(normal[2]) > 0.9)
    {
        v1 = Eigen::Vector3f(1, 0, 0).cross(normal).normalized();
    }
    else
    {
        v1 = Eigen::Vector3f(0, 0, 1).cross(normal).normalized();
    }

    // v2 是法向量与 v1 的叉乘，确保与法向量和 v1 都垂直
    v2 = normal.cross(v1).normalized();

    // 使用规则网格生成点云
    float step = plane_size / resolution;  // 根据分辨率设置步长

    for (int i = -resolution / 2; i < resolution / 2; ++i)
    {
        for (int j = -resolution / 2; j < resolution / 2; ++j)
        {
            pcl::PointXYZ point;
            point.x = centroid_point.x + i * step * v1[0] + j * step * v2[0];
            point.y = centroid_point.y + i * step * v1[1] + j * step * v2[1];
            point.z = centroid_point.z + i * step * v1[2] + j * step * v2[2];

            plane_cloud->points.push_back(point);
        }
    }

    // 保存生成的平面点云
    if (plane_cloud->size())
    {
        plane_cloud->width    = plane_cloud->size();
        plane_cloud->height   = 1;
        std::string save_path = file_name + ".pcd";
        pcl::io::savePCDFileASCII(save_path, *plane_cloud);
        std::cout << "Saved point cloud to " << save_path << std::endl;
    }
    else
    {
        std::cerr << "WARNING: Plane cloud is empty" << std::endl;
    }
}

int main()
{
    // 假设中心点和拟合平面参数已经计算出来
    pcl::PointXYZ centroid_point(0.0f, 0.0f, 0.0f);  // 这里使用示例中心点
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 这里使用拟合平面的示例系数
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0f;   // a
    coefficients->values[1] = 1.0f;   // b
    coefficients->values[2] = 0.0f;   // c
    coefficients->values[3] = -1.0f;  // d

    // 生成并保存密集的平面点云
    ShowPlane(centroid_point, coefficients, "plane1", 1.0f, 100);

    return 0;
}
