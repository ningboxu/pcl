#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>

// 保存点云的函数
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud, std::string name)
{
    std::string save_path = name + ".pcd";
    if (pointcloud->size())
    {
        pointcloud->width = pointcloud->size();
        pointcloud->height = 1;
        pcl::io::savePCDFileASCII(save_path, *pointcloud);
        std::cout << "Saved point cloud to " << save_path << std::endl;
    }
    else
    {
        std::cerr << "WARNING: Point cloud " << name << " is empty." << std::endl;
    }
}

// 可视化向量并生成点云
void ShowVector(const Eigen::Vector3f& vec, std::string name, Eigen::Vector3f start_p, float length)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f vec_tmp = vec.normalized();
    for (float i = 0; i < length;)
    {
        Eigen::Vector3f p = start_p + i * vec_tmp;
        pc->push_back(pcl::PointXYZ(p(0), p(1), p(2)));
        i += 0.01;  // 控制生成点的间隔
    }
    SavePointCloud(pc, name);
}

int main()
{
    // 假设已经有了平面法向量和中心点
    Eigen::Vector3f normal(0.599527, -0.797945, 0.0620601);  // 拟合得到的平面法向量
    Eigen::Vector3f centroid_point(0.0f, 0.0f, 0.0f);        // 平面的中心点

    // 可视化法向量，长度为1
    ShowVector(normal, "normal_vector", centroid_point, 1.0f);

    // 计算平面上的两个正交向量
    Eigen::Vector3f v1, v2;
    if (std::fabs(normal[2]) > 0.9) {
        v1 = Eigen::Vector3f(1, 0, 0).cross(normal).normalized();  // 使用 X 轴作为参考
    } else {
        v1 = Eigen::Vector3f(0, 0, 1).cross(normal).normalized();  // 使用 Z 轴作为参考
    }
    v2 = normal.cross(v1).normalized();

    // 可视化 v1 和 v2，长度为1
    ShowVector(v1, "v1_vector", centroid_point, 1.0f);
    ShowVector(v2, "v2_vector", centroid_point, 1.0f);

    return 0;
}

