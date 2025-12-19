#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include <chrono>  // 用于计时

int main(int argc, char** argv)
{
    // 检查是否提供了文件路径
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <point_cloud.pcd/point_cloud.ply>" << std::endl;
        return -1;
    }

    std::string file_path = argv[1];

    // 1. 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    std::string extension = file_path.substr(file_path.find_last_of(".") + 1);

    if (extension == "pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", file_path.c_str());
            return -1;
        }
    }
    else if (extension == "ply")
    {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", file_path.c_str());
            return -1;
        }
    }
    else
    {
        std::cerr << "Unsupported file format. Please use PCD or PLY."
                  << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud: " << file_path << " with "
              << cloud->points.size() << " points." << std::endl;

    // 2. 移除 NaN 点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // 3. 检查 Z 轴范围
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto& point : cloud->points)
    {
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
    }
    std::cout << "Z-axis range: [" << min_z << ", " << max_z << "]"
              << std::endl;

    // 4. 对点云进行下采样（可选）
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);  // 调整下采样体素大小
    sor.filter(*cloud);
    std::cout << "After downsampling: " << cloud->points.size() << " points."
              << std::endl;

    // 5. 创建 ConvexHull 对象，并设置输入点云
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setDimension(3);  // 强制生成3D凸包
    chull.setInputCloud(cloud);

    // 6. 统计计算耗时
    auto start = std::chrono::high_resolution_clock::now();

    // 7. 存储凸包生成的网格
    pcl::PolygonMesh hull_mesh;
    chull.reconstruct(hull_mesh);

    // 8. 获取凸包的体积和表面积
    double volume = chull.getTotalVolume();  // 获取体积
    double area   = chull.getTotalArea();    // 获取表面积

    // 记录结束时间

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    // 打印结果
    std::cout << "Convex Hull Volume: " << volume << std::endl;
    std::cout << "Convex Hull Area: " << area << std::endl;
    std::cout << "Time elapsed: " << elapsed.count() << " seconds."
              << std::endl;

    return 0;
}
