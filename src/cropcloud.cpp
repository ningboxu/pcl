#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <random>

// 生成随机测试点云
void generateTestPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            int num_points)
{
    // 使用随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 设置x, y, z范围
    std::uniform_real_distribution<> dis_x(-5.0, 5.0);  // x范围: [-5, 5]
    std::uniform_real_distribution<> dis_y(-5.0, 5.0);  // y范围: [-5, 5]
    std::uniform_real_distribution<> dis_z(0.0, 5.0);   // z范围: [0, 5]

    // 生成随机点
    for (int i = 0; i < num_points; ++i)
    {
        pcl::PointXYZ point;
        point.x = dis_x(gen);
        point.y = dis_y(gen);
        point.z = dis_z(gen);
        cloud->points.push_back(point);
    }

    cloud->width    = num_points;
    cloud->height   = 1;  // 无序点云
    cloud->is_dense = true;
}

// 裁剪函数
void cropPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float x0,
                    float x1, float y0, float y1, float z0, float z1)
{
    // 创建通过滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;

    // 裁剪x轴范围
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x0, x1);
    pass.filter(*output_cloud);  // 输出临时结果

    // 裁剪y轴范围
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(output_cloud);  // 继续使用临时裁剪后的点云
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y0, y1);
    pass_y.filter(*output_cloud);  // 输出裁剪后的点云

    // 裁剪z轴范围
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(output_cloud);  // 使用经过x和y裁剪后的点云
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z0, z1);
    pass_z.filter(*output_cloud);  // 最终裁剪后的点云
}

int main()
{
    // 生成并存储原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 生成1000个随机点的测试点云
    generateTestPointCloud(cloud, 1000);

    // 保存生成的点云以供观察
    pcl::io::savePCDFileASCII("generated_test_cloud.pcd", *cloud);
    std::cout << "Generated test point cloud with " << cloud->points.size()
              << " points." << std::endl;

    // 设置裁剪范围，例如：x的范围[-2, 2]，y的范围[-3, 3]，z的范围[1, 4]
    float x0 = -2.0, x1 = 2.0;
    float y0 = -3.0, y1 = 3.0;
    float z0 = 1.0, z1 = 4.0;

    // 调用裁剪函数
    cropPointCloud(cloud, cropped_cloud, x0, x1, y0, y1, z0, z1);

    // 保存裁剪后的点云
    pcl::io::savePCDFileASCII("cropped_test_cloud.pcd", *cropped_cloud);
    std::cout << "Saved cropped point cloud with "
              << cropped_cloud->points.size() << " points." << std::endl;

    return 0;
}
