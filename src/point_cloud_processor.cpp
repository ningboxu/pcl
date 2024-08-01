#include <iostream>
#include <limits>
#include "point_cloud_processor.h"

void removeInvalidPoints(const std::string& input_file,
                         const std::string& output_file)
{
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", input_file.c_str());
        return;
    }

    // 检查并统计NaN点和0点
    size_t nan_count  = 0;
    size_t zero_count = 0;
    for (const auto& point : cloud->points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            nan_count++;
        }
        else if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f)
        {
            zero_count++;
        }
    }

    std::cout << "Original cloud size: " << cloud->size() << std::endl;
    std::cout << "NaN points count: " << nan_count << std::endl;
    std::cout << "Zero points count: " << zero_count << std::endl;

    // 删除NaN点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, nan_index);

    // 删除0点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_no_nan->points)
    {
        if (!(point.x == 0.0f && point.y == 0.0f && point.z == 0.0f))
        {
            cloud_filtered->points.push_back(point);
        }
    }
    cloud_filtered->width    = cloud_filtered->points.size();
    cloud_filtered->height   = 1;  // 单行点云
    cloud_filtered->is_dense = true;

    // 打印处理后点云的大小
    std::cout << "Filtered cloud size: " << cloud_filtered->size() << std::endl;

    // 保存处理后的点云
    pcl::io::savePLYFile(output_file, *cloud_filtered);
}
