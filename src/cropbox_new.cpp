#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

// 定义一个函数来执行点云裁剪操作
void cropPointCloud(const std::string& input_file,
                    const std::string& output_file,
                    const Eigen::Vector4f& min_pt,
                    const Eigen::Vector4f& max_pt,
                    const Eigen::Vector3f& translation,
                    const Eigen::Vector3f& rotation, bool negative)
{
    // 定义点云类型为 pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 从 PCD 文件中读取点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file\n");
        return;
    }
    std::cout << "Loaded " << cloud->width * cloud->height
              << " data points from " << input_file
              << " with the following fields: " << std::endl;

    // 创建 CropBox 滤波器对象
    pcl::CropBox<pcl::PointXYZ> crop;

    // 设置输入点云
    crop.setInputCloud(cloud);

    // 设置裁剪区域的最小和最大边界
    crop.setMin(min_pt);
    crop.setMax(max_pt);

    // 设置裁剪区域的平移量
    crop.setTranslation(translation);

    // 设置裁剪区域的旋转量
    crop.setRotation(rotation);

    // 设置是否取反裁剪结果
    crop.setNegative(negative);

    // 执行裁剪操作
    crop.filter(*cloud_filtered);

    // 输出裁剪后的点云信息
    std::cout << "Filtered cloud contains "
              << cloud_filtered->width * cloud_filtered->height
              << " data points." << std::endl;

    // 将裁剪后的点云保存为 PCD 文件
    pcl::io::savePCDFileASCII(output_file, *cloud_filtered);
    std::cout << "Filtered cloud saved to " << output_file << std::endl;
}

int main(int argc, char** argv)
{
    // 定义裁剪区域的最小和最大边界
    Eigen::Vector4f min_pt(-1.0, -1.0, -1.0, 1.0);
    Eigen::Vector4f max_pt(1.0, 1.0, 1.0, 1.0);

    // 定义裁剪区域的平移量
    Eigen::Vector3f translation(0.0, 0.0, 0.0);

    // 定义裁剪区域的旋转量
    Eigen::Vector3f rotation(0.0, 0.0, 0.0);

    // 定义是否取反裁剪结果
    bool negative = false;

    // 调用裁剪函数
    cropPointCloud("input_cloud.pcd", "output_cloud.pcd", min_pt, max_pt,
                   translation, rotation, negative);

    return 0;
}