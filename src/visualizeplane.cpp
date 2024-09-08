#include <glog/logging.h>
#include <pcl/common/centroid.h>  // 用于计算点云的质心
#include <pcl/features/moment_of_inertia_estimation.h>  // 用于计算点云的特征信息
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>  // 用于可视化
#include <Eigen/Geometry>
#include <chrono>  // 用于统计时间
#include <cmath>   // 用于计算平方根等数学操作
#include <iostream>
#include <thread>  // 用于 std::this_thread::sleep_for

// #include "utils.h"
void ConvertPointCloudToMeters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (auto &point : cloud->points)
    {
        point.x /= 1000.0f;  // 将 x 坐标从 mm 转换为 m
        point.y /= 1000.0f;  // 将 y 坐标从 mm 转换为 m
        point.z /= 1000.0f;  // 将 z 坐标从 mm 转换为 m
    }
}
void visualizePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const pcl::ModelCoefficients::Ptr &coefficients)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 添加原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(
        cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "sample cloud");

    // 添加拟合平面
    viewer->addPlane(*coefficients, "plane");

    // 设置显示点云的尺寸
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    // 开始可视化
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv)
{
    using namespace std::chrono;

    // 记录程序总开始时间
    auto program_start = high_resolution_clock::now();

    if (argc < 2)
    {
        std::cerr << "Please provide a path to the point cloud file!\n";
        return -1;
    }

    std::string file_path = argv[1];

    // 日志设置
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold  = google::ERROR;
    FLAGS_minloglevel      = google::INFO;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir          = "./";

    // 加载点云数据
    auto start_load = high_resolution_clock::now();  // 记录开始时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (file_path.substr(file_path.find_last_of(".") + 1) == "pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PCD file\n");
            return -1;
        }
    }
    else if (file_path.substr(file_path.find_last_of(".") + 1) == "ply")
    {
        pcl::PLYReader reader;
        if (reader.read(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PLY file\n");
            return -1;
        }
    }
    else
    {
        std::cerr
            << "Unsupported file format! Supported formats are PCD and PLY.\n";
        return -1;
    }
    auto end_load = high_resolution_clock::now();  // 记录结束时间
    LOG(INFO) << "Loaded " << cloud->width * cloud->height
              << " data points from " << file_path;
    std::cout << "Loading time: "
              << duration_cast<milliseconds>(end_load - start_load).count()
              << " ms" << std::endl;

    // 将点云单位从毫米转换为米
    auto start_convert = high_resolution_clock::now();  // 记录开始时间
    ConvertPointCloudToMeters(cloud);
    LOG(INFO) << "Converted point cloud units from mm to meters.";
    auto end_convert = high_resolution_clock::now();  // 记录结束时间
    std::cout
        << "Conversion time: "
        << duration_cast<milliseconds>(end_convert - start_convert).count()
        << " ms" << std::endl;

    // 计算点云中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    std::cout << "Centroid of point cloud: (" << centroid[0] << ", "
              << centroid[1] << ", " << centroid[2] << ")" << std::endl;
    Eigen::Vector3f centroid_vec(centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ centroid_point(centroid[0], centroid[1], centroid[2]);

    // 设置平面分割器
    auto start_seg = high_resolution_clock::now();  // 记录开始时间
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    auto end_seg = high_resolution_clock::now();  // 记录结束时间

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    std::cout << "Segmentation time: "
              << duration_cast<milliseconds>(end_seg - start_seg).count()
              << " ms" << std::endl;

    // 打印平面模型
    std::cout << "Model coefficients: ";
    for (auto val : coefficients->values) std::cout << val << " ";
    std::cout << std::endl;

    // 可视化平面
    visualizePlane(cloud, coefficients);

    return 0;
}
