#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>

// 计算残差（每个点到平面的距离）
double calculateError(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const Eigen::Vector4f& coefficients)
{
    double total_error = 0.0;
    double a = coefficients[0], b = coefficients[1], c = coefficients[2],
           d = coefficients[3];

    for (const auto& point : cloud->points)
    {
        double distance =
            std::abs(a * point.x + b * point.y + c * point.z + d) /
            std::sqrt(a * a + b * b + c * c);
        total_error += distance * distance;
    }

    return total_error / cloud->points.size();  // 返回均方误差
}

// 使用RANSAC拟合平面
Eigen::Vector4f fitPlaneRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.001);  // 设置为 1mm 以适应 z 轴 5mm 的尺寸

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return Eigen::Vector4f(0, 0, 0, 0);  // 返回一个无效的平面系数
    }

    // 返回拟合的平面模型系数
    return Eigen::Vector4f(coefficients->values[0], coefficients->values[1],
                           coefficients->values[2], coefficients->values[3]);
}

// 使用最小二乘法拟合平面
Eigen::Vector4f fitPlaneLeastSquares(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 计算点云的中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // 减去质心，将点云归一化
    Eigen::MatrixXf centered(cloud->points.size(), 3);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        centered(i, 0) = cloud->points[i].x - centroid[0];
        centered(i, 1) = cloud->points[i].y - centroid[1];
        centered(i, 2) = cloud->points[i].z - centroid[2];
    }

    // 计算协方差矩阵
    Eigen::Matrix3f covariance =
        (centered.transpose() * centered) / cloud->points.size();

    // 对协方差矩阵进行特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(
        0);  // 最小特征值对应的特征向量就是法向量

    // 平面方程的系数 ax + by + cz + d = 0
    float d = -normal.dot(centroid.head<3>());
    return Eigen::Vector4f(normal[0], normal[1], normal[2], d);
}

// 输出平面系数
void printPlaneCoefficients(const std::string& method_name,
                            const Eigen::Vector4f& coefficients)
{
    std::cout << method_name << " Plane coefficients: " << coefficients[0]
              << " " << coefficients[1] << " " << coefficients[2] << " "
              << coefficients[3] << std::endl;
}

// 可视化点云和拟合平面
void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
               const Eigen::Vector4f& coefficients)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 添加点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(
        cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "sample cloud");

    // 添加平面
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize(4);
    plane_coeff.values[0] = coefficients[0];
    plane_coeff.values[1] = coefficients[1];
    plane_coeff.values[2] = coefficients[2];
    plane_coeff.values[3] = coefficients[3];

    // 指定平面的大小，确保它覆盖足够的区域
    viewer->addPlane(plane_coeff, 0.0, 0.0, 0.0, "plane",
                     10);  // 10 代表平面的尺寸扩大倍数

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);

    // 调整相机视角以确保平面可见
    viewer->setCameraPosition(0, 0, -3, 0, -1, 0);  // 根据点云范围调整相机位置

    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char** argv)
{
    // 检查是否提供了点云路径参数
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_ply_file>" << std::endl;
        return -1;
    }

    std::string file_path = argv[1];

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 从命令行提供的PLY文件加载点云
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file %s\n", file_path.c_str());
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->points.size()
              << " points from " << file_path << std::endl;

    // 计时并使用RANSAC拟合平面
    auto start_ransac = std::chrono::high_resolution_clock::now();
    Eigen::Vector4f plane_coefficients_ransac = fitPlaneRANSAC(cloud);
    auto end_ransac = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_ransac = end_ransac - start_ransac;
    printPlaneCoefficients("RANSAC", plane_coefficients_ransac);

    // 计时并使用最小二乘法拟合平面
    auto start_least_squares = std::chrono::high_resolution_clock::now();
    Eigen::Vector4f plane_coefficients_least_squares =
        fitPlaneLeastSquares(cloud);
    auto end_least_squares = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_least_squares =
        end_least_squares - start_least_squares;
    printPlaneCoefficients("Least Squares", plane_coefficients_least_squares);

    // 计算误差
    double ransac_error = calculateError(cloud, plane_coefficients_ransac);
    double least_squares_error =
        calculateError(cloud, plane_coefficients_least_squares);

    std::cout << "RANSAC MSE: " << ransac_error
              << ", Time: " << duration_ransac.count() << " seconds"
              << std::endl;
    std::cout << "Least Squares MSE: " << least_squares_error
              << ", Time: " << duration_least_squares.count() << " seconds"
              << std::endl;

    // 可视化RANSAC结果
    visualize(cloud, plane_coefficients_ransac);

    return 0;
}
