#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace pcl;
using namespace std;

// 计算点到平面的投影
Eigen::Vector3f projectPointToPlane(const Eigen::Vector3f& point,
                                    const Eigen::Vector3f& normal, float D)
{
    // 计算点到平面的距离
    float distance = (point.dot(normal) + D) / normal.norm();
    // 计算投影点
    return point - distance * normal;
}

// 拟合圆的最小二乘法
void fitCircleToPoints(const std::vector<Eigen::Vector3f>& points,
                       Eigen::Vector3f& center, float& radius)
{
    // 最小二乘法拟合圆，平面上用2D圆拟合，Z值固定
    Eigen::MatrixXf A(points.size(), 3);
    Eigen::VectorXf b(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        float x = points[i].x();
        float y = points[i].y();
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1;
        b(i)    = x * x + y * y;
    }

    Eigen::VectorXf solution =
        (A.transpose() * A).ldlt().solve(A.transpose() * b);

    // 计算圆心和半径，圆心是2D坐标，Z坐标保持不变
    center = Eigen::Vector3f(solution(0) / 2, solution(1) / 2, points[0].z());
    radius = std::sqrt(solution(2) + center.head<2>().squaredNorm());
}

// 生成拟合圆的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCircleCloud(
    const Eigen::Vector3f& center, float radius, int num_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 在圆上均匀分布 num_points 个点
    for (int i = 0; i < num_points; ++i)
    {
        float angle = 2.0f * M_PI * i / num_points;
        float x     = center.x() + radius * cos(angle);
        float y     = center.y() + radius * sin(angle);
        float z     = center.z();  // 保持圆在平面上，Z坐标不变

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        circle_cloud->points.push_back(point);
    }

    // 也可以把圆心加入点云中
    pcl::PointXYZ center_point;
    center_point.x = center.x();
    center_point.y = center.y();
    center_point.z = center.z();
    circle_cloud->points.push_back(center_point);

    // 设置点云的宽度和高度
    circle_cloud->width  = circle_cloud->points.size();
    circle_cloud->height = 1;

    return circle_cloud;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    // 从命令行参数获取点云文件路径
    std::string input_file = argv[1];

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file\n");
        return -1;
    }

    std::cout << "Loaded point cloud from: " << input_file << std::endl;

    // 假设平面参数已经给定
    float A = -0.0180648f;
    float B = 0.00186412f;
    float C = 0.999835f;
    float D = -0.356201f;

    // 打印平面方程
    std::cout << "Plane Equation: A = " << A << ", B = " << B << ", C = " << C
              << ", D = " << D << std::endl;

    // 创建存储投影后点云的容器
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_top_plane_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 平面的法向量
    Eigen::Vector3f planeNormal(A, B, C);

    // 将点云投影到平面上
    for (const auto& point : cloud->points)
    {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f projectedPoint = projectPointToPlane(p, planeNormal, D);

        pcl::PointXYZ projectedPCLPoint;
        projectedPCLPoint.x = projectedPoint.x();
        projectedPCLPoint.y = projectedPoint.y();
        projectedPCLPoint.z = projectedPoint.z();

        // 添加投影后的点到点云
        convex_top_plane_cloud->points.push_back(projectedPCLPoint);
    }

    // 设置投影点云的 width 和 height
    convex_top_plane_cloud->width = convex_top_plane_cloud->points.size();
    convex_top_plane_cloud->height = 1;  // 可以设置为 1，因为这是一个散点云

    // 保存投影后的点云
    std::string output_file = "projected_points.pcd";
    pcl::io::savePCDFileASCII(output_file, *convex_top_plane_cloud);
    std::cout << "Saved projected points to " << output_file << std::endl;

    // 拟合圆并找到圆心
    std::vector<Eigen::Vector3f> projectedPoints3D;
    for (const auto& point : convex_top_plane_cloud->points)
    {
        projectedPoints3D.push_back(Eigen::Vector3f(point.x, point.y, point.z));
    }

    Eigen::Vector3f circleCenter;
    float radius;
    fitCircleToPoints(projectedPoints3D, circleCenter, radius);

    // 输出圆心和半径
    std::cout << "Circle center: (" << circleCenter.x() << ", "
              << circleCenter.y() << ", " << circleCenter.z() << ")"
              << std::endl;
    std::cout << "Radius: " << radius << std::endl;

    // 生成拟合圆的点云（包括圆心）
    pcl::PointCloud<pcl::PointXYZ>::Ptr circleCloud =
        generateCircleCloud(circleCenter, radius, 100);

    // 保存拟合圆的点云
    std::string circle_output_file = "fitted_circle.pcd";
    pcl::io::savePCDFileASCII(circle_output_file, *circleCloud);
    std::cout << "Saved fitted circle to " << circle_output_file << std::endl;

    return 0;
}
