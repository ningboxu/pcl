#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <limits>

// 定义一个结构体来存储最小点和最大点
struct MinMaxPoints
{
    pcl::PointXYZ minPoint;
    pcl::PointXYZ maxPoint;
};

MinMaxPoints findMinMaxPointOnAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, char axis)
{
    MinMaxPoints result;
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud->points)
    {
        float value = 0.0f;
        switch (axis)
        {
        case 'x':
            value = point.x;
            break;
        case 'y':
            value = point.y;
            break;
        case 'z':
            value = point.z;
            break;
        default:
            std::cerr << "Invalid axis specified. Please use 'x', 'y' or 'z'."
                      << std::endl;
            return result;
        }

        if (value < minValue)
        {
            minValue        = value;
            result.minPoint = point;
        }

        if (value > maxValue)
        {
            maxValue        = value;
            result.maxPoint = point;
        }
    }

    return result;
}

int main()
{
    // 示例点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.emplace_back(1.0, 2.0, 3.0);
    cloud->points.emplace_back(-1.0, 4.0, 0.5);
    cloud->points.emplace_back(3.0, -2.0, 1.5);

    // 查找x轴上的最小和最大点
    MinMaxPoints minMaxX = findMinMaxPointOnAxis(cloud, 'x');
    std::cout << "Minimum point on x-axis: (" << minMaxX.minPoint.x << ", "
              << minMaxX.minPoint.y << ", " << minMaxX.minPoint.z << ")"
              << std::endl;
    std::cout << "Maximum point on x-axis: (" << minMaxX.maxPoint.x << ", "
              << minMaxX.maxPoint.y << ", " << minMaxX.maxPoint.z << ")"
              << std::endl;

    return 0;
}
