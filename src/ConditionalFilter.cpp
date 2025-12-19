#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char** argv) {
    // 定义点云对象指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/xnb/test/pcl/data/pc_2024-09-03_10-12-29.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return -1;
    }

    // 定义条件滤波对象
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    // 添加y方向的范围条件
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.094)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.548)));

    // 添加z方向的范围条件
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.745)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, -0.037)));

    // 使用条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(false);
    condrem.filter(*cloud_filtered);

    // 保存滤波后的点云
    pcl::io::savePCDFile("filtered_output.pcd", *cloud_filtered);

    std::cout << "Cloud before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud after filtering: " << cloud_filtered->points.size() << std::endl;

    return 0;
}

