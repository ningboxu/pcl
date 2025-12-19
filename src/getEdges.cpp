#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <cmath>

// 凸包边缘提取函数
pcl::PointCloud<pcl::PointXYZ>::Ptr extractConvexHullEdges(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    chull.setInputCloud(cloud);
    chull.reconstruct(*hull_cloud);
    return hull_cloud;
}

// 法向量变化边缘提取函数
pcl::PointCloud<pcl::PointXYZ>::Ptr extractNormalBasedEdges(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(50);  // 使用50个最近邻点计算法向量
    ne.compute(*normals);

    // 提取法向量变化较大的点作为边缘点
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    float angle_threshold = M_PI / 6.0;  // 法向量变化角度阈值，30度（可以调整）
    for (size_t i = 1; i < normals->points.size(); ++i)
    {
        const auto& n1 = normals->points[i - 1];
        const auto& n2 = normals->points[i];

        // 计算法向量之间的夹角
        float dot_product = n1.normal_x * n2.normal_x +
                            n1.normal_y * n2.normal_y +
                            n1.normal_z * n2.normal_z;
        float angle = std::acos(dot_product);

        // 如果夹角大于阈值，则认为是边缘点
        if (angle > angle_threshold)
        {
            edge_cloud->points.push_back(cloud->points[i]);
        }
    }

    // 设置点云的宽度和高度以确保可以保存
    edge_cloud->width    = edge_cloud->points.size();
    edge_cloud->height   = 1;
    edge_cloud->is_dense = false;  // 设置为无序点云

    std::cout << "Extracted " << edge_cloud->points.size()
              << " edge points based on normal variation." << std::endl;
    return edge_cloud;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_ply_file>" << std::endl;
        return -1;
    }

    // 加载点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file %s\n", argv[1]);
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->points.size()
              << " points." << std::endl;

    // 提取凸包边缘点
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_edges =
        extractConvexHullEdges(cloud);
    convex_hull_edges->width    = convex_hull_edges->points.size();
    convex_hull_edges->height   = 1;
    convex_hull_edges->is_dense = false;
    pcl::io::savePCDFileASCII("convex_hull_edges.pcd", *convex_hull_edges);
    std::cout << "Saved convex hull edge points to 'convex_hull_edges.pcd'."
              << std::endl;

    // 提取法向量变化边缘点
    pcl::PointCloud<pcl::PointXYZ>::Ptr normal_based_edges =
        extractNormalBasedEdges(cloud);
    pcl::io::savePCDFileASCII("normal_based_edges.pcd", *normal_based_edges);
    std::cout << "Saved normal-based edge points to 'normal_based_edges.pcd'."
              << std::endl;

    return 0;
}
