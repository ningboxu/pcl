#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
#include <iostream>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_file.ply>" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];

    // 创建所需的对象
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // 创建点云和法线云对象
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(
        new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

    // 读取点云数据
    if (reader.read(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return -1;
    }
    std::cerr << "PointCloud has: " << cloud->size() << " data points."
              << std::endl;

    // 计算点法线
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // 创建分割对象并设置所有参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.005);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    // 获取圆柱体内点和系数
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder
              << std::endl;

    // 提取圆柱体的点云
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
    extract.filter(*cloud_cylinder);

    if (cloud_cylinder->points.empty())
    {
        std::cerr << "Can't find the cylindrical component." << std::endl;
    }
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: "
                  << cloud_cylinder->size() << " data points." << std::endl;
        writer.write("cylinder_inliers.ply", *cloud_cylinder, false);
    }

    // 获取点云的Z轴范围，并找出Z最小值和最大值对应的点
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    PointT min_point, max_point;

    for (const auto& point : cloud->points)
    {
        if (point.z < z_min)
        {
            z_min     = point.z;
            min_point = point;
        }
        if (point.z > z_max)
        {
            z_max     = point.z;
            max_point = point;
        }
    }

    // 获取圆柱体参数
    float cx     = coefficients_cylinder->values[0];
    float cy     = coefficients_cylinder->values[1];
    float cz     = coefficients_cylinder->values[2];
    float dx     = coefficients_cylinder->values[3];
    float dy     = coefficients_cylinder->values[4];
    float dz     = coefficients_cylinder->values[5];
    float radius = coefficients_cylinder->values[6];

    // 将Z最小值点和Z最大值点投影到圆柱体轴线上
    auto project_to_axis = [&](const PointT& point)
    {
        float t =
            (dx * (point.x - cx) + dy * (point.y - cy) + dz * (point.z - cz)) /
            (dx * dx + dy * dy + dz * dz);
        PointT projected_point;
        projected_point.x = cx + t * dx;
        projected_point.y = cy + t * dy;
        projected_point.z = cz + t * dz;
        return projected_point;
    };

    PointT proj_min = project_to_axis(min_point);
    PointT proj_max = project_to_axis(max_point);

    // 生成圆柱体点云，确保从投影的Z最小点到Z最大点生成
    float z_step = 0.01;  // Z轴方向上的步长
    int num_theta = 360;  // 每层的角度分辨率（360表示每1度一个点）
    pcl::PointCloud<PointT>::Ptr generated_cylinder(
        new pcl::PointCloud<PointT>);

    // 从投影的z_min点生成到z_max点
    for (float z = proj_min.z; z <= proj_max.z; z += z_step)
    {
        float current_z_offset = (z - proj_min.z);
        float x_offset         = current_z_offset * dx;
        float y_offset         = current_z_offset * dy;
        float z_coord          = proj_min.z + current_z_offset * dz * (-1);

        for (int i = 0; i < num_theta; ++i)
        {
            float theta = 2 * M_PI * i / num_theta;  // 角度

            PointT point;
            float r_x = radius * cos(theta);
            float r_y = radius * sin(theta);

            point.x = proj_min.x + r_x + x_offset;
            point.y = proj_min.y + r_y + y_offset;
            point.z = z_coord;

            generated_cylinder->points.push_back(point);
        }
    }

    generated_cylinder->width    = generated_cylinder->points.size();
    generated_cylinder->height   = 1;
    generated_cylinder->is_dense = true;

    // 保存生成的圆柱体点云
    writer.write("generated_cylinder.ply", *generated_cylinder, false);

    return 0;
}
