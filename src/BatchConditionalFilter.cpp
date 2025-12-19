#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <iostream>

void processPointCloud(const std::string& input_file,
                       const std::string& output_file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return;
    }

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(
        new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT,
                                                -0.094)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT,
                                                0.548)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT,
                                                -0.745)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT,
                                                -0.037)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(false);
    condrem.filter(*cloud_filtered);

    pcl::io::savePCDFile(output_file, *cloud_filtered);
}

int main(int argc, char** argv)
{
    // 指定数据目录
    std::string base_directory = "/home/xnb/test/pcl/data/livox";

    boost::filesystem::recursive_directory_iterator end;
    for (boost::filesystem::recursive_directory_iterator iter(base_directory);
         iter != end; ++iter)
    {
        const auto& entry = *iter;
        if (entry.path().extension() == ".pcd")
        {
            std::string input_path = entry.path().string();
            std::string output_path =
                input_path.substr(0, input_path.size() - 4) + "_filtered.pcd";
            processPointCloud(input_path, output_path);
            std::cout << "Processed: " << input_path << " -> " << output_path
                      << std::endl;
        }
    }

    return 0;
}
