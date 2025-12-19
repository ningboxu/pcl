#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sys/stat.h>            // 用于创建目录
#include <unistd.h>              // 用于获取当前工作目录
#include <boost/filesystem.hpp>  // 用于处理目录和文件操作
#include <cmath>
#include <fstream>  // 用于文件操作
#include <iostream>
#include <limits>  // 用于初始化最大值和最小值
#include <map>
#include <numeric>
#include <vector>

// 定义栅格结构体
struct Grid
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud{
        new pcl::PointCloud<pcl::PointXYZ>};
    double x_mean{0.0};
    double x_stddev{0.0};
    double x_min{std::numeric_limits<double>::max()};
    double x_max{std::numeric_limits<double>::lowest()};

    void addPoint(const pcl::PointXYZ& point)
    {
        grid_cloud->points.push_back(point);
        // 更新最小值和最大值
        if (point.x < x_min) x_min = point.x;
        if (point.x > x_max) x_max = point.x;
    }

    void computeStatistics()
    {
        if (grid_cloud->points.empty()) return;

        // 计算x的平均值
        double sum = 0.0;
        for (const auto& point : grid_cloud->points)
        {
            sum += point.x;
        }
        x_mean = sum / grid_cloud->points.size();

        // 计算x的标准差
        double accum = 0.0;
        for (const auto& point : grid_cloud->points)
        {
            accum += (point.x - x_mean) * (point.x - x_mean);
        }
        x_stddev = std::sqrt(accum / grid_cloud->points.size());
    }
};

// 栅格化点云类
class PointCloudGridProcessor
{
public:
    PointCloudGridProcessor(float grid_size_y, float grid_size_z)
        : grid_size_y_(grid_size_y), grid_size_z_(grid_size_z)
    {
    }

    void process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        grid_map_.clear();

        for (const auto& point : input_cloud->points)
        {
            auto grid_key = std::make_pair(
                static_cast<int>(std::floor(point.y / grid_size_y_)),
                static_cast<int>(std::floor(point.z / grid_size_z_)));

            grid_map_[grid_key].addPoint(point);
        }

        for (auto& grid_entry : grid_map_)
        {
            grid_entry.second.computeStatistics();
        }
    }

    void logStatistics() const
    {
        for (const auto& grid_entry : grid_map_)
        {
            const auto& grid_key = grid_entry.first;
            const auto& stats    = grid_entry.second;

            LOG(INFO) << "Grid (" << grid_key.first << ", " << grid_key.second
                      << "): Mean X = " << stats.x_mean
                      << ", Stddev X = " << stats.x_stddev
                      << ", Min X = " << stats.x_min
                      << ", Max X = " << stats.x_max
                      << ", Points Count = " << stats.grid_cloud->points.size();
        }
    }

    // 将统计数据保存到CSV文件中
    void saveStatisticsToCSV(const std::string& csv_file_path) const
    {
        std::ofstream file(csv_file_path);

        // 写入表头
        file << "Grid Y,Grid Z,Mean X,Stddev X,Min X,Max X,Points Count\n";

        for (const auto& grid_entry : grid_map_)
        {
            const auto& grid_key = grid_entry.first;
            const auto& stats    = grid_entry.second;

            file << grid_key.first << "," << grid_key.second << ","
                 << stats.x_mean << "," << stats.x_stddev << "," << stats.x_min
                 << "," << stats.x_max << "," << stats.grid_cloud->points.size()
                 << "\n";
        }

        file.close();
    }

private:
    float grid_size_y_;
    float grid_size_z_;
    std::map<std::pair<int, int>, Grid> grid_map_;
};

// 批量处理文件夹中的点云数据
void processPointCloudsInDirectory(const std::string& input_directory,
                                   const std::string& output_directory,
                                   float grid_size_y, float grid_size_z)
{
    namespace fs = boost::filesystem;

    // 检查输出目录是否存在，如果不存在则创建
    if (!fs::exists(output_directory))
    {
        fs::create_directory(output_directory);
    }

    // 遍历输入目录
    for (const auto& entry : fs::directory_iterator(input_directory))
    {
        if (fs::is_directory(entry))  // 处理每个文件夹
        {
            std::string folder_name = entry.path().filename().string();
            std::string folder_output_dir =
                output_directory + "/" + folder_name;

            // 创建子目录，如10cm、20cm等
            if (!fs::exists(folder_output_dir))
            {
                fs::create_directory(folder_output_dir);
            }

            // 遍历文件夹中的所有.pcd文件
            for (const auto& file_entry : fs::directory_iterator(entry))
            {
                std::string file_path = file_entry.path().string();
                if (file_entry.path().extension() == ".pcd")
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                        new pcl::PointCloud<pcl::PointXYZ>);

                    // 读取点云数据
                    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path,
                                                            *cloud) == -1)
                    {
                        PCL_ERROR("Couldn't read file %s \n",
                                  file_path.c_str());
                        continue;
                    }

                    // 创建并处理点云的栅格化对象
                    PointCloudGridProcessor processor(grid_size_y, grid_size_z);
                    processor.process(cloud);
                    processor.logStatistics();

                    // 将每个点云文件的统计数据保存为独立的CSV文件
                    std::string csv_file_name =
                        folder_output_dir + "/" +
                        file_entry.path().stem().string() + ".csv";
                    processor.saveStatisticsToCSV(csv_file_name);
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    // 设置日志输出目录为当前工作目录，即build目录
    FLAGS_log_dir = ".";

    // 设置日志级别和输出选项
    FLAGS_stderrthreshold =
        google::ERROR;  // 只将ERROR级别及以上的日志输出到stderr
    FLAGS_minloglevel = google::INFO;  // 最低日志级别为INFO
    FLAGS_logtostderr = false;  // 不将日志输出到stderr，写入日志文件
    FLAGS_colorlogtostderr = true;  // 启用颜色日志输出

    // 设置栅格大小
    float grid_size_y = 0.005f;
    float grid_size_z = 0.005f;

    // 输入和输出目录
    std::string input_directory = "/home/xnb/test/test_data/filtered_livox";
    std::string output_directory =
        "/home/xnb/test/pcl/data/output_csv";  // 新的输出路径

    // 批量处理点云数据并保存到指定输出路径
    processPointCloudsInDirectory(input_directory, output_directory,
                                  grid_size_y, grid_size_z);

    return 0;
}
