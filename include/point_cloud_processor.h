#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <string>

void removeInvalidPoints(const std::string& input_file, const std::string& output_file);

#endif // POINT_CLOUD_PROCESSOR_H

