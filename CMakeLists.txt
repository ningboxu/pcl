cmake_minimum_required(VERSION 3.10)
project(PointCloudProcessing)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找PCL库
find_package(PCL 1.8 REQUIRED)

# 包含PCL头文件
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

# 链接PCL库
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 获取src目录及其子目录下的所有源文件
file(GLOB_RECURSE SRC_FILES "src/*.cpp")
list(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/point_cloud_processor.cpp")

# 为每个包含main函数的源文件生成一个可执行文件
foreach(SRC_FILE ${SRC_FILES})
    # 获取源文件名而不包括扩展名
    get_filename_component(EXECUTABLE_NAME ${SRC_FILE} NAME_WE)
    
    # 添加可执行文件
    add_executable(${EXECUTABLE_NAME} ${SRC_FILE} src/point_cloud_processor.cpp)
    
    # 链接PCL库
    target_link_libraries(${EXECUTABLE_NAME} ${PCL_LIBRARIES})
endforeach()
