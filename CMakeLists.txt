cmake_minimum_required(VERSION 3.10)
project(PointCloudProcessing)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置构建类型
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

# 设置编译器标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-deprecated")

# 查找PCL库
find_package(PCL 1.8 REQUIRED COMPONENTS common io features filters segmentation visualization surface)

# 查找Boost库，特别是文件系统组件
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 查找OpenMP库
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 包含头文件目录
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

# 链接库目录
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

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
    
    # 链接所需的库
    target_link_libraries(${EXECUTABLE_NAME}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        glog
        )
endforeach()
