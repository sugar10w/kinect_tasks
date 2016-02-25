CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

PROJECT(Kinect_tasks)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED) # libusb and freenect2 needed

INCLUDE_DIRECTORIES(
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ./include/
)

LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS}
)

ADD_DEFINITIONS(
    ${PCL_DEFINITIONS}
)

# 各模块源代码
## kinect2pcl: Kinect原始数据和pcl之间的转化
SET(kinect2pcl_src
    src/kinect2pcl/kinect_parameters.cpp
    src/kinect2pcl/point_cloud_builder.cpp
    src/kinect2pcl/image_rebuild.cpp
)
## object_builder: 采集物品信息
SET(object_builder_src
    src/object_builder/target_locator.cpp
)    
## rgb_filter: 处理BGR信息
SET(rgb_filter_src
    src/rgb_filter/color_block_filter.cpp
    src/rgb_filter/entropy_filter.cpp
    src/rgb_filter/line_filter.cpp
    src/rgb_filter/color_fixer.cpp
)
## pcl_filter: 处理点云信息
SET(pcl_filter_src
    src/pcl_filter/point_cloud_minus.cpp
)    

# 各演示程序
## display_people_detection: 实时显示合成的点云并标出画面中的站立人物
ADD_EXECUTABLE(display_people_detection
    src/display_people_detection.cpp
    ${kinect2pcl_src}
)
TARGET_LINK_LIBRARIES(display_people_detection
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

## demo_object_rgb_filter: 演示RGB目标提取方法
ADD_EXECUTABLE(demo_object_rgb_filter
    src/rgb_filter/demo_object_rgb_filter.cpp
    ${rgb_filter_src}
)
TARGET_LINK_LIBRARIES(demo_object_rgb_filter
    ${OpenCV_LIBS}
)

## kinect_take_raw: 采集物体的原始信息（旋转展台）
ADD_EXECUTABLE(kinect_take_raw
    src/object_builder/kinect_take_raw.cpp
)
TARGET_LINK_LIBRARIES(kinect_take_raw
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
)

## object_builder_from_raw: 从物体的原始信息合成出物体的全方向点云
ADD_EXECUTABLE(object_builder_from_raw
    src/object_builder/object_builder_from_raw.cpp
    src/load_rgbd.cpp
    ${kinect2pcl_src}
    ${object_builder_src}
    ${pcl_filter_src}
)
TARGET_LINK_LIBRARIES(object_builder_from_raw
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)


