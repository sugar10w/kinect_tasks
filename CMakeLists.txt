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
    src/kinect2pcl/kinect_parameters.cpp    ### Kinect标定数据(注意，这里需要和point_cloud_builder,Protonect相互配合)
    src/kinect2pcl/point_cloud_builder.cpp  ### 将原始数据合成为点云 
    src/kinect2pcl/image_rebuild.cpp        ### 将点云投影到二维图片
)
## object_builder: 采集、分离、识别物品
SET(object_builder_src
    src/object_builder/target_locator.cpp   ### 确定目标的位置，以及基本的目标处理操作
    src/object_builder/cluster_divider.cpp  ### 欧几里得聚类
    src/object_builder/object_cluster.cpp   ### 针对物体的点云类的封装
)    
## rgb_filter: 处理BGR信息
SET(rgb_filter_src
    src/rgb_filter/color_block_filter.cpp   ### 找色块
    src/rgb_filter/entropy_filter.cpp       ### 计算熵
    src/rgb_filter/line_filter.cpp          ### 找直线
    src/rgb_filter/color_fixer.cpp          ### 填补黑色点
    src/rgb_filter/rgb_object_filter.cpp    ### BGR方法总结
)
## pcl_filter: 处理点云信息
SET(pcl_filter_src
    src/pcl_filter/point_cloud_minus.cpp    ### 点云的相减
    src/pcl_filter/plane_filter.cpp         ### 过滤平面
)    

# 各演示程序

## display_3d: 实时显示三维点云
ADD_EXECUTABLE(display_3d
    src/display_3d.cpp
    ${kinect2pcl_src}
)
TARGET_LINK_LIBRARIES(display_3d
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)        

## display_3d_object_finder: 实时标出画面中的物品
ADD_EXECUTABLE(display_3d_object_finder
    src/display_3d_object_finder.cpp
    ${kinect2pcl_src}
    ${rgb_filter_src}
    ${pcl_filter_src}
    ${object_builder_src}
)
TARGET_LINK_LIBRARIES(display_3d_object_finder
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}    
)    

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

## object_finder: 提取目标
ADD_EXECUTABLE(object_finder 
    src/object_finder.cpp
    src/load_rgbd.cpp
    ${kinect2pcl_src}
    ${rgb_filter_src}
    ${pcl_filter_src}
    ${object_builder_src}
)
TARGET_LINK_LIBRARIES(object_finder
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

## demo_object_rgb_filter: 演示RGB目标提取方法
ADD_EXECUTABLE(demo_object_rgb_filter
    src/demo_object_rgb_filter.cpp
    ${rgb_filter_src}
)
TARGET_LINK_LIBRARIES(demo_object_rgb_filter
    ${OpenCV_LIBS}
)

## kinect_take_raw: 采集物体的原始信息（旋转展台）
ADD_EXECUTABLE(kinect_take_raw
    src/kinect_take_raw.cpp
)
TARGET_LINK_LIBRARIES(kinect_take_raw
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
)

## object_builder_from_raw: 从物体的原始信息合成出物体的全方向点云
ADD_EXECUTABLE(object_builder_from_raw
    src/object_builder_from_raw.cpp
    src/load_rgbd.cpp
    ${kinect2pcl_src}
    ${pcl_filter_src}
    ${object_builder_src}
)
TARGET_LINK_LIBRARIES(object_builder_from_raw
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)


