/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.28
 *
 * 测试 rgb_object_filter
 * 以及 pcl_filter
 *
 */


#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>

#include "object_builder/cluster_divider.h"
#include "kinect2pcl/point_cloud_builder.h"
#include "rgb_filter/rgb_object_filter.h"
#include "pcl_filter/plane_filter.h"
#include "load_rgbd.h"
#include "common.h"

using namespace tinker::vision;

PointCloudPtr GetCloudFromMask(cv::Mat& raw_img, cv::Mat& depth_img, cv::Mat mask)
{
    cv::Mat depth = depth_img.clone();
    depth.setTo(0, ~mask);
    PointCloudBuilder builder(depth, raw_img);
    PointCloudPtr cloud = builder.getPointCloud();
    return cloud;
} 

int main(int argc, char* argv[])
{
    /* 获取depth, registered */
    if (argc!=3)
    {
       std::cout<<"Usage: "<<argv[0]<<" depth img"<<std::endl;
       return -1;
    }
    std::string filename = argv[2];
	std::string depthfilename = argv[1];

    cv::Mat raw_img = cv::imread(filename);
	cv::Mat depth_img = loadDepth(depthfilename);

	/* 输入检查 */
    if (raw_img.empty()) 
	{ std::cout<<"Cannot open "<<filename<<std::endl; return -1; }
	else if (depth_img.empty())
	{ std::cout<<"Cannot open "<<depthfilename<<std::endl; return -1; }
	else if (raw_img.cols!=depth_img.cols || raw_img.rows!=depth_img.rows)
	{ std::cout<<filename<<" and "<<depthfilename 
        <<" do not match."<<std::endl;
	   	return -1; }
	else if (raw_img.cols>600 || raw_img.rows>600)
	{ std::cout<<filename<<"is larger than 600*600."<<std::endl; return -1; }

    RgbObjectFilter rgb_filter(raw_img);
    cv::Mat object_mask = rgb_filter.GetObjectMask();
    cv::Mat back_mask   = rgb_filter.GetBackMask();
    cv::Mat lines_mask  = rgb_filter.GetLinesMask();

    //raw (xyzMat required)
    PointCloudBuilder raw_builder(depth_img, raw_img);
    raw_builder.setBuildXyzMat(true);
    PointCloudPtr cloud_raw = raw_builder.getPointCloud();
    cv::Mat cloud_raw_mat = raw_builder.getXyzMat();

    
    //back
    PointCloudPtr cloud_back = GetCloudFromMask(raw_img, depth_img, back_mask);
    pcl::io::savePCDFile("back.pcd", *cloud_back, true);
    
    PlaneFilter plane_filter(cloud_back);

    //object
    PointCloudPtr cloud_object = GetCloudFromMask(raw_img, depth_img, object_mask);
    plane_filter.Filter(cloud_object);
    pcl::io::savePCDFile("object.pcd", *cloud_object, true);

    //lines
    PointCloudPtr cloud_lines = GetCloudFromMask(raw_img, depth_img, ~lines_mask);
    //plane_filter.Filter(cloud_lines);
    pcl::io::savePCDFile("lines.pcd", *cloud_lines, true);
   

    //raw with no plane
    //plane_filter.Filter(cloud_raw);
    //pcl::io::savePCDFile("raw_no_plane.pcd", *cloud_raw, true); 



    // prepare for the cloud to display :::
    PointCloudPtr cloud_display(new PointCloud);
    float leaf_size_ = 2.0f;
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_raw);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud_display);

    *cloud_display += *cloud_object;

    // initialize the viewer and set the cloud inside 
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(cloud_display);
    viewer.addPointCloud<PointT> (cloud_display, rgb_handler, "cloud_object");

    // Get the clusters and Draw them on the viewer
    ClusterDivider cluster_divider(cloud_object);
    std::vector<ObjectCluster> clusters = cluster_divider.GetDividedCluster();
    for (int i=0; i<clusters.size(); ++i)
    {
        clusters[i].DrawBoundingBox(viewer, i);
    }

    viewer.spin();
}
