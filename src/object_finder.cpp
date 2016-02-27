/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.25
 *
 * 测试 rgb_object_filter
 *
 */

#include "rgb_filter/rgb_object_filter.h"

#include <pcl/io/pcd_io.h>

#include "kinect2pcl/point_cloud_builder.h"
#include "load_rgbd.h"

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

    PointCloudPtr cloud_object = GetCloudFromMask(raw_img, depth_img, object_mask);
    pcl::io::savePCDFile("object.pcd", *cloud_object);

    PointCloudPtr cloud_back = GetCloudFromMask(raw_img, depth_img, back_mask);
    pcl::io::savePCDFile("back.pcd", *cloud_back);
    
    PointCloudPtr cloud_lines = GetCloudFromMask(raw_img, depth_img, ~lines_mask);
    pcl::io::savePCDFile("lines.pcd", *cloud_lines);


}
