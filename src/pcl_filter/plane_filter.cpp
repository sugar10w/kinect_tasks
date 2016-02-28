/*
 * Created by sugar10w, 2016.2.27 
 * Last edited by sugar10w, 2016.2.28
 *
 * 找到点云中的平面
 * 从其他点云中去除平面
 *
 */

#include"pcl_filter/plane_filter.h"

#include<iostream>
#include<vector>

#include<pcl/io/pcd_io.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/sample_consensus/sac_model_plane.h>  // //
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>

namespace tinker {
namespace vision {    


PlaneFilter::PlaneFilter(PointCloudPtr &cloud, float leaf_size)
    : leaf_size_(leaf_size)
{

    //降采样
    PointCloudPtr cloud_downsampled(new PointCloud);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud_downsampled);

    // prepare 
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //segmentation
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true); //TODO ???
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(6);

    seg.setInputCloud(cloud_downsampled);
    seg.segment(*inliers, *coefficients);

    //coefficients to plane_   TODO 直接采用统一格式
    plane_.resize(4);
    plane_(0) = coefficients->values[0];
    plane_(1) = coefficients->values[1];
    plane_(2) = coefficients->values[2];
    plane_(3) = coefficients->values[3];

}

PlaneFilter::~PlaneFilter()
{
}

void PlaneFilter::Filter(PointCloudPtr &cloud)
{

    // 取出点
    pcl::IndicesPtr inliers(new std::vector<int>);
    pcl::SampleConsensusModelPlane<PointT>::Ptr plane_model(
            new pcl::SampleConsensusModelPlane<PointT>(cloud));
    plane_model->selectWithinDistance(plane_, leaf_size_, *inliers);

    // extract
    PointCloudPtr cloud_no_plane(new PointCloud);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_plane);
    
    cloud = cloud_no_plane;
}

Eigen::VectorXf PlaneFilter::GetPlane()
{
    return plane_;    
}

}
}
