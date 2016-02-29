
#include "object_builder/cluster_divider.h"

//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include "pcl_rebuild/ImageRebuild.h"
//#include "pcl_rebuild/ClusterDivider.h"

#include "common.h"

namespace tinker
{
namespace vision
{

    ClusterDivider::ClusterDivider(PointCloudPtr point_cloud)
        :point_cloud_(point_cloud)
    { }

    std::vector<ObjectCluster> ClusterDivider::GetDividedCluster()
    {
        PointCloudPtr cloud_filtered (new PointCloud);
        float leaf_size_ = 1.0f;

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud (point_cloud_);
        vg.setLeafSize (leaf_size_, leaf_size_, leaf_size_);  // TODO set it manually
        vg.filter (*cloud_filtered);

        // the kdtree for ec
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud_filtered);

        // ec
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (2*leaf_size_); // 4cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        //TODO extract ??
        std::vector<ObjectCluster> divided_clouds;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PointCloudPtr cloud_cluster(new PointCloud);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            ObjectCluster object_cluster(cloud_cluster);
            divided_clouds.push_back(object_cluster);
        }
        return divided_clouds;
    } 


}
}

