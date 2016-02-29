/*
 * Created by 郭嘉丞, 2016.1
 * Last edited by sugar10w, 2016.2.29
 *
 * 将一个点云拆分为多个点云
 *
 */

#ifndef __CLUSTER_DIVIDER_H__
#define __CLUSTER_DIVIDER_H__

#include <opencv2/opencv.hpp>

#include "object_builder/object_cluster.h"
#include "common.h"

namespace tinker
{
namespace vision
{
class ClusterDivider
{
public:
    ClusterDivider(PointCloudPtr point_cloud);
    std::vector<ObjectCluster> GetDividedCluster();
private:
    PointCloudPtr point_cloud_;
};
}
}

#endif

