/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.25
 *
 * TODO 记录用于描述物体的点云
 *
 */

#ifndef __OBJECTFINDER_OBJECT_CLUSTER_H__
#define __OBJECTFINDER_OBJECT_CLUSTER_H__


#include <pcl/visualization/pcl_visualizer.h>

#include "common.h"

namespace tinker {
namespace vision {

class ObjectCluster
{
public:
    // 新建物体
    ObjectCluster(const PointCloudPtr& input_cloud);
    // 在viewer上绘制方框
    void DrawBoundingBox(pcl::visualization::PCLVisualizer& viewer, int object_number);
private:
    // 大概的位置和颜色信息
    float x_min, x_max, y_min, y_max, z_min, z_max;
    int r_avg, g_avg, b_avg;
    // 点云
    PointCloudPtr cloud;
    
};

}
}

#endif //OBJECTFINDER_OBJECT_CLUSTER_H__
