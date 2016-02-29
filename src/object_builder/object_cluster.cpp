/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.25
 *
 * TODO 记录用于描述物体的点云
 *
 */

#include "object_builder/object_cluster.h"

#include <sstream>

namespace tinker {
namespace vision {

// 新建物体
ObjectCluster::ObjectCluster(const PointCloudPtr& input_cloud)
    : cloud(input_cloud)
{
    int n = cloud->width * cloud->height;
    int r_sum = 0, g_sum = 0, b_sum = 0;

    x_min = y_min = z_min = 10000;
    x_max = y_max = z_max = -10000;

    for (int i=0; i<n; ++i)
    {
        PointT point = cloud->points[i];
        if (point.x<x_min) x_min = point.x;
        if (point.x>x_max) x_max = point.x;
        if (point.y<y_min) y_min = point.y;
        if (point.y>y_max) y_max = point.y;
        if (point.z<z_min) z_min = point.z;
        if (point.z>z_max) z_max = point.z;
        r_sum += point.r;
        g_sum += point.g;
        b_sum += point.b;
    }

    r_avg = r_sum / n; if (r_avg>255) r_avg = 255;
    g_avg = g_sum / n; if (g_avg>255) g_avg = 255;
    b_avg = b_sum / n; if (b_avg>255) b_avg = 255;

}

// 在viewer上绘制方框
void ObjectCluster::DrawBoundingBox(pcl::visualization::PCLVisualizer& viewer, int object_number)
{
    std::stringstream box_name;
    box_name << "object_box_" << object_number;
    viewer.removeShape(box_name.str());

    viewer.addCube(
            x_min, x_max, y_min, y_max, z_min, z_max,
            (double)r_avg/256, (double)g_avg/256, (double)b_avg/256,
            box_name.str());
    viewer.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
            box_name.str());

}

}
}
