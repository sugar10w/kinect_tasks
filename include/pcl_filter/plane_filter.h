/*
 * Created by sugar10w, 2016.2.27 
 * Last edited by sugar10w, 2016.2.28
 *
 * 找到点云中的平面
 * 从其他点云中去除平面
 *
 */

#ifndef __PCL_PLANE_FILTER__
#define __PCL_PLANE_FILTER__

#include"common.h"

namespace tinker {
namespace vision {

class PlaneFilter
{
public:
    /* cloud 将用于提取平面的点云
     * leaf_size 降采样尺寸 */
    PlaneFilter(PointCloudPtr &cloud, float leaf_size=6.0f);
    ~PlaneFilter();

    /* cloud 将在cloud中去除平面 */
    void Filter(PointCloudPtr &cloud);
    
    /* 获取平面信息 */
    Eigen::VectorXf GetPlane();

private:
    float leaf_size_;
    Eigen::VectorXf plane_;
    
};    

}
}

#endif //__PCL_PLANE_FILTER__
