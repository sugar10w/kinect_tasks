/*
 * Created by sugar10w, 2016.1
 * Last edited by sugar10w, 2016.2.28
 *
 * 用附近点的平均颜色填充图片内的无效点(纯黑)
 *
 */

#ifndef _OBJECTFINDER_COLORFIXER_
#define _OBJECTFINDER_COLORFIXER_

#include<opencv2/opencv.hpp>

namespace tinker{
namespace vision{

/* 填充图片内的无效点 */
cv::Mat FixColor(const cv::Mat & img, int range = 2);


}
}

#endif // _OBJECTFINDER_COLORFIXER_

