/*
 * Created by sugar10w, 2016.2.2
 * Last edited by sugar10w, 2015.2.25
 *
 * 过滤BGR图片、获取mask的接口
 *
 */


#ifndef __IRGB_FILTER_H_
#define __IRGB_FILTER_H_

#include<opencv2/opencv.hpp>

namespace tinker
{
namespace vision
{

class IRgbFilter
{
public:
    /* 保留图片中的目标部分 */
    virtual void Filter(cv::Mat & img) = 0;

    /* 获取蒙版，目标为白色，舍去部分为黑色 */
    virtual cv::Mat GetMask(const cv::Mat & img) = 0;

    /* 设置处理时忽略的区域 */
    virtual void SetIgnoreMask(const cv::Mat & ignore_mask) = 0;

    virtual ~IRgbFilter() { }
};

}
}

#endif // __IRGB_FILTER_H_
