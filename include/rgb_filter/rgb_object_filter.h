/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.28
 *
 * 处理BGR图片，分离并获取物体、线段、背景等蒙版。
 *
 */

#ifndef __RGB_OBJECT_FILTER_H__
#define __RGB_OBJECT_FILTER_H__

#include <opencv2/opencv.hpp>

namespace tinker {
namespace vision {

class RgbObjectFilter
{
public:
    RgbObjectFilter(cv::Mat& raw_img);
    ~RgbObjectFilter();
    cv::Mat GetObjectMask();
    cv::Mat GetBackMask();
    cv::Mat GetLinesMask();
    std::vector<cv::Vec4i> GetLines();
private:
    cv::Mat raw_color_mask,
            raw_lines_mask,
            raw_entropy_mask;
    std::vector<cv::Vec4i> lines_;
    cv::Rect GetMarginRect(cv::Mat& img);
    void OpenImage(cv::Mat& img,
            int refill_kernel_size = 0,
            int erode_kernel_size = 2,
            int dilate_kernel_size = 2);
};

}
}

#endif // __RGB_OBJECT_FILTER_H__
