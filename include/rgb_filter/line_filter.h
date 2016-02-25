/* 
 * Created by yht on 04/10/15.
 * Last edited by sugar10w, 2016.2.25
 *
 * 检测BGR图片中的线段。
 *
 */

#ifndef __LINE_FILTER_H__
#define __LINE_FILTER_H__

#include "rgb_filter/interface_rgb_filter.h"

namespace tinker
{
namespace vision
{
    class LineFilter : public IRgbFilter
    {
    public:
        /* 设置降采样倍数；设置角度的正弦值限制（只有更接近水平或竖直的线段才会被保留；设为负数则不做限制） */
        LineFilter(int downsample_ratio = 4, float slope_range = -1);
        ~LineFilter();

        /* 将线段处涂黑 */
        void Filter(cv::Mat & image_mat);

        /* 线段为黑色，线段以外的部分为白色 */
        cv::Mat GetMask(const cv::Mat & image_mat);

        /* 设置忽略区域 */
        void SetIgnoreMask(const cv::Mat & ignore_mask);
    
    private:

        /* 将边缘图像降采样。 打开removeContent则去除内部色块仅保留轮廓 */
        cv::Mat Downsample(const cv::Mat &img, bool remove_content = true);

        /* 暴力将水平线段和垂直线段加到线段数组中 */
        void BruteRemoveVerticals(const cv::Mat &img, std::vector<cv::Vec4i> &lines);

        /* 检查线段的斜率 */
        bool CheckSlope(const cv::Vec4i line);

        /* 忽略区域内不会检查线段 */
        cv::Mat ignore_mask_;
        /* 降采样倍数 */
        int downsample_ratio_;
        /* 倾斜角的正切/余切限制；设为负数则不检查斜率 */
        float slope_range_;
    };
}
}

#endif

