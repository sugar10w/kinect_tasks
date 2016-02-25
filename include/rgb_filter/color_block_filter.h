/*
 * Created by sugar10w, 2016.1.29
 * Last edited by sugar10w, 2016.2.25
 *
 * 找到图片中不是背景的（纯）色块。
 *
 */

#ifndef __COLOR_BLOCK_FILTER_H_
#define __COLOR_BLOCK_FILTER_H_


#include<vector>
#include"rgb_filter/interface_rgb_filter.h"

namespace tinker
{
namespace vision
{
    class ColorBlockFilter : public IRgbFilter
    {
    public:
        ColorBlockFilter(int downsample_ratio = 3);
        ~ColorBlockFilter();

        /* 保留非背景的纯色块区域 */
        void Filter(cv::Mat & img);
        
        /* 返回非背景的纯色块位置 */
        cv::Mat GetMask(const cv::Mat & img);
        
        /* 设置忽略区域 */
        void SetIgnoreMask(const cv::Mat & ignore_mask);

        /* 根据原图和边缘，返回填充了纯色的图像（认为是背景的部分填黑色） */
        cv::Mat FillColor(const cv::Mat & img);
    
    private:

        /* 获得图像的降采样边缘，作为划分色块的依据 */
        cv::Mat GetOutline(const cv::Mat & img);

        /* 判断边缘是否合理；合理则返回此区域的代表性颜色，不合理则返回黑色(0,0,0) */
        cv::Scalar GetContourColor(const cv::Mat & img, const std::vector<std::vector<cv::Point> > & contours, const int idx);

        /* 降采样倍数 */
        const int downsample_ratio_;

    };

}
}


#endif // __COLOR_BLOCK_FILTER_H_
