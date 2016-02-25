/* 
 * Created by 郭嘉丞 on 15/10/10.
 * Last edited by sugar10w, 2016.2.25
 *
 * 计算相邻区域的颜色灰度分布熵，过滤BGR图片
 *
 */

#ifndef __ENTROPY_FILTER_H__
#define __ENTROPY_FILTER_H__

#include "rgb_filter/interface_rgb_filter.h"

namespace tinker
{
namespace vision
{

class EntropyFilter : public IRgbFilter
{
public:
    /* filter_size取熵方格的大小，threshold阈值 */
    EntropyFilter(int filter_size, double threshold);
    ~EntropyFilter();    

    /* 将熵低于阈值的区域设为标记色flag_color */
    void Filter(cv::Mat & image_mat);
    /* 获取蒙版，熵低于阈值的区域为0 */
    cv::Mat GetMask(const cv::Mat & image_mat);
    /* 设置忽略区域 */
    void SetIgnoreMask(const cv::Mat & ignore_mask);
    /* 熵低于阈值时的标记色 */
    static const cv::Vec3b flag_color;

private:
    /* 在局部方格内统计熵值 */
    cv::Mat GetEntropyImage(const cv::Mat & image_mat);
    /* 在初始化时，计算entropy_table_表，用于熵的计算 */
    void BuildEntropyTable();
    /* log表，用于熵的计算 */
    double * entropy_table_;

    /* 取熵方格的大小 */
    int filter_size_;
    /* 阈值 */
    double threshold_;
};

}
}

#endif // __ENTROPY_FILTER_H__

