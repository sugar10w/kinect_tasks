/* 
 * Created by 郭嘉丞 on 15/10/10.
 * Last edited by sugar10w, 2016.2.25
 *
 * 计算相邻区域的颜色灰度分布熵，过滤BGR图片
 *
 */

#include "rgb_filter/entropy_filter.h"

#include <iostream>
#include <cmath>

namespace tinker
{
namespace vision
{
    using namespace std;
	
	/* 熵低于阈值时的标记色 */
    const cv::Vec3b EntropyFilter::flag_color(255, 0, 255);
	
	/* filter_size取熵方格的大小，threshold阈值 */
    EntropyFilter::EntropyFilter(int filter_size, double threshold)
        :entropy_table_(new double[filter_size*filter_size + 1]), 
        filter_size_(filter_size), threshold_(threshold)
    {
        BuildEntropyTable();
    }

    EntropyFilter::~EntropyFilter()
    {
        delete [] entropy_table_;
    }

	/* 在初始化时，计算entropy_table_表，用于熵的计算 */
    void EntropyFilter::BuildEntropyTable()
    {
        entropy_table_[0] = 0;
        double gray_levels = filter_size_ * filter_size_;
        for (int i = 1; i <= filter_size_ * filter_size_; i++)
         {
            entropy_table_[i] = log((double) i / gray_levels) / log(2);
        }
    }

	/* 将熵低于阈值的区域设为标记色flag_color */
    void EntropyFilter::Filter(cv::Mat & image_mat)
    {
        assert(image_mat.type()==CV_8UC3 || image_mat.type()==CV_8UC4);

        cv::Mat entropy_mat = GetEntropyImage(image_mat);

		if (image_mat.type()==CV_8UC3)
        {
            for (int i=0; i<entropy_mat.rows; ++i)
                for (int j=0; j<entropy_mat.cols; ++j)
                    if (entropy_mat.at<double>(i,j) < threshold_)
                        image_mat.at<cv::Vec3b>(i,j) = flag_color; 
        }
        else if (image_mat.type()==CV_8UC4)
        {
            for (int i=0; i<entropy_mat.rows; ++i)
                for (int j=0; j<entropy_mat.cols; ++j)
                    if (entropy_mat.at<double>(i,j) < threshold_)
                        image_mat.at<cv::Vec4b>(i,j) = cv::Vec4b(flag_color[0], flag_color[1], flag_color[2], 255);
        }

    }

	/* 获取蒙版，熵低于阈值的区域为0 */
    cv::Mat EntropyFilter::GetMask(const cv::Mat & image_mat)
    {
        cv::Mat output_mat = cv::Mat::zeros(image_mat.rows, image_mat.cols, CV_8UC1);
        cv::Mat entropy_mat = GetEntropyImage(image_mat);
        for (int i=0; i<entropy_mat.rows; ++i)
            for (int j=0; j<entropy_mat.cols; ++j)
                if (entropy_mat.at<double>(i,j) > threshold_)
                    output_mat.at<uchar>(i,j)=255;
        return output_mat;        
    }

	/* 在局部方格内统计熵值 */
    cv::Mat EntropyFilter::GetEntropyImage(const cv::Mat & image_mat)
    {
        cv::Mat entropy_image(image_mat.rows, image_mat.cols, CV_64FC1);
        cv::Mat gray_scale_image;
        cv::cvtColor(image_mat, gray_scale_image, CV_BGR2GRAY);
        int gray_levels = filter_size_ * filter_size_;
        int *gray_scale = new int[gray_levels];
        int mov_start = filter_size_ / 2;
        for (int i = 0; i < image_mat.rows; i++)
        {
            for (int j = 0; j < image_mat.cols; j++)
            {
                if (i - mov_start < 0 || j - mov_start < 0
                    || i - mov_start + filter_size_ >= image_mat.rows ||
                    j - mov_start + filter_size_ >= image_mat.cols)
                {
                    entropy_image.at<double>(i, j) = 0;
                }
                else
                {
                    int x = i - mov_start;
                    int y = j - mov_start;
                    for (int k = 0; k < gray_levels; k++)
                    {
                        gray_scale[k] = 0;
                    }
                    for (int movx = 0; movx < filter_size_; movx++)
                        for (int movy = 0; movy < filter_size_; movy++)
                        {
                            double gray = gray_scale_image.at<uchar>(movx + x, movy + y);
                            gray = gray * ((double) gray_levels) / 256;
                            gray_scale[int(gray)]++;
                        }
                    double entropy = 0;
                    for (int k = 0; k < gray_levels; k++)
                    {
                        double add_entropy = double(gray_scale[k]) * entropy_table_[gray_scale[k]] / double(gray_levels);
                        entropy += add_entropy;
                    }
                    entropy_image.at<double>(i, j) = entropy / entropy_table_[1];
                }
            }
        }
        delete [] gray_scale;
        return entropy_image;
    }

    /* 设置忽略区域 */
    void  EntropyFilter::SetIgnoreMask(const cv::Mat & ignore_mask)
    {
        assert(ignore_mask.type() == CV_8UC1);

        //TODO
    }


}
}

