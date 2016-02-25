/*
 * Created by sugar10w, 2016.1.29
 * Last edited by sugar10w, 2016.2.25
 *
 * 找到图片中不是背景的（纯）色块。
 *
 */

#include"rgb_filter/color_block_filter.h"

#include<iostream>
#include<vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace tinker
{
namespace vision
{

    ColorBlockFilter::ColorBlockFilter(int downsample_ratio)
        : downsample_ratio_(downsample_ratio)
    {
    }

    ColorBlockFilter::~ColorBlockFilter()
    {
    }

    /* 返回蒙版,指示非背景的纯色块位置 */
    cv::Mat ColorBlockFilter::GetMask(const cv::Mat & img)
    {
        /* 获得降采样后的边缘 */
        cv::Mat contour = ~ GetOutline(img);
		//cv::imshow("contour", contour);

        cv::Mat output = cv::Mat::zeros(img.size(), CV_8UC1);
        cv::Mat filled = cv::Mat::zeros(contour.size(), CV_8UC3); // 降采样尺寸图片,准备填色 
      
        cv::Mat resized; // 降采样后的图形,用于采集和计算颜色 
        cv::resize(img, resized, contour.size());

        /* 提取降采样边缘 cv::findContours */
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours( contour, contours, CV_RETR_LIST , CV_CHAIN_APPROX_NONE);
        
        /* 准备记录每一个区域的颜色和大小 */
        cv::Scalar * colors = new cv::Scalar[(int)contours.size()];
        double * cont_size = new double[(int)contours.size()];
        int maxColorIndex = -1; double maxSize = -1;

        /* 逐个计算色块颜色和面积,找到面积最大的色块,*/
        for(int i = 0 ; i < (int)contours.size() ; ++i )
        {
            cont_size[i] = cv::contourArea(contours[i]);
            colors[i] = GetContourColor(resized, contours, i);
            if ( (colors[i][0] || colors[i][1] || colors[i][2] ) 
					&& cont_size[i]>maxSize )
            { 
                maxSize = cont_size[i];
                maxColorIndex = i; 
            }
        }

        /* 最大面积过小则不处理,否则将
         * 与面积最大色块颜色相近的色块,面积大于最大面积1/5的色块,
         * 都标记为背景色块(0,0,0) */
		//std::cout<<maxColorIndex<<" "<<maxSize
		//	<<" color"<<colors[maxColorIndex]<<std::endl;
        if (maxSize > 2.3392e-3 * filled.rows * filled.cols /* 约50 */ ) 
            for (int i=0; i<(int)contours.size(); ++i)
            {
                if (colors[i][0] || colors[i][1] || colors[i][2])
                {
                    if (cont_size[i]*5 > maxSize /* 检查面积 */  ||
						( colors[i][0] - colors[maxColorIndex][0] ) *
					   	( colors[i][0] - colors[maxColorIndex][0] ) +
                        ( colors[i][1] - colors[maxColorIndex][1] ) *
					   	( colors[i][1] - colors[maxColorIndex][1] ) +
                        ( colors[i][2] - colors[maxColorIndex][2] ) *
					   	( colors[i][2] - colors[maxColorIndex][2] ) < 1170  /* 检查颜色 */
						)
                        colors[i]=cv::Scalar(0,0,0);
                }
            }

        /* 在降采样尺度的图片filled上填色 */
        for (int i=0; i<(int)contours.size(); ++i)
        {            
            if (colors[i][0] || colors[i][1] || colors[i][2])
              cv::drawContours(filled, contours, i, colors[i], CV_FILLED, 4);
        }

        delete[] colors;
        delete[] cont_size;

        //cv::imshow("raw filled", filled);

        /* 对filled的腐蚀膨胀 */
        
        //int erode_kernel_size = 1;
        //cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        //    cv::Size(2 * erode_kernel_size + 1, 2 * erode_kernel_size + 1),
        //    cv::Point(erode_kernel_size, erode_kernel_size));
        //cv::erode(filled, filled, erode_kernel);
        //cv::imshow("1 erode ", filled);

        int dilate_kernel_size = 1;
        cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1),
            cv::Point(dilate_kernel_size, dilate_kernel_size));
        cv::dilate(filled, filled, dilate_kernel);
        //cv::imshow("2 dilate ", filled);

        /* 将降采样的蒙版结果升像素到output上 */
        for (int i=0; i<output.rows; ++i)
            for (int j=0; j<output.cols; ++j)
            {
                cv::Vec3b point = filled.at<cv::Vec3b>
					(i/downsample_ratio_,j/downsample_ratio_);
                if (point[0] || point[1] || point[2] )
                    output.at<uchar>(i,j)=255;
            }

        return output;
    }

    /* 保留非背景的纯色块区域 */
    void ColorBlockFilter::Filter(cv::Mat & img)
    {
        cv::Mat mask = GetMask(img);
        img.setTo(0, ~mask);
    }

    /* 获得图像的降采样边缘，作为划分色块的依据 */
    cv::Mat ColorBlockFilter::GetOutline(const cv::Mat & img)
    {
        cv::Mat canny_img;
        cv::Mat down_img = cv::Mat::zeros(img.rows/downsample_ratio_+1, img.cols/downsample_ratio_+1, CV_8UC1);
        
        /* 先Canny, */
        cv::Canny(img, canny_img, 50, 200, 3);

        /* 再降采样 */
        for (int i=0; i<canny_img.rows; ++i)
            for (int j=0; j<canny_img.cols; ++j)
                if (canny_img.at<uchar>(i, j)) 
                {
                    down_img.at<uchar>(i/downsample_ratio_, j/downsample_ratio_)=255;

                    /* 为了配合findContours函数，需要保证**对角格**封闭 */ 
                    if (j/downsample_ratio_+1 < canny_img.cols/downsample_ratio_) 
                        down_img.at<uchar>(i/downsample_ratio_, j/downsample_ratio_+1)=255;
                }
        
        return down_img;
    }

    /* 根据原图和边缘，返回填充了纯色的图像（认为是背景的部分填黑色） */
    cv::Mat ColorBlockFilter::FillColor(const cv::Mat & img)
    {
        assert( img.type() == CV_8UC3 );
        //assert( outline.type() == CV_8UC1 );

        cv::Mat 
            filled, 
            contour = ~ GetOutline(img); /* 将从Canny的结果反色（findContours将提取白色部分的边缘） */

        /* findContours提取边缘 */
        std::vector<std::vector<cv::Point> > contours;
        findContours( contour, contours, CV_RETR_LIST , CV_CHAIN_APPROX_NONE);

        cv::resize(img, filled, contour.size());

        
        
        /* 逐个计算色块颜色，舍去面积最大的色块，并删除与面积最大色块颜色相近的色块 */
        cv::Scalar * colors = new cv::Scalar[(int)contours.size()];
        int maxColorIndex = -1; 
        double maxSize = -1;
        for(int i = 0 ; i < (int)contours.size() ; ++i )
        {
            double cont_size = cv::contourArea(contours[i]);
            colors[i] = GetContourColor(filled, contours, i);
            if ( (colors[i][0] || colors[i][1] || colors[i][2] ) && cont_size>maxSize )
            { 
                maxSize = cont_size;
                maxColorIndex = i; 
            }
        }


//
// maxColorIndex = -1;
//
        for (int i=0; i<(int)contours.size(); ++i)
        {
            if (colors[i][0] || colors[i][1] || colors[i][2])
            {
                if ( // maxColorIndex != -1 &&
                    ( colors[i][0] - colors[maxColorIndex][0] ) *
				   	( colors[i][0] - colors[maxColorIndex][0] ) +
                    ( colors[i][1] - colors[maxColorIndex][1] ) *
				   	( colors[i][1] - colors[maxColorIndex][1] ) +
                    ( colors[i][2] - colors[maxColorIndex][2] ) *
				   	( colors[i][2] - colors[maxColorIndex][2] ) < 1170
                    )
                    colors[i]=cv::Scalar(0,0,0);
            }
        }
        
        /* 开始填色 */
        for (int i=0; i<(int)contours.size(); ++i)
        {
            cv::drawContours(filled, contours, i, colors[i], CV_FILLED, 4);
        }

        delete colors;

        return filled;
    }

    /* 判断边缘所决定的区域是否合理；合理则返回此区域的代表性颜色，不合理则返回黑色(0,0,0) */
    cv::Scalar ColorBlockFilter::GetContourColor(const cv::Mat & img, const std::vector<std::vector<cv::Point> > & contours, const int idx)
    {
        const std::vector<cv::Point> & contour = contours[idx];
        const cv::Scalar invalid_color(0, 0, 0);

        /* 检查区域的大小 */
        double full_size = img.rows * img.cols;
        double cont_size = cv::contourArea(contour);
        //if (/*cont_size < full_size*2.3392e-4 /*约5*//* || */ cont_size > full_size*9.3568e-3 /*约200*/ ) return invalid_color;

        /* 获取区域的位置和长宽: */
        cv::Point point = contour[0];
        int xmin, xmax, ymin, ymax, width, height;
        xmin = xmax = point.x; ymin = ymax = point.y;
        for (int i=0; i<(int)contour.size(); ++i)
        {
            point = contour[i];
            xmin = xmin<point.x ? xmin : point.x;
            xmax = xmax>point.x ? xmax : point.x;
            ymin = ymin<point.y ? ymin : point.y;
            ymax = ymax>point.y ? ymax : point.y;
        }
        width = xmax - xmin + 1;
        height = ymax - ymin + 1;
        /* 去除边缘的色块(因为无法保证面积的准确性,也不能保证物体图像的完整) */
        if (xmin<=1 || ymin<=1 || xmax>=img.cols-2 || ymax>=img.rows-2) return invalid_color;
        /* 去除长宽比太大的色块(书架边框) */
        if (height*3.5<width || width*3.5<height) return invalid_color;
        /* 去除过分稀疏的色块(书架边框) */
        if (cont_size*10<height*width) return invalid_color;

        /* 计算区域的BGR平均颜色;
         * 把边缘的内部画在图片上，然后按照图片判断内部 */
        int b=0, g=0, r=0, cnt=0;
        cv::Mat inside = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
        cv::drawContours(inside, contours, idx, cv::Scalar(255,255,255), CV_FILLED, 4);
        for (int i=0; i<inside.rows; ++i)
            for (int j=0; j<inside.cols; ++j)
                if (inside.at<cv::Vec3b>(i,j)[0])
                {
                    cv::Vec3b pt = img.at<cv::Vec3b>(i,j);
                    ++cnt;
                    b+=pt[0];
                    g+=pt[1];
                    r+=pt[2];
                }
        b/=cnt; g/=cnt; r/=cnt;

        /* 此区域的信息 */
/*        std::cout<<idx<<std::endl;
        std::cout<<width<<"*"<<height<<"; size="<<cont_size<<"; %="<<cont_size/width/height<<std::endl;
        std::cout<<xmin<<"~"<<xmax<<" "<<ymin<<"~"<<ymax<<std::endl;
        std::cout<<"B "<<b<<", G "<<g<<", R "<<r<<std::endl<<std::endl;
*/

        if (b||g||r) return cv::Scalar(b, g, r);
        else return cv::Scalar(1, 0, 0);  //避免(0,0,0)误判
    }

    /* 设置忽略区域 */
    void  ColorBlockFilter::SetIgnoreMask(const cv::Mat & ignore_mask)
    {
        assert(ignore_mask.type() == CV_8UC1);

        //TODO
    }

}
}
