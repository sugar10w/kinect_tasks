/*
 * Created by yht on 04/10/15.
 * Last edited by sugar10w, 2016.2.25
 *
 * 检测BGR图片中的线段。
 *
 */


#include "rgb_filter/line_filter.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace tinker
{
namespace vision
{

    LineFilter::LineFilter(int downsample_ratio, float slope_range)
        : downsample_ratio_(downsample_ratio), slope_range_(slope_range)
    {
    }

    LineFilter::~LineFilter()
    {
    }

    /* 将边缘图像降采样。 打开removeContent则去除内部色块仅保留轮廓 */
    cv::Mat LineFilter::Downsample(const cv::Mat &img, bool remove_content)
    { 
      assert(img.type() == CV_8UC1);
      
      /* 降采样图像 */
      cv::Mat d_img = cv::Mat::zeros(img.rows/downsample_ratio_+1, img.cols/downsample_ratio_+1, CV_8UC1);   
      
      /* 降采样 */    
      for (int y=0; y<img.rows; ++y)
        for (int x=0; x<img.cols; ++x) 
          if (img.at<uchar>(y,x)) 
            d_img.at<uchar>(y/downsample_ratio_, x/downsample_ratio_)=255;
      
      if (!remove_content) return d_img;

      /* 再处理图像 */
      cv::Mat o_img = cv::Mat::zeros(img.rows/downsample_ratio_+1, img.cols/downsample_ratio_+1, CV_8UC1);

      /* 先去除内部零散的黑点,*/
      for (int y=1; y<d_img.rows-1; ++y)
          for (int x=1; x<d_img.cols-1; ++x)
              if (d_img.at<uchar>(y,x)==0)
              {
                  int c=0;
                  if ( d_img.at<uchar>(y-1,x)==255 ) ++c;
                  if ( d_img.at<uchar>(y+1,x)==255 ) ++c;
                  if ( d_img.at<uchar>(y,x-1)==255 ) ++c;
                  if ( d_img.at<uchar>(y,x+1)==255 ) ++c;
                  if (c >= 2) d_img.at<uchar>(y,x) = 1;
              }
      /* 再去除色块内部白色区域 */
      for (int y=1; y<d_img.rows-1; ++y)
          for (int x=1; x<d_img.cols-1; ++x)
              if (d_img.at<uchar>(y,x)==0) 
                  o_img.at<uchar>(y,x)=0;
              else
              {
                  if ( d_img.at<uchar>(y-1,x) && d_img.at<uchar>(y+1,x) && d_img.at<uchar>(y,x-1) && d_img.at<uchar>(y,x+1) ) 
                      o_img.at<uchar>(y,x)=0;
                  else 
                      o_img.at<uchar>(y,x)=255;
              }
      return o_img;

    }

    /* 暴力将水平线段和垂直线段加到线段数组中 */
    void LineFilter::BruteRemoveVerticals(const cv::Mat &img, std::vector<cv::Vec4i> &lines)
    {
      // TODO 优化这里的算法

      /* 可忍断点数 */
      const int max_cntNull = 1;
      /* 最低长度 */
      const int min_length = 0.1 * (img.rows>img.cols ? img.rows : img.cols);

      /* 垂直线段 */
      for (int x=0; x<img.cols; ++x)
      {
        int cntPoint = 0, cntNull = 0;
        for (int y=0; y<img.rows; ++y)
        {
          if (img.at<uchar>(y,x)!=0) ++cntPoint; else ++cntNull;
          if (cntNull > max_cntNull) { cntNull=0; cntPoint=0;  }
          if (cntPoint > min_length) 
          {
            while (y<img.rows && img.at<uchar>(y,x)!=0) { ++y; ++cntPoint;  }
            cv::Vec4i line;
            line[0] = line[2] = x;
            line[1] = y-cntPoint; line[3] = y;
            lines.push_back(line);
            cntPoint = 0; cntNull = 0;
          }
        }
      }

      /* 水平线段 */
      for (int y=0; y<img.rows; ++y)
      {
        int cntPoint =0, cntNull = 0;
        for (int x=0; x<img.cols; ++x)
        {
          if (img.at<uchar>(y,x)!=0) ++cntPoint; else ++cntNull;
          if (cntNull > max_cntNull) { cntNull=0; cntPoint = 0;}
          if (cntPoint > min_length)
          {
            while (x<img.cols && img.at<uchar>(y,x)!=0) { ++x; ++cntPoint;}
            cv::Vec4i line;
            line[0]=x-cntPoint; line[2]=x;
            line[1] = line[3] = y;
            lines.push_back(line);
            cntPoint = 0; cntNull = 0;
          }
        }
      }
    }

    /* 检查线段的斜率 */
    bool LineFilter::CheckSlope(const cv::Vec4i l)
    {
        if (slope_range_<0) return true;
        if (l[0]==l[2]) return true;
        if (l[1]==l[3]) return true;
        double dx = l[0]-l[2]; if (dx<0) dx=-dx;
        double dy = l[1]-l[3]; if (dy<0) dy=-dy;
        return dx/dy<slope_range_ || dy/dx<slope_range_;
    }
    
    /* 将线段处涂黑 */
    void LineFilter::Filter(cv::Mat & img)
    {
        cv::Mat mask = GetMask(img);
        img.setTo(0, ~mask);
    }

    /* 线段为黑色，线段以外的部分为白色 */
    cv::Mat LineFilter::GetMask(const cv::Mat & img)
    {
        cv::Mat cannied, downsampled, downsampled_mask;
        cv::Mat output = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

        /* Canny, 降采样 */
        Canny(img, cannied, 50, 200, 3);
        downsampled = Downsample(cannied, true);
        
        /* 遮盖忽略区域 */
        if (!ignore_mask_.empty())
        {
            downsampled_mask = Downsample(ignore_mask_, false);
            downsampled &= ~downsampled_mask;
        }

        //imshow("downsampled ", downsampled);

        /* Hough，暴力 取出线段 */
        lines_.clear();
        HoughLinesP(downsampled, lines_, 1, CV_PI/180, 10, (int)(downsampled.rows*0.2), 3);
        BruteRemoveVerticals(downsampled, lines_);

        /* 应用降采样得到的结果，在output上绘制 */
        int width = downsample_ratio_ * 4;
        for (size_t i = 0; i < lines_.size(); i++)
        {
            cv::Vec4i & l = lines_[i];
            if (slope_range_>0 && CheckSlope(l))
            {
                l[0]=l[0]*downsample_ratio_+downsample_ratio_/2; if (l[0]>=output.cols) l[0] = output.cols-1;
                l[1]=l[1]*downsample_ratio_+downsample_ratio_/2; if (l[1]>=output.rows) l[1] = output.rows-1;
                l[2]=l[2]*downsample_ratio_+downsample_ratio_/2; if (l[2]>=output.cols) l[2] = output.cols-1;
                l[3]=l[3]*downsample_ratio_+downsample_ratio_/2; if (l[3]>=output.rows) l[3] = output.rows-1;
                
                line(output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), width, 4);
            }
        } 
        return ~output;
    }

    /* 设置忽略区域 */
    void  LineFilter::SetIgnoreMask(const cv::Mat & ignore_mask)
    {
        assert(ignore_mask.empty() || ignore_mask.type() == CV_8UC1);
        ignore_mask_ = ignore_mask.clone();
    }
    
    /* 获取Lines坐标 */
    std::vector<cv::Vec4i> LineFilter::GetLines()
    {
        return lines_;
    }

}
}
