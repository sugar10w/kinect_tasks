/*
 * Created by sugar10w, 2016.1
 * Last edited by sugar10w, 2016.2.25
 *
 * 用附近点的平均颜色填充图片内的无效点(纯黑纯白)
 *
 */

#include"rgb_filter/color_fixer.h"

namespace tinker{
namespace vision{

/* 判断无效点（纯黑或纯白） */
static bool isUselessPoint(cv::Vec3b point)
{
  return point[0]+point[1]+point[2]==0 
        || point[0]==255&&point[1]==255&&point[2]==255 ;
} 


/* 填充图片内的无效点 */
cv::Mat FixColor(const cv::Mat & img, int range)
{
  assert(img.type()==CV_8UC3);

  /* 选取周围(range*2+1)^2个有效点取BGR平均 */
  cv::Mat img2 = img.clone();
  for (int y=range; y<img.rows-range; ++y)
    for (int x=range; x<img.cols-range; ++x)
      if ( isUselessPoint(img.at<cv::Vec3b>(y,x)))
      {
         int b=0, g=0, r=0; int cnt=0;
         for (int i=y-range; i<=y+range; ++i)
           for (int j=x-range; j<=x+range; ++j)
            if ( ! isUselessPoint(img.at<cv::Vec3b>(i,j)))
            {
              b+=img.at<cv::Vec3b>(i,j)[0]; 
              g+=img.at<cv::Vec3b>(i,j)[1]; 
              r+=img.at<cv::Vec3b>(i,j)[2]; 
              ++cnt;
             }
         if (cnt>=range*range)
           img2.at<cv::Vec3b>(y,x) = cv::Vec3b( b/cnt, g/cnt, r/cnt);
         else
           img2.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0);
       }
  return img2;
}

}
}
