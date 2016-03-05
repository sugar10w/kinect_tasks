/*
 * Created by 郭嘉丞 on 15/9/25.
 * Last edited by sugar10w, 2016.2.21
 *
 * 利用Kinect标定数据，将三维点云重新转换为二维图片。
 * 以及根据三维点云，在高像素(1920*1080)图片中找到对应的位置图片。
 *
 */ 

#include "kinect2pcl/image_rebuild.h"

#include <iostream>
#include "kinect2pcl/kinect_parameters.h"

namespace tinker {
namespace vision {    

using std::cerr;
using std::endl;
using std::cout;

struct PixelNumber
{
    int nx;
    int ny;
};

/* 标记LowRes(512*424)与HiRes(1920*1080)之间关系的一组常数
 * LowRes的上下边缘是无效点，HiRes的左右边缘是无效点 */
const int kLowResWidth = 512;
const int kLowResHeight = 424, kLowResTop = 19, kLowResBottom = 35;
const int kHiResWidth = 1920, kHiResLeft = 255, kHiResRight = 157;
const int kHiResHeight = 1080;
const int kLowResActualHeight = kLowResHeight - kLowResTop - kLowResBottom;
const int kHiResActualWidth  = kHiResWidth - kHiResLeft - kHiResRight; 

/* 利用Kinect标定矩阵，将三维点投影到二维平面 */
static PixelNumber GetPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4]);
/* lowRes到HiRes点的转化 */
static PixelNumber& lowResToHiRes(PixelNumber& point);
 /* 扩展这个矩形 */
static cv::Rect GetExtendedRect(cv::Rect rect, bool hiRes, float k = 2.0f);

/* 利用Kinect标定值，将点云中的每个点重新绘制到二维平面上 */
cv::Mat Get2DImageFromPointCloud(PointCloudPtr cloud)
{
    cv::Mat rebuiltImage = cv::Mat::zeros(kLowResHeight, kLowResWidth, CV_8UC3);
    
    /* 用于局部截取的尺寸 */
    int minPixelX = INT32_MAX, maxPixelX = -1;
    int minPixelY = INT32_MAX, maxPixelY = -1;

    for (unsigned i=0; i<cloud->points.size(); i++)
    {
        const pcl::PointXYZRGB & point = cloud->points[i];
        PixelNumber pxNumber = GetPixelNumber(point, projectionParameter);

        /* 舍去超出原视野边界的意外点 */
        if(pxNumber.nx >= kLowResWidth || pxNumber.nx < 0)
        { /* cerr << pxNumber.nx << " " << pxNumber.ny << endl; */ continue; }
        if(pxNumber.ny >= kLowResHeight || pxNumber.nx < 0)
        { /* cerr << pxNumber.nx << " " << pxNumber.ny << endl; */ continue; }

        /* 绘制点 */
        cv::Vec3b &pixel = rebuiltImage.at<cv::Vec3b>(pxNumber.ny, pxNumber.nx);
        pixel[0] = point.b;
        pixel[1] = point.g;
        pixel[2] = point.r;

        /* 刷新截取尺寸 */
        if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
        if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
        if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
        if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
    }

    /* 检查空图片 */
    if(maxPixelX < 0 || minPixelX < 0) return cv::Mat();

    /* 截取目标位置图片 */
    return cv::Mat(rebuiltImage,
                   cv::Range(minPixelY, maxPixelY+1),
                   cv::Range(minPixelX, maxPixelX+1));
}

/* 利用Kinect标定值，找到点云在原图片(lowRes或hiRes)中的位置*/
cv::Rect GetHDRectFromPointCloud(PointCloudPtr cloud, bool hiRes)
{ 
    int minPixelX = INT32_MAX, maxPixelX = -1;
    int minPixelY = INT32_MAX, maxPixelY = -1;
    for (unsigned i=0; i<cloud->points.size(); i++)
     {
        const pcl::PointXYZRGB & point = cloud->points[i];
        PixelNumber pxNumber = GetPixelNumber(point, projectionParameter);
        if (hiRes) lowResToHiRes(pxNumber);

        if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
        if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
        if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
        if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
    }

    int width  = (hiRes)? kHiResWidth  : kLowResWidth;
    int height = (hiRes)? kHiResHeight : kLowResHeight; 
    minPixelX = minPixelX < 0 ? 0 : minPixelX;
    maxPixelX = (maxPixelX >= width) ? width - 1 : maxPixelX;
    minPixelY = minPixelY < 0 ? 0 : minPixelY;
    maxPixelY = (maxPixelY >= height) ? height - 1 : maxPixelY;
    
     return  cv::Rect(minPixelX, minPixelY, 
                      maxPixelX-minPixelX+1,
                      maxPixelY-minPixelY+1 );
             
}

/* 利用Kinect标定值，将点云在图片(lowRes或hiRes)的位置切出来 */
cv::Mat GetHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat &totalImage, bool hiRes, bool extend)
{
    if (!extend)
         return cv::Mat(totalImage, GetHDRectFromPointCloud(cloud, hiRes));
    else
        return cv::Mat(totalImage, GetExtendedRect(GetHDRectFromPointCloud(cloud, hiRes),hiRes));
}

/* 利用Kinect标定矩阵，将三维点投影到二维平面 */
static PixelNumber GetPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4])
{
    double projectedPoint[3];   //2d point in the homogeneous coordinate
    for(int i=0; i<3; i++)
    {
        projectedPoint[i] = projectionMat[i][0] * point.x +
                            projectionMat[i][1] * point.y +
                            projectionMat[i][2] * point.z +
                            projectionMat[i][3];
    }
    PixelNumber pixelNumber;
    pixelNumber.nx = (int)(projectedPoint[0] / projectedPoint[2]);
    pixelNumber.ny = (int)(projectedPoint[1] / projectedPoint[2]);
    return pixelNumber;
}

/* lowRes到HiRes点的转化 */ 
static PixelNumber& lowResToHiRes(PixelNumber &point)
{
  point.nx = (point.nx * kHiResActualWidth / kLowResWidth ) + kHiResLeft;
  point.ny = (point.ny - kLowResTop ) * kHiResHeight / kLowResActualHeight;
  return point;
}

/* 扩展这个矩形 */
static cv::Rect GetExtendedRect(cv::Rect rect, bool hiRes, float k)
{
    float cx=rect.x+rect.width/2, cy=rect.y+rect.height/2;
    int l = cx-k/2*rect.width,
        r = cx+k/2*rect.width,
        t = cy-k/2*rect.height,
        b = cy+k/2*rect.height;
    int width = (hiRes)? kHiResWidth : kLowResWidth,
        height = (hiRes)? kHiResHeight : kLowResHeight;
    if (l<0) l=0;
    if (r>=width) r=width-1;
    if (t<0) t=0;
    if (b>=height) b=height-1;

    rect.x = l;
    rect.width = r-l+1;
    rect.y = t;
    rect.height = b-t+1;

    return rect;
}

}
}
