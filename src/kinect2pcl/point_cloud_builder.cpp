/*
 * Created by 郭嘉丞 on 15/9/12.
 * Last edited by sugar10w, 2016.2.24
 *
 * 将逐点对应的
 *   depth cv::Mat, CV_32FC1 
 *   registered cv::Mat, CV_8UC3
 * 合成为
 *   PointCloud<PointXYZRGB>
 * 
 */

#include "kinect2pcl/point_cloud_builder.h"

#include <iostream>
#include "kinect2pcl/kinect_parameters.h"

using std::cerr;
using std::cout;
using std::endl;

// 输入数据
// depthMatrix, 512*424, CV_32FC1
// imageMatrix, 512*424, CV_8UC3
PointCloudBuilder::PointCloudBuilder(const cv::Mat &depthMatrix, const cv::Mat &imageMatrix)
        : depthMat(depthMatrix), imageMat(imageMatrix)
{
}

// 获取点云
PointCloudPtr PointCloudBuilder::getPointCloud()
{
	if(!pointCloud)
	{
		pointCloud = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
		buildPointCloud();
 	}
	return pointCloud;
}

// 构建点云
void PointCloudBuilder::buildPointCloud()
{
    // 深度信息预处理
    depthMat = depthMat * depthToZ[0] + depthToZ[1];
    for (int y = 0; y < imageMat.rows; y++)
    {
        for (int x = 0; x < imageMat.cols; x++)
        {
            double Z = depthMat.at<float>(y, x);

            // 过滤无效点
            if (Z < MIN_DEPTH_CM) continue;

            // 解出点的坐标
            cv::Mat solvedXY = getPointXY(x, y, Z, projectionParameter);
            if(solvedXY.rows == 0 || solvedXY.cols == 0) continue;
            double X = solvedXY.at<double>(0);
            double Y = solvedXY.at<double>(1);

            // 点云中加入新点
            pcl::PointXYZRGB newPoint;
            newPoint.b = imageMat.at<cv::Vec3b>(y, x)[0];
            newPoint.g = imageMat.at<cv::Vec3b>(y, x)[1];
            newPoint.r = imageMat.at<cv::Vec3b>(y, x)[2];
            newPoint.x = (float) X;
            newPoint.y = (float) Y;
            newPoint.z = (float) Z;
            pointCloud->points.push_back(newPoint);
        }
    }

    // 修正点云尺寸
    pointCloud->width = (int) pointCloud->points.size();
    pointCloud->height = 1;
    //cout << "Build success" << endl;
}

// 从 P*[?,?,Z,1]=k[px,py,1] 解出坐标 [X,Y]
cv::Mat PointCloudBuilder::getPointXY(int pixelx, int pixely, double depth, double p[3][4])
{
    // 构建并填充矩阵A
    double matrixAval[2][2] = {
            {p[0][0] - p[2][0] * pixelx, p[0][1] - p[2][1] * pixelx},
            {p[1][0] - p[2][0] * pixely, p[1][1] - p[2][1] * pixely}
     };
    cv::Mat matrixA(2, 2, CV_64FC1, &matrixAval);

    // 构建并填充矩阵b
    double matrixbval[2] = {
            p[2][2] * depth * pixelx + p[2][3] * pixelx - p[0][2] * depth - p[0][3],
            p[2][2] * depth * pixely + p[2][3] * pixely - p[1][2] * depth - p[1][3]
    } ;
    cv::Mat matrixb(2, 1, CV_64FC1, &matrixbval);

    // 解出方程 Ax=b
    cv::Mat solvedXY;
    if (!cv::solve(matrixA, matrixb, solvedXY))
    {
        cerr << "singular matrix A at x,y" << pixelx << " " << pixely << endl;
        return cv::Mat();
    } 
    return solvedXY;
}
