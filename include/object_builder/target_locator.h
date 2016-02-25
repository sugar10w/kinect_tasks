/*
 * Created by sugar10w, 2016.1.19
 * Last edited by sugar10w, 2016.2.25
 *
 * 根据旋转展台的形状，确定转轴和物体位置。
 * 利用物体的位置取出局部图，简化计算量。
 * 针对旋转展台的转速，处理局部点云的旋转。
 *
 * TODO Kinect2对5cm尺度物体，且同平面附近没有其他大型物体干扰时，局部深度信息存在2倍的畸变
 *
 */

#ifndef _OBJECTBUILDER_LOCATECENTER_
#define _OBJECTBUILDER_LOCATECENTER_

#include"common.h"

const float RAW_Z_ZOOM =  0.5f;  //TODO 尺寸大于10cm的物体，使用1;  宽度小于10cm的物体，使用0.5
const float RAW_RADIUS_Z = 4.0f; //TODO 旋转展台半径，需要Z_ZOOM修正
const float ROTATING_SPEED =  6.28 / 62.5; // 旋转展台顺时针旋转

const float CUT_WIDTH = 25.0f,   // 裁剪用水平直径
            CUT_HEIGHT = 30.0f,  // 裁剪用高度
            EXTRA_DEPTH = 1.0f;  // 裁剪用，超过旋转面的深度

void ResetZZoom(float zoom = 1.0f);

PointT LocateCenterPoint(PointCloudPtr cloud);

PointCloudPtr CutNearCenter(PointCloudPtr cloud, PointT center);

PointCloudPtr MoveToCenter(PointCloudPtr cloud, PointT center);

PointCloudPtr RotateAfterTime(PointCloudPtr cloud, float time);

#endif //_OBJECTBUILDER_LOCATECENTER_
