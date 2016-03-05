/*
 * Created by 郭嘉丞 on 15/9/12.
 * Last edited by sugar10w, 2016.2.24
 *
 * Kinect标定数据
 *
 * 三维点(X,Y,Z). 与其二维投影 (px,py)关系是
 * P * [X,Y,Z,1] = [x,y,k] = k*[px,py,1]
 *
 */

#ifndef KINECTDATAANALYZER_KINECTPARAMETERS_H
#define KINECTDATAANALYZER_KINECTPARAMETERS_H

#include <opencv2/opencv.hpp>

namespace tinker {
namespace vision {    

extern double depthToZ[2];  //from depth data(0~1) to Z in the camera coordinate(cm)
extern double projectionParameter[3][4];  //the projection matrix values

}
}

#endif //KINECTDATAANALYZER_KINECTPARAMETERS_H
