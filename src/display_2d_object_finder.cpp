/* TODO 重写
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 *
 * 二维画面，用矩形标出找到的物体
 *
 *
 */

/** @file Protonect.cpp Main application file. */


#include <sys/time.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pcl_rebuild/PointCloudBuilder.h"
#include "pcl_rebuild/ClusterDivider.h"
#include "pcl_rebuild/LineFilter.h"
#include "pcl_rebuild/EntropyFilter.h"
#include "pcl_rebuild/ColorChecker.h"
#include "pcl_rebuild/ImageRebuild.h"

using std::cout;
using std::endl;
using namespace tinker::vision;
using namespace libfreenect2;

int MAX_FRAMES = 30;
int DELAY_MS = 60000/MAX_FRAMES;

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

typedef struct {
  cv::Point point;
  cv::Scalar color;
} PointT;

//The following demostrates how to create a custom logger
// logger
#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
   {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
   {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
   {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};

void delay_ms(int time_ms)
{
  struct timeval t_start, t_now;
  int cnt, last_cnt;
  gettimeofday(&t_start, NULL);
  while (true)
  {
    gettimeofday(&t_now, NULL);
    cnt = ( t_now.tv_sec - t_start.tv_sec )*1000 + t_now.tv_usec/1000 - t_start.tv_usec/1000;
    if (cnt>time_ms) break;
  }
}

/*void write_depth_data(const cv::Mat & depth_matrix, std::string filename)
{
  //depth: 32FC1
  std::ofstream fout(filename.c_str(), std::ofstream::binary);
  fout.write((char *)&depth_matrix.rows, sizeof(int));
  fout.write((char *)&depth_matrix.cols, sizeof(int));
  fout.write((char *)depth_matrix.data, depth_matrix.cols * depth_matrix.rows * sizeof(float));
}
*/


int main(int argc, char *argv[])
{

  std::string program_path(argv[0]);
  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  // create a console logger with debug level (default is console logger with info level)
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  std::string serial = freenect2.getDefaultDeviceSerialNumber();

//  bool viewer_enabled = true;

    dev = freenect2.openDevice(serial);

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

// listeners
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

// start
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

// registration setup
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

//---------------------------------------------------------------
  std::vector<PointT> interestingPoint;
  srand((unsigned)time(NULL));
  int framecount=0; 

// loop start  
  
  while(!protonect_shutdown)
  {
    
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

// registration
    registration->apply(rgb, depth, &undistorted, &registered);
        
      framecount++;
	  std::cout<<"["<<framecount<<"]"<<std::endl;

     cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
     depthMatrix /= 4500.0f;
      cv::Mat rgbMatrix(rgb->height, rgb->width , CV_8UC4, rgb->data);    
      cv::Mat display(720, 1280, CV_8UC3);
      cv::resize(rgbMatrix, display, display.size());

//      gettimeofday(&t_now, NULL); float usedTime=t_now.tv_sec-t_start.tv_sec+0.000001f*(t_now.tv_usec-t_start.tv_usec);
//     if (Status==3) output_file<<usedTime<<std::endl;

//      std::string prefixDepth = "depth";
//      std::string prefixRegistered = "registered";
//      std::string prefixRgb = "rgb";
//      sprintf(buffer, "%d", framecount);

    cv::Mat registeredMatrix(registered.height, registered.width, CV_8UC4, registered.data);
//    cv::imwrite(prefixRegistered+buffer+".png", registeredMatrix);
//    cv::imwrite(prefixRgb+buffer+".png", rgbMatrix);
//    write_depth_data(depthMatrix, prefixDepth+buffer+".bin");


//      cv::imwrite("reg.png", registeredMatrix);      
//      cv::Mat mat = cv::imread("reg.png");

// loop end
//      if (framecount == MAX_FRAMES) protonect_shutdown=true;
      LineFilter line_filter;
      line_filter.Filter(depthMatrix, registeredMatrix);
      EntropyFilter entropy_filter(5, 0.4);
      entropy_filter.Filter(depthMatrix, registeredMatrix);
      
      PointCloudBuilder * builder = new PointCloudBuilder(depthMatrix, registeredMatrix);
      PointCloudPtr pointCloud = builder->GetPointCloud();
      IPointCloudDivider * divider = new ClusterDivider(pointCloud);
      std::vector<PointCloudPtr> divided_point_clouds = divider->GetDividedPointClouds();
      std::vector<cv::Rect> rects ; 
      
      cv::rectangle(display, cv::Rect(display.cols*kHighResLeft/kHighResWidth, 0, display.cols*kHighResActualWidth/kHighResWidth-1 , display.rows), cv::Scalar(0, 255, 0), 5);
      for (int i=0; i<(int)divided_point_clouds.size(); ++i)
      {
        cv::Rect rect = GetHDRectFromPointCloud(divided_point_clouds[i], rgbMatrix, true);
		if (rect.height > rect.width*5) continue;
        float k = (float)display.cols / (float)rgbMatrix.cols; 
        rect.x *= k; rect.y *=k; rect.width *=k; rect.height *=k;

        PointT pt; 
		pt.point = cv::Point(rect.x+rect.width/2, rect.y+rect.height/2);
		
		float minDis= 0.05 * 0.05 *  display.rows * display.rows;
		bool flag = false;
		for (int i=0; i<(int)interestingPoint.size(); ++i) 
	    { 
		    PointT pt2 = interestingPoint[i];
			float dis =(pt2.point.x-pt.point.x)*(pt2.point.x-pt.point.x) 
			      + (pt2.point.y-pt.point.y)*(pt2.point.y-pt.point.y);
			if (dis  < minDis)
		     {   
				 minDis = dis;
				 pt.color = pt2.color;
				 flag = true;
			 }
			
	    }
		if (!flag) pt.color = cv::Scalar(64+rand()%128, 64+rand()%128, 64+rand()%128);

        if ((int)interestingPoint.size()<500) interestingPoint.push_back(pt);
        cv::rectangle(display, rect, pt.color, 2);
      }
      
      for (int i=0; i<(int)interestingPoint.size(); ++i)
      {
	   PointT pt = interestingPoint[i];
       cv::circle(display, pt.point, 2, pt.color, 2);
      }
      

      std::cout<<">>>"<<std::endl;

      cv::imshow("Display", display);
      char c = cv::waitKey(100); 

      if (c==27 || c=='q') {  protonect_shutdown = true; break; }
      else if (c=='c') interestingPoint.clear(); 

      listener.release(frames);
   }

// stop
  dev->stop();
  dev->close();

  delete registration;

  return 0;
}
