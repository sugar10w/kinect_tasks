/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 * Copyright (c) 2011 individual OpenKinect contributors.
 * 
 * Last edited by sugar10w, 2016.2.25
 *
 * 摆拍物体的图片。
 * 拍摄中途不要移动Kinect2或者遮挡物体。
 *
 * -1号照片：拍摄不变的背景，尽量选择比较空的桌面和书架。
 * 0号照片：放置旋转展台后的场景。-1号和0号将确定目标和旋转轴的位置。
 * >=1号照片：正在旋转中的物体，拍摄持续时间为1分钟。
 *
 * TODO 整理代码
 *
 */

#include <sys/time.h>
#include <signal.h>
#include <iostream>
#include <cstdio>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;

int MAX_FRAMES = 30;
int DELAY_MS = 60000/MAX_FRAMES;

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

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
    if (cnt!=last_cnt && cnt%200==0) { std::cout<<"."; last_cnt = cnt; }
    if (cnt>time_ms) break;
  }
}

void write_depth_data(const cv::Mat & depth_matrix, std::string filename)
{
  //depth: 32FC1
  std::ofstream fout(filename.c_str(), std::ofstream::binary);
  fout.write((char *)&depth_matrix.rows, sizeof(int));
  fout.write((char *)&depth_matrix.cols, sizeof(int));
  fout.write((char *)depth_matrix.data, depth_matrix.cols * depth_matrix.rows * sizeof(float));
}

int getNumber(char* s)
{
  int t=0;  char *p=s;
  while ('0'<=*p && *p<='9') 
  {
    t=t*10+*p-'0';
    ++p;
  }
  return t;
}

int main(int argc, char *argv[])
{
  if (argc==2)
  {
    int t = getNumber(argv[1]);
    if (t>3&&t<120)
    {
      MAX_FRAMES = t;
      DELAY_MS = 60000/t;
    }
  }

  std::string program_path(argv[0]);
  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  // create a console logger with debug level (default is console logger with info level)
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));

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

  bool viewer_enabled = true;

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
  
  std::ofstream output_file("kinect_info.txt");
  output_file<<MAX_FRAMES<<std::endl;

  int Status = 0;

  int framecount = -2;
  viewer_enabled = false;

  int count = 0;
  char buffer[20];

  struct timeval t_start, t_now;
  gettimeofday(&t_start, NULL);

// loop start
  while(!protonect_shutdown)
  {

    if (Status==0)
    {
      std::cout<<"\n\n\nPlease Clear the Stage. Press [space][Enter] to continue.\n\n\n"<<std::endl;
      char c; while ((c=getchar())!=' ') ; while ((c=getchar())!='\n');
    }
    else if (Status==1)
    {
      std::cout<<"\n\n\nPlease Set the Rolling Plane. Press [space][Enter] to continue.\n\n\n"<<std::endl;
      char c; while ((c=getchar())!=' ') ;  while ((c=getchar())!='\n');
    }
    else if (Status==2)
    {
      std::cout<<"\n\n\nPlease Put on the target. Press [space][Enter] to START SCANNING.\n\n\n"<<std::endl;
      char c; while ((c=getchar())!=' ') ;  while ((c=getchar())!='\n');
      Status = 3;
    }
 
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

// registration
    registration->apply(rgb, depth, &undistorted, &registered);
        
      framecount++;
      std::cout << "-------------------[" << framecount << "]---------------" << std::endl;

      cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
      depthMatrix /= 4500.0f;
      cv::Mat rgbMatrix(rgb->height, rgb->width , CV_8UC4, rgb->data);   

      gettimeofday(&t_now, NULL); float usedTime=t_now.tv_sec-t_start.tv_sec+0.000001f*(t_now.tv_usec-t_start.tv_usec);
      if (Status==3) output_file<<usedTime<<std::endl;

      std::string prefixDepth = "depth";
      std::string prefixRegistered = "registered";
      std::string prefixRgb = "rgb";
      sprintf(buffer, "%d", framecount);

    cv::Mat registeredMatrix(registered.height, registered.width, CV_8UC4, registered.data);
    cv::imwrite(prefixRegistered+buffer+".png", registeredMatrix);
    cv::imwrite(prefixRgb+buffer+".png", rgbMatrix);
    write_depth_data(depthMatrix, prefixDepth+buffer+".bin");

// loop end
      if (framecount == MAX_FRAMES) protonect_shutdown=true;

      listener.release(frames);
      
      switch (Status)
      {
       case 0: Status = 1; break;
       case 1: Status = 2; break;
       case 2: Status = 3; break;
       case 3: delay_ms(DELAY_MS); break;
      }

  }

// stop
  dev->stop();
  dev->close();

  delete registration;

  return 0;
}
