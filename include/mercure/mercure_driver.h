/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/

#pragma once

//#include <ros/ros.h>
//#include <ros/package.h>
#include<opencv2/opencv.hpp>
#include "DxImageProc.h"
#include "GxIAPI.h"
#include <chrono>

namespace camera{

#define ACQ_TRANSFER_SIZE       (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB  64//acq_transfer_number_usb

#define ACQ_FRAME_WIDTH   1280 // 1280 
#define ACQ_FRAME_HEIGHT  720  // 720

#define ACQ_BUFFER_NUM 3

struct Param
{
  int exp_auto_;
  int w_auto_;
  int gain_auto_;
  double exp_time_; // us
  double w_red_;
  double w_green_;
  double w_blue_;
  double gain_;
};

class MercureDriver
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;

  PGX_FRAME_BUFFER pFrameBuffer_;
  uint8_t* rgbImagebuf_;   
  Param param_;

public:
  explicit MercureDriver();
  GX_STATUS init_sdk();
  void GetVision();
  void operator >> (cv::Mat& Image); // API
  void LoadParam();
  ~MercureDriver();
};

} // namespace camera