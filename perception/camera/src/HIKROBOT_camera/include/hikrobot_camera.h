#ifndef MYCAMERA_H
#define MYCAMERA_H
#include"MvCameraControl.h"
#include"iostream"
#include"opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
class Mycamera
{
  public:
      Mycamera();
      ~Mycamera();
  //声明相关变量及函数等
  static int EnumDevices(MV_CC_DEVICE_INFO_LIST* pstDevList);

  // ch:连接相机
  int connectCamera(string id);

  //设置是否为触发模式
  int setTriggerMode(unsigned int TriggerModeNum);

  //开启相机采集
  int startCamera();

  //发送软触发
  int softTrigger();

  //读取buffer
  int ReadBuffer(Mat &image);

  //设置心跳时间
  int setHeartBeatTime(unsigned int time);

  //设置曝光时间
  int setExposureTime(float ExposureTimeNum);
  private:
      void*               m_hDevHandle;

  public:
      unsigned char*  m_pBufForSaveImage;         // 用于保存图像的缓存
      unsigned int    m_nBufSizeForSaveImage;

      unsigned char*  m_pBufForDriver;            // 用于从驱动获取图像的缓存
      unsigned int    m_nBufSizeForDriver;
      };
#endif // MYCAMERA_H
