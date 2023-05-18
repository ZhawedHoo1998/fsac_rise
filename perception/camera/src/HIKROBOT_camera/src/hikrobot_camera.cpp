#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "hikrobot_camera.h"

using namespace cv;

// 定义相机信息结构体
struct CameraInfo
{
    MV_CC_DEVICE_HANDLE hDevice;            // 相机句柄
    unsigned char* pData;                   // 图像数据指针
    MV_FRAME_OUT_INFO_EX stFrameInfo;       // 图像信息
    image_transport::Publisher publisher;  // 图像发布者
};

// 定义相机信息数组
constexpr int nCameraNum = 2;
CameraInfo cameraInfoArray[nCameraNum];

// 回调函数，获取相机图像数据并发布话题
void ImageCallback(unsigned int nIndex)
{
    CameraInfo& cameraInfo = cameraInfoArray[nIndex];

    // 获取图像数据
    int nRet = MV_CC_GetOneFrameTimeout(cameraInfo.hDevice, &cameraInfo.pData, &cameraInfo.stFrameInfo, 1000);
    if (nRet != MV_OK)
    {
        ROS_ERROR("MV_CC_GetOneFrameTimeout failed, ret = %d", nRet);
        return;
    }

    // 将图像数据转换为OpenCV格式
    Mat matImage(cameraInfo.stFrameInfo.nHeight, cameraInfo.stFrameInfo.nWidth, CV_8UC1, cameraInfo.pData);

    // 发布图像话题
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matImage).toImageMsg();
    cameraInfo.publisher.publish(msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mv_camera");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK)
    {
        ROS_ERROR("MV_CC_EnumDevices failed, ret = %d", nRet);
        return -1;
    }
    if (stDeviceList.nDeviceNum < nCameraNum)
    {
        ROS_ERROR("not enough devices found");
        return -1;
    }

    // 创建相机句柄并打开相机
    for (int i = 0; i < nCameraNum; i++)
    {
        CameraInfo& cameraInfo = cameraInfoArray[i];

        // 创建相机句柄
        nRet = MV_CC_CreateHandle(&cameraInfo.hDevice, stDeviceList.pDeviceInfo[i]);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_CreateHandle failed, ret = %d", nRet);
            return -1;
        }

        // 打开相机
        nRet = MV_CC_OpenDevice(cameraInfo.hDevice);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_OpenDevice failed, ret = %d", nRet);
            MV_CC_DestroyHandle(cameraInfo.hDevice);
            return -1;
        }

        // 设置触发模式为连续采集
        nRet = MV_CC_SetEnumValue(cameraInfo.hDevice, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_SetEnumValue failed, ret = %d", nRet);
            MV_CC_CloseDevice(cameraInfo.hDevice);
            MV_CC_DestroyHandle(cameraInfo.hDevice);
            return -1;
        }

        // 开始采集
        nRet = MV_CC_StartGrabbing(cameraInfo.hDevice);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_StartGrabbing failed, ret = %d", nRet);
            MV_CC_CloseDevice(cameraInfo.hDevice);
            MV_CC_DestroyHandle(cameraInfo.hDevice);
            return -1;
        }

        // 创建图像发布者
        cameraInfo.publisher = it.advertise("camera/image_" + std::to_string(i), 1);

        // 分别启动两个线程获取图像数据并发布话题
        std::thread thread(ImageCallback, i);
        thread.detach();
    }

    ros::spin();

    // 停止采集并关闭相机
    for (int i = 0; i < nCameraNum; i++)
    {
        CameraInfo& cameraInfo = cameraInfoArray[i];

        MV_CC_StopGrabbing(cameraInfo.hDevice);
        MV_CC_CloseDevice(cameraInfo.hDevice);
        MV_CC_DestroyHandle(cameraInfo.hDevice);
        delete[] cameraInfo.pData;
    }

    return 0;
}




#include "mycamera.h"

MV_CC_DEVICE_INFO_LIST m_stDevList;       // ch:设备信息列表结构体变量，用来存储设备列表
MV_CC_DEVICE_INFO* m_Device=NULL;         //设备对象

Mycamera::Mycamera()
{
    m_hDevHandle    = NULL;
}

Mycamera::~Mycamera()
{
    if (m_hDevHandle)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle    = NULL;
    }
}

//查询设备列表
int Mycamera::EnumDevices(MV_CC_DEVICE_INFO_LIST* pstDevList)
{
    int temp= MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, pstDevList);
    if (MV_OK != temp)
    {
        return -1;
    }
    return 0;
}

//连接相机
int  Mycamera::connectCamera(string id)
{
    int temp= EnumDevices(&m_stDevList);
    if(temp!=0)
        //设备更新成功接收命令的返回值为0，返回值不为0则为异常
        return -1;
    if(m_stDevList.nDeviceNum==0)
        //未找到任何相机
        return 2;
    for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
            continue;
        }
        if(id== (char*)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName||id== (char*)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber)
        {
            m_Device= m_stDevList.pDeviceInfo[i];
            break;
        }else
        {
            continue;
        }
    }
    if(m_Device==NULL)
    {
        //未找到指定名称的相机
        return 3;
    }
    temp  = MV_CC_CreateHandle(&m_hDevHandle, m_Device);
    if(temp  !=0)
        return -1;

    temp  = MV_CC_OpenDevice(m_hDevHandle);
    if (temp  !=0)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle = NULL;
        return -1;
    }else
    {
        setTriggerMode(1);
        return 0;
    }
    if (m_Device->nTLayerType == MV_GIGE_DEVICE)
    {
       //std::cout<<"Gige Camera"<<std::endl;
    }
}
//启动相机采集
int Mycamera::startCamera()
{
    int temp=MV_CC_StartGrabbing(m_hDevHandle);
    if(temp!=0)
    {
        return -1;
    }else
    {
        return 0;
    }
}
//发送软触发
int Mycamera::softTrigger()
{
    int tempValue= MV_CC_SetCommandValue(m_hDevHandle, "TriggerSoftware");
    if(tempValue!=0)
    {
        return -1;
    }else
    {
        return 0;
    }
}
//读取相机中的图像
int Mycamera::ReadBuffer(Mat &image)
{
    unsigned int nRecvBufSize = 0;
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    int temp = MV_CC_GetIntValue(m_hDevHandle, "PayloadSize", &stParam);
    if (temp != 0)
    {
        return -1;
    }
    nRecvBufSize = stParam.nCurValue;
    m_pBufForDriver = (unsigned char *)malloc(nRecvBufSize);
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    tempValue= MV_CC_GetOneFrameTimeout(m_hDevHandle, m_pBufForDriver, nRecvBufSize, &stImageInfo, 700);
    if(tempValue!=0)
    {
        return -1;
    }
    m_nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
    m_pBufForSaveImage = (unsigned char*)malloc(m_nBufSizeForSaveImage);

    bool isMono;//判断是否为黑白图像
    switch (stImageInfo.enPixelType)
    {
    case PixelType_Gvsp_Mono8:
    case PixelType_Gvsp_Mono10:
    case PixelType_Gvsp_Mono10_Packed:
    case PixelType_Gvsp_Mono12:
    case PixelType_Gvsp_Mono12_Packed:
        isMono=true;
        break;
    default:
        isMono=false;
        break;
    }

    if(isMono)
    {
        image=Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC1,m_pBufForDriver);
    }
    else
    {
        //转换图像格式为BGR8
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
        stConvertParam.nWidth = stImageInfo.nWidth;                 //ch:图像宽 | en:image width
        stConvertParam.nHeight = stImageInfo.nHeight;               //ch:图像高 | en:image height
        stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;         //ch:输入数据大小 | en:input data size
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format
        //stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format  适用于OPENCV的图像格式
        stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed; //ch:输出像素格式 | en:output pixel format
        stConvertParam.pDstBuffer = m_pBufForSaveImage;                    //ch:输出数据缓存 | en:output data buffer
        stConvertParam.nDstBufferSize = m_nBufSizeForSaveImage;            //ch:输出缓存大小 | en:output buffer size
        MV_CC_ConvertPixelType(m_hDevHandle, &stConvertParam);
        image=Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC3,m_pBufForSaveImage);
    }
    return 0;
}
//设置心跳时间
int Mycamera::setHeartBeatTime(unsigned int time)
{
    //心跳时间最小为500ms
    if(time<500)
        time=500;
    int temp=MV_CC_SetIntValue(m_hDevHandle, "GevHeartbeatTimeout", time);
    if(temp!=0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
//设置曝光时间
int Mycamera::setExposureTime(float ExposureTimeNum)
{
    int temp= MV_CC_SetFloatValue(m_hDevHandle, "ExposureTime",ExposureTimeNum );
    if(temp!=0)
        return -1;
    return 0;
}
