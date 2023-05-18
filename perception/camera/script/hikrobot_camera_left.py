# _*_coding:UTF-8_*_
import sys
sys.path.append("/opt/MVS/Samples/64/Python/MvImport")
from MvCameraControl_class import *
import cv2 as cv
import numpy as np
from CameraParams_const import *
from CameraParams_header import *
from MvCameraControl_header import *
from MvCameraControl_class import *
from MvErrorDefine_const import *
from PixelType_const import *
from PixelType_header import *

class AccessToImages():

    def FindDevices(self):
        device_list = MV_CC_DEVICE_INFO_LIST()
        find_device_ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, device_list)
        if find_device_ret == 0:
            return device_list
        else:
            return False

    def GetImages(self, show_device_num, device_list):
        i = 0
        if show_device_num <= device_list.nDeviceNum:
            device_info = cast(device_list.pDeviceInfo[int(show_device_num)], POINTER(MV_CC_DEVICE_INFO)).contents
            cam = MvCamera()
            handle_ret = cam.MV_CC_CreateHandle(device_info)
            if handle_ret != 0:
                print("创建句柄失败")
            # 打开设备
            open_device = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if open_device != 0:
                print('开启失败')

            # 获取相机int型节点值  "PayloadSize" 为当前节点得名称,一帧节点的大小
            camera_int_param = MVCC_INTVALUE()
            int_value_ret = cam.MV_CC_GetIntValue("PayloadSize", camera_int_param)
            if int_value_ret != 0:
                print('获取图像信息失败')
            image_data_size = (c_ubyte * camera_int_param.nCurValue)()
            image_data_length = camera_int_param.nCurValue
            # 开始抓流
            grab_ret = cam.MV_CC_StartGrabbing()
            if grab_ret != 0:
                print('抓起流失败')
            # 获取数据大小

            # 下面将获得每一帧的图像
            while True:
                frame_info = MV_FRAME_OUT_INFO_EX()
                memset(byref(frame_info), 0, sizeof(frame_info))

                frame_ret = cam.MV_CC_GetOneFrameTimeout(image_data_size, image_data_length, frame_info, 1000)
                if frame_ret != 0:
                    print('获取帧信息失败')

                nRGBSize = frame_info.nWidth * frame_info.nHeight
                convert_param = MV_CC_PIXEL_CONVERT_PARAM()
                memset(byref(convert_param), 0, sizeof(convert_param))
                convert_param.nWidth = frame_info.nWidth
                convert_param.nHeight = frame_info.nHeight
                convert_param.pSrcData = image_data_size
                convert_param.nSrcDataLen = frame_info.nFrameLen
                convert_param.enSrcPixelType = frame_info.enPixelType
                convert_param.enDstPixelType = frame_info.enPixelType
                convert_param.pDstBuffer = (c_ubyte * nRGBSize)()
                convert_param.nDstBufferSize = nRGBSize
                ret = cam.MV_CC_ConvertPixelType(convert_param)
                if ret != 0:
                    print("convert pixel fail! ret[0x%x]" % ret)
                    del image_data_size
                    sys.exit()
                img_buff = (c_ubyte * convert_param.nDstLen)()
                cdll.msvcrt.memcpy(byref(img_buff), convert_param.pDstBuffer, convert_param.nDstLen)
                img = np.array(img_buff, dtype=np.int8)
                Img = img.reshape(1024, 1280)
                Img = cv.flip(Img, -1)
                Img = cv.flip(Img, -1)
                cv.imshow('img' + str(show_device_num), Img)
                cv.waitKey(25)


if __name__ == '__main__':

    a = AccessToImages()
    device_list = a.FindDevices()
    if a:
        a.GetImages(0, device_list)
    # print(device_list.nDeviceNum)
