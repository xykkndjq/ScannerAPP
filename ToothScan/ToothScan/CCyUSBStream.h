#ifndef CCYUSBSTREAM_H
#define CCYUSBSTREAM_H

#include <windows.h>
#include <stdio.h>
#include "CyAPI.h"
#include <dbt.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <opencv2\opencv.hpp>
#include <string>

namespace Communication
{
	#define MAX_QUEUE_SZ 64
	#define ImageRow 1024
	#define ImageCol 1280
	#define TN 10000
	#define SCALE 256
	#define QueueSize 8
	#define TimeOut 100
	#define ImageQueueSize 12

	using std::cout;
	using std::endl;
	using std::vector;
	using cv::Mat;
	using std::string;

	static uchar OpenDLP[4] = { 0XAA, 0XBB, 0X04, 0X01};//打开光机
	static uchar CloseDLP[4] = { 0XAA, 0XBB, 0X04, 0X04};//关闭光机
	static uchar RealMode[5] = { 0XAA, 0XBB, 0X05, 0X05, 0X00};//实时模式
	static uchar TrigerMode[5] = { 0XAA, 0XBB, 0X05, 0X05, 0X01};//触发模式
	static uchar OpenLight[5] = { 0XAA, 0XBB, 0X05, 0X06, 0X01 };//投白光

	//光机亮度
	static uchar MaxDLPLight[5] = { 0XAA, 0XBB, 0X05, 0X07, 0X00 };//最亮
	static uchar MidDLPLight[5] = { 0XAA, 0XBB, 0X05, 0X07, 0X01 };//中等
	static uchar MinDLPLight[5] = { 0XAA, 0XBB, 0X05, 0X07, 0X02 };//最弱
	static uchar ScanDLPLight[11] = { 0XAA, 0XBB, 0X0B, 0X07, 0X03, 0X50, 0X00, 0X60, 0X00, 0X5D, 0X00 };//最弱

	static uchar SMReset[4] = { 0XAA, 0XBB, 0X04, 0X03};
	static uchar SMRotOneDeg[9] = { 0XAA, 0XBB, 0X09, 0X02, 0X00, 0X00, 0X00, 0X00, 0X00 };

	class CCyUSBStream
	{
		public:
			 CCyUSBStream();
			 ~CCyUSBStream();
		
			 CCyUSBDevice *m_USBDevice;

			 int InitCyUSBParameter(void *handle);//初始化USB//return 0:正常的初始化成功，1：代表USBDevice初始化失败,2:代表没有USBDevice,3:代表有多于一个USBDevice
			 int OpenUSB(void *handle);//return 0:正常的初始化成功，1：代表USBDevice初始化失败,2:代表没有USBDevice,3:代表有多于一个USBDevice
			 int CloseUSB();//清空USBdevice和端口
			 void InitUSBBufferParameter();

			 bool OpenDLPFunction();//打开光机
			 bool ClosedDLPFunction();//关闭光机
			 bool ResetDLPFunction();//电机归位
			 bool TestDLPFunction();//电机
			 bool SetMaxDLPLight();//设置光机最亮
			 bool SetMidDLPLight();//设置光机中等
			 bool SetMinDLPLight();//设置光机最弱
			 bool SetScanDLPLight();//设置光机扫描亮度
									//实时显示
			 bool RealModeFuction();//实时获取数据
			 bool initRealTimeParameter();//实时初始化
			 bool realTimeImageStream(cv::Mat &v_img_L, cv::Mat &v_img_R);//获取左右相机各一张图像
			 void clearRealTimeStream();//清理实时数据流

										//trigger模式取一个角度数据
			 bool TriggerModeFunction();//改为trigger模式获取数据
			 bool SMRotOneDegFunction(double v_ddeg_x, double v_ddeg_y, bool v_bcali, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set);
			//控制电机旋转
			//v_ddeg_x：大电机旋转角度； v_bpositiveOrien_x：大电机是否正转（true为正转，false为反转）；
			//v_ddeg_y：小电机旋转角度； v_bpositiveOrien_y：小电机是否正转（true为正转，false为反转）； 
			//v_bcali：是否为标定的旋转；（true为标定旋转，false为扫描旋转）；
			void AbortXferLoop();//清理内存

		private:
			
			CCyUSBEndPoint *m_EndPtIn;
			CCyUSBEndPoint *m_EndPtOut;
			PUCHAR *m_buffers;
			CCyIsoPktInfo	**m_isoPktInfos;
			PUCHAR			*m_contexts;
			OVERLAPPED		m_inOvLap[MAX_QUEUE_SZ];
			
			long m_endptInLength;
			int m_validTotalBuffersSize;

			UCHAR *m_totalRealTimeBuffers;
			UCHAR *m_totalScanImageBuffers;
			UCHAR *m_totalCalibImageBuffers;

			bool m_startSave = false;
			//leftcamera
			vector<cv::Mat> m_img_L_set;
			
			int m_left_CAFlag = 0;
			bool m_left_start = false;
			bool m_left_start_row = false;
			bool m_left_end = false;
			Mat m_left_PerImage;
			Mat	m_left_image_raw;
			int m_left_im_u;
			int m_left_im_v;
			int m_left_bmpNum = 1;
			//rightcamera
			vector<cv::Mat> m_img_R_set;
			int m_right_CAFlag = 0;
			bool m_right_start = false;
			bool m_right_start_row = false;
			bool m_right_end = false;
			Mat m_right_PerImage;
			Mat	m_right_image_raw;
			int m_right_im_u;
			int m_right_im_v;
			int m_right_bmpNum = 1;

			void triggerScanImageStream();
			void triggerCalibImageStream();
			void AbortXferLoopIn(int pending);//清理内存

			void TakeValidRealTimeInfo(long length);
			void TakeValidCalibInfo(long length);
			void TakeValidScanInfo(long length);
			bool TakeInitFlag(byte *inBuf, long length, int &statPosition);
			void TakeValidImage(Mat &PerImage, vector<cv::Mat> &img_set);

			void TakeValidImage(Mat &PerImage, Mat &image_raw);
	};
}
#endif //CCYUSBSTREAM_H