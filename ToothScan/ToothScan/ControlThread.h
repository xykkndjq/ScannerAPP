#ifndef ControlThread_H
#define ControlThread_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QSemaphore>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <iostream>

//#include "VTK_render.h"
#include "./include/3DScan.h"
//#include "3DScan_cuda.cuh"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <opencv2\opencv.hpp>

#define IMG_ROW    1024
#define IMG_COL   1280

#define EXPOSURETIME 16666

#define CALI_ROTATE_POS_CNT 12    //定义标定过程的位置数 13
#define SCAN_ROTATE_POS_CNT 12    //定义扫描过程的位置数 11

#define CALI_ROTATE_POS_CNT2 11    //定义标定过程的位置数 13
#define SCAN_ROTATE_POS_CNT2 10    //定义扫描过程的位置数 11    

extern int DataSize;
//extern int BufferSize;
extern unsigned char *totalNormalScanImageBuffer;
extern scan::RasterScan *rs;

extern cv::Mat rt_r;
extern std::vector<cv::Mat> scanner_rt;

extern float c_scan_x;
extern float c_scan_y;

extern QSemaphore freeSpace;
extern QSemaphore usedSpace;

static byte OpenDLP[5] = { 0xaa,0xbb,0x05,0x01,0x00 }; //开启及配置DLP
static byte CloseDLP[5] = { 0xaa,0xbb,0x05,0x01,0x01 }; //关闭DLP
static byte ProjectOneSet_CALI_MAX[11] = { 0xaa,0xbb,0x0b,0x01,0x02,  0xdf,0x02, 0xa2,0x03, 0x84,0x03 }; //标定
static byte ProjectOneSet_CALI_MID[11] = { 0xaa,0xbb,0x0b,0x01,0x02,  0xf5,0x00, 0x36,0x01, 0x2c,0x01 }; //标定
static byte ProjectOneSet_CALI_MIN[11] = { 0xaa,0xbb,0x0b,0x01,0x02,  0x83,0x00, 0x9e,0x00, 0x99,0x00 }; //标定
static byte ProjectOneSet_SCAN_MAX[11] = { 0xaa,0xbb,0x0b,0x01,0x03,  0xdf,0x02, 0xa2,0x03, 0x84,0x03 }; //扫描
static byte ProjectOneSet_SCAN_MID[11] = { 0xaa,0xbb,0x0b,0x01,0x03,  0xf5,0x00, 0x36,0x01, 0x2c,0x01 }; //扫描
static byte ProjectOneSet_SCAN_MIN[11] = { 0xaa,0xbb,0x0b,0x01,0x03,  0x83,0x00, 0x9e,0x00, 0x99,0x00 }; //扫描


static byte WhiteDisplay[5] = { 0xaa,0xbb,0x05,0x01,0x05 }; //显示白屏
//小电机
static byte SMYEnable[5] = { 0xaa,0xbb,0x05,0x02,0x00 }; //使能
static byte SMYDisable[5] = { 0xaa,0xbb,0x05,0x02,0x01 }; //失能
static byte SMYRotOneDegPos[9] = { 0xaa,0xbb,0x09,0x02,0x02,0x00,0x00,0x00,0x00 }; //步进电机正转一个固定角度 deg = SMXRotOneDeg[5] * 256 + SMXRotOneDeg[6]
static byte SMYRotOneDegNeg[9] = { 0xaa,0xbb,0x09,0x02,0x02,0x01,0x00,0x00,0x00 }; //步进电机反转一个固定角度 deg = SMXRotOneDeg[5] * 256 + SMXRotOneDeg[6]
static byte SMYRotOneCirclePos[6] = { 0xaa,0xbb,0x06,0x02,0x03,0x00 }; //步进电机正转一圈
static byte SMYRotOneCircleNeg[6] = { 0xaa,0xbb,0x06,0x02,0x03,0x01 }; //步进电机反转一圈
//大电机
static byte SMXEnable[5] = { 0xaa,0xbb,0x05,0x03,0x00 }; //使能
static byte SMXDisable[5] = { 0xaa,0xbb,0x05,0x03,0x01 }; //失能
static byte SMXRotOneDegPos[9] = { 0xaa,0xbb,0x09,0x03,0x02,0x00,0x00,0x00,0x00 }; //步进电机正转一个固定角度 deg = SMXRotOneDeg[5] * 256 + SMXRotOneDeg[6]
static byte SMXRotOneDegNeg[9] = { 0xaa,0xbb,0x09,0x03,0x02,0x01,0x00,0x00,0x00 }; //步进电机反转一个固定角度 deg = SMXRotOneDeg[5] * 256 + SMXRotOneDeg[6]
static byte SMXRotOneCirclePos[6] = { 0xaa,0xbb,0x06,0x03,0x03,0x00 }; //步进电机正转一圈
static byte SMXRotOneCircleNeg[6] = { 0xaa,0xbb,0x06,0x03,0x03,0x01 }; //步进电机反转一圈

//static int SMX_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 45, -90, 45, 45, -90, 45, 45, -90, 45, 45, 135, 135, 45 };
//static int SMY_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 0,  0,  45,  0,  0,  45,  0,  0,  -60, 0,  0,  0,  -30 };
//static int SM_CALI_BRIGHTNESS[CALI_ROTATE_POS_CNT] = { 3,  3,  1,  1,  1,  2,  2,  2,  2,  2,  3,  2,  3 };//亮度等级，3最亮，1为默认
//
//static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 90, -45, -45, -45, -45, -45, -45, -135, 0,   0,  -45 };
//static int SMY_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 30, 0,   0,   0,   0,   0,   0,   0,    -30, 80, -80 };
//static int SM_SCAN_BRIGHTNESS[SCAN_ROTATE_POS_CNT] = { 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2 };//亮度等级，3最亮，1为默认

static int SMY_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 30, -60, -150, -45, -90, 0,-45, -45, 0, 45, 45, -45 };
static int SMX_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 0,  0,  0,  0,  0,  -30,  0,  0,  60,  0,  0, -30 };
static int SM_CALI_BRIGHTNESS[CALI_ROTATE_POS_CNT] = { 2,  2,  2,  2,  3,  3,  3,  2,  2,  2,  2,  2 };//亮度等级，3最亮，1为默认

																									   //static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
static int SMY_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 0, 45, 45, 45, 45, 45, 45, 45, 115, -70,  -70,  70 };
static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { -45, 0,   0,   0,   0,   0,   0,   0,    -35, 0,  0, 80 };
static int SM_SCAN_BRIGHTNESS[SCAN_ROTATE_POS_CNT] = { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 1 };//亮度等级，3最亮，1为默认

static int SMY_CALI_ROTATE_DEGREE2[CALI_ROTATE_POS_CNT2] = { 0,   0,  -30,  30,-25,   0,  25, -30,   0,  30,    0 };
static int SMX_CALI_ROTATE_DEGREE2[CALI_ROTATE_POS_CNT2] = { 0, 30,  30, 30, 20,  20,  20,  10,  10,  10,    0 };
static int SM_CALI_BRIGHTNESS2[CALI_ROTATE_POS_CNT2] = { 2,   2,    2,   2,  2,   2,   2,   2,   2,   2,    0 };//亮度等级，3最亮，1为默认

																												//static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
static int SMY_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,/*-60,   0,  60, -50,   0,  50,*/   0, -45, -90, -135,  45,  90, 135,175, 0 };
static int SMX_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,/* 30,  30,  30,  20,  20,  20,*/ -40, -40, -40,  -40, -40, -40, -40,-40, 0 };

static int SM_SCAN_BRIGHTNESS2[SCAN_ROTATE_POS_CNT2] = { 2,/*  2,   2,   2,   2,   2,   2,*/   3,   3,   3,    3,   3,   3,   3,  3, 2 }; //亮度等级，3最亮，1为默认
////static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
//static int SMX_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,-60,   0,  60, -50,   0,  50,   0, -40,  -80, -140,    40,   80,  140, 0 };
//static int SMY_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0, 30,  30,  30,  20,  20,  20, -30, -30,  -30,  -30,  -30,  -30,  -30,  0 };
//static int SM_SCAN_BRIGHTNESS2[SCAN_ROTATE_POS_CNT2] = { 2,  2,   2,   2,   2,   2,   2,   3,   3,    3,    3,    3,   3,    3,  2 };//亮度等级，3最亮，1为默认


using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

enum triggerType
{
	SOFTWARE,
	HARDWARE
};

//const triggerType chosenTrigger = SOFTWARE;
const triggerType chosenTrigger = HARDWARE;

class ControlThread : public QObject
{
	Q_OBJECT

public:
	ControlThread(QObject *parent=0);
	~ControlThread();

	int point_cloud_index;
	bool serial_port_open;
	vector<bool> point_cloud_right;
	QMutex* mutex;

	QSerialPort *serial;

	vector<vector<double>> points_cloud_globle;

	float l_scan_x;
	float l_scan_y;
	float l_scan_z;
	
	float c_scan_z;
	//float c_scan_x;
	//float c_scan_y;

	void setFlage(bool flag = true);  //设置标志位，何时关闭子线程

	bool  isStop;

public slots:
	void controlNormalScan();
	void controlCalibrationScan();
	void controlGlobalCaliScan();
	void compensationControlScan();

private:

	void InitParameters();
	void OpenOrCloseSerialPort();
	void WriteByte(byte* szWriteBuffer, int number);
	int ReadData(byte* RXBuff);
	
	/*****************************1、相机配置函数*******************************************/
	//1.1 相机触发设置
	int ConfigureTrigger(INodeMap & nodeMap, CameraPtr pCam);
	//1.2 复位触发
	int ResetTrigger(INodeMap & nodeMap, CameraPtr pCam);
	//1.3 触发后图像获取函数
	int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, gcstring deviceSerialNumber, int index, ImagePtr &convertedImage);
	//1.4 设置相机图片规格
	int ConfigureCustomImageSettings(INodeMap & nodeMap);
	//1.5 曝光时间设置
	int ConfigureExposure(INodeMap & nodeMap);

	void getfirstimage();

	///*****************************2、串口初始化函数*******************************************/
	//HANDLE hComm;
	//OVERLAPPED OverLapped;
	//COMSTAT Comstat;
	//DWORD dwCommEvents;
	////2.1 打开串口
	//bool OpenPort();
	////2.2 设置串口
	//bool SetupDCB(int rate_arg);
	////2.3 设置超时时间
	//bool SetupTimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalConstant, DWORD WriteTotalMultiplier, DWORD WriteTotalConstant);
	////2.4 串口接收
	//void ReceiveWaitChar();
	////2.5 串口接收，返回当前缓存区中的所有字节数，最大不超过128字节
	//void ReceiveChar(byte RXBuff[]);
	////2.6 获取当前接收缓存区中的字节数
	//int ReceiveBufferNum();
	////2.7 串口发送
	//bool WriteStr(char* szWriteBuffer);
	//bool WriteByte(byte* szWriteBuffer, DWORD dwSend);
	////2.8 串口初始化全过程
	//void serialPortInit();

	/*****************************3、下位机动作函数*******************************************/
	//3.1步进电机旋转一个角度 (1:x 2:y) //这里写一个函数，首先发送旋转指令，然后在函数里用延时等待旋转结束，然后return
	void cameraInitParameters();

	bool SMX_RotOneDeg(double deg);
	bool SMY_RotOneDeg(double deg);
	//3.2电机使能
	void SM_Enable();
	//3.3电机失能
	void SM_Disable();
	//3.4DLP投影一组标定图片
	void DLP_ProjectOneSet_Cali(int caliCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set);
	//3.5DLP投影一组扫描图片
	void DLP_ProjectOneSet_Scan(int scanCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set);

	void DLP_ProjectOneSet_Global(int scanCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set);
};

#endif // ControlThread_H