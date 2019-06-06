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
#define MODELPLYn
//#include "3DScan_cuda.cuh"
//#include "Spinnaker.h"
//#include "SpinGenApi/SpinnakerGenApi.h"

#include <opencv2\opencv.hpp>

#include "CCyUSBStream.h"

#define IMG_ROW    1024
#define IMG_COL   1280

#define EXPOSURETIME 16666

//#define CALI_ROTATE_POS_CNT 12    //定义标定过程的位置数 13
//#define SCAN_ROTATE_POS_CNT 12    //定义扫描过程的位置数 11

#define CALI_ROTATE_POS_CNT2 11    //定义标定过程的位置数 13
#define SCAN_ROTATE_POS_CNT2 10    //定义扫描过程的位置数 11    
#define SCAN_ALLJAW_POS		  7

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
extern QSemaphore g_event;

//static int SMX_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 45, -90, 45, 45, -90, 45, 45, -90, 45, 45, 135, 135, 45 };
//static int SMY_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 0,  0,  45,  0,  0,  45,  0,  0,  -60, 0,  0,  0,  -30 };
//static int SM_CALI_BRIGHTNESS[CALI_ROTATE_POS_CNT] = { 3,  3,  1,  1,  1,  2,  2,  2,  2,  2,  3,  2,  3 };//亮度等级，3最亮，1为默认
//
//static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 90, -45, -45, -45, -45, -45, -45, -135, 0,   0,  -45 };
//static int SMY_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 30, 0,   0,   0,   0,   0,   0,   0,    -30, 80, -80 };
//static int SM_SCAN_BRIGHTNESS[SCAN_ROTATE_POS_CNT] = { 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2 };//亮度等级，3最亮，1为默认

//static int SMY_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 30, -60, -150, -45, -90, 0,-45, -45, 0, 45, 45, -45 };
//static int SMX_CALI_ROTATE_DEGREE[CALI_ROTATE_POS_CNT] = { 0,  0,  0,  0,  0,  -30,  0,  0,  60,  0,  0, -30 };
//static int SM_CALI_BRIGHTNESS[CALI_ROTATE_POS_CNT] = { 2,  2,  2,  2,  3,  3,  3,  2,  2,  2,  2,  2 };//亮度等级，3最亮，1为默认
//
//																									   //static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
//static int SMY_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 0, 45, 45, 45, 45, 45, 45, 45, 115, -70,  -70,  70 };
//static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { -45, 0,   0,   0,   0,   0,   0,   0,    -35, 0,  0, 80 };
//static int SM_SCAN_BRIGHTNESS[SCAN_ROTATE_POS_CNT] = { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 1 };//亮度等级，3最亮，1为默认
//
static int SMY_CALI_ROTATE_DEGREE2[CALI_ROTATE_POS_CNT2] = { 0,   0,  -30,  30,-25,   0,  25, -30,   0,  30,    0 };
static int SMX_CALI_ROTATE_DEGREE2[CALI_ROTATE_POS_CNT2] = { 0, 30,  30, 30, 20,  20,  20,  10,  10,  10,    0 };
static int SM_CALI_BRIGHTNESS2[CALI_ROTATE_POS_CNT2] = { 2,   2,    2,   2,  2,   2,   2,   2,   2,   2,    0 };//亮度等级，3最亮，1为默认

																												//static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
static int SMY_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,/*-60,   0,  60, -50,   0,  50,*/   0, -45, -90, -135,  179,  135, 90,45, 0 };
static int SMX_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,/* 30,  30,  30,  20,  20,  20,*/ -30, -30, -30,  -30, -30, -30, -30,-30, 0 };

//static int SMY_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,0,   -45,  -90, -135,   -180,  135,   90, 45, 0, 45,  90,  135, 180,-135, -90 ,-45,0};
//static int SMX_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0, -20,  -20,  -20,  -20,  -20,  -20, -20, -20, -40,  -40, -40, -40, -40,-40, -40,-40,0};


// static int ALLJAWY_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = {   0,-45,-90, 45, 90,  0,-45,-90, 45, 90,0 };
// static int ALLJAWX_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = { -30,-30,-30,-30,-30,-60,-60,-60,-60,-60,0 };

// static int ALLJAWY_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = { 0,-45,-90, -90, -45,  0,45,90, 90, 45,0 };
// static int ALLJAWX_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = { -50,-50,-50,-80,-80,-80,-80,-80,-50,-50,0 };

static int ALLJAWY_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = { 0,-45, -45,  0,45,45,0 };
static int ALLJAWX_SCAN_ROTATE_DEGREE2[SCAN_ALLJAW_POS] = { -50,-50,-80,-80,-80,-50,0 };
//
//static int SM_SCAN_BRIGHTNESS2[SCAN_ROTATE_POS_CNT2] = { 2,/*  2,   2,   2,   2,   2,   2,*/   3,   3,   3,    3,   3,   3,   3,  3, 2 }; //亮度等级，3最亮，1为默认
////static int SMX_SCAN_ROTATE_DEGREE[SCAN_ROTATE_POS_CNT] = { 15, 30,   0,   0,   0,   0,   0,   0,    35, 0,  0, -80 };
//static int SMX_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0,-60,   0,  60, -50,   0,  50,   0, -40,  -80, -140,    40,   80,  140, 0 };
//static int SMY_SCAN_ROTATE_DEGREE2[SCAN_ROTATE_POS_CNT2] = { 0, 30,  30,  30,  20,  20,  20, -30, -30,  -30,  -30,  -30,  -30,  -30,  0 };
//static int SM_SCAN_BRIGHTNESS2[SCAN_ROTATE_POS_CNT2] = { 2,  2,   2,   2,   2,   2,   2,   3,   3,    3,    3,    3,   3,    3,  2 };//亮度等级，3最亮，1为默认


using namespace std;
//using namespace Spinnaker;
//using namespace Spinnaker::GenApi;
//using namespace Spinnaker::GenICam;
using namespace Communication;

class ControlThread : public QObject
{
	Q_OBJECT

public:
	ControlThread(QObject *parent=0);
	~ControlThread();

	CCyUSBStream l_usbStream;

	vector<vector<double>> points_cloud_globle;

	float l_scan_x;
	float l_scan_y;
	float l_scan_z;
	
	float c_scan_z;
	//float c_scan_x;
	//float c_scan_y;

	void setFlage(bool flag = true);  //设置标志位，何时关闭子线程

	bool  isStop;

	vector<cv::Mat> calibImageCamera;
signals:
	void calibImageSignal(int endFlag);
	void calibFinishSignal(QString errorStr);
public slots:
	//void normalControlScan();
	//void normalAllJawControlScan();
	void controlCalibrationScan();
	void compensationControlScan();
	//void controlGlobalCaliScan();

	void allJawScan();
	void normalScan();
private:
	//double m_ddeg_x = 0.0;
	//bool m_bpositiveOrien_x;//大电机转向
	//double m_ddeg_y = 0.0;
	//bool m_bpositiveOrien_y;//小电机反转
	//bool m_bcali;//标定

	void InitParameters();
	//void SMRotDegAnalysis(double v_ddeg_x, double v_ddeg_y, bool v_bcali);
};

#endif // ControlThread_H