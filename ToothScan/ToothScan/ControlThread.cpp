#include "ControlThread.h"
#define bufferSize 30
extern int DataSize = bufferSize;
//extern int BufferSize = 5;
extern unsigned char *totalNormalScanImageBuffer = (unsigned char *)malloc(bufferSize * 34 * 1280 * 1024 * sizeof(unsigned char));

extern cv::Mat rt_r = cv::Mat::eye(4, 4, CV_64FC1);
extern std::vector<cv::Mat> scanner_rt(9, rt_r);

extern float c_scan_x = 0.0;
extern float c_scan_y = 0.0;

extern QSemaphore freeSpace(DataSize);
extern QSemaphore usedSpace(0);
extern scan::RasterScan *rs = new scan::RasterScan();
#define DEPLOY1
class CCon {
public:
	CCon(QSemaphore &freeSpace, QSemaphore &usedSpace):m_freeSpace(freeSpace), m_usedSpace(usedSpace)
	{
		freeSpace.acquire();
		cout << "freeSpace.acquire();" << endl;
	}
	~CCon() {
		usedSpace.release();
		cout << "usedSpace.release();" << endl;
	}
private:
	QSemaphore &m_freeSpace;
	QSemaphore &m_usedSpace;
};

ControlThread::ControlThread(QObject *parent)
	: QObject(parent)
{
	isStop = false;
	//point_cloud_index = 0;
	//serial_port_open = false;
	//point_cloud_right.resize(CALI_ROTATE_POS_CNT2 - 1);
	InitParameters();
	l_scan_x = 0.0;
	l_scan_y = 0.0;

	//scan_times = 0;
}

ControlThread::~ControlThread()
{
}

void ControlThread::InitParameters()
{
	cout << " RasterScan Start!!" << endl;
	rs->InitRasterScan("SystemCalibration.yml");

	rt_r.at<double>(0, 0) = rs->Rot_r.at<double>(0, 0);
	rt_r.at<double>(0, 1) = rs->Rot_r.at<double>(0, 1);
	rt_r.at<double>(0, 2) = rs->Rot_r.at<double>(0, 2);
	rt_r.at<double>(1, 0) = rs->Rot_r.at<double>(1, 0);
	rt_r.at<double>(1, 1) = rs->Rot_r.at<double>(1, 1);
	rt_r.at<double>(1, 2) = rs->Rot_r.at<double>(1, 2);
	rt_r.at<double>(2, 0) = rs->Rot_r.at<double>(2, 0);
	rt_r.at<double>(2, 1) = rs->Rot_r.at<double>(2, 1);
	rt_r.at<double>(2, 2) = rs->Rot_r.at<double>(2, 2);
	rt_r.at<double>(0, 3) = rs->tvec_r.at<double>(0, 0);
	rt_r.at<double>(1, 3) = rs->tvec_r.at<double>(1, 0);
	rt_r.at<double>(2, 3) = rs->tvec_r.at<double>(2, 0);

	//plane_center = cv::Mat::zeros(4, 1, CV_64FC1);//-79.2578 -23.4466 105.556
	//plane_center.at<double>(0, 0) = 50; plane_center.at<double>(1, 0) = 40; plane_center.at<double>(3, 0) = 1;
	//camera_center = cv::Mat::zeros(4, 1, CV_64FC1);
	//camera_center.at<double>(0) = -79.2578; camera_center.at<double>(1) = -23.4466; camera_center.at<double>(2) = 105.556; camera_center.at<double>(3) = 1;
	//float start_xy = sqrt(pow(-camera_center.at<double>(0) - plane_center.at<double>(0), 2) + pow(-camera_center.at<double>(1) - plane_center.at<double>(1), 2));
	cout << rt_r << endl;

	//rt_r = rt_r.inv();

	Mat pre_move = Mat::eye(4, 4, CV_64FC1);
	pre_move.at<double>(0, 0) = 1; pre_move.at<double>(0, 1) = 0; pre_move.at<double>(0, 2) = 0; pre_move.at<double>(0, 3) = -50;
	pre_move.at<double>(1, 0) = 0; pre_move.at<double>(1, 1) = 1; pre_move.at<double>(1, 2) = 0; pre_move.at<double>(1, 3) = -40;
	pre_move.at<double>(2, 0) = 0; pre_move.at<double>(2, 1) = 0; pre_move.at<double>(2, 2) = 1; pre_move.at<double>(2, 3) = 0;
	pre_move.at<double>(3, 0) = 0; pre_move.at<double>(3, 1) = 0; pre_move.at<double>(3, 2) = 0; pre_move.at<double>(3, 3) = 1;

	Mat pre_pitch = Mat::eye(4, 4, CV_64FC1);
	//double pre_alpha = ToRad(0);  // x axis
	//pre_pitch.at<double>(0, 0) = 1; pre_pitch.at<double>(0, 1) = 0; pre_pitch.at<double>(0, 2) = 0; pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = 0; pre_pitch.at<double>(1, 1) = cosf(pre_alpha); pre_pitch.at<double>(1, 2) = sinf(pre_alpha); pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = 0; pre_pitch.at<double>(2, 1) = -sinf(pre_alpha); pre_pitch.at<double>(2, 2) = cosf(pre_alpha); pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//double pre_alpha = ToRad(90);  // z axis
	//pre_pitch.at<double>(0, 0) = cosf(pre_alpha); pre_pitch.at<double>(0, 1) = 0; pre_pitch.at<double>(0, 2) = sinf(pre_alpha); pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = 0; pre_pitch.at<double>(1, 1) = 1; pre_pitch.at<double>(1, 2) = 0; pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = -sinf(pre_alpha); pre_pitch.at<double>(2, 1) = 0; pre_pitch.at<double>(2, 2) = cosf(pre_alpha); pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//double pre_alpha = ToRad(90);  // z axis
	//pre_pitch.at<double>(0, 0) = cosf(pre_alpha); pre_pitch.at<double>(0, 1) = -sinf(pre_alpha); pre_pitch.at<double>(0, 2) = 0; pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = sinf(pre_alpha); pre_pitch.at<double>(1, 1) = cosf(pre_alpha); pre_pitch.at<double>(1, 2) =0; pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = 0; pre_pitch.at<double>(2, 1) = 0; pre_pitch.at<double>(2, 2) = 1; pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//----------------------------------------- init scan ---------------------------------------
	cv::FileStorage fs_g("D:/dentalimage/dentalimage2/external_parameter.yml", cv::FileStorage::READ);  //read the cameras file：external_parameter.yml
	for (int image_index = 0; image_index < 9; image_index++)
	{
		stringstream ss;
		ss << image_index;
		string index_;
		ss >> index_;
		cv::Mat rt;
		fs_g["rt_mat" + index_] >> rt;
		scanner_rt[image_index] = rt/*.inv()*/;
	}
	fs_g.release();

	//l_usbStream.InitCyUSBParameter();//初始化
	//bool closedFlag = l_usbStream.ClosedDLPFunction();
	//cout << "closedFlag = " << closedFlag << endl;
	//_sleep(1000);
	//bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
	//cout << "openFlag = " << closedFlag << endl;
	//cout << "初始化光机，等待5秒。。。 " << endl;
}

void ControlThread::setFlage(bool flag)
{
	isStop = flag;
}

//void ControlThread::SMRotDegAnalysis(double v_ddeg_x, double v_ddeg_y, bool v_bcali)
//{
//	if (v_ddeg_x >= 0)
//	{
//		m_ddeg_x = v_ddeg_x;
//		m_bpositiveOrien_x = true;
//	}
//	else
//	{
//		m_ddeg_x = abs(v_ddeg_x);
//		m_bpositiveOrien_x = false;
//	}
//	if (v_ddeg_y >= 0)
//	{
//		m_ddeg_y = v_ddeg_y;
//		m_bpositiveOrien_y = true;
//	}
//	else
//	{
//		m_ddeg_y = abs(v_ddeg_y);
//		m_bpositiveOrien_x = false;
//	}
//	m_bcali = v_bcali;
//}

//void ControlThread::normalControlScan()
//{
//	bool l_bcali = false;
//	//vector<cv::Mat> image_groups_left, image_groups_right;
//	int imageSize = IMG_ROW * IMG_COL;
//	int bufferBias = 0;
//	
//	//l_usbStream.InitCyUSBParameter();//初始化
//#ifdef DEPLOY
//	 	bool closedFlag = l_usbStream.ClosedDLPFunction();
//	 	cout << "closedFlag = " << closedFlag << endl;
//	 	_sleep(1000);
//	 	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
//	 	cout << "openFlag = " << closedFlag << endl;
//	 	cout << "初始化光机，等待5秒。。。 " << endl;
//	 	_sleep(4000);
//#else
//	bool resetFlag = l_usbStream.ResetDLPFunction();
//	cout << "resetFlag = " << resetFlag << endl;
//	_sleep(4000);
//#endif
//
//
//	l_usbStream.SetScanDLPLight();
//	clock_t time1, time2, time3, time4;
//	for (int scan_index = 0; scan_index < SCAN_ROTATE_POS_CNT2; scan_index++)
//	{
//		double d_scan_x = 0.0;
//		double d_scan_y = 0.0;
//		c_scan_x = SMX_SCAN_ROTATE_DEGREE2[scan_index];
//		c_scan_y = SMY_SCAN_ROTATE_DEGREE2[scan_index];
//
//		d_scan_x = (c_scan_x - l_scan_x);
//		d_scan_y = (c_scan_y - l_scan_y);
//		
//		l_scan_x = c_scan_x;
//		l_scan_y = c_scan_y;
//
//		if (c_scan_x < -90 || c_scan_x > 90)
//		{
//			return;
//		}
//		
//
//		vector<cv::Mat> imgL_set, imgR_set;
//
//		time1 = clock();
//		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y,  l_bcali, imgL_set, imgR_set);
//		time2 = clock();
//
//	if (scan_index == SCAN_ROTATE_POS_CNT2 - 1)
//		{
//			continue;
//		}
//
//		if (imgL_set.size() < 19 || imgR_set.size() < 19)
//		{
//			cout << "USB has a problem, because of insufficient data..." << endl;
//			return;
//		}		
//		
//		CCon coc(freeSpace, usedSpace);
//		//freeSpace.acquire();
//		vector<cv::Mat> images_l, images_r;
//		vector<cv::Mat> image_rgb;
//		//unsigned char* im_l = 0;
//		//unsigned char* im_r = 0;
//		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
//		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
//		int imageBias = 0;
//		time3 = clock();
//		for (int image_index = 0; image_index < 19; image_index++)
//		{
//			
//			ostringstream filename_L;
//			cv::flip(imgL_set[image_index], imgL_set[image_index], -1);
//			filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";
//			//cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);
//
//
//			ostringstream filename_R;
//			cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
//			filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";
//			//cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);
//			if (image_index == 0)
//			{
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//			}
//
//			if (image_index>0 && image_index<16)
//			{
//				//images_l.push_back(imgL_set[image_index]);
//				//images_r.push_back(imgR_set[image_index]);
//				//memcpy(im_l + (image_index-1) * 1280 * 1024, (unsigned char*)imgL_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
//				//memcpy(im_r + (image_index-1) * 1280 * 1024, (unsigned char*)imgR_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgL_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//				
//			}
//			else if (image_index>=16)
//			{
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//			}
//		}
//		time4 = clock();
//		bufferBias++;
//		//usedSpace.release();
//		cout << "The ControlThread: " << scan_index << " has finished." << endl;
//
//		cout << "The rotation and projection time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
//		cout << "The memcpy time is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
//	}
//
//	//3、关闭DLP
//#ifdef DEPLOY
//	l_usbStream.ClosedDLPFunction();
//#endif
//	//l_usbStream.AbortXferLoop();
//	//cout << "关闭DLP。。 " << endl;
//}

//void ControlThread::normalAllJawControlScan()
//{
//	bool l_bcali = false;
//	//vector<cv::Mat> image_groups_left, image_groups_right;
//	int imageSize = IMG_ROW * IMG_COL;
//	int bufferBias = 0;
//
//	//l_usbStream.InitCyUSBParameter();//初始化
//#ifdef DEPLOY
//	bool closedFlag = l_usbStream.ClosedDLPFunction();
//	cout << "closedFlag = " << closedFlag << endl;
//	_sleep(1000);
//	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
//	cout << "openFlag = " << closedFlag << endl;
//	cout << "初始化光机，等待5秒。。。 " << endl;
//	_sleep(4000);
//#else
//	bool resetFlag = l_usbStream.ResetDLPFunction();
//	cout << "resetFlag = " << resetFlag << endl;
//	_sleep(4000);
//#endif
//
//	l_usbStream.SetScanDLPLight();//设置光机亮度
//
//	clock_t time1, time2, time3, time4;
//	for (int scan_index = 0; scan_index < SCAN_ALLJAW_POS; scan_index++)
//	{
//		double d_scan_x = 0.0;
//		double d_scan_y = 0.0;
//		c_scan_x = ALLJAWX_SCAN_ROTATE_DEGREE2[scan_index];
//		c_scan_y = ALLJAWY_SCAN_ROTATE_DEGREE2[scan_index];
//
//		d_scan_x = (c_scan_x - l_scan_x);
//		d_scan_y = (c_scan_y - l_scan_y);
//
//		l_scan_x = c_scan_x;
//		l_scan_y = c_scan_y;
//
//		if (c_scan_x < -90 || c_scan_x > 90)
//		{
//			return;
//		}
//
//
//		vector<cv::Mat> imgL_set, imgR_set;
//
//		time1 = clock();
//		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, imgL_set, imgR_set);
//		time2 = clock();
//
//		if (scan_index == SCAN_ALLJAW_POS - 1)
//		{
//			continue;
//		}
//
//		if (imgL_set.size() < 19 || imgR_set.size() < 19)
//		{
//			cout << "USB has a problem, because of insufficient data..." << endl;
//			return;
//		}
//
//		CCon coc(freeSpace, usedSpace);
//		//freeSpace.acquire();
//		vector<cv::Mat> images_l, images_r;
//		vector<cv::Mat> image_rgb;
//		//unsigned char* im_l = 0;
//		//unsigned char* im_r = 0;
//		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
//		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
//		int imageBias = 0;
//		time3 = clock();
//		for (int image_index = 0; image_index < 19; image_index++)
//		{
//
//			ostringstream filename_L;
//			cv::flip(imgL_set[image_index], imgL_set[image_index], -1);
//			filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";
//			cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);
//
//
//			ostringstream filename_R;
//			cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
//			filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";
//			cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);
//			if (image_index == 0)
//			{
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//			}
//
//			if (image_index>0 && image_index<16)
//			{
//				//images_l.push_back(imgL_set[image_index]);
//				//images_r.push_back(imgR_set[image_index]);
//				//memcpy(im_l + (image_index-1) * 1280 * 1024, (unsigned char*)imgL_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
//				//memcpy(im_r + (image_index-1) * 1280 * 1024, (unsigned char*)imgR_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgL_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//
//			}
//			else if (image_index >= 16)
//			{
//				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
//				imageBias++;
//			}
//		}
//		time4 = clock();
//		bufferBias++;
//		//usedSpace.release();
//		cout << "The ControlThread: " << scan_index << " has finished." << endl;
//
//		cout << "The rotation and projection time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
//		cout << "The memcpy time is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
//	}
//
//	//3、关闭DLP
//#ifdef DEPLOY
//	l_usbStream.ClosedDLPFunction();
//#endif
//	//l_usbStream.AbortXferLoop();
//	//cout << "关闭DLP。。 " << endl;
//}

void ControlThread::allJawScan()
{
	bool l_bcali = false;
	//vector<cv::Mat> image_groups_left, image_groups_right;
	int imageSize = IMG_ROW * IMG_COL;
	int bufferBias = 0;

	// 	bool closedFlag = l_usbStream.ClosedDLPFunction();
#ifdef DEPLOY
	bool closedFlag = l_usbStream.ClosedDLPFunction();
	cout << "closedFlag = " << closedFlag << endl;
	_sleep(1000);
	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
	cout << "openFlag = " << closedFlag << endl;
	cout << "初始化光机，等待5秒。。。 " << endl;
	_sleep(4000);
#else
	bool resetFlag = l_usbStream.ResetDLPFunction();
	cout << "resetFlag = " << resetFlag << endl;
	_sleep(4000);
#endif

	//l_usbStream.SetScanDLPLight();
	l_usbStream.SetMinDLPLight();
	clock_t time1, time2, time3, time4;
	for (int scan_index = 0; scan_index < SCAN_ALLJAW_POS; scan_index++)
	{
		double d_scan_x = 0.0;
		double d_scan_y = 0.0;
		c_scan_x = ALLJAWX_SCAN_ROTATE_DEGREE2[scan_index];
		c_scan_y = ALLJAWY_SCAN_ROTATE_DEGREE2[scan_index];

		d_scan_x = (c_scan_x - l_scan_x);
		d_scan_y = (c_scan_y - l_scan_y);

		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;

		if (c_scan_x < -90 || c_scan_x > 90)
		{
			return;
		}
		

		vector<cv::Mat> imgL_set, imgR_set;
		if (scan_index == SCAN_ALLJAW_POS - 1)
		{
			l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, false, imgL_set, imgR_set);
			continue;
		}
		time1 = clock();
		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, true, imgL_set, imgR_set);
		time2 = clock();

		if (imgL_set.size() < SCAN_IMAGE_NUMBER || imgR_set.size() < SCAN_IMAGE_NUMBER)
		{

			cout << "USB has a problem, because of insufficient data..." << endl;
			return;
		}

		CCon coc(freeSpace, usedSpace);
		//freeSpace.acquire();
		vector<cv::Mat> images_l, images_r;
		vector<cv::Mat> image_rgb;
		//unsigned char* im_l = 0;
		//unsigned char* im_r = 0;
		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		int imageBias = 0;
		time3 = clock();
		for (int image_index = 0; image_index < SCAN_IMAGE_NUMBER; image_index++)
		{

			ostringstream filename_L;
			//cv::flip(imgL_set[image_index], imgL_set[image_index], -1);
			filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";

			cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);


			ostringstream filename_R;
			//cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
			filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";

			cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);
			if (image_index == 0)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}

			if (image_index > 0 && image_index < UNWRAPING_IMAGE_NUMBER+1)
			{
				//images_l.push_back(imgL_set[image_index]);
				//images_r.push_back(imgR_set[image_index]);
				//memcpy(im_l + (image_index-1) * 1280 * 1024, (unsigned char*)imgL_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
				//memcpy(im_r + (image_index-1) * 1280 * 1024, (unsigned char*)imgR_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgL_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;

			}
			else if (image_index >= UNWRAPING_IMAGE_NUMBER + 1)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}
		}
		time4 = clock();
		bufferBias++;
		//usedSpace.release();
		cout << "The ControlThread: " << scan_index << " has finished." << endl;

		cout << "The rotation and projection time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		cout << "The memcpy time is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
	}

	//3、关闭DLP
#ifdef DEPLOY
	l_usbStream.ClosedDLPFunction();
#endif
	//l_usbStream.ClosedDLPFunction();
	//l_usbStream.AbortXferLoop();
	//cout << "关闭DLP。。 " << endl;
}

void ControlThread::controlCalibrationScan()
{
	bool l_bcali = true;
	//l_usbStream.InitCyUSBParameter();//初始化

#ifdef DEPLOY
	bool closedFlag = l_usbStream.ClosedDLPFunction();
	cout << "closedFlag = " << closedFlag << endl;
	_sleep(1000);
	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
	cout << "openFlag = " << closedFlag << endl;
	cout << "初始化光机，等待5秒。。。 " << endl;
	_sleep(4000);
#else
	bool resetFlag = l_usbStream.ResetDLPFunction();
	cout << "resetFlag = " << resetFlag << endl;
	_sleep(4000);
#endif

	l_usbStream.SetDLPLight(1);
	//l_usbStream.SetMidDLPLight();
	vector<vector<cv::Mat>> image_groups;
	for (int scan_index = 0; scan_index < CALI_ROTATE_POS_CNT2; scan_index++)
	{
		if (scan_index == 1)
		{
			l_usbStream.SetDLPLight(2);
		}

		c_scan_x = SMX_CALI_ROTATE_DEGREE2[scan_index];
		c_scan_y = SMY_CALI_ROTATE_DEGREE2[scan_index];

		double d_scan_x = (c_scan_x - l_scan_x);
		double d_scan_y = (c_scan_y - l_scan_y);
		//double d_scan_z = (c_scan_z - l_scan_z);
		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;

		if (c_scan_x < -90 || c_scan_x > 90)
		{
			return;
		}

		vector<cv::Mat> imgL_set, imgR_set;
		//cout << "******************************************************************" << endl;
		//cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;

		//SMRotDegAnalysis(d_scan_x, d_scan_y, true);
		if (scan_index == CALI_ROTATE_POS_CNT2 - 1)
		{
			l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, false, imgL_set, imgR_set);
			continue;
		}
		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, true, imgL_set, imgR_set);


	
		if (imgL_set.size() < CALI_IMAGE_NUMBER || imgR_set.size() < CALI_IMAGE_NUMBER)
		{
			cout << "USB has a problem, because of insufficient data..." << endl;
			return;
		}
		vector<cv::Mat> images_l, images_r;
		vector<cv::Mat> image_rgb;
		//unsigned char* im_l = 0;
		//unsigned char* im_r = 0;
		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		int imageBias = 0;

		cout << "开始存图片。。 " << endl;
		vector<cv::Mat> group;
		for (size_t j = 0; j < CALI_IMAGE_NUMBER; j++)
		{
			//cv::flip(imgL_set[j], imgL_set[j], -1);
			group.push_back(imgL_set[j]);
			if (j == 0)
			{
				calibImageCamera.push_back(imgL_set[j]);
			}

			ostringstream filename_L;
			filename_L << "D:\\dentalimage\\dentalimage2\\CaliPic\\" << scan_index << "_" << j << "_" << "L" << ".png";
			cv::imwrite(filename_L.str().c_str(), imgL_set[j]);
		}
		for (size_t j = 0; j < CALI_IMAGE_NUMBER; j++)
		{
			//cv::flip(imgR_set[j], imgR_set[j], -1);
			group.push_back(imgR_set[j]);
			if (j == 0)
			{
				calibImageCamera.push_back(imgR_set[j]);
				emit calibImageSignal(0);
			}
			ostringstream filename_R;
			filename_R << "D:\\dentalimage\\dentalimage2\\CaliPic\\" << scan_index << "_" << j << "_" << "R" << ".png";

			cv::imwrite(filename_R.str().c_str(), imgR_set[j]);
		}
		image_groups.push_back(group);
		//4、保存图片
		cout << "一个角度标定图片存储完毕。。 " << endl;
	}
	cout << "全部标定图片存储完毕。。 " << endl;
	////3、关闭DLP
//	l_usbStream.ClosedDLPFunction();
	//l_usbStream.AbortXferLoop();
	rs->PreCalibration(1280, 960, 10, image_groups);

	for (int image_index = 0; image_index < CALI_ROTATE_POS_CNT2 - 1; image_index++)
	{
		cv::Mat image_l, image_r;
		bool f_l, f_r;
		rs->CalibrationImagePrint(image_l, image_r, image_index, f_l, f_r);
		calibImageCamera.push_back(image_l);
		calibImageCamera.push_back(image_r);
		int endFlag = 0;
		if (image_index == (CALI_ROTATE_POS_CNT2 - 2))
		{
			endFlag = 1;
			emit calibImageSignal(endFlag);
		}
		else if(image_index != (CALI_ROTATE_POS_CNT2 - 2))
		{
			emit calibImageSignal(endFlag);
		}
	}
	orth::Point3d error;
	rs->SystemCalibration("SystemCalibration.yml", error);
	cout << error << endl;
	string errorStr = std::to_string(error.x) + ", " + std::to_string(error.y) + ", " + std::to_string(error.z) + ".";
	QString errorQstr = QString::QString::fromStdString(errorStr);
	emit calibFinishSignal(errorQstr);

	InitParameters();
}


//void ControlThread::controlGlobalCaliScan()
//{
//	bool l_bcali = true;
//	//l_usbStream.InitCyUSBParameter();//初始化
//
//#ifdef DEPLOY
//	bool closedFlag = l_usbStream.ClosedDLPFunction();
//	cout << "closedFlag = " << closedFlag << endl;
//	_sleep(1000);
//	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
//	cout << "openFlag = " << closedFlag << endl;
//	cout << "初始化光机，等待5秒。。。 " << endl;
//	_sleep(4000);
//#else
//	bool resetFlag = l_usbStream.ResetDLPFunction();
//	cout << "resetFlag = " << resetFlag << endl;
//	_sleep(4000);
//#endif
//
//	l_usbStream.SetScanDLPLight();//设置光机亮度
//	vector<Mat> image_groups_left, image_groups_right;
//	for (int scan_index = 0; scan_index < SCAN_ROTATE_POS_CNT2; scan_index++)
//	{
//		
//		c_scan_x = SMX_SCAN_ROTATE_DEGREE2[scan_index];
//		c_scan_y = SMY_SCAN_ROTATE_DEGREE2[scan_index];
//
//		double d_scan_x = (c_scan_x - l_scan_x);
//		double d_scan_y = (c_scan_y - l_scan_y);
//		//double d_scan_z = (c_scan_z - l_scan_z);
//		l_scan_x = c_scan_x;
//		l_scan_y = c_scan_y;
//
//		cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;
//		//l_scan_z = c_scan_z;
//		if (c_scan_x<-90 || c_scan_x>90)
//		{
//			return;
//		}
//		
//		vector<cv::Mat> imgL_set, imgR_set;
//		cout << "******************************************************************" << endl;
//		cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;
//
//		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, imgL_set, imgR_set);
//
//		if (scan_index == SCAN_ROTATE_POS_CNT2 - 1)
//		{
//			continue;
//		}
//
//		cout << "开始存图片。。 " << endl;
//
//		cv::flip(imgL_set[0], imgL_set[0], -1);
//		image_groups_left.push_back(imgL_set[0]);
//		ostringstream filename_L;
//		filename_L << "D:\\dentalimage\\dentalimage2\\GloPic\\" << scan_index << "_0_" << "L" << ".png";
//
//		//cv::imwrite(filename_L.str().c_str(), imgL_set[0]);
//
//		cv::flip(imgR_set[0], imgR_set[0], -1);
//		image_groups_right.push_back(imgR_set[0]);
//		ostringstream filename_R;
//		filename_R << "D:\\dentalimage\\dentalimage2\\GloPic\\" << scan_index << "_0_" << "R" << ".png";
//
//		//cv::imwrite(filename_R.str().c_str(), imgR_set[0]);
//
//		//4、保存图片
//		cout << "标定图片存储完毕。。 " << endl;
//	}
//
//	////3、关闭DLP
//	//l_usbStream.ClosedDLPFunction();
//
//	vector<double> mask_points;
//	rs->PlaneRTCalculate(image_groups_left, image_groups_right, "D:/dentalimage/dentalimage2/external_parameter.yml", mask_points);
//	//ColoredPoints(mask_points, 2);
//	InitParameters();
//}

void ControlThread::compensationControlScan()
{
	bool l_bcali = false;
	
	//l_usbStream.InitCyUSBParameter();//初始化

#ifdef DEPLOY
	bool closedFlag = l_usbStream.ClosedDLPFunction();
	cout << "closedFlag = " << closedFlag << endl;
	_sleep(1000);
	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
	cout << "openFlag = " << closedFlag << endl;
	cout << "初始化光机，等待5秒。。。 " << endl;
	_sleep(4000);
#else
	bool resetFlag = l_usbStream.ResetDLPFunction();
	cout << "resetFlag = " << resetFlag << endl;
	_sleep(4000);
#endif

	int imageSize = IMG_ROW * IMG_COL;
	double d_scan_x = (c_scan_x - l_scan_x);
	double d_scan_y = (c_scan_y - l_scan_y);
	//double d_scan_z = (c_scan_z - l_scan_z);
	l_scan_x = 0;
	l_scan_y = 0;

	if (c_scan_x<-90 || c_scan_x>90)
	{
		return;
	}
	CCon coc(freeSpace, usedSpace);
	//freeSpace.acquire();

	cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;

	vector<cv::Mat> imgL_set, imgR_set;
	cout << "******************************************************************" << endl;
	cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;

	///**************************************扫描过程*****************************************/
	l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, true, imgL_set, imgR_set);

	if (imgL_set.size() < SCAN_IMAGE_NUMBER || imgR_set.size() < SCAN_IMAGE_NUMBER)
	{

		cout << "USB has a problem, because of insufficient data..." << endl;
		return;
	}

	//存进总数组
	vector<cv::Mat> images_l, images_r;
	vector<cv::Mat> image_rgb;
	//unsigned char* im_l = 0;
	//unsigned char* im_r = 0;
	//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
	//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
	int imageBias = 0;
	for (int image_index = 0; image_index < SCAN_IMAGE_NUMBER; image_index++)
	{
		//cv::flip(imgL_set[image_index], imgL_set[image_index], -1);
		ostringstream filename_L;
		filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << curModelSize << "_" << image_index << "_" << "L" << ".png";

		cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);

		//cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
		ostringstream filename_R;
		filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << curModelSize << "_" << image_index << "_" << "R" << ".png";

		cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);

		if (image_index == 0)
		{
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;
		}

		if (image_index>0 && image_index<UNWRAPING_IMAGE_NUMBER+1)
		{
			//images_l.push_back(imgL_set[image_index]);
			//images_r.push_back(imgR_set[image_index]);
			//memcpy(im_l + (image_index-1) * 1280 * 1024, (unsigned char*)imgL_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
			//memcpy(im_r + (image_index-1) * 1280 * 1024, (unsigned char*)imgR_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgL_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;

		}
		else if (image_index >= UNWRAPING_IMAGE_NUMBER + 1)
		{
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;
		}
	}
	curModelSize += 1;
	l_usbStream.SMRotOneDegFunction(-d_scan_x, -d_scan_y, l_bcali, false, imgL_set, imgR_set);

	//3、关闭DLP
#ifdef DEPLOY
	l_usbStream.ClosedDLPFunction();
#endif
	//l_usbStream.ClosedDLPFunction();//电机失能
									//WriteByte(CloseDLP, 5);
	//cout << "关闭DLP。。 " << endl;
	//usedSpace.release();
}

void ControlThread::normalScan()
{
	bool l_bcali = false;
	vector<cv::Mat> image_groups_left, image_groups_right;
	int imageSize = IMG_ROW * IMG_COL;
	int bufferBias = 0;


#ifdef DEPLOY
	bool closedFlag = l_usbStream.ClosedDLPFunction();
	cout << "closedFlag = " << closedFlag << endl;
	_sleep(1000);
	bool openFlag = l_usbStream.OpenDLPFunction();//打开光机
	cout << "openFlag = " << closedFlag << endl;
	cout << "初始化光机，等待5秒。。。 " << endl;
	_sleep(4000);
#else
	bool resetFlag = l_usbStream.ResetDLPFunction();
	cout << "resetFlag = " << resetFlag << endl;
	_sleep(4000);
#endif

	l_usbStream.SetMinDLPLight();//设置光机亮度
	clock_t time1, time2, time3, time4, time5, time6;

	curModelSize = 0;
	for (int scan_index = 0; scan_index < SCAN_ROTATE_POS_CNT2; scan_index++)
	{
		
		double d_scan_x = 0.0;
		double d_scan_y = 0.0;
		c_scan_x = SMX_SCAN_ROTATE_DEGREE2[scan_index];
		c_scan_y = SMY_SCAN_ROTATE_DEGREE2[scan_index];

		d_scan_x = (c_scan_x - l_scan_x);
		d_scan_y = (c_scan_y - l_scan_y);

		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;

		if (c_scan_x < -90 || c_scan_x > 90)
		{
			return;
		}


		vector<cv::Mat> imgL_set, imgR_set;
		//2、电机旋转
		if (scan_index == SCAN_ROTATE_POS_CNT2 - 1)
		{
			//l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, false, imgL_set, imgR_set);
			curModelSize += (SCAN_ROTATE_POS_CNT2-1);
			//continue;
		}
		time1 = clock();
		l_usbStream.SMRotOneDegFunction(d_scan_x, d_scan_y, l_bcali, true, imgL_set, imgR_set);
		time2 = clock();
		if (imgL_set.size() < SCAN_IMAGE_NUMBER || imgR_set.size() < SCAN_IMAGE_NUMBER)
		{
			cout << "USB has a problem, because of insufficient data..." << endl;
			return;
		}
		CCon coc(freeSpace, usedSpace);
		//freeSpace.acquire();
		//cout << "开始存图片。。 " << endl;

		vector<cv::Mat> images_l, images_r;
		vector<cv::Mat> image_rgb;
		//unsigned char* im_l = 0;
		//unsigned char* im_r = 0;
		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		int imageBias = 0;
		time3 = clock();
		for (int image_index = 0; image_index < SCAN_IMAGE_NUMBER; image_index++)
		{

			ostringstream filename_L;

			//cv::flip(imgL_set[image_index], imgL_set[image_index], -1);
			filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";

			cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);

			imgL_set.push_back(cv::imread(filename_L.str(), 0));


			ostringstream filename_R;

			//cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
			filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";
			cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);
			if (image_index == 0)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}

			if (image_index > 0 && image_index < UNWRAPING_IMAGE_NUMBER+1)
			{
				//images_l.push_back(imgL_set[image_index]);
				//images_r.push_back(imgR_set[image_index]);
				//memcpy(im_l + (image_index-1) * 1280 * 1024, (unsigned char*)imgL_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
				//memcpy(im_r + (image_index-1) * 1280 * 1024, (unsigned char*)imgR_set[image_index].data, 1280 * 1024 * sizeof(unsigned char));
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgL_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;

			}
			else if (image_index >= UNWRAPING_IMAGE_NUMBER + 1)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}
		}
		time4 = clock();
		bufferBias++;
		//usedSpace.release();
		cout << "The ControlThread: " << scan_index << " has finished." << endl;

		cout << "The rotation and projection time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		cout << "The memcpy time is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
	}

	//3、关闭DLP

#ifdef DEPLOY
	l_usbStream.ClosedDLPFunction();
#endif
// 	l_usbStream.ClosedDLPFunction();
// 	cout << "关闭DLP。。 " << endl;
}

