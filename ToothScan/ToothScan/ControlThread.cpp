#include "ControlThread.h"

extern int DataSize = SCAN_ROTATE_POS_CNT2 - 1;
//extern int BufferSize = 5;
extern unsigned char *totalNormalScanImageBuffer = (unsigned char *)malloc((SCAN_ROTATE_POS_CNT2-1) * 34 * 1280 * 1024 * sizeof(unsigned char));

extern cv::Mat rt_r = cv::Mat::eye(4, 4, CV_64FC1);
extern std::vector<cv::Mat> scanner_rt(9, rt_r);

extern float c_scan_x = 0.0;
extern float c_scan_y = 0.0;

extern QSemaphore freeSpace(DataSize);
extern QSemaphore usedSpace(0);
extern scan::RasterScan *rs = new scan::RasterScan();

ControlThread::ControlThread(QObject *parent)
	: QObject(parent)
{
	isStop = false;
	//point_cloud_index = 0;
	serial_port_open = false;
	//point_cloud_right.resize(CALI_ROTATE_POS_CNT2 - 1);
	InitParameters();
	OpenOrCloseSerialPort();
	WriteByte(OpenDLP, 5);
	WriteByte(OpenDLP, 5);
	WriteByte(OpenDLP, 5);
	WriteByte(OpenDLP, 5);
	cout << "等待1秒。。。 " << endl;
	_sleep(1000);

	l_scan_x = 0.0;
	l_scan_y = 0.0;

	//scan_times = 0;
}

ControlThread::~ControlThread()
{
}

///*****************************2、串口初始化函数*******************************************/
void ControlThread::OpenOrCloseSerialPort()
{
	if (serial_port_open)
	{
		serial->clear();
		serial->close();
		serial->deleteLater();
		serial_port_open = false;
	}
	else
	{
		serial = new QSerialPort;
		serial->setPortName("COM2");
		serial->open(QIODevice::ReadWrite);
		serial->setBaudRate(QSerialPort::Baud9600);//设置波特率为115200
		serial->setDataBits(QSerialPort::Data8);//设置数据位8
		serial->setParity(QSerialPort::NoParity); //校验位设置为0
		serial->setStopBits(QSerialPort::OneStop);//停止位设置为1
		serial->setFlowControl(QSerialPort::NoFlowControl);//设置为无流控制
	}
}

void ControlThread::InitParameters()
{
	cout << " RasterScan Start!!" << endl;
	rs->InitRasterScan("D:/dentalimage/dentalimage2/SystemCalibration.yml");

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
}

void ControlThread::setFlage(bool flag)
{
	isStop = flag;
}

int ControlThread::ReadData(byte* RXBuff)
{
	int	RXsize = 0;
	if (serial->waitForReadyRead(10000)) {
		QByteArray responseData = serial->readAll();
		while (serial->waitForReadyRead(10))
			responseData += serial->readAll();

		RXBuff = (byte*)(responseData.data());
		RXsize = responseData.size();
	}
	else
	{
		cout << "Read wrong!!" << endl;
	}

	return RXsize;
}

void ControlThread::WriteByte(byte* szWriteBuffer, int number)
{
	QByteArray byArr;
	byArr.resize(number);
	for (int i = 0; i<number; i++) {
		byArr[i] = szWriteBuffer[i];
	}
	//serial->write(byArr);
	//if (serial->waitForBytesWritten(1000))
	//{
	//	cout << "write done !!" << endl;
	//}
	//else
	//{
	//	cout << "write false !!" << endl;
	//}
	//cout << " serial = " << 0 << endl;

	serial->write(byArr);
	if (serial->waitForBytesWritten(1000))
	{
		//cout << "write done !!" << endl;
	}
	else
	{
		//cout << "write false !!" << endl;
	}
}

//1.1 相机触发设置
int ControlThread::ConfigureTrigger(INodeMap & nodeMap, CameraPtr pCam)
{
	int result = 0;

	try
	{
		//
		// Ensure trigger mode off
		//
		// *** NOTES ***
		// The trigger must be disabled in order to configure whether the source
		// is software or hardware.
		//
		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

		//cout << "Trigger mode disabled..." << endl;

		//
		// Select trigger source
		//
		// *** NOTES ***
		// The trigger source must be set to hardware or software while trigger 
		// mode is off.
		//
		CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
		if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
		{
			cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		if (chosenTrigger == SOFTWARE)
		{
			// Set trigger mode to software
			CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
			if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
			{
				cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());

			cout << "Trigger source set to software..." << endl;
		}
		else if (chosenTrigger == HARDWARE)
		{
			// Set trigger mode to hardware ('Line0')
			CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
			if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
			{
				cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());

			//cout << "Trigger source set to hardware..." << endl;
		}

		//
		// Turn trigger mode on
		//
		// *** LATER ***
		// Once the appropriate trigger source has been set, turn trigger mode 
		// on in order to retrieve images using the trigger.
		//

		CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
		if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
		{
			cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());


		// Set acquisition mode to continuous  设置数据获取模式及开启数据获取
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
		// Begin acquiring images
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}
//1.2 复位触发
int ControlThread::ResetTrigger(INodeMap & nodeMap, CameraPtr pCam)
{
	int result = 0;

	try
	{
		// End acquisition  关闭数据获取
		pCam->EndAcquisition();

		//
		// Turn trigger mode back off
		//
		// *** NOTES ***
		// Once all images have been captured, turn trigger mode back off to
		// restore the camera to a clean state.
		//
		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			cout << "Unable to disable trigger mode (node retrieval). Non-fatal error..." << endl;
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			cout << "Unable to disable trigger mode (enum entry retrieval). Non-fatal error..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

		//cout << "Trigger mode disabled..." << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}
//1.3 触发后图像获取函数
int ControlThread::AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, gcstring deviceSerialNumber, int index, ImagePtr &convertedImage)
{
	int result = 0;

	try
	{
		// Retrieve the next received image
		ImagePtr pResultImage = pCam->GetNextImage();//硬触发主要用这一句进行数据获取
		if (pResultImage->IsIncomplete())
		{
			cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
		}
		else
		{
			// Convert image to mono 8
			convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
			//cout << "Image saved" << endl;
		}
		// Release image
		pResultImage->Release();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}
//1.4 设置相机图片规格
int ControlThread::ConfigureCustomImageSettings(INodeMap & nodeMap)
{
	int result = 0;

	//cout << endl << endl << "*** CONFIGURING CUSTOM IMAGE SETTINGS ***" << endl << endl;

	try
	{
		//1、设置像素格式为Mono8
		CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
		if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
		{
			// Retrieve the desired entry node from the enumeration node
			CEnumEntryPtr ptrPixelFormatMono8 = ptrPixelFormat->GetEntryByName("Mono8");
			if (IsAvailable(ptrPixelFormatMono8) && IsReadable(ptrPixelFormatMono8))
			{
				// Retrieve the integer value from the entry node
				int64_t pixelFormatMono8 = ptrPixelFormatMono8->GetValue();

				// Set integer as new value for enumeration node
				ptrPixelFormat->SetIntValue(pixelFormatMono8);

				//cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
			}
			else
			{
				cout << "Pixel format mono 8 not available..." << endl;
			}
		}
		else
		{
			cout << "Pixel format not available..." << endl;
		}
		//2、设置像素offset X
		CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
		if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX))
		{
			ptrOffsetX->SetValue(384);
			cout << "Offset X set to 384 ..." << endl;
		}
		else
		{
			cout << "Offset X not available..." << endl;
		}
		//3、设置像素offset Y
		CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
		if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY))
		{
			ptrOffsetY->SetValue(512);
			cout << "Offset Y set to 512 ..." << endl;
		}
		else
		{
			cout << "Offset Y not available..." << endl;
		}
		//3、设置像素宽
		CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
		if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
		{
			int64_t widthToSet = IMG_COL;

			ptrWidth->SetValue(widthToSet);

			cout << "Width set to" << IMG_COL << "..." << endl << endl;
		}
		else
		{
			cout << "Width not available..." << endl;
		}
		//3、设置像素长
		CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
		if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
		{
			int64_t heightToSet = IMG_ROW;

			ptrHeight->SetValue(heightToSet);

			cout << "Height set to " << IMG_ROW << "..." << endl << endl;
		}
		else
		{
			cout << "Height not available..." << endl << endl;
		}
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	return result;
}
//1.5 曝光时间设置
int ControlThread::ConfigureExposure(INodeMap & nodeMap)
{
	int result = 0;

	//cout << endl << endl << "*** CONFIGURING EXPOSURE ***" << endl << endl;

	try
	{
		//
		// Turn off automatic exposure mode
		//
		// *** NOTES ***
		// Automatic exposure prevents the manual configuration of exposure 
		// time and needs to be turned off.
		//
		// *** LATER ***
		// Exposure time can be set automatically or manually as needed. This
		// example turns automatic exposure off to set it manually and back
		// on in order to return the camera to its default state.
		//
		CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
		if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
		{
			cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
			return -1;
		}

		CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
		if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
		{
			cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
			return -1;
		}

		ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

		//cout << "Automatic exposure disabled..." << endl;

		//
		// Set exposure time manually; exposure time recorded in microseconds
		//
		// *** NOTES ***
		// The node is checked for availability and writability prior to the 
		// setting of the node. Further, it is ensured that the desired exposure 
		// time does not exceed the maximum. Exposure time is counted in 
		// microseconds. This information can be found out either by 
		// retrieving the unit with the GetUnit() method or by checking SpinView.
		// 
		CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
		if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
		{
			cout << "Unable to set exposure time. Aborting..." << endl << endl;
			return -1;
		}

		// Ensure desired exposure time does not exceed the maximum
		const double exposureTimeMax = ptrExposureTime->GetMax();
		double exposureTimeToSet = EXPOSURETIME;

		if (exposureTimeToSet > exposureTimeMax)
		{
			exposureTimeToSet = exposureTimeMax;
		}

		ptrExposureTime->SetValue(exposureTimeToSet);

		//cout << "Exposure time set to " << exposureTimeToSet << " us..." << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

/*****************************3、下位机动作函数*******************************************/
//3.1步进电机旋转一个角度 (1:x 2:y) //这里写一个函数，首先发送旋转指令，然后在函数里用延时等待旋转结束，然后return
bool ControlThread::SMY_RotOneDeg(double deg)
{
	byte RYBuff[128] = {};
	//下发命令
	uint deg_y = abs(deg) * 266.6666;
	if (deg > 0)
	{
		SMYRotOneDegNeg[6] = deg_y / 65536;
		SMYRotOneDegNeg[7] = (deg_y - (deg_y / 65536) * 65536) / 256;
		SMYRotOneDegNeg[8] = deg_y % 256;
		//cout << "电机X旋转..." << deg << endl;
		WriteByte(SMYRotOneDegNeg, 9);//转动角度
	}
	else if (deg < 0)
	{
		SMYRotOneDegPos[6] = deg_y / 65536;
		SMYRotOneDegPos[7] = (deg_y - (deg_y / 65536) * 65536) / 256;
		SMYRotOneDegPos[8] = deg_y % 256;
		//cout << "电机X旋转..." << deg << endl;
		WriteByte(SMYRotOneDegPos, 9);//转动角度
		
	}

	else
	{
		//cout << "电机X旋转..." << deg << endl;
		return 1;
	}
	_sleep(100);
	int BufferNum = ReadData(RYBuff);
	cout << RYBuff << endl;
	//_sleep(600);
	////接收命令
	//for (int k = 0; k < 100; k++)
	//{
	//	_sleep(100);
	//	
	//	int BufferNum = ReadData(RXBuff);
	//	if (BufferNum > 1)
	//	{
	//		//byte* Receive = ReceiveChar();//接收上行数据
	//		
	//		cout << "X接收上行数据..." << BufferNum << endl;
	//		cout << "..................................." << endl;
	//		for (size_t i = 0; i < BufferNum - 1; i++)
	//		{
	//			//cout << "第几次..." << i << endl;
	//			byte rx0 = RXBuff[0];
	//			byte rx1 = RXBuff[1];
	//			if (rx0 == 0xef && rx1 == 0x01)//根据接收数据返回调用结果
	//			{
	//				return 1;
	//			}
	//		}
	//	}
	//	else if (k == 99)
	//	{
	//		cout << "步进电机X启动运行错误..." << endl;
	//		return -1;
	//	}
	//	else
	//	{
	//		//WriteByte(RotateOneDeg, 3);//发送下行数据
	//	}
	//}
	return 1;
}
bool ControlThread::SMX_RotOneDeg(double deg)
{
	byte RXBuff[128] = {};
	//下发命令
	uint deg_x = abs(deg) * 355.5555;
	if (deg > 0)
	{
		SMXRotOneDegPos[6] = deg_x / 65536;
		SMXRotOneDegPos[7] = (deg_x - (deg_x / 65536) * 65536) / 256;
		SMXRotOneDegPos[8] = deg_x % 256;
		//cout << "电机Y旋转..." << deg << endl;
		WriteByte(SMXRotOneDegPos, 9);
	}
	else if (deg < 0)
	{
		SMXRotOneDegNeg[6] = deg_x / 65536;
		SMXRotOneDegNeg[7] = (deg_x - (deg_x / 65536) * 65536) / 256;
		SMXRotOneDegNeg[8] = deg_x % 256;
		//cout << "电机Y旋转..." << deg << endl;
		WriteByte(SMXRotOneDegNeg, 9);
	}
	else
	{
		//cout << "电机Y旋转..." << deg << endl;
		return 1;
	}
	_sleep(100);
	int BufferNum = ReadData(RXBuff);
	cout << RXBuff << endl;
	//_sleep(600);
	////接收命令
	//for (int k = 0; k < 100; k++)
	//{
	//	_sleep(100);
	//	int BufferNum = ReadData(RXBuff);
	//	if (BufferNum > 1)
	//	{
	//		//byte* Receive = ReceiveChar();//接收上行数据
	//		cout << "Y接收上行数据..." << BufferNum << endl;
	//		cout << "..................................." << endl;
	//		for (size_t i = 0; i < BufferNum - 1; i++)
	//		{
	//			//cout << "第几次..." << i << endl;
	//			byte rx0 = RXBuff[0];
	//			byte rx1 = RXBuff[1];
	//			if (rx0 == 0xef && rx1 == 0x02)//根据接收数据返回调用结果
	//			{
	//				return 1;
	//			}
	//		}
	//	}
	//	else if (k == 99)
	//	{
	//		cout << "步进电机Y启动运行错误..." << endl;
	//		return -1;
	//	}
	//	else
	//	{
	//		//WriteByte(RotateOneDeg, 3);//发送下行数据
	//	}
	//}
	return 1;
}
//3.2电机使能
void ControlThread::SM_Enable()
{
	WriteByte(SMXEnable, 5);//电机使能
	_sleep(100);
	WriteByte(SMYEnable, 5);//电机使能
	_sleep(100);
}
//3.3电机失能
void ControlThread::SM_Disable()
{
	WriteByte(SMXDisable, 5);//电机使能
	_sleep(100);
	WriteByte(SMYDisable, 5);//电机使能
	_sleep(100);
}
//3.4DLP投影一组标定图片
void ControlThread::DLP_ProjectOneSet_Cali(int caliCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set)
{
	///*********相机初始化操作*********/
	//1、获取相机列表及参数
	SystemPtr system = System::GetInstance();// Retrieve singleton reference to system object
	CameraList camList = system->GetCameras();// Retrieve list of cameras from the system
	unsigned int numCameras = camList.GetSize();
	//cout << "Number of cameras detected: " << numCameras << endl << endl;

	CameraPtr pCam_L, pCam_R;
	pCam_L = camList.GetByIndex(0);
	pCam_R = camList.GetByIndex(1);
	INodeMap & nodeMapTLDevice_L = pCam_L->GetTLDeviceNodeMap();
	INodeMap & nodeMapTLDevice_R = pCam_R->GetTLDeviceNodeMap();
	pCam_L->Init();
	pCam_R->Init();
	INodeMap & nodeMap_L = pCam_L->GetNodeMap();
	INodeMap & nodeMap_R = pCam_R->GetNodeMap();

	CStringPtr ptrStringSerial_L = nodeMapTLDevice_L.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_L = ptrStringSerial_L->GetValue();
	CStringPtr ptrStringSerial_R = nodeMapTLDevice_R.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_R = ptrStringSerial_R->GetValue();

	//2、图片格式设置
	ConfigureCustomImageSettings(nodeMap_L);
	ConfigureCustomImageSettings(nodeMap_R);

	//3、相机曝光设置
	ConfigureExposure(nodeMap_L);
	ConfigureExposure(nodeMap_R);

	//4、相机初始化及触发设置
	ConfigureTrigger(nodeMap_L, pCam_L);
	ConfigureTrigger(nodeMap_R, pCam_R);
	//cout << "start acquisition !!" << endl;
	pCam_L->BeginAcquisition();
	pCam_R->BeginAcquisition();

	//2.1投影及触发一组
	//cout << "开始一组标定投影。。 " << endl;
	switch (SM_CALI_BRIGHTNESS[caliCnt])
	{
	case 1:
		WriteByte(ProjectOneSet_CALI_MIN, 11);
		break;
	case 2:
		WriteByte(ProjectOneSet_CALI_MID, 11);
		break;
	case 3:
		WriteByte(ProjectOneSet_CALI_MAX, 11);
		break;
	default:
		break;
	}
	////2.2一组投影采集，获取图片

	for (size_t i = 0; i < 31; i++)
	{
		ImagePtr img_L, img_R;
		cv::Mat mat_L, mat_R;
		AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, 0, img_L);
		AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, 0, img_R);
		mat_L = cv::Mat::zeros(1024, 1280, CV_8UC1);
		mat_R = cv::Mat::zeros(1024, 1280, CV_8UC1);
		memcpy(mat_L.data, img_L->GetData(), img_L->GetImageSize());
		memcpy(mat_R.data, img_R->GetData(), img_R->GetImageSize());
		img_L_set.push_back(mat_L);
		img_R_set.push_back(mat_R);
	}
	//cout << "end acquisition !!" << endl;
	/*****************************************************************************************/
	ResetTrigger(nodeMap_L, pCam_L);// Reset trigger
	ResetTrigger(nodeMap_R, pCam_R);

	pCam_L->DeInit();// Deinitialize camera
	pCam_R->DeInit();
	/*****************************************************************************************/
	// Clear camera list before releasing system
	camList.Clear();
}
//3.5DLP投影一组扫描图片
void ControlThread::DLP_ProjectOneSet_Scan(int scanCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set)
{
	///*********相机初始化操作*********/
	//1、获取相机列表及参数
	SystemPtr system = System::GetInstance();// Retrieve singleton reference to system object
	CameraList camList = system->GetCameras();// Retrieve list of cameras from the system
	unsigned int numCameras = camList.GetSize();
	//cout << "Number of cameras detected: " << numCameras << endl << endl;

	CameraPtr pCam_L, pCam_R;
	pCam_L = camList.GetByIndex(0);
	pCam_R = camList.GetByIndex(1);
	INodeMap & nodeMapTLDevice_L = pCam_L->GetTLDeviceNodeMap();
	INodeMap & nodeMapTLDevice_R = pCam_R->GetTLDeviceNodeMap();
	pCam_L->Init();
	pCam_R->Init();
	INodeMap & nodeMap_L = pCam_L->GetNodeMap();
	INodeMap & nodeMap_R = pCam_R->GetNodeMap();

	CStringPtr ptrStringSerial_L = nodeMapTLDevice_L.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_L = ptrStringSerial_L->GetValue();
	CStringPtr ptrStringSerial_R = nodeMapTLDevice_R.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_R = ptrStringSerial_R->GetValue();

	//2、图片格式设置
	ConfigureCustomImageSettings(nodeMap_L);
	ConfigureCustomImageSettings(nodeMap_R);

	//3、相机曝光设置
	ConfigureExposure(nodeMap_L);
	ConfigureExposure(nodeMap_R);

	//4、相机初始化及触发设置
	ConfigureTrigger(nodeMap_L, pCam_L);
	ConfigureTrigger(nodeMap_R, pCam_R);

	//cout << "start acquisition !!" << endl;
	pCam_L->BeginAcquisition();
	pCam_R->BeginAcquisition();

	//2.1投影及触发一组
	//cout << "开始一组扫描投影。。 " << endl;
	switch (SM_SCAN_BRIGHTNESS[scanCnt])
	{
	case 1:
		WriteByte(ProjectOneSet_SCAN_MIN, 11);
		break;
	case 2:
		WriteByte(ProjectOneSet_SCAN_MID, 11);
		break;
	case 3:
		WriteByte(ProjectOneSet_SCAN_MAX, 11);
		break;
	default:
		break;
	}
	//cout << "serical write done !!" << endl;

	for (size_t i = 0; i < 19; i++)
	{
		ImagePtr img_L, img_R;
		cv::Mat mat_L, mat_R;
		AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, 0, img_L);
		AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, 0, img_R);
		mat_L = cv::Mat::zeros(1024, 1280, CV_8UC1);
		mat_R = cv::Mat::zeros(1024, 1280, CV_8UC1);
		memcpy(mat_L.data, img_L->GetData(), img_L->GetImageSize());
		memcpy(mat_R.data, img_R->GetData(), img_R->GetImageSize());
		img_L_set.push_back(mat_L);
		img_R_set.push_back(mat_R);
	}

	/*****************************************************************************************/
	ResetTrigger(nodeMap_L, pCam_L);// Reset trigger
	ResetTrigger(nodeMap_R, pCam_R);

	pCam_L->DeInit();// Deinitialize camera
	pCam_R->DeInit();
	/*****************************************************************************************/
	// Clear camera list before releasing system
	camList.Clear();
}

void ControlThread::DLP_ProjectOneSet_Global(int scanCnt, vector<cv::Mat> &img_L_set, vector<cv::Mat> &img_R_set)
{
	///*********相机初始化操作*********/
	//1、获取相机列表及参数
	SystemPtr system = System::GetInstance();// Retrieve singleton reference to system object
	CameraList camList = system->GetCameras();// Retrieve list of cameras from the system
	unsigned int numCameras = camList.GetSize();
	//cout << "Number of cameras detected: " << numCameras << endl << endl;

	CameraPtr pCam_L, pCam_R;
	pCam_L = camList.GetByIndex(0);
	pCam_R = camList.GetByIndex(1);
	INodeMap & nodeMapTLDevice_L = pCam_L->GetTLDeviceNodeMap();
	INodeMap & nodeMapTLDevice_R = pCam_R->GetTLDeviceNodeMap();
	pCam_L->Init();
	pCam_R->Init();
	INodeMap & nodeMap_L = pCam_L->GetNodeMap();
	INodeMap & nodeMap_R = pCam_R->GetNodeMap();

	CStringPtr ptrStringSerial_L = nodeMapTLDevice_L.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_L = ptrStringSerial_L->GetValue();
	CStringPtr ptrStringSerial_R = nodeMapTLDevice_R.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_R = ptrStringSerial_R->GetValue();

	//2、图片格式设置
	ConfigureCustomImageSettings(nodeMap_L);
	ConfigureCustomImageSettings(nodeMap_R);

	//3、相机曝光设置
	ConfigureExposure(nodeMap_L);
	ConfigureExposure(nodeMap_R);

	//4、相机初始化及触发设置
	ConfigureTrigger(nodeMap_L, pCam_L);
	ConfigureTrigger(nodeMap_R, pCam_R);

	//2.1投影及触发一组
	//cout << "开始一组扫描投影。。 " << endl;
	switch (SM_SCAN_BRIGHTNESS[scanCnt])
	{
	case 1:
		WriteByte(ProjectOneSet_SCAN_MIN, 11);
		break;
	case 2:
		WriteByte(ProjectOneSet_SCAN_MID, 11);
		break;
	case 3:
		WriteByte(ProjectOneSet_SCAN_MAX, 11);
		break;
	default:
		break;
	}
	////2.2一组投影采集，获取图片
	//for (size_t i = 0; i < 19; i++)
	//{
	//	ImagePtr img_L, img_R;
	//	AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, i, img_L);
	//	AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, i, img_R);

	//	img_L_set.push_back(img_L);
	//	img_R_set.push_back(img_R);
	//}
	//cout << "start acquisition !!" << endl;
	pCam_L->BeginAcquisition();
	pCam_R->BeginAcquisition();
	for (size_t i = 0; i < 19; i++)
	{
		ImagePtr img_L, img_R;
		cv::Mat mat_L, mat_R;
		AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, 0, img_L);
		AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, 0, img_R);
		mat_L = cv::Mat::zeros(1024, 1280, CV_8UC1);
		mat_R = cv::Mat::zeros(1024, 1280, CV_8UC1);
		memcpy(mat_L.data, img_L->GetData(), img_L->GetImageSize());
		memcpy(mat_R.data, img_R->GetData(), img_R->GetImageSize());
		img_L_set.push_back(mat_L);
		img_R_set.push_back(mat_R);
	}


	//for (size_t i = 0; i < 19; i++)
	//{
	//	ImagePtr img_L, img_R;
	//	cv::Mat mat_L, mat_R;
	//	double image_mean = 0;
	//	do
	//	{
	//		AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, i, img_L);
	//		mat_L = cv::Mat::zeros(1024, 1280, CV_8UC1);
	//		memcpy(mat_L.data, img_L->GetData(), img_L->GetImageSize());
	//		image_mean = cal_mean_stddev(mat_L);
	//		cout << "                                                                         image_mean = " << image_mean << endl;
	//	} while (image_mean<8);
	//	image_mean = 0;
	//	do
	//	{
	//		AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, i, img_R);
	//		mat_R = cv::Mat::zeros(1024, 1280, CV_8UC1);
	//		memcpy(mat_R.data, img_R->GetData(), img_R->GetImageSize());
	//		image_mean = cal_mean_stddev(mat_R);
	//		cout << "                                                                         image_mean = " << image_mean << endl;
	//	} while (image_mean<8);

	//	img_L_set.push_back(mat_L);
	//	img_R_set.push_back(mat_R);
	//}

	/*****************************************************************************************/
	ResetTrigger(nodeMap_L, pCam_L);// Reset trigger
	ResetTrigger(nodeMap_R, pCam_R);

	pCam_L->DeInit();// Deinitialize camera
	pCam_R->DeInit();
	/*****************************************************************************************/
	// Clear camera list before releasing system
	camList.Clear();
}

void ControlThread::getfirstimage()
{
	///*********相机初始化操作*********/
	//1、获取相机列表及参数
	SystemPtr system = System::GetInstance();// Retrieve singleton reference to system object
	CameraList camList = system->GetCameras();// Retrieve list of cameras from the system
	unsigned int numCameras = camList.GetSize();
	//cout << "Number of cameras detected: " << numCameras << endl << endl;

	CameraPtr pCam_L, pCam_R;
	pCam_L = camList.GetByIndex(0);
	pCam_R = camList.GetByIndex(1);
	INodeMap & nodeMapTLDevice_L = pCam_L->GetTLDeviceNodeMap();
	INodeMap & nodeMapTLDevice_R = pCam_R->GetTLDeviceNodeMap();
	pCam_L->Init();
	pCam_R->Init();
	INodeMap & nodeMap_L = pCam_L->GetNodeMap();
	INodeMap & nodeMap_R = pCam_R->GetNodeMap();

	CStringPtr ptrStringSerial_L = nodeMapTLDevice_L.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_L = ptrStringSerial_L->GetValue();
	CStringPtr ptrStringSerial_R = nodeMapTLDevice_R.GetNode("DeviceSerialNumber");
	gcstring deviceSerialNumber_R = ptrStringSerial_R->GetValue();

	//2、图片格式设置
	ConfigureCustomImageSettings(nodeMap_L);
	ConfigureCustomImageSettings(nodeMap_R);

	//3、相机曝光设置
	ConfigureExposure(nodeMap_L);
	ConfigureExposure(nodeMap_R);

	//4、相机初始化及触发设置
	ConfigureTrigger(nodeMap_L, pCam_L);
	ConfigureTrigger(nodeMap_R, pCam_R);


	//ImagePtr img_L, img_R;
	//AcquireImages(pCam_L, nodeMap_L, nodeMapTLDevice_L, deviceSerialNumber_L, 0, img_L);
	//AcquireImages(pCam_R, nodeMap_R, nodeMapTLDevice_R, deviceSerialNumber_R, 0, img_R);

	/*****************************************************************************************/
	ResetTrigger(nodeMap_L, pCam_L);// Reset trigger
	ResetTrigger(nodeMap_R, pCam_R);

	pCam_L->DeInit();// Deinitialize camera
	pCam_R->DeInit();
	/*****************************************************************************************/
	// Clear camera list before releasing system
	camList.Clear();

	//for (int image_index = 1; image_index < 16; image_index++)
	//{
	//	cv::Mat mat_L = cv::Mat(1024, 1280, CV_8UC1);
	//	memcpy(mat_L.data, imgR_set[image_index]->GetData(), imgR_set[image_index]->GetImageSize());
	//	images_l.push_back(mat_L);
	//	ostringstream filename_L;
	//	filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";
	//	cv::imwrite(filename_L.str().c_str(), mat_L);

	//	cv::Mat mat_R = cv::Mat(1024, 1280, CV_8UC1);
	//	memcpy(mat_R.data, imgL_set[image_index]->GetData(), imgL_set[image_index]->GetImageSize());
	//	images_r.push_back(mat_R);
	//	ostringstream filename_R;
	//	filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";
	//	cv::imwrite(filename_R.str().c_str(), mat_R);
	//}
}

void ControlThread::controlNormalScan()
{
	vector<cv::Mat> image_groups_left, image_groups_right;
	int imageSize = IMG_ROW * IMG_COL;
	int bufferBias = 0;
	////1、开启及配置DLP
	SM_Enable();//电机使能
	clock_t time1, time2, time3, time4, time5, time6;
	for (int scan_index = 0; scan_index < SCAN_ROTATE_POS_CNT2; scan_index++)
	{
		double d_scan_x = 0.0;
		double d_scan_y = 0.0;
		c_scan_x = SMX_SCAN_ROTATE_DEGREE2[scan_index];
		c_scan_y = SMY_SCAN_ROTATE_DEGREE2[scan_index];

		d_scan_x = (c_scan_x - l_scan_x);
		d_scan_y = (c_scan_y - l_scan_y);
		//double d_scan_z = (c_scan_z - l_scan_z);
		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;
		//2、电机旋转
		time1 = clock();
		SMX_RotOneDeg(d_scan_x);
		_sleep(100);
		SMY_RotOneDeg(d_scan_y);
		_sleep(100);
		time2 = clock();

		if (scan_index == SCAN_ROTATE_POS_CNT2 - 1)
		{
			//3、关闭DLP
			SM_Disable();//电机失能
			continue;
		}
		freeSpace.acquire();
		//cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;
		//l_scan_z = c_scan_z;
		if (c_scan_x<-120 || c_scan_x>120)
		{
			return;
		}

		vector<cv::Mat> imgL_set, imgR_set;
		///**************************************扫描过程*****************************************/
		//开始投影多组光栅
		time3 = clock();
		DLP_ProjectOneSet_Scan(SM_SCAN_BRIGHTNESS[scan_index], imgL_set, imgR_set);
		time4 = clock();
		//cout << "开始存图片。。 " << endl;

		vector<cv::Mat> images_l, images_r;
		vector<cv::Mat> image_rgb;
		//unsigned char* im_l = 0;
		//unsigned char* im_r = 0;
		//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
		int imageBias = 0;
		time5 = clock();
		for (int image_index = 0; image_index < 19; image_index++)
		{
			
			ostringstream filename_L;
			filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "L" << ".png";
			cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);


			ostringstream filename_R;
			cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
			filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << scan_index << "_" << image_index << "_" << "R" << ".png";
			cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);
			if (image_index == 0)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}

			if (image_index>0 && image_index<16)
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
			else if (image_index>=16)
			{
				memcpy(totalNormalScanImageBuffer + bufferBias * 34 * imageSize + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
				imageBias++;
			}
		}
		time6 = clock();
		bufferBias++;
		usedSpace.release();
		cout << "The ControlThread: " << scan_index << " has finished." << endl;

		cout << "The rotation time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		cout << "The projection time is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
		cout << "The memcpy time is " << (double)(time6 - time5) / CLOCKS_PER_SEC << " s;" << endl;
	}

	//3、关闭DLP
	SM_Disable();//电机失能
				 //WriteByte(CloseDLP, 5);
	cout << "关闭DLP。。 " << endl;
}

void ControlThread::controlCalibrationScan()
{
	vector<vector<cv::Mat>> image_groups;
	for (int scan_index = 0; scan_index < CALI_ROTATE_POS_CNT2; scan_index++)
	{
		c_scan_x = SMX_CALI_ROTATE_DEGREE2[scan_index];
		c_scan_y = SMY_CALI_ROTATE_DEGREE2[scan_index];

		double d_scan_x = (c_scan_x - l_scan_x);
		double d_scan_y = (c_scan_y - l_scan_y);
		//double d_scan_z = (c_scan_z - l_scan_z);
		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;

		cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;
		//l_scan_z = c_scan_z;
		if (c_scan_x<-120 || c_scan_x>120)
		{
			return;
		}

		vector<cv::Mat> imgL_set, imgR_set;
		cout << "******************************************************************" << endl;
		cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;

		///**************************************扫描过程*****************************************/
		////1、开启及配置DLP
		SM_Enable();//电机使能

					//2、开始投影多组光栅
		SMX_RotOneDeg(d_scan_x);
		_sleep(100);
		SMY_RotOneDeg(d_scan_y);
		_sleep(100);

		if (scan_index == CALI_ROTATE_POS_CNT2 - 1)
		{
			//3、关闭DLP
			SM_Disable();//电机失能
							//WriteByte(CloseDLP, 5);
			continue;
		}

		DLP_ProjectOneSet_Cali(SM_CALI_BRIGHTNESS2[scan_index], imgL_set, imgR_set);
		//存进总数组

		cout << "开始存图片。。 " << endl;
		vector<cv::Mat> group;
		for (size_t j = 0; j < 31; j++)
		{
			group.push_back(imgL_set[j]);
			if (j == 0)
			{
				calibImageCamera.push_back(imgL_set[j]);
			}
			
			ostringstream filename_L;
			filename_L << "D:\\dentalimage\\dentalimage2\\CaliPic\\" << scan_index << "_" << j << "_" << "L" << ".png";
			cv::imwrite(filename_L.str().c_str(), imgL_set[j]);
		}
		for (size_t j = 0; j < 31; j++)
		{
			cv::flip(imgR_set[j], imgR_set[j], -1);
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
	SM_Disable();//电机失能
					//WriteByte(CloseDLP, 5);
					//cout << "关闭DLP。。 " << endl;
					//cout << "开始存图片。。 " << endl;
					/*****************************************************************************************/

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
	rs->SystemCalibration("D:/dentalimage/dentalimage2/SystemCalibration.yml", error);
	cout << error << endl;
	InitParameters();
}

void ControlThread::controlGlobalCaliScan()
{
	vector<Mat> image_groups_left, image_groups_right;
	for (int scan_index = 0; scan_index < SCAN_ROTATE_POS_CNT2; scan_index++)
	{
		c_scan_x = SMX_SCAN_ROTATE_DEGREE2[scan_index];
		c_scan_y = SMY_SCAN_ROTATE_DEGREE2[scan_index];

		double d_scan_x = (c_scan_x - l_scan_x);
		double d_scan_y = (c_scan_y - l_scan_y);
		//double d_scan_z = (c_scan_z - l_scan_z);
		l_scan_x = c_scan_x;
		l_scan_y = c_scan_y;

		cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;
		//l_scan_z = c_scan_z;
		if (c_scan_x<-120 || c_scan_x>120)
		{
			return;
		}

		vector<cv::Mat> imgL_set, imgR_set;
		cout << "******************************************************************" << endl;
		cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;

		///**************************************扫描过程*****************************************/
		////1、开启及配置DLP
		SM_Enable();//电机使能

					//2、开始投影多组光栅
		SMX_RotOneDeg(d_scan_x);
		_sleep(100);
		SMY_RotOneDeg(d_scan_y);
		_sleep(100);

		if (scan_index == SCAN_ROTATE_POS_CNT2 - 1)
		{
			//3、关闭DLP
			SM_Disable();//电机失能
						 //WriteByte(CloseDLP, 5);
			continue;
		}

		DLP_ProjectOneSet_Global(SM_CALI_BRIGHTNESS2[scan_index], imgL_set, imgR_set);
		//存进总数组

		cout << "开始存图片。。 " << endl;

		cv::flip(imgR_set[0], imgR_set[0], -1);
		image_groups_left.push_back(imgL_set[0]);
		ostringstream filename_L;
		filename_L << "D:\\dentalimage\\dentalimage2\\GloPic\\" << scan_index << "_0_" << "L" << ".png";
		cv::imwrite(filename_L.str().c_str(), imgL_set[0]);

		image_groups_right.push_back(imgR_set[0]);
		ostringstream filename_R;
		filename_R << "D:\\dentalimage\\dentalimage2\\GloPic\\" << scan_index << "_0_" << "R" << ".png";
		cv::imwrite(filename_R.str().c_str(), imgR_set[0]);

		//4、保存图片
		cout << "标定图片存储完毕。。 " << endl;

	}

	////3、关闭DLP
	SM_Disable();//电机失能
				 //WriteByte(CloseDLP, 5);
				 //cout << "关闭DLP。。 " << endl;
				 //cout << "开始存图片。。 " << endl;
				 /*****************************************************************************************/

	vector<double> mask_points;
	rs->PlaneRTCalculate(image_groups_left, image_groups_right, "D:/dentalimage/dentalimage2/external_parameter.yml", mask_points);
	//ColoredPoints(mask_points, 2);
	InitParameters();
}

void ControlThread::compensationControlScan()
{
	freeSpace.acquire();
	int imageSize = IMG_ROW * IMG_COL;
	double d_scan_x = (c_scan_x - l_scan_x);
	double d_scan_y = (c_scan_y - l_scan_y);
	//double d_scan_z = (c_scan_z - l_scan_z);
	l_scan_x = 0;
	l_scan_y = 0;

	cout << "l_scan_x = " << l_scan_x << "; l_scan_y = " << l_scan_y << endl;

	vector<cv::Mat> imgL_set, imgR_set;
	cout << "******************************************************************" << endl;
	cout << "SM Rot。。。 x = " << d_scan_x << " , y = " << d_scan_y << /*" , z = " << d_scan_z <<*/ endl;

	///**************************************扫描过程*****************************************/
	
	////1、开启及配置DLP
	SM_Enable();//电机使能

	SMX_RotOneDeg(d_scan_x);
	_sleep(100);
	SMY_RotOneDeg(d_scan_y);
	_sleep(100);
	DLP_ProjectOneSet_Scan(2, imgL_set, imgR_set);
	//存进总数组
	vector<cv::Mat> images_l, images_r;
	vector<cv::Mat> image_rgb;
	//unsigned char* im_l = 0;
	//unsigned char* im_r = 0;
	//im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
	//im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
	int imageBias = 0;
	for (int image_index = 0; image_index < 19; image_index++)
	{
		
		ostringstream filename_L;
		filename_L << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << points_cloud_globle.size() << "_" << image_index << "_" << "L" << ".png";
		cv::imwrite(filename_L.str().c_str(), imgL_set[image_index]);

		cv::flip(imgR_set[image_index], imgR_set[image_index], -1);
		ostringstream filename_R;
		filename_R << "D:\\dentalimage\\dentalimage2\\ScanPic\\" << points_cloud_globle.size() << "_" << image_index << "_" << "R" << ".png";
		cv::imwrite(filename_R.str().c_str(), imgR_set[image_index]);

		if (image_index == 0)
		{
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;
		}

		if (image_index>0 && image_index<16)
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
		else if (image_index >= 16)
		{
			memcpy(totalNormalScanImageBuffer + imageBias * imageSize, (unsigned char*)imgR_set[image_index].data, imageSize * sizeof(unsigned char));
			imageBias++;
		}
	}
	SMX_RotOneDeg(-d_scan_x);
	_sleep(100);
	SMY_RotOneDeg(-d_scan_y);
	_sleep(100);
	//3、关闭DLP
	SM_Disable();//电机失能
				 //WriteByte(CloseDLP, 5);
	cout << "关闭DLP。。 " << endl;
	usedSpace.release();
}

