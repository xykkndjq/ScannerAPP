#include "ScanMainGUI.h"

ScanMainGUI::ScanMainGUI(QWidget *parent)
	: QWidget(parent)
{
	
	this->initVariable();
	this->constructIHM();
	//this->setConnections();
	ui.setupUi(this);
	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(pushBtnClicked()));
	connect(ui.frontviewBtn, SIGNAL(clicked()), this, SLOT(fontviewBtnClicked()));
	connect(ui.rightViewBtn, SIGNAL(clicked()), this, SLOT(rightViewBtnClicked()));
	connect(ui.leftViewBtn, SIGNAL(clicked()), this, SLOT(leftViewBtnClicked()));
	connect(ui.backViewBtn, SIGNAL(clicked()), this, SLOT(backViewBtnClicked()));
	connect(ui.jawViewBtn, SIGNAL(clicked()), this, SLOT(jawViewBtnClicked()));
	connect(ui.narrowViewBtn, SIGNAL(clicked()), this, SLOT(narrowViewBtnClicked()));
	connect(ui.enlargeViewBtn, SIGNAL(clicked()), this, SLOT(enlargeViewBtnClicked()));
	connect(ui.modelMoveStateSetBtn, SIGNAL(clicked()), this, SLOT(modelMoveStateSetBtnClicked()));
	connect(ui.delSelectedBtn, SIGNAL(clicked()), this, SLOT(delSelectedBtnClicked()));
	connect(ui.confirmSelectedBtn, SIGNAL(clicked()), this, SLOT(delSelectedBtnClicked()));
	
	
}

ScanMainGUI::~ScanMainGUI()
{
	if (controlScanQThread->isRunning())
	{
		controlScanQThread->quit();
	}
	controlScanQThread->wait();
	
	if (controlComputeQThread->isRunning())
	{
		controlComputeQThread->quit();
	}
	controlComputeQThread->wait();
}


void ScanMainGUI::initVariable()
{
	glWidget = new GLWidget(this);
	//tabMainPage = new TabMainGUI();
	//tabMainPage->showMaximized();

	//scanTipWidget = new ScanTipWidget(this);

	//初始化线程
	//ControlScanThread = new ControlThread;
	//ControlComputeThread = new ComputeThread;

// 	cv::Mat model_matrix = cv::Mat::eye(4, 4, CV_64FC1);
// 	cv::Mat view_matrix = rt_r/*.inv()*/;
// 	glWidget->SetMatrix(model_matrix, view_matrix);
// 
// 	//创建子线程对象
// 	controlScanQThread = new QThread(this);
// 	controlComputeQThread = new QThread(this);
// 	//把子线程（对象）添加到自定义的线程类（对象）中
// 	ControlScanThread->moveToThread(controlScanQThread);
// 	ControlComputeThread->moveToThread(controlComputeQThread);
// 
// 	//cameraWindow
// 	cameraWindow = new QDockWidget(QStringLiteral("相机显示"), this);
// 	cameraImageLabel = new QLabel(cameraWindow);
// 	cameraImageLabel->setPixmap(QPixmap(":/MainWidget/Resources/images/alljaw.png"));
// 	cameraImageLabel->setScaledContents(true);
// 
// 	autoTunePushButton = new QPushButton(QStringLiteral("自动调节"),cameraWindow);
// 	spinCameraBox = new QSpinBox(cameraWindow);
// 	sliderCamera = new QSlider(Qt::Horizontal, cameraWindow);
// 	spinCameraBox->setRange(0, 130);
// 	sliderCamera->setRange(0, 130);
// 
// 	//buttontoolButtons
// 	leftWatchButton = new QToolButton();
// 	rightWatchButton = new QToolButton();
// 	topWatchButton = new QToolButton();
// 	bottomWatchButton = new QToolButton();
// 	frontWatchButton = new QToolButton();
// 	backWatchButton = new QToolButton();
}

void ScanMainGUI::constructIHM()
{
	//glWidget->showMaximized();
	glWidget->setGeometry(0,0,1920,1080);
	glWidget->setWindowSize(QSize(1920, 1080));
	glWidget->showFullScreen();
	return;
	//相机设置
	cameraWindow->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable); // 设置停靠窗口特性，可移动,可关闭
	cameraWindow->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);//设置可停靠区域为主窗口左边和右边
	cameraWindow->setGeometry(0, 0, 350, 400);
	
	cameraImageLabel->setGeometry(0,20,350,370);
	sliderCamera->setGeometry(0, 371, 200, 20);
	spinCameraBox->setGeometry(201, 371, 60, 20);
	autoTunePushButton->setGeometry(262,371,85,20);
	
	//cameraWindow->setContentsMargins(0, 0, 0, 0);
	
	//BottomTool底部工具栏
	leftWatchButton->setFixedSize(40,40);
	leftWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/LeftView.png"));
	leftWatchButton->setIconSize(QSize(leftWatchButton->width(), leftWatchButton->height()));
	leftWatchButton->setStyleSheet("border-style:flat");

	rightWatchButton->setFixedSize(40, 40);
	rightWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/RightView.png"));
	rightWatchButton->setIconSize(QSize(rightWatchButton->width(), rightWatchButton->height()));
	rightWatchButton->setStyleSheet("border-style:flat");

	topWatchButton->setFixedSize(40, 40);
	topWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/TopView.png"));
	topWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	topWatchButton->setStyleSheet("border-style:flat");

	bottomWatchButton->setFixedSize(40, 40);
	bottomWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BottomView.png"));
	bottomWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	bottomWatchButton->setStyleSheet("border-style:flat");

	frontWatchButton->setFixedSize(40, 40);
	frontWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/FrontView.png"));
	frontWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	frontWatchButton->setStyleSheet("border-style:flat");

	backWatchButton->setFixedSize(40, 40);
	backWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BackView.png"));
	backWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	backWatchButton->setStyleSheet("border-style:flat");
	
	QWidget *bottomWidget = new QWidget(this);
	QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
	QWidget *bottomButtonWidget = new QWidget(bottomWidget);
	bottomButtonWidget->setFixedSize(300, 100);
	QHBoxLayout *bottomButtonHLayout = new QHBoxLayout(bottomButtonWidget);

	bottomButtonHLayout->addWidget(topWatchButton);
	bottomButtonHLayout->addWidget(bottomWatchButton);
	bottomButtonHLayout->addWidget(frontWatchButton);
	bottomButtonHLayout->addWidget(backWatchButton);
	bottomButtonHLayout->addWidget(leftWatchButton);
	bottomButtonHLayout->addWidget(rightWatchButton);
	
	
	bottomHLayout->addStretch();
	bottomHLayout->addWidget(bottomButtonWidget);
	bottomHLayout->addStretch();

	bottomWidget->setGeometry(0, 900, 1920, 100);

	//提示窗口
	scanTipWidget->setFixedSize(300,700);
	scanTipWidget->setGeometry(1615,80,300,700);
	scanTipWidget->setVisible(false);
}

void ScanMainGUI::setConnections()
{
	QObject::connect(sliderCamera, &QSlider::valueChanged, spinCameraBox, &QSpinBox::setValue);
	void (QSpinBox:: *spinBoxSignal)(int) = &QSpinBox::valueChanged;
	QObject::connect(spinCameraBox, spinBoxSignal, sliderCamera, &QSlider::setValue);
	spinCameraBox->setValue(35);

	//扫描
	//connect(tabMainPage->scanButton, SIGNAL(clicked()), this, SLOT(CalculatePointCloud()));
	connect(tabMainPage, SIGNAL(scanDataSignal(QJsonObject)), this, SLOT(doScanDialogSlot(QJsonObject)));
	connect(scanTipWidget->forwardStepButton,SIGNAL(clicked()),this,SLOT(judgeForwardStep()));//正常扫描
	connect(ControlComputeThread, SIGNAL(computeFinish()), this, SLOT(JawScan()));
	connect(scanTipWidget->compensationButton, SIGNAL(clicked()), this, SLOT(compensationScan()));
	
	connect(this, SIGNAL(compensationSignal(int)), ControlScanThread, SLOT(compensationControlScan()));
	connect(this, SIGNAL(compensationSignal(int)), ControlComputeThread, SLOT(compensationComputeScan(int)));
	connect(this, SIGNAL(startControlNormalScan(int)), ControlScanThread, SLOT(controlNormalScan()));
	connect(this, SIGNAL(startControlNormalScan(int)), ControlComputeThread, SLOT(controlComputeScan(int)));

	connect(ControlComputeThread, SIGNAL(showModeltoGlSingel(int)),this, SLOT(updateMeshModel(int)));
	//Calibrate标定子页面
	connect(tabMainPage->calibratePushButton, SIGNAL(clicked()), this, SLOT(ToothCalibrateSlot()));
	connect(tabMainPage->globalCaliPushButton, SIGNAL(clicked()), this, SLOT(GlobalCalibrateSlot()));
}

void ScanMainGUI::ToothCalibrateSlot()
{
	ControlScanThread->controlCalibrationScan();
}

void ScanMainGUI::GlobalCalibrateSlot()
{
	ControlScanThread->controlGlobalCaliScan();
}

void ScanMainGUI::CalculatePointCloud()
{
	if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
	{
		emit startControlNormalScan(chooseJawIndex);
		return;
	}
	if (controlScanQThread->isRunning() == false)
	{
		//启动子线程，但没有启动线程处理函数
		controlScanQThread->start();
		ControlScanThread->setFlage(false);
	}
	if (controlComputeQThread->isRunning() == false)
	{
		//启动子线程，但没有启动线程处理函数
		controlComputeQThread->start();
		ControlComputeThread->setFlage(false);
	}
	emit startControlNormalScan(chooseJawIndex);
}

void ScanMainGUI::updateMeshModel(int refreshIndex)
{
	if (refreshIndex == 1)
	{
		glWidget->vertData.clear();
		glWidget->totalFaceNum = 0;
	}
	
	if (chooseJawIndex == 1)
	{
		int scan_index = ControlComputeThread->upper_mModel.size() - 1;
		glWidget->mm = ControlComputeThread->upper_mModel[scan_index];
	}
	else if (chooseJawIndex == 2)
	{
		int scan_index = ControlComputeThread->lower_mModel.size() - 1;
		glWidget->mm = ControlComputeThread->lower_mModel[scan_index];
	}
	else if (chooseJawIndex == 3)
	{
		int scan_index = ControlComputeThread->all_mModel.size() - 1;
		glWidget->mm = ControlComputeThread->all_mModel[scan_index];
	}

	glWidget->makeObject();
	this->showMaximized();
}

void ScanMainGUI::doScanDialogSlot(QJsonObject scanObj)
{
	scanTipWidget->setVisible(true);
	cout <<"caseType"<< scanObj.value("caseType").toInt() << endl;
	cout << "upperJaw" << scanObj.value("upperJaw").toInt() << endl;
	if (scanObj.value("caseType").toInt() == 1)
	{
		if (scanObj.value("upperJaw").toInt() == 1)
		{
			scanTipWidget->upperPlaceConstructIHM1();
			globalTipIndex = 1;
			forwardIndex = 1;
			chooseJawIndex = 1;
		}
		else if (scanObj.value("lowerJaw").toInt() == 1)
		{
			scanTipWidget->lowerPlaceConstructIHM1();
			globalTipIndex = 2;
			forwardIndex = 1;
			chooseJawIndex = 2;
		}
		else if (scanObj.value("allJaw").toInt() == 1)
		{
			scanTipWidget->allPlaceConstructIHM1();
			globalTipIndex = 3;
			forwardIndex = 1;
			chooseJawIndex = 3;
		}
	}
	else if (scanObj.value("caseType").toInt() == 2)
	{

	}
	this->showMaximized();
	scanTipWidget->showMaximized();
}

void ScanMainGUI::judgeForwardStep()
{
	scanTipWidget->setVisible(false);
	cout <<"forwardIndex: "<< forwardIndex << endl;
	if (forwardIndex == 1)
	{
		CalculatePointCloud();
	}
	else if (forwardIndex == 2)
	{
		if (allJawIndex != 6)
		{
			JawScan();
		}
		else if (allJawIndex == 6)
		{
			//上颌下颌全颌配准
			JawScan();
		}
	}
	else if (forwardIndex == 3)
	{
		scanTipWidget->setVisible(false);
	}
}

void ScanMainGUI::JawScan()
{
	scanTipWidget->setVisible(true);
	if (globalTipIndex == 1)
	{
		scanTipWidget->upperCompenConstructIHM2();
		forwardIndex = 3;
	}
	else if (globalTipIndex == 2)
	{
		scanTipWidget->lowerCompenConstructIHM2();
		forwardIndex = 3;
	}
	else if (globalTipIndex == 3)
	{
		if (compensationFlag == true)
		{
			--allJawIndex;
		}
		if (allJawIndex == 1)
		{
			scanTipWidget->allCompenConstructIHM2();
			forwardIndex = 2;
			compensationFlag = false;
		}
		else if (allJawIndex == 2)
		{
			scanTipWidget->allPlaceConstructIHM3();
			forwardIndex = 1;
			chooseJawIndex = 2;
		}
		else if (allJawIndex == 3)
		{
			scanTipWidget->allCompenConstructIHM4();
			forwardIndex = 2;
			compensationFlag = false;
		}
		else if (allJawIndex == 4)
		{
			scanTipWidget->allPlaceConstructIHM5();
			forwardIndex = 1;
			chooseJawIndex = 1;
		}
		else if (allJawIndex == 5)
		{
			scanTipWidget->allCompenConstructIHM6();
			forwardIndex = 2;
			compensationFlag = false;
		}
		else if (allJawIndex == 6)
		{
			scanTipWidget->allFinishConstructIHM7();
			forwardIndex = 3;
		}
		allJawIndex++;
	}
	this->showMaximized();
	scanTipWidget->showMaximized();
}

void ScanMainGUI::compensationScan()
{
	compensationFlag = true;
	//float ax = 0, ay = 0;
	//------------相机目标旋转
	//------------相机目标旋转
	//------------相机目标旋转
	//------------相机目标旋转

	glWidget->GetMotorRot(c_addscan_x, c_addscan_y);

	//c_addscan_x = ax;
	//c_addscan_y = ay;

	if (c_addscan_y>-90 && c_addscan_y<90)
	{
		if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
		{
			emit compensationSignal(chooseJawIndex);
			return;
		}
		if(controlScanQThread->isRunning() == false)
		{
			//启动子线程，但没有启动线程处理函数
			controlScanQThread->start();
			ControlScanThread->setFlage(false);
		}
		if (controlComputeQThread->isRunning() == false)
		{
			//启动子线程，但没有启动线程处理函数
			controlComputeQThread->start();
			ControlComputeThread->setFlage(false);
		}
		emit compensationSignal(chooseJawIndex);
	}
}


void ScanMainGUI::fontviewBtnClicked()
{
	//glWidget->frontView();
	glWidget->mainView();
}

void ScanMainGUI::pushBtnClicked()
{
	cout << "pushBtnClicked" << endl;
// 	QColor clearColor(Qt::GlobalColor::white);
// 	glWidget->setClearColor(clearColor);
	glWidget->upwardView();
}

void ScanMainGUI::rightViewBtnClicked()
{
	glWidget->rightView();
}

void ScanMainGUI::leftViewBtnClicked()
{
	glWidget->leftView();
}

void ScanMainGUI::backViewBtnClicked()
{
	glWidget->backView();
}

void ScanMainGUI::jawViewBtnClicked()
{
	//glWidget->jawView();
	glWidget->overView();
}

void ScanMainGUI::narrowViewBtnClicked()
{
	glWidget->shrinkView();
}

void ScanMainGUI::enlargeViewBtnClicked()
{
	glWidget->enlargeView();
}

void ScanMainGUI::modelMoveStateSetBtnClicked()
{
	//glWidget->setModelMoveState(ui.modelMoveStateSetBtn->isChecked());
	glWidget->selectRegion(ui.modelMoveStateSetBtn->isChecked());
}

void ScanMainGUI::delSelectedBtnClicked()
{
	glWidget->delSelected();
}

void ScanMainGUI::confirmSelectedBtnClicked()
{
	glWidget->confirmSelRegion();
}
