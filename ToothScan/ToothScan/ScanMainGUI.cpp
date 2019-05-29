#include "ScanMainGUI.h"
#include "TaskManager.h"
#include "plyio.h"
#define NUITEST
void ScanMainGUI::saveModelFile(pCScanTask pTask) {
	orth::MeshModel meshModel;
	pTask->pTeethModel->getMeshModel(meshModel);
	int mModelVSize = meshModel.size();
	tinyply::plyio io;
	std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data())+ tabMainPage->ToChineseStr(patientNameQStr).data() + pTask->Get_ModelFileName() + ".ply";
	cout << "pathname: " << modelNameStr << endl;
	//finish_model_io.writeModel(modelNameStr, "stlb");
	//orth::ModelIO finish_model_io(&meshModel);
	//finish_model_io.writeModel(modelNameStr, "stlb");
	io.write_ply_example(modelNameStr, meshModel, true);
	cout << "saveModelFile finish" << endl;
}
bool ScanMainGUI::isModelFileExit(pCScanTask &pTask) {
	std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + pTask->Get_ModelFileName() + ".ply";
	return QFile::exists(modelNameStr.c_str());
}
void ScanMainGUI::loadModelFile(pCScanTask &pTask) {
	std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + pTask->Get_ModelFileName() + ".ply";
	orth::MeshModel meshModel;
	//orth::ModelIO finish_model_io(&meshModel);
	//finish_model_io.loadModel(modelNameStr);
 	tinyply::plyio io;
 	io.read_ply_file(modelNameStr, meshModel);
	pTask->m_mAllModel = meshModel;
	glWidget->mm = meshModel;
	pTask->pTeethModel = glWidget->makeObject();
	pTask->pTeethModel->m_model = meshModel;
}

void ScanMainGUI::hideAllPanel()
{
	ui.CutJawPanel->setVisible(false);
	ui.CutJawFinishPanel->setVisible(false);
	ui.ScanJawGroup->setVisible(false);
	ui.compensationScanPanel->setVisible(false);
	ui.StitchingPanel->setVisible(false);
	ui.StitchingFinishPanel->setVisible(false);
	ui.OralSubstitutePanel->setVisible(false);
	ui.TeethStitchingPanel->setVisible(false);
	ui.orderInfoPanel->setVisible(false);
	ui.DentalImplantPanel->setVisible(false);
}

void styleControl2(QObject *obj) {
	QObjectList list = obj->children();
	QWidget *b;
	QBoxLayout *pBoxLayout;
	foreach(QObject *obj, list)
	{
		pBoxLayout = static_cast<QBoxLayout*>(obj);
		b = static_cast<QWidget*>(obj);
		// 		if (b&&
		// 			obj->metaObject()->className() == QStringLiteral("QGroupBox")) {
		// // 			QString strStyleSheet = b->styleSheet();
		// // 			strStyleSheet += "border:0px groove gray;border-radius:5px;padding:2px 4px;border-color: rgb(128, 128, 128);";
		// // 			b->setStyleSheet(strStyleSheet);
		// 			QGraphicsDropShadowEffect *shadow_effect = new QGraphicsDropShadowEffect();
		// 
		// 			shadow_effect->setOffset(0, 0);
		// 
		// 			shadow_effect->setColor(Qt::gray);
		// 
		// 			shadow_effect->setBlurRadius(8);
		// 			b->setGraphicsEffect(shadow_effect);
		// 		}else 
		if (obj->metaObject()->className() == QStringLiteral("QPushButton")
			|| obj->metaObject()->className() == QStringLiteral("QDateTimeEdit")
			|| obj->metaObject()->className() == QStringLiteral("QLineEdit")
			|| obj->metaObject()->className() == QStringLiteral("QTextEdit")
			)
		{

			if (b) {
				QString strStyleSheet = b->styleSheet();
				strStyleSheet += "border:0px groove gray;border-radius:5px;background: rgb(255, 255, 255);";
				b->setStyleSheet(strStyleSheet);
				//b->setStyleSheet("border:1px groove gray;border-radius:10px;padding:2px 4px;border - color: rgb(128, 128, 128);background - color: rgb(255, 255, 255);");
				QGraphicsDropShadowEffect *shadow_effect = new QGraphicsDropShadowEffect();

				shadow_effect->setOffset(0, 0);

				shadow_effect->setColor(Qt::gray);

				shadow_effect->setBlurRadius(8);
				b->setGraphicsEffect(shadow_effect);
			}
		}
		if (pBoxLayout) {
			styleControl2(pBoxLayout);
		}
	}
}

ScanMainGUI::ScanMainGUI(QWidget *parent)
	: QWidget(parent),
	m_bsplitModelFlag(false)
{

	this->initVariable();
	this->constructIHM();
	ui.setupUi(this);
	this->setConnections();
	ui.CutJawPanel->move(1600, 230);
	ui.CutJawFinishPanel->move(1600, 230);
	ui.ScanJawGroup->move(1600, 230);
	ui.compensationScanPanel->move(1600, 230);
	ui.StitchingPanel->move(1600, 230);
	ui.StitchingFinishPanel->move(1600, 230);
	ui.OralSubstitutePanel->move(1600, 230);
	ui.TeethStitchingPanel->move(1600, 230);
	ui.orderInfoPanel->move(1600, 100);
	// 	ui.ScanJawGroup->setStyleSheet("background-color:rgb(225,225,225);");
	// 	ui.CutJawFinishPanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.CutJawPanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.compensationScanPanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.StitchingPanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.StitchingFinishPanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.OralSubstitutePanel->setStyleSheet("background-color:rgb(215,215,215);");
	// 	ui.TeethStitchingPanel->setStyleSheet("background-color:rgb(215,215,215);");
	resetValue();


	QGraphicsDropShadowEffect *defaultShadow = new QGraphicsDropShadowEffect();
	defaultShadow->setBlurRadius(9.0);
	defaultShadow->setColor(Qt::lightGray);
	defaultShadow->setOffset(0, 0);
	ui.ScanJawScanBtn->setGraphicsEffect(defaultShadow);
	setWindowFlags(Qt::FramelessWindowHint);
	styleControl2(this);
	ui.progressBar->setVisible(false);
	// 	QGraphicsDropShadowEffect *shadow_effect = new QGraphicsDropShadowEffect(ui.CutJawPanel);
	// 	shadow_effect->setOffset(-5, 5);
	// 	shadow_effect->setColor(Qt::gray);
	// 	shadow_effect->setBlurRadius(8);
	// 	ui.CutJawPanel->setGraphicsEffect(shadow_effect);
	// 	this->setAttribute(Qt::WA_TranslucentBackground);
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

	tabMainPage = new TabMainGUI();
	tabMainPage->showMaximized();

	scanTipWidget = new ScanTipWidget(this);

	//初始化线程
	ControlScanThread = new ControlThread;
	ControlComputeThread = new ComputeThread;

	cv::Mat model_matrix = cv::Mat::eye(4, 4, CV_64FC1);
	cv::Mat view_matrix = rt_r/*.inv()*/;
	glWidget->SetMatrix(model_matrix, view_matrix);

	//创建子线程对象
	controlScanQThread = new QThread(this);
	controlComputeQThread = new QThread(this);
	//把子线程（对象）添加到自定义的线程类（对象）中
	ControlScanThread->moveToThread(controlScanQThread);
	ControlComputeThread->moveToThread(controlComputeQThread);
	controlScanQThread->start();
	controlComputeQThread->start();

	//cameraWindow
	cameraWindow = new QDockWidget(QStringLiteral("相机显示"), this);
	cameraImageLabel = new QLabel(cameraWindow);
	cameraImageLabel->setStyleSheet("background-color:rgb(0,0,0);");
	cameraImageLabel->setScaledContents(true);

	autoTunePushButton = new QPushButton(QStringLiteral("自动调节"), cameraWindow);
	spinCameraBox = new QSpinBox(cameraWindow);
	sliderCamera = new QSlider(Qt::Horizontal, cameraWindow);
	spinCameraBox->setRange(0, 130);
	sliderCamera->setRange(0, 130);

	//toptoolButtons
	leftWatchButton = new QToolButton();
	rightWatchButton = new QToolButton();
	topWatchButton = new QToolButton();
	bottomWatchButton = new QToolButton();
	frontWatchButton = new QToolButton();
	backWatchButton = new QToolButton();
	enlargeButton = new QToolButton();
	shrinkButton = new QToolButton();
	//bottomtoolButtons
	selectRegionButton = new QToolButton();
	deleteModelButton = new QToolButton();
	m_closeBtn = new CTeethImgBtn("./Resources/images/closebtn.png", "", this);
	m_closeBtn->setFixedSize(30, 30);
	m_closeBtn->move(1880,20);
}

void ScanMainGUI::constructIHM()
{
	//glWidget->showMaximized();
	glWidget->setGeometry(0, 0, 1920, 1080);
	glWidget->setWindowSize(QSize(1920, 1080));
	glWidget->showFullScreen();

	//相机设置
	cameraWindow->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable); // 设置停靠窗口特性，可移动,可关闭
	cameraWindow->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);//设置可停靠区域为主窗口左边和右边
	cameraWindow->setGeometry(0, 0, 320, 276);

	cameraImageLabel->setGeometry(0, 20, 320, 256);
	sliderCamera->setGeometry(0, 256, 175, 20);
	spinCameraBox->setGeometry(175, 256, 60, 20);
	autoTunePushButton->setGeometry(235, 256, 85, 20);

	//cameraWindow->setContentsMargins(0, 0, 0, 0);

	//BottomTool底部工具栏
	leftWatchButton->setFixedSize(80, 80);
	leftWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/LeftView.png"));
	leftWatchButton->setIconSize(QSize(leftWatchButton->width(), leftWatchButton->height()));
	leftWatchButton->setStyleSheet("border-style:flat");
	leftWatchButton->setToolTip(QStringLiteral("左视图"));

	rightWatchButton->setFixedSize(80, 80);
	rightWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/RightView.png"));
	rightWatchButton->setIconSize(QSize(rightWatchButton->width(), rightWatchButton->height()));
	rightWatchButton->setStyleSheet("border-style:flat");
	rightWatchButton->setToolTip(QStringLiteral("右视图"));

	topWatchButton->setFixedSize(80, 80);
	topWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/TopView.png"));
	topWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	topWatchButton->setStyleSheet("border-style:flat");
	topWatchButton->setToolTip(QStringLiteral("俯视图"));

	bottomWatchButton->setFixedSize(80, 80);
	bottomWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BottomView.png"));
	bottomWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	bottomWatchButton->setStyleSheet("border-style:flat");
	bottomWatchButton->setToolTip(QStringLiteral("仰视图"));

	frontWatchButton->setFixedSize(80, 80);
	frontWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/FrontView.png"));
	frontWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	frontWatchButton->setStyleSheet("border-style:flat");
	frontWatchButton->setToolTip(QStringLiteral("前视图"));

	backWatchButton->setFixedSize(80, 80);
	backWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BackView.png"));
	backWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	backWatchButton->setStyleSheet("border-style:flat");
	backWatchButton->setToolTip(QStringLiteral("后视图"));

	enlargeButton->setFixedSize(80, 80);
	enlargeButton->setIcon(QIcon(":/MainWidget/Resources/images/enlarge.png"));
	enlargeButton->setIconSize(QSize(enlargeButton->width(), enlargeButton->height()));
	enlargeButton->setStyleSheet("border-style:flat");
	enlargeButton->setToolTip(QStringLiteral("放大"));

	shrinkButton->setFixedSize(80, 80);
	shrinkButton->setIcon(QIcon(":/MainWidget/Resources/images/shrink.png"));
	shrinkButton->setIconSize(QSize(shrinkButton->width(), shrinkButton->height()));
	shrinkButton->setStyleSheet("border-style:flat");
	shrinkButton->setToolTip(QStringLiteral("缩小"));

	selectRegionButton->setFixedSize(80, 80);
	selectRegionButton->setIcon(QIcon(":/MainWidget/Resources/images/SelectRegion.png"));
	selectRegionButton->setIconSize(QSize(enlargeButton->width(), enlargeButton->height()));
	selectRegionButton->setStyleSheet("border-style:flat");
	selectRegionButton->setToolTip(QStringLiteral("框选"));

	deleteModelButton->setFixedSize(80, 80);
	deleteModelButton->setIcon(QIcon(":/MainWidget/Resources/images/DeleteSelected.png"));
	deleteModelButton->setIconSize(QSize(shrinkButton->width(), shrinkButton->height()));
	deleteModelButton->setStyleSheet("border-style:flat");
	deleteModelButton->setToolTip(QStringLiteral("删除框选"));

	//顶部工具栏
	QWidget *topWidget = new QWidget(this);
	QHBoxLayout *topHLayout = new QHBoxLayout(topWidget);
	topHLayout->addStretch();
	topHLayout->addWidget(topWatchButton);
	topHLayout->addWidget(bottomWatchButton);
	topHLayout->addWidget(frontWatchButton);
	topHLayout->addWidget(backWatchButton);
	topHLayout->addWidget(leftWatchButton);
	topHLayout->addWidget(rightWatchButton);
	topHLayout->addWidget(enlargeButton);
	topHLayout->addWidget(shrinkButton);
	topHLayout->addStretch();
	topHLayout->addWidget(m_closeBtn);
	topWidget->setGeometry(0, 10, 1920, 100);

	//底部工具栏
	QWidget *bottomWidget = new QWidget(this);
	QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
	bottomHLayout->addStretch();
	bottomHLayout->addWidget(selectRegionButton);
	bottomHLayout->addWidget(deleteModelButton);
	bottomHLayout->addStretch();
	
	bottomWidget->setGeometry(0, 900, 1920, 100);

	//提示窗口
	scanTipWidget->setFixedSize(300, 700);
	scanTipWidget->setGeometry(1615, 80, 300, 700);
	scanTipWidget->setVisible(false);
}

void ScanMainGUI::setConnections()
{
	QObject::connect(sliderCamera, &QSlider::valueChanged, spinCameraBox, &QSpinBox::setValue);
	void (QSpinBox:: *spinBoxSignal)(int) = &QSpinBox::valueChanged;
	QObject::connect(spinCameraBox, spinBoxSignal, sliderCamera, &QSlider::setValue);
	spinCameraBox->setValue(35);
	//软件打开时，提示是否USB连接成功
	connect(this, SIGNAL(usbDeviceSignal()), this, SLOT(usbDeviceSlot()));

	//订单管理信息
	connect(tabMainPage, SIGNAL(scanDataSignal(QJsonObject)), this, SLOT(doScanDialogSlot(QJsonObject)));
	connect(tabMainPage, SIGNAL(showOrderInfoSignal(COrderInfo)), this, SLOT(showOrderInfo(COrderInfo)));

	//扫描提示页面
	connect(scanTipWidget->forwardStepButton, SIGNAL(clicked()), this, SLOT(judgeForwardStep()));//扫描 完成 下一步 按钮
	connect(scanTipWidget->backStepButton, SIGNAL(clicked()), this, SLOT(judgeBackStep()));//后退 按钮
	//补充扫描
	connect(scanTipWidget->compensationButton, SIGNAL(clicked()), this, SLOT(compensationScan()));
	//撤销补扫
	connect(scanTipWidget->discardButton, SIGNAL(clicked()), this, SLOT(discardCompensationSlot()));

	connect(scanTipWidget, SIGNAL(updateModelAngle()), this, SLOT(setRotationWaverSlot()));//RotationAndWaver

	//MeshandGpa
	connect(this, SIGNAL(gpaMeshSignal(int)), ControlComputeThread, SLOT(GPAMeshing(int)));
	//保存切割后模型
	connect(this, SIGNAL(saveCutModelSignal()), this, SLOT(saveCutModelSlot()));
	//保存所有模型到文件
	connect(this, SIGNAL(saveModeltoFileSignal()), this, SLOT(saveModeltoFileSlot()));

	//远距离配准，将上颌、下颌以及全颌进行配准
	connect(this, SIGNAL(farRegistrationSignal()), ControlComputeThread, SLOT(FarRegistrationSlot()));
	connect(ControlComputeThread, SIGNAL(finishFarRegSignal()), this, SLOT(updateRegMeshSlot()));
	connect(ControlComputeThread, SIGNAL(finishFarRegSignal()), this, SLOT(JawScan()));
	//完成配准后，显示或者隐藏模型连接																		   
	connect(scanTipWidget->upperShowButton, SIGNAL(clicked()), this, SLOT(ShowHideUpperModel()));//显示或隐藏上颌模型
	connect(scanTipWidget->lowerShowButton, SIGNAL(clicked()), this, SLOT(ShowHideLowerModel()));//显示或隐藏上颌模型

	//扫描控制和计算
	connect(this, SIGNAL(compensationSignal(int)), ControlScanThread, SLOT(compensationControlScan()));
	connect(this, SIGNAL(compensationSignal(int)), ControlComputeThread, SLOT(compensationComputeScan(int)));
	connect(this, SIGNAL(startControlNormalScan(int)), ControlScanThread, SLOT(normalControlScan()));
	connect(this, SIGNAL(startControlNormalScan(int)), ControlComputeThread, SLOT(normalComputeScan(int)));
	connect(this, SIGNAL(startAllJawNormalScan()), ControlScanThread, SLOT(normalAllJawControlScan()));
	connect(this, SIGNAL(startAllJawNormalScan()), ControlComputeThread, SLOT(normalAllJawComputeScan()));

	connect(this, SIGNAL(startControlCalibrationSignal()), ControlScanThread, SLOT(controlCalibrationScan()));
	connect(ControlComputeThread, SIGNAL(computeFinish()), this, SLOT(JawScan()));
	//展示模型
	connect(ControlComputeThread, SIGNAL(showModeltoGlSingel(int)), this, SLOT(updateMeshModel(int)));
	connect(this, SIGNAL(updateMeshModelSingel(int)), this, SLOT(updateMeshModel(int)));
	connect(this, SIGNAL(updateModelsVtSingle()), this, SLOT(updateModelsVtSlot()));
	connect(ControlComputeThread, SIGNAL(cameraShowSignal()), this, SLOT(updateCamera()));

	//Calibrate标定子页面
	connect(tabMainPage->calibratePushButton, SIGNAL(clicked()), this, SLOT(ToothCalibrateSlot()));
	connect(tabMainPage->globalCaliPushButton, SIGNAL(clicked()), this, SLOT(GlobalCalibrateSlot()));
	connect(ControlScanThread, SIGNAL(calibImageSignal(int)), this, SLOT(calibImageCameraSlot(int)));//展示标定照片

	//模型工具栏操作
	connect(this->topWatchButton, SIGNAL(clicked()), this, SLOT(topModelWatchSlot()));
	connect(this->bottomWatchButton, SIGNAL(clicked()), this, SLOT(bottomModelWatchSlot()));
	connect(this->leftWatchButton, SIGNAL(clicked()), this, SLOT(leftModelWatchSlot()));
	connect(this->rightWatchButton, SIGNAL(clicked()), this, SLOT(rightModelWatchSlot()));
	connect(this->frontWatchButton, SIGNAL(clicked()), this, SLOT(frontModelWatchSlot()));
	connect(this->backWatchButton, SIGNAL(clicked()), this, SLOT(backModelWatchSlot()));
	connect(this->enlargeButton, SIGNAL(clicked()), this, SLOT(enlargeModelSlot()));
	connect(this->shrinkButton, SIGNAL(clicked()), this, SLOT(shrinkModelSlot()));
	connect(this->selectRegionButton, SIGNAL(clicked()), this, SLOT(selectRegionSlot()));
	connect(this->deleteModelButton, SIGNAL(clicked()), this, SLOT(deleteSelectedRegionSlot()));

	//水平切面
	connect(this, SIGNAL(cutSurfaceSignal(bool)), this, SLOT(showCutSurfaceSlot(bool)));
	connect(scanTipWidget->spinCutBox, SIGNAL(valueChanged(int)), this, SLOT(moveCutSurfaceSpinSlot()));
	connect(scanTipWidget->sliderCut, SIGNAL(valueChanged(int)), this, SLOT(moveCutSurfaceSliderSlot()));
	connect(scanTipWidget->cutHeightButton, SIGNAL(clicked()), this, SLOT(cutModelSlot()));
	connect(scanTipWidget->discardCutHeightButton, SIGNAL(clicked()), this, SLOT(discardCutModelSlot()));

	connect(ui.ScanJawScanBtn, SIGNAL(clicked()), this, SLOT(scanJawScanBtnClick()));
	connect(ui.ScanJawBackStepBtn, SIGNAL(clicked()), this, SLOT(ScanJawBackStepBtnClick()));
	connect(ui.compensationBtn, SIGNAL(clicked()), this, SLOT(compensationBtnClick()));
	connect(ui.discardBtn, SIGNAL(clicked()), this, SLOT(discardBtnClick()));
	connect(ui.compensationScanPanelNextBtn, SIGNAL(clicked()), this, SLOT(compensationScanPanelNextBtnClick()));
	connect(ui.compensationScanPanelBackBtn, SIGNAL(clicked()), this, SLOT(compensationScanPanelBackBtnClick()));

	connect(ui.cutHeightSlider, &QSlider::valueChanged, ui.cutHeightSpinBox, &QSpinBox::setValue);
	void (QSpinBox:: *spinBoxSignal2)(int) = &QSpinBox::valueChanged;
	connect(ui.cutHeightSpinBox, spinBoxSignal2, ui.cutHeightSlider, &QSlider::setValue);
	connect(ui.cutHeightSpinBox, SIGNAL(valueChanged(int)), this, SLOT(movecutHeightSpinBoxSlot()));
	connect(ui.cutHeightSlider, SIGNAL(valueChanged(int)), this, SLOT(movecutHeightSliderSlot()));

	connect(ui.cutModelBtn, SIGNAL(clicked()), this, SLOT(cutModelBtnClick()));
	connect(ui.unDoCutBtn, SIGNAL(clicked()), this, SLOT(unDoCutBtnClick()));
	connect(ui.saveCutHeightCutBtn, SIGNAL(clicked()), this, SLOT(saveCutHeightCutBtnClick()));
	connect(ui.cutPaneNextStepBtn, SIGNAL(clicked()), this, SLOT(cutPaneNextStepBtnClick()));
	connect(ui.cutPanelBackBtn, SIGNAL(clicked()), this, SLOT(cutPanelBackBtnClick()));
	connect(ui.CutJawFinishPanelNextStepBtn, SIGNAL(clicked()), this, SLOT(CutJawFinishPanelNextStepBtnClick()));


	connect(this, SIGNAL(startNormalScan()), ControlScanThread, SLOT(normalScan()));
	connect(this, SIGNAL(startNormalScan()), ControlComputeThread, SLOT(normalComputeScan()));

	connect(this, SIGNAL(startAllJawScan()), ControlScanThread, SLOT(allJawScan()));
	connect(this, SIGNAL(startAllJawScan()), ControlComputeThread, SLOT(allJawComputeScan()));

	connect(ControlComputeThread, SIGNAL(showTaskModel()), this, SLOT(updateTaskModel()));
	connect(ControlComputeThread, SIGNAL(cameraShowSignal()), this, SLOT(updateCamera()));
	/*补扫*/
	connect(this, SIGNAL(compensationScanSignal()), ControlScanThread, SLOT(compensationControlScan()));
	connect(this, SIGNAL(compensationScanSignal()), ControlComputeThread, SLOT(compensationCompute()));
	connect(this, SIGNAL(gpaTaskMeshSignal()), ControlComputeThread, SLOT(GPAMeshing()));
	connect(this, SIGNAL(taskTeethSititSignal()), ControlComputeThread, SLOT(taskTeethSitit()));
	/*补扫*/
	/*mesh*/
	connect(ControlComputeThread, SIGNAL(meshFinish()), this, SLOT(meshFinishSlot()));
	connect(ControlComputeThread, SIGNAL(StitchFinish()), this, SLOT(StitchFinishSlot()));
	connect(ControlComputeThread, SIGNAL(taskTeethSititFinish()), this, SLOT(taskTeethSititFinishSlot()));
	/*mesh*/
	/*Stitching*/
	connect(this, SIGNAL(taskSititchingSignal()), ControlComputeThread, SLOT(Stitching()));
	connect(ui.stitchingPanelBtn, SIGNAL(clicked()), this, SLOT(stitchingPanelBtnClick()));

	connect(ui.stitchingBackBtn, SIGNAL(clicked()), this, SLOT(stitchingBackBtnClick()));
	connect(ui.stitchingUpperJawBtn, SIGNAL(clicked()), this, SLOT(stitchingUpperJawBtnClick()));
	connect(ui.stitchingLowerJawBtn, SIGNAL(clicked()), this, SLOT(stitchingLowerJawBtnClick()));
	connect(ui.stitchingFNextBtn, SIGNAL(clicked()), this, SLOT(stitchingFNextBtnClick()));
	connect(ui.OralSubstitutePanelNextBtn, SIGNAL(clicked()), this, SLOT(OralSubstitutePanelNextBtnClick()));
	connect(ui.teethStitchingPanelNextBtn, SIGNAL(clicked()), this, SLOT(teethStitchingPanelNextBtnClick()));
	connect(ui.oralSubstitutePanelBackBtn, SIGNAL(clicked()), this, SLOT(oralSubstitutePanelBackBtnClick()));
	connect(ui.teethStitchingPanelBackBtn, SIGNAL(clicked()), this, SLOT(teethStitchingPanelBackBtnClick()));
	connect(ui.CutJawFinishPanelBackBtn, SIGNAL(clicked()), this, SLOT(cutJawFinishPanelBackBtnClick()));
	connect(ui.StitchingFinishPanelBackBtn, SIGNAL(clicked()), this, SLOT(stitchingFinishPanelBackBtnClick()));
	connect(this->m_closeBtn, SIGNAL(clicked()), this, SLOT(closeBtnClicked()));
	connect(ui.upperJawBtn, SIGNAL(clicked()), this, SLOT(upperJawBtnBtnClick()));
	connect(ui.lowJawBtn, SIGNAL(clicked()), this, SLOT(lowJawBtnClick()));
	connect(ControlComputeThread, SIGNAL(progressBarResetSignal()), ui.progressBar,SLOT(reset()));
	connect(ControlComputeThread, SIGNAL(progressBarSetMinSignal(int)), ui.progressBar, SLOT(setMinimum(int)));
	connect(ControlComputeThread, SIGNAL(progressBarSetMaxSignal(int)), ui.progressBar, SLOT(setMaximum(int)));
	connect(ControlComputeThread, SIGNAL(progressBarSetValueSignal(int)), ui.progressBar, SLOT(setValue(int)));
	connect(ControlComputeThread, SIGNAL(progressBarsetOrientation(Qt::Orientation)), ui.progressBar, SLOT(setOrientation(Qt::Orientation)));
	connect(ControlComputeThread, SIGNAL(progressBarVisibleSignal(bool)), ui.progressBar, SLOT(setVisible(bool)));
	connect(ControlComputeThread, SIGNAL(progressBarSetSignal(int, int,  bool)), this, SLOT(progressBarSetSlot(int, int, bool)));
	for (int i = 0; i < 32; i++)
	{
		QString strWidgetName = "teethBtn_" + QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10);
		QPushButton * pButton = findChild<QPushButton*>(strWidgetName);
		if (pButton) {
			pButton->setObjectName(QString::number(i, 10));
			pButton->setCheckable(true);
			connect(pButton, SIGNAL(clicked()), this, SLOT(ToothButtonListPress()));
		}
	}
// 	void progressBarSetMinSignal(int min);
// 	void progressBarSetMaxSignal(int max);
// 	void progressBarSetValueSignal(int value);
// 	void progressBarsetOrientation(Qt::Orientation);
}

void ScanMainGUI::closeBtnClicked()
{
	controlScanQThread->terminate();
	controlComputeQThread->terminate();
	controlComputeQThread->wait();
	controlScanQThread->wait();
	tabMainPage->show();
	tabMainPage->showMaximized();
	this->hide();
}

void ScanMainGUI::ToothCalibrateSlot()
{
	QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("开始标定!"));
	box.setStandardButtons(QMessageBox::Yes);
	box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
	box.exec();
	if (controlScanQThread->isRunning() == false)
	{
		//启动子线程，但没有启动线程处理函数
		controlScanQThread->start();
		ControlScanThread->setFlage(false);
	}
	emit startControlCalibrationSignal();
}

void ScanMainGUI::GlobalCalibrateSlot()
{
	ControlScanThread->controlGlobalCaliScan();
}

void ScanMainGUI::CalculatePointCloud()
{
	cout <<"controlScanQThread->isRunning() = "<< controlScanQThread->isRunning() << endl;
	cout << "controlComputeQThread->isRunning() = " << controlComputeQThread->isRunning() << endl;
	cout <<"chooseJawIndex" <<chooseJawIndex << endl;
	if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
	{
		if (chooseJawIndex == 3)
		{
			emit startAllJawNormalScan();
			return;
		}
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
	if (chooseJawIndex == 3)
	{
		emit startAllJawNormalScan();
		return;
	}


	emit startControlNormalScan(chooseJawIndex);
}

void ScanMainGUI::splitModelCalculatePointCloud(pCScanTask pScanTask)
{
	if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
	{
		if (pScanTask->Get_ScanType() == eAllJawScan) {
			emit startAllJawScan();
			return;
		}
		emit startNormalScan();
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
	if (pScanTask->Get_ScanType() == eAllJawScan) {
		emit startAllJawScan();
		return;
	}
	emit startNormalScan();
}

void ScanMainGUI::updateMeshModel(int refreshIndex)
{
	if (refreshIndex == 1)//清空之前窗口上所有的模型
	{
		glWidget->m_ModelsVt.clear();
	}

	if (chooseJawIndex == 1)
	{
		if (refreshIndex == 2 && glWidget->m_ModelsVt.size() != 0)
		{
			/*for (int i = 0; i < glWidget->m_ModelsVt.size(); i++)
			{
				upper_ModelsVt.push_back(glWidget->m_ModelsVt[i]);
			}*/
			upper_ModelsVt = glWidget->m_ModelsVt;
			glWidget->m_ModelsVt.clear();
		}
		if (ControlComputeThread->upper_mModel.size() != 0)
		{
			int scan_index = ControlComputeThread->upper_mModel.size() - 1;
			glWidget->mm = ControlComputeThread->upper_mModel[scan_index];
		}
		else
		{
			glWidget->update();
			this->showMaximized();
			return;
		}
		upperTeethModel = glWidget->makeObject();
	}
	else if (chooseJawIndex == 2)
	{
		if (refreshIndex == 2 && glWidget->m_ModelsVt.size() != 0)
		{
			/*for (int i = 0; i < glWidget->m_ModelsVt.size(); i++)
			{
				lower_ModelsVt.push_back(glWidget->m_ModelsVt[i]);
			}*/
			lower_ModelsVt = glWidget->m_ModelsVt;
			glWidget->m_ModelsVt.clear();
		}
		if (ControlComputeThread->lower_mModel.size() != 0)
		{
			int scan_index = ControlComputeThread->lower_mModel.size() - 1;
			glWidget->mm = ControlComputeThread->lower_mModel[scan_index];
		}
		else
		{
			glWidget->update();
			this->showMaximized();
			return;
		}
		lowerTeethModel = glWidget->makeObject();
	}
	else if (chooseJawIndex == 3)
	{
		if (refreshIndex == 2 && glWidget->m_ModelsVt.size() != 0)
		{
			/*for (int i = 0; i < glWidget->m_ModelsVt.size(); i++)
			{
				all_ModelsVt.push_back(glWidget->m_ModelsVt[i]);
			}*/
			all_ModelsVt = glWidget->m_ModelsVt;
			glWidget->m_ModelsVt.clear();
		}
		if (ControlComputeThread->all_mModel.size() != 0)
		{
			int scan_index = ControlComputeThread->all_mModel.size() - 1;
			glWidget->mm = ControlComputeThread->all_mModel[scan_index];
		}
		else
		{
			glWidget->update();
			this->showMaximized();
			return;
		}
		allTeethModel = glWidget->makeObject();
	}

	this->showMaximized();
}

void ScanMainGUI::doScanDialogSlot(QJsonObject scanObj)
{
	controlScanQThread->start();
	controlComputeQThread->start();
	cameraWindow->setVisible(true);
	resetValue();
	glWidget->m_ModelsVt.clear();
	scanTipWidget->setVisible(true);
	m_bsplitModelFlag = false;
	cout << "caseType" << scanObj.value("caseType").toInt() << endl;
	cout << "upperJaw" << scanObj.value("upperJaw").toInt() << endl;
	filePath = scanObj.value("filePath").toString();//保存文件路径 
	patientNameQStr = scanObj.value("patientName").toString();
	if (scanObj.value("caseType").toInt() == 1)
	{
		CTaskManager::getInstance()->resetCurrentTask();
		showScanJawGroup();
// 		if (scanObj.value("upperJaw").toInt() == 1)
// 		{
// 			scanTipWidget->upperPlaceConstructIHM1();
// 			globalTipIndex = 1;
// 			forwardIndex = 1;
// 			chooseJawIndex = 1;
// 		}
// 		else if (scanObj.value("lowerJaw").toInt() == 1)
// 		{
// 			scanTipWidget->lowerPlaceConstructIHM1();
// 			globalTipIndex = 2;
// 			forwardIndex = 1;
// 			chooseJawIndex = 2;
// 		}
// 		else if (scanObj.value("allJaw").toInt() == 1)
// 		{
// 			scanTipWidget->allPlaceConstructIHM1();
// 			globalTipIndex = 3;
// 			forwardIndex = 1;
// 			chooseJawIndex = 3;
// 		}
	}
	else if (scanObj.value("caseType").toInt() == 2)
	{
		m_bsplitModelFlag = true;
		CTaskManager::getInstance()->resetCurrentTask();
		showScanJawGroup();
	}
	glWidget->m_ModelsVt.clear();
	tabMainPage->hide();
	this->showMaximized();
	scanTipWidget->showMaximized();

}
void ScanMainGUI::showScanJawGroup(bool bBack) {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		ui.ScanJawGroup->setVisible(true);
		ui.ScanJawBackStepBtn->setVisible(false);
		//ui.ScanJawGroupTipImage->
		QString l_strLabelImagePath = "";
		switch (pCurrentTask->Get_ScanType())
		{
		case eUpperJawScan:
			l_strLabelImagePath = "./Resources/images/upperjaw_yes.png";
			break;
		case eLowerJawScan:
			l_strLabelImagePath = "./Resources/images/lowerjaw_yes.png";
			break;
		case eAllJawScan:
			l_strLabelImagePath = "./Resources/images/alljaw.png";
			break;
		default:
			break;
		}
		ui.ScanJawGroupTipImage->setPixmap(QPixmap(l_strLabelImagePath));
		ui.noticeTipsLabel->setVisible(false);
		QString str;
		if (pCurrentTask->Get_Gingiva() == true) {
			str = QString::fromLocal8Bit("请插入带牙龈的") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str());
			ui.noticeTipsLabel->setText(QString::fromLocal8Bit("不要插入扫描杆"));
			ui.noticeTipsLabel->setVisible(true);
		}
		else if (pCurrentTask->Get_Gingiva() == false && pCurrentTask->Get_DentalImplant() == true) {
			str = QString::fromLocal8Bit("请插入带扫描杆的") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str());
			ui.noticeTipsLabel->setText(QString::fromLocal8Bit("不要插入牙龈"));
			ui.noticeTipsLabel->setVisible(true);
		}else
			str = QString::fromLocal8Bit("请插入") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str());
		//str.sprintf("%s",pCurrentTask->Get_TaskName());
		ui.ScanJawTips->setText(str);
	}
}
void ScanMainGUI::showOralSubstitutePanel(bool bBack) {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pCurrentTask) {
		return;
	}
	ui.OralSubstitutePanel->setVisible(true);
	pCGroupScan pGroupScan = static_pointer_cast<CGroupScan>(pCurrentTask);
	if (!pGroupScan)
		return;
	int nStep = 8 / pGroupScan->m_vtTeeth.size();
	for (int i = 0; i < 8; i++) {
		QString strLabelName = "teethNoLabel";
		strLabelName.sprintf("teethNoLabel%d", 1 + i);
		QLabel * plabel = findChild<QLabel*>(strLabelName);
		if (plabel){
			plabel->setText("");
			plabel->hide();
		}
	}
	for (int i = 0; i < pGroupScan->m_vtTeeth.size(); i++) {
		QString strLabelName = "teethNoLabel";
		strLabelName.sprintf("teethNoLabel%d", 1 + i * nStep);
		QLabel * plabel = findChild<QLabel*>(strLabelName);
		if (plabel) {
			//cout << QString::number((pGroupScan->m_vtTeeth[i] % 7 + 1) * 10 + pGroupScan->m_vtTeeth[i] + 1).toStdString() << endl;
			plabel->show();
			plabel->setText(QString::number((pGroupScan->m_vtTeeth[i] / 8 + 1) * 10 + pGroupScan->m_vtTeeth[i]%8 + 1));
		}
	}
}
void ScanMainGUI::showSubstitutePanel(bool bBack) {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pCurrentTask) {
		return;
	}
	ui.StitchingPanel->setVisible(true);
	QString str;
	str = QString::fromLocal8Bit("拼接") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str());
	ui.StitchingPanelTips->setText(str);
}
void ScanMainGUI::showStitchingFinishPanel(bool bBack) {
	ui.StitchingFinishPanel->setVisible(true);
	ui.stitchingUpperJawBtn->setVisible(false);
	ui.stitchingLowerJawBtn->setVisible(false);
	//pCurrentTask->Set_TaskPro(eProgressScan);
	glWidget->m_ModelsVt.clear();
	if (m_pupperTeethModel) {
		ui.stitchingUpperJawBtn->setVisible(true);
		glWidget->m_ModelsVt.push_back(m_pupperTeethModel);
// 		pCScanTask ptask= CTaskManager::getInstance()->getTask(eUpperJawScan);
// 		if (ptask) {
// 			saveModelFile(ptask);
// 		}
	}
	if (m_plowerTeethModel) {
		ui.stitchingLowerJawBtn->setVisible(true);
		glWidget->m_ModelsVt.push_back(m_plowerTeethModel);
// 		pCScanTask ptask = CTaskManager::getInstance()->getTask(eLowerJawScan);
// 		if (ptask) {
// 			saveModelFile(ptask);
// 		}
	}
	glWidget->update();
}
void ScanMainGUI::showCutJawPanel(bool bBack) {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		ui.compensationScanPanel->setVisible(false);
		ui.CutJawPanel->setVisible(true);
		QString str;
		//str.sprintf("%s", pCurrentTask->Get_TaskName());
		str = QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str()) + QString::fromLocal8Bit("增加扫描完成，可以水平切割模型数据");
		ui.CutJawPanelTipsLabel->setText(str);
		glWidget->showBkGround(true);
		ui.unDoCutBtn->setEnabled(false);
	}
	if (bBack) {
		//回退操作
	}
}
void ScanMainGUI::showcompensationScanPanel(bool bBack) {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pCurrentTask) {
		return;
	}
	if (pCurrentTask->Get_TaskPro() == eTaskProgress::eProgressScan) {
		//该增加补扫了
		ui.compensationScanPanel->setVisible(true);
		QString str;
		str = QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str()) + QString::fromLocal8Bit("扫描完成，可以增加模型数据");
		ui.compensationScanPanelTipLabel->setText(str);
	}
	if (bBack) {
		//回退操作
	}
}
void ScanMainGUI::judgeForwardStep()
{
	scanTipWidget->setVisible(false);
	cout << "forwardIndex: " << forwardIndex << endl;
	if (forwardIndex == 1)
	{
		if (allJawIndex == 4 || allJawIndex == 7)
		{
			bool bShow = false;
			glWidget->showBkGround(false);
			glWidget->update();
		}
		CalculatePointCloud();
	}
	else if (forwardIndex == 2)
	{
		emit gpaMeshSignal(chooseJawIndex);
	}
	else if (forwardIndex == 3)
	{
		JawScan();
	}
	else if (forwardIndex == 4)
	{
		emit saveModeltoFileSignal();
		scanTipWidget->setVisible(false);
	}
	else if (forwardIndex == 5)
	{
		emit farRegistrationSignal();
		//JawScan();
	}
}

void ScanMainGUI::JawScan()
{
	showcompensationScanPanel();
	return;
	if (m_bsplitModelFlag) {		//分模
	
	}
	else
	{
		scanTipWidget->setVisible(true);
		if (globalTipIndex == 1)
		{
			if (compensationFlag == true)
			{
				--upperJawIndex;
			}
			if (upperJawIndex == 1)
			{
				scanTipWidget->upperCompenConstructIHM2();
				forwardIndex = 2;
				compensationFlag = false;
			}
			else if (upperJawIndex == 2)
			{
				scanTipWidget->upperCutConstructIHM3();
				forwardIndex = 3;
				emit cutSurfaceSignal(true);
			}
			else if (upperJawIndex == 3)
			{
				ControlComputeThread->showUpperModelFlag = true;
				ControlComputeThread->regUpperModelFlag = true;
				scanTipWidget->upperFinishConstructIHM4();
				forwardIndex = 4;
				emit cutSurfaceSignal(false);
			}
			++upperJawIndex;
		}
		else if (globalTipIndex == 2)
		{
			if (compensationFlag == true)
			{
				--lowerJawIndex;
			}

			if (lowerJawIndex == 1)
			{
				scanTipWidget->lowerCompenConstructIHM2();
				forwardIndex = 2;
				compensationFlag = false;
			}
			else if (lowerJawIndex == 2)
			{
				scanTipWidget->lowerCutConstructIHM3();
				forwardIndex = 3;
				emit cutSurfaceSignal(true);
			}
			else if (lowerJawIndex == 3)
			{
				ControlComputeThread->showLowerModelFlag = true;
				ControlComputeThread->regLowerModelFlag = true;
				scanTipWidget->lowerFinishConstructIHM4();
				forwardIndex = 4;
				emit cutSurfaceSignal(false);
			}
			++lowerJawIndex;
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
				cout <<"forwardIndex = " << forwardIndex <<endl;
				compensationFlag = false;
			}
			else if (allJawIndex == 2)
			{
				scanTipWidget->allCutConstructIHM3();
				forwardIndex = 3;
				emit cutSurfaceSignal(true);
			}
			else if (allJawIndex == 3)
			{
				scanTipWidget->allPlaceConstructIHM4();
				forwardIndex = 1;
				chooseJawIndex = 2;
				emit cutSurfaceSignal(false);
			}
			else if (allJawIndex == 4)
			{
				scanTipWidget->allCompenConstructIHM5();
				forwardIndex = 2;
				compensationFlag = false;
			}
			else if (allJawIndex == 5)
			{
				scanTipWidget->allCutConstructIHM6();
				forwardIndex = 3;
				emit cutSurfaceSignal(true);
			}
			else if (allJawIndex == 6)
			{
				scanTipWidget->allPlaceConstructIHM7();
				forwardIndex = 1;
				chooseJawIndex = 1;
				emit cutSurfaceSignal(false);
			}
			else if (allJawIndex == 7)
			{
				scanTipWidget->allCompenConstructIHM8();
				forwardIndex = 2;
				compensationFlag = false;
			}
			else if (allJawIndex == 8)
			{
				scanTipWidget->allCutConstructIHM9();
				forwardIndex = 5;
				emit cutSurfaceSignal(true);
			}
			else if (allJawIndex == 9)
			{
				scanTipWidget->allFinishConstructIHM10();
				forwardIndex = 4;
				chooseJawIndex = 3;
				emit cutSurfaceSignal(false);
			}
			allJawIndex++;
		}

		this->showMaximized();
		scanTipWidget->showMaximized();
	}
}

void ScanMainGUI::deleteAllCutModel()
{
	if (chooseJawIndex == 1)
	{
		if (upperCutModelNum > 0)
		{
			for (int i = 0; i < upperCutModelNum; i++)
			{
				ControlComputeThread->upper_mModel.pop_back();
			}
		}
	}
	else if (chooseJawIndex == 2)
	{
		if (lowerCutModelNum > 0)
		{
			for (int i = 0; i < lowerCutModelNum; i++)
			{
				ControlComputeThread->lower_mModel.pop_back();
			}
		}
	}
	else if (chooseJawIndex == 3)
	{
		if (allCutModelNum > 0)
		{
			for (int i = 0; i < lowerCutModelNum; i++)
			{
				ControlComputeThread->all_mModel.pop_back();
			}
		}
	}
}

void ScanMainGUI::resetValue()
{
	m_pupperTeethModel = nullptr;
	m_plowerTeethModel = nullptr;
	m_pallTeethModel = nullptr;
	m_bsplitModelFlag = false;
	ui.progressBar->hide();
	hideAllPanel();
}

void ScanMainGUI::judgeBackStep()
{
	scanTipWidget->setVisible(true);
	if (globalTipIndex == 1)
	{
		if (upperJawIndex == 2)
		{
			scanTipWidget->upperPlaceConstructIHM1();
			globalTipIndex = 1;
			forwardIndex = 1;
			chooseJawIndex = 1;
			ControlComputeThread->upper_mModel.clear();
			//ControlComputeThread->upper_points_cloud_end2.clear();
			//ControlComputeThread->upper_points_cloud_globle2.clear();
			cameraImageLabel->setStyleSheet("background-color:rgb(0,0,0);");
			emit updateMeshModelSingel(1);
		}
		else if (upperJawIndex == 3)
		{
			scanTipWidget->upperCompenConstructIHM2();
			forwardIndex = 2;
			compensationFlag = false;
			//ControlComputeThread->upper_mModel.pop_back();
			deleteAllCutModel();
			emit cutSurfaceSignal(false);
			emit updateModelsVtSingle();
		}
		else if (upperJawIndex == 4)
		{
			scanTipWidget->upperCutConstructIHM3();
			forwardIndex = 3;
			//ControlComputeThread->upper_mModel.pop_back();
			emit cutSurfaceSignal(true);
			emit updateMeshModelSingel(1);
		}
		--upperJawIndex;
	}
	else if (globalTipIndex == 2)
	{
		if (lowerJawIndex == 2)
		{
			scanTipWidget->lowerPlaceConstructIHM1();
			globalTipIndex = 2;
			forwardIndex = 1;
			chooseJawIndex = 2;
			ControlComputeThread->lower_mModel.clear();
			//ControlComputeThread->lower_points_cloud_end2.clear();
			//ControlComputeThread->lower_points_cloud_globle2.clear();
			cameraImageLabel->setStyleSheet("background-color:rgb(0,0,0);");
			emit updateMeshModelSingel(1);
		}
		else if (lowerJawIndex == 3)
		{
			scanTipWidget->lowerCompenConstructIHM2();
			forwardIndex = 2;
			compensationFlag = false;
			//ControlComputeThread->lower_mModel.pop_back();
			deleteAllCutModel();
			emit cutSurfaceSignal(false);
			emit updateModelsVtSingle();
		}
		else if (lowerJawIndex == 4)
		{
			scanTipWidget->lowerCutConstructIHM3();
			forwardIndex = 3;
			//ControlComputeThread->lower_mModel.pop_back();
			emit cutSurfaceSignal(true);
			emit updateMeshModelSingel(1);
		}
		--lowerJawIndex;
	}
	else if (globalTipIndex == 3)
	{
		if (allJawIndex == 2)
		{
			scanTipWidget->allPlaceConstructIHM1();
			globalTipIndex = 3;
			forwardIndex = 1;
			chooseJawIndex = 3;
			ControlComputeThread->all_mModel.clear();
			//ControlComputeThread->all_points_cloud_end2.clear();
			//ControlComputeThread->all_points_cloud_globle2.clear();
			cameraImageLabel->setStyleSheet("background-color:rgb(0,0,0);");
			emit updateMeshModelSingel(1);
		}
		else if (allJawIndex == 3)
		{
			scanTipWidget->allCompenConstructIHM2();
			forwardIndex = 2;
			compensationFlag = false;
			//ControlComputeThread->all_mModel.pop_back();
			deleteAllCutModel();
			emit cutSurfaceSignal(false);
			emit updateModelsVtSingle();
		}
		else if (allJawIndex == 4)
		{
			scanTipWidget->allCutConstructIHM3();
			forwardIndex = 3;
			//ControlComputeThread->all_mModel.pop_back();
			emit cutSurfaceSignal(true);
			emit updateMeshModelSingel(1);
		}
		else if (allJawIndex == 5)
		{
			scanTipWidget->allPlaceConstructIHM4();
			forwardIndex = 1;
			chooseJawIndex = 2;
			ControlComputeThread->lower_mModel.clear();
			//ControlComputeThread->lower_points_cloud_end2.clear();
			//ControlComputeThread->lower_points_cloud_globle2.clear();
			emit cutSurfaceSignal(false);
			emit updateMeshModelSingel(1);
		}
		else if (allJawIndex == 6)
		{
			scanTipWidget->allCompenConstructIHM5();
			forwardIndex = 2;
			compensationFlag = false;
			//ControlComputeThread->lower_mModel.pop_back();
			deleteAllCutModel();
			emit cutSurfaceSignal(false);
			emit updateModelsVtSingle();
		}
		else if (allJawIndex == 7)
		{
			scanTipWidget->allCutConstructIHM6();
			forwardIndex = 3;
			//ControlComputeThread->lower_mModel.pop_back();
			emit cutSurfaceSignal(true);
			emit updateMeshModelSingel(1);
		}
		else if (allJawIndex == 8)
		{
			scanTipWidget->allPlaceConstructIHM7();
			forwardIndex = 1;
			chooseJawIndex = 1;
			ControlComputeThread->upper_mModel.clear();
			//ControlComputeThread->upper_points_cloud_end2.clear();
			//ControlComputeThread->upper_points_cloud_globle2.clear();
			emit cutSurfaceSignal(false);
			emit updateMeshModelSingel(1);
		}
		else if (allJawIndex == 9)
		{
			scanTipWidget->allCompenConstructIHM8();
			forwardIndex = 2;
			compensationFlag = false;
			//ControlComputeThread->upper_mModel.pop_back();
			deleteAllCutModel();
			emit cutSurfaceSignal(false);
			emit updateModelsVtSingle();
		}
		else if (allJawIndex == 10)
		{
			scanTipWidget->allCutConstructIHM9();
			forwardIndex = 3;
			//ControlComputeThread->upper_mModel.pop_back();
			emit cutSurfaceSignal(true);
			emit updateMeshModelSingel(1);
		}
		--allJawIndex;
	}

	this->showMaximized();
	scanTipWidget->showMaximized();
}

void ScanMainGUI::compensationScan()
{
	compensationFlag = true;
	//float ax = 0, ay = 0;
	//glWidget->GetMotorRot(ax, ay);
	c_scan_x = ax;
	c_scan_y = ay;
	cout << "c_scan_x: " << c_scan_x << "; c_scan_y: " << c_scan_y;
	if (c_scan_x > -90 && c_scan_x < 90)
	{
		if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
		{
			emit compensationSignal(chooseJawIndex);
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
		emit compensationSignal(chooseJawIndex);
	}
}

void ScanMainGUI::discardCompensationSlot()
{
	if (chooseJawIndex == 1)
	{
		if (ControlComputeThread->addUpperCompensationNum > 0)
		{
			ControlComputeThread->upper_mModel.pop_back();
			glWidget->m_ModelsVt.pop_back();
			glWidget->update();
			--ControlComputeThread->addUpperCompensationNum;
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("上颌没有增扫数据!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
	}
	else if (chooseJawIndex == 2)
	{
		if (ControlComputeThread->addLowerCompensationNum > 0)
		{
			ControlComputeThread->lower_mModel.pop_back();
			glWidget->m_ModelsVt.pop_back();
			glWidget->update();
			--ControlComputeThread->addLowerCompensationNum;
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("下颌没有增扫数据!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
	}
	else if (chooseJawIndex == 3)
	{
		if (ControlComputeThread->addAllCompensationNum > 0)
		{
			ControlComputeThread->all_mModel.pop_back();
			glWidget->m_ModelsVt.pop_back();
			glWidget->update();
			--ControlComputeThread->addAllCompensationNum;
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("全颌没有增扫数据!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
	}
}

void ScanMainGUI::topModelWatchSlot()
{
	glWidget->upwardView();
}

void ScanMainGUI::bottomModelWatchSlot()
{
	glWidget->overView();
}

void ScanMainGUI::leftModelWatchSlot()
{
	glWidget->leftView();
}

void ScanMainGUI::rightModelWatchSlot()
{
	glWidget->rightView();
}

void ScanMainGUI::frontModelWatchSlot()
{
	glWidget->mainView();
}

void ScanMainGUI::backModelWatchSlot()
{
	glWidget->backView();
}

void ScanMainGUI::enlargeModelSlot()
{
	glWidget->enlargeView();
}

void ScanMainGUI::shrinkModelSlot()
{
	glWidget->shrinkView();
}

void ScanMainGUI::selectRegionSlot()
{
	if (bSelected == false)
	{
		bSelected = true;
	}
	else
	{
		bSelected = false;
	}
	glWidget->selectRegion(bSelected);
}

void ScanMainGUI::deleteSelectedRegionSlot()
{
	glWidget->delSelected();
}

void ScanMainGUI::showCutSurfaceSlot(bool bShow)
{
	glWidget->showBkGround(bShow);
	glWidget->update();
}

void ScanMainGUI::moveCutSurfaceSpinSlot()
{
	int curValue = scanTipWidget->spinCutBox->value();
	scanTipWidget->cutHeightLineEdit->setText(QString::number(curValue, 10));
	if (curValue > scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveUp();
		scanTipWidget->globalSpinCutValue = curValue;
	}
	else if (curValue < scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveDown();
		scanTipWidget->globalSpinCutValue = curValue;
	}

}

void ScanMainGUI::moveCutSurfaceSliderSlot()
{
	int curValue = scanTipWidget->sliderCut->value();
	scanTipWidget->cutHeightLineEdit->setText(QString::number(curValue, 10));
	if (curValue > scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveUp();
		scanTipWidget->globalSpinCutValue = curValue;
	}
	else if (curValue < scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveDown();
		scanTipWidget->globalSpinCutValue = curValue;
	}
}

void ScanMainGUI::movecutHeightSpinBoxSlot()
{
	int curValue = ui.cutHeightSpinBox->value();
	ui.cutHeightSpinBox->setValue(curValue);
	if (curValue > scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveUp();
		scanTipWidget->globalSpinCutValue = curValue;
	}
	else if (curValue < scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveDown();
		scanTipWidget->globalSpinCutValue = curValue;
	}

}

void ScanMainGUI::movecutHeightSliderSlot()
{
	int curValue = ui.cutHeightSlider->value();
	ui.cutHeightSpinBox->setValue(curValue);
	if (curValue > scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveUp();
		scanTipWidget->globalSpinCutValue = curValue;
	}
	else if (curValue < scanTipWidget->globalSpinCutValue)
	{
		glWidget->bgGroundmoveDown();
		scanTipWidget->globalSpinCutValue = curValue;
	}
}

void ScanMainGUI::ToothButtonListPress()
{
	int toothButtonIndex = sender()->objectName().toInt();
	QString strWidgetName = "teethBtn_" + QString::number((toothButtonIndex / 8 + 1) * 10 + toothButtonIndex % 8 + 1, 10);
	QPushButton * pButton = findChild<QPushButton*>(strWidgetName);
	if (pButton) {
		glWidget->m_cutToothIndex = toothButtonIndex;
	}
}

void ScanMainGUI::cutModelSlot()
{
	glWidget->cutModelUnderBg();
	emit saveCutModelSignal();
}

void ScanMainGUI::setRotationWaverSlot()
{
	glWidget->GetMotorRot(ax, ay);

	while (ay < -180 || ay>180)
	{
		if (ay > 180)
		{
			float temp = ay - 180;
			ay = -180 + temp;
		}
		if (ay < -180)
		{
			float temp = ay + 180;
			ay = 180 + temp;
		}
	}

	ax = round(ax * 10);
	ax = ax / 10;

	ay = round(ay * 10);
	ay = ay / 10;
	scanTipWidget->rotationLineEdit->setText(QString("%1").arg(ay));
	scanTipWidget->waverLineEdit->setText(QString("%1").arg(ax));

	ui.rotationLineEdit->setText(QString("%1").arg(ay));
	ui.waverLineEdit->setText(QString("%1").arg(ax));

	ax -= 68.3;
}

void ScanMainGUI::updateCamera()
{
	QImage cameraImage;
	cameraImage = Mat2QImage(ControlComputeThread->camera_image);
	cameraImageLabel->setPixmap(QPixmap::fromImage(cameraImage));
	cameraImageLabel->setScaledContents(true);
}

QImage ScanMainGUI::Mat2QImage(const cv::Mat &InputMat)
{
	cv::Mat TmpMat;
	// convert the color space to RGB
	if (InputMat.channels() == 1)
	{
		cv::cvtColor(InputMat, TmpMat, CV_GRAY2RGB);
	}
	else
	{
		cv::cvtColor(InputMat, TmpMat, CV_BGR2RGB);
	}
	// construct the QImage using the data of the mat, while do not copy the data
	QImage Result = QImage((const uchar*)(TmpMat.data), TmpMat.cols, TmpMat.rows, QImage::Format_RGB888);
	// deep copy the data from mat to QImage
	Result.bits();
	return Result;
}

void ScanMainGUI::saveCutModelSlot()
{
	if (chooseJawIndex == 1)
	{
		/*int size = glWidget->m_ModelsVt.size();
		glWidget->m_ModelsVt[size - 1];*/
		orth::MeshModel mm;
		upperTeethModel->getMeshModel(mm);
		ControlComputeThread->upper_mModel.push_back(mm);
		++upperCutModelNum;
	}
	else if (chooseJawIndex == 2)
	{
		orth::MeshModel mm;
		lowerTeethModel->getMeshModel(mm);
		ControlComputeThread->lower_mModel.push_back(mm);
		++lowerCutModelNum;
	}
	else if (chooseJawIndex == 3)
	{
		orth::MeshModel mm;
		allTeethModel->getMeshModel(mm);
		ControlComputeThread->all_mModel.push_back(mm);
		++allCutModelNum;
	}
	emit updateMeshModelSingel(1);
}

void ScanMainGUI::discardCutModelSlot()
{
	if (chooseJawIndex == 1)
	{
		if (upperCutModelNum > 0)
		{
			ControlComputeThread->upper_mModel.pop_back();
			--upperCutModelNum;
			emit updateMeshModelSingel(1);
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("上颌没有切割模型!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
			//QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("上颌没有切割模型!"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		}
	}
	else if (chooseJawIndex == 2)
	{
		if (lowerCutModelNum > 0)
		{
			ControlComputeThread->lower_mModel.pop_back();
			--lowerCutModelNum;
			emit updateMeshModelSingel(1);
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("下颌没有切割模型!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
	}
	else if (chooseJawIndex == 3)
	{
		if (allCutModelNum > 0)
		{
			ControlComputeThread->all_mModel.pop_back();
			--allCutModelNum;
			emit updateMeshModelSingel(1);
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("全颌没有切割模型!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
	}

}

void ScanMainGUI::updateModelsVtSlot()
{
	glWidget->m_ModelsVt.clear();
	if (chooseJawIndex == 1)
	{
		/*for (int i = 0; i < upper_ModelsVt.size(); i++)
		{
			glWidget->m_ModelsVt.push_back(upper_ModelsVt[i]);
		}*/
		glWidget->m_ModelsVt = upper_ModelsVt;
	}
	else if (chooseJawIndex == 2)
	{
		glWidget->m_ModelsVt = lower_ModelsVt;
	}
	else if (chooseJawIndex == 3)
	{
		glWidget->m_ModelsVt = all_ModelsVt;
	}
	glWidget->update();
	this->showMaximized();
}

void ScanMainGUI::saveModeltoFileSlot()
{
	if (chooseJawIndex == 1)
	{
		int mModelVSize = ControlComputeThread->upper_mModel.size();
		/*for (int index = 0; index < mModelVSize - 1; index++)
		{
			orth::ModelIO model_io(&ControlComputeThread->upper_mModel[index]);
			std::string fileStr = filePath.toStdString() + tabMainPage->ToChineseStr(patientNameQStr).data() + "_UpperJaw_" + std::to_string(index) + ".stl";
			cout << "fileStr: " << fileStr << endl;
			model_io.writeModel(fileStr,"stl");
		}*/
		orth::ModelIO finish_model_io(&ControlComputeThread->upper_mModel[mModelVSize - 1]);
		std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + "_FinalUpperJawModel.stl";
		cout << "pathname: " << modelNameStr << endl;
		finish_model_io.writeModel(modelNameStr, "stlb");
	}
	else if (chooseJawIndex == 2)
	{
		int mModelVSize = ControlComputeThread->lower_mModel.size();
		/*for (int index = 0; index < mModelVSize - 1; index++)
		{
			orth::ModelIO model_io(&ControlComputeThread->lower_mModel[index]);
			std::string fileStr = filePath.toStdString() + tabMainPage->ToChineseStr(patientNameQStr).data() + "_LowerJaw_" + std::to_string(index) + ".stl";
			model_io.writeModel(fileStr, "stl");
		}*/
		orth::ModelIO finish_model_io(&ControlComputeThread->lower_mModel[mModelVSize - 1]);
		std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + "_FinalLowerJawModel.stl";
		cout << "pathname: " << modelNameStr << endl;
		finish_model_io.writeModel(modelNameStr, "stlb");
	}
	else if (chooseJawIndex == 3)
	{
		int mModelVSize = ControlComputeThread->all_mModel.size();
		/*for (int index = 0; index < mModelVSize - 1; index++)
		{
			orth::ModelIO model_io(&ControlComputeThread->all_mModel[index]);
			std::string fileStr = filePath.toStdString() + tabMainPage->ToChineseStr(patientNameQStr).data() + "_AllJaw_" + std::to_string(index) + ".stl";
			model_io.writeModel(fileStr, "stl");
		}*/
		orth::ModelIO all_finish_model_io(&ControlComputeThread->all_mModel[mModelVSize - 1]);
		std::string modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + "_FinalAllJawModel.stl";
		cout << "pathname: " << modelNameStr << endl;
		all_finish_model_io.writeModel(modelNameStr, "stlb");

		mModelVSize = ControlComputeThread->upper_mModel.size();
		orth::ModelIO upper_finish_model_io(&ControlComputeThread->upper_mModel[mModelVSize - 1]);
		modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + "_FinalUpperJawModel.stl";
		cout << "pathname: " << modelNameStr << endl;
		upper_finish_model_io.writeModel(modelNameStr, "stlb");

		mModelVSize = ControlComputeThread->lower_mModel.size();
		orth::ModelIO lower_finish_model_io(&ControlComputeThread->lower_mModel[mModelVSize - 1]);
		modelNameStr = string(tabMainPage->ToChineseStr(filePath).data()) + tabMainPage->ToChineseStr(patientNameQStr).data() + "_FinalLowerJawModel.stl";
		cout << "pathname: " << modelNameStr << endl;
		lower_finish_model_io.writeModel(modelNameStr, "stlb");
	}
	glWidget->m_ModelsVt.clear();
	glWidget->update();

	this->setVisible(false);
	tabMainPage->showMaximized();
}

void ScanMainGUI::scanJawScanBtnClick()
{
#ifdef UITEST
	ui.ScanJawGroup->setVisible(false);
	ui.CutJawPanel->setVisible(true);
	return;
#endif

	ui.ScanJawGroup->setVisible(false);
	glWidget->m_ModelsVt.clear();
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		pCurrentTask->m_mModel.clear();
		if (isModelFileExit(pCurrentTask) && QMessageBox::information(NULL, QStringLiteral("提示"), QStringLiteral("模型已经存在是否重新扫描"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::No) {
			//加载模型
			loadModelFile(pCurrentTask);
			//cutPaneNextStepBtnClick();
			showCutJawPanel();

		}
		else
		{
			pCurrentTask->Set_TaskPro(eProgressScan);
			splitModelCalculatePointCloud(pCurrentTask);
		}
	}
}
void ScanMainGUI::ScanJawBackStepBtnClick()
{
	ShowLastScanTask();
// 	pCScanTask pCurrentTask = CTaskManager::getInstance()->getLastTask();
// 	if (pCurrentTask) {
// 		switch (pCurrentTask->Get_TaskType())
// 		{
// 		case eScan: {
// 			break;
// 		}
// 		case eUpperStitching: {
// 			break;
// 		}
// 		case eLowerStitching: {
// 			break;
// 		}
// 		case eUpperTeethStit: {
// 			break;
// 		}
// 		case eLowerTeethStit: {
// 			break;
// 		}
// 		default:
// 			break;
// 		}
// 	}
}

void ScanMainGUI::ShowLastScanTask(bool bCurrentTaskSame)
{
	hideAllPanel();
	glWidget->showBkGround(false);
	pCScanTask pScanTask = CTaskManager::getInstance()->getLastScanTask(bCurrentTaskSame);
	if (!pScanTask) {
		showScanJawGroup();
		return;
	}
	if (pScanTask->Get_TaskType() != eScan)
		return;
	hideAllPanel();
	switch (pScanTask->Get_ScanType()) 
	{
	case etoothCrown:
	case einlayScan:
		showOralSubstitutePanel();
		break;
	case eUpperJawScan:
	case eLowerJawScan:
	case eAllJawScan:
		showScanJawGroup();
		break;
	default:
		break;
	}
}

void ScanMainGUI::compensationBtnClick()
{
	ui.compensationScanPanel->setVisible(false);
	ui.discardBtn->setEnabled(true);
	compensationFlag = true;
	//float ax = 0, ay = 0;
	//glWidget->GetMotorRot(ax, ay);
	c_scan_x = ax;
	c_scan_y = ay;
	cout << "c_scan_x: " << c_scan_x << "; c_scan_y: " << c_scan_y;
	if (c_scan_x > -90 && c_scan_x < 90)
	{
		if (controlScanQThread->isRunning() == true && controlComputeQThread->isRunning() == true)  //判断线程占用
		{
			emit compensationScanSignal();
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
		emit compensationScanSignal();
	}
}
void ScanMainGUI::discardBtnClick() {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		if (pCurrentTask->m_nAddModel > 0) {
			pCurrentTask->m_mModel.pop_back();
			pCurrentTask->m_points_cloud_globle.pop_back();
			pCurrentTask->m_points_cloud_end._Pop_back_n(pCurrentTask->m_points_cloud_end_addSize[pCurrentTask->m_points_cloud_end_addSize.size() - 1]);
			pCurrentTask->m_points_cloud_end_addSize.pop_back();
			pCurrentTask->m_nAddModel--;
			glWidget->m_ModelsVt.pop_back();
			glWidget->update();
		}
		if (pCurrentTask->m_nAddModel == 0) {
			ui.discardBtn->setEnabled(false);
		}
	}
}

//下一步 合并吧
void ScanMainGUI::compensationScanPanelNextBtnClick() {	//下一步切割
#ifdef UITEST
	ui.compensationScanPanel->setVisible(false);
	ui.StitchingPanel->setVisible(true);
	return;
#endif
	hideAllPanel();
	showCutJawPanel();
}
//上一步
void ScanMainGUI::compensationScanPanelBackBtnClick() {
	// 	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	// 	if (pCurrentTask) {
	// 		emit gpaTaskMeshSignal();
	// 	}
// 	hideAllPanel();
// 	//ui.ScanJawGroup->setVisible(true);
// 	showScanJawGroup(true);
	hideAllPanel();
	ShowLastScanTask(true);
}
void ScanMainGUI::cutModelBtnClick() {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		vector<orth::MeshModel> tmpMeshModelVt;
		for (int i = 0; i < glWidget->m_ModelsVt.size(); i++) {
			orth::MeshModel meshModel;
			pCTeethModel pTeeth = static_pointer_cast<CTeethModel>(glWidget->m_ModelsVt[i]);
			if (pTeeth) {
				pTeeth->getMeshModel(meshModel);
			}
			tmpMeshModelVt.push_back(meshModel);
		}
		pCurrentTask->m_mCutModel.push_back(tmpMeshModelVt);
		//orth::MeshModel meshModel;
		//pCurrentTask->pTeethModel->getMeshModel(meshModel);
		
		glWidget->cutModelUnderBg();
		ui.unDoCutBtn->setEnabled(true);
	}
}
void ScanMainGUI::unDoCutBtnClick() {
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		if (pCurrentTask->m_mCutModel.size() > 0) {
			orth::MeshModel meshModel;
			pCurrentTask->m_mModel = pCurrentTask->m_mCutModel[pCurrentTask->m_mCutModel.size() - 1];
// 			pCurrentTask->pTeethModel->m_model = meshModel;
// 			pCurrentTask->pTeethModel->makeObject();
			for (int i = 0; i < glWidget->m_ModelsVt.size();i++) {
				pCTeethModel pTeeth = static_pointer_cast<CTeethModel>(glWidget->m_ModelsVt[i]);
				if (pTeeth){
					pTeeth->m_model = pCurrentTask->m_mModel[i];
					pTeeth->makeObject();
				}
			}
			pCurrentTask->m_mCutModel.pop_back();
			glWidget->update();
		}
		else {
			ui.unDoCutBtn->setEnabled(false);
		}
	}
}
void ScanMainGUI::saveCutHeightCutBtnClick() {


}
void ScanMainGUI::cutPanelBackBtnClick() {
	//showcompensationScanPanel(true);
	ShowLastScanTask(true);
}
void ScanMainGUI::oralSubstitutePanelBackBtnClick() {
	//showcompensationScanPanel(true);
	ShowLastScanTask();
}
void ScanMainGUI::teethStitchingPanelBackBtnClick() {
	//showcompensationScanPanel(true);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {

		pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pCurrentTask);
		if (!pTask)
			return;
		pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
		if (!pDstTask || !pSrcTask)
			return;
		if(pDstTask->m_mRegistrationModels.size())
			pDstTask->m_mRegistrationModels.pop_back();
	}
	ShowLastScanTask(true);
}
void ScanMainGUI::cutJawFinishPanelBackBtnClick() {
	//showcompensationScanPanel(true);
	ShowLastScanTask(true);
}
void ScanMainGUI::stitchingFinishPanelBackBtnClick() {
	//showcompensationScanPanel(true);
	ShowLastScanTask(true);
}
void ScanMainGUI::cutPaneNextStepBtnClick() {
#ifdef UITEST
	ui.CutJawPanel->setVisible(false);
	ui.compensationScanPanel->setVisible(true);
	return;
#endif
	hideAllPanel();
	glWidget->showBkGround(false);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask && pCurrentTask->m_mModel.size()>0) {
		glWidget->cutModelUnderBg();
		vector<orth::MeshModel> tmpMeshModelVt;
		for (int i = 0; i < glWidget->m_ModelsVt.size(); i++) {
			orth::MeshModel meshModel;
			pCTeethModel pTeeth = static_pointer_cast<CTeethModel>(glWidget->m_ModelsVt[i]);
			if (pTeeth) {
				pTeeth->getMeshModel(meshModel);
			}
			tmpMeshModelVt.push_back(meshModel);
		}
		pCurrentTask->m_mModel = tmpMeshModelVt;
		pCurrentTask->m_mCutModel.clear();
		glWidget->update();
		emit gpaTaskMeshSignal();
		return;
	}	
	pCScanTask pNextTask = CTaskManager::getInstance()->getNextTask();
	if (pCurrentTask->Get_ScanType() == eAllJawScan) {
		m_pallTeethModel = pCurrentTask->pTeethModel;
	}
	saveModelFile(pCurrentTask);

	if (!pNextTask&&pCurrentTask) {
		ui.CutJawPanel->setVisible(false);
		ui.StitchingFinishPanel->setVisible(true);
		QString str;
		ui.stitchingUpperJawBtn->setVisible(false);
		ui.stitchingLowerJawBtn->setVisible(false);
		if (pCurrentTask->Get_ScanType() == eUpperJawScan) {
			m_pupperTeethModel = pCurrentTask->pTeethModel;
			ui.stitchingUpperJawBtn->setChecked(true);
			ui.stitchingUpperJawBtn->setVisible(true);
		}
		else if (pCurrentTask->Get_ScanType() == eLowerJawScan) {
			m_plowerTeethModel = pCurrentTask->pTeethModel;
			ui.stitchingLowerJawBtn->setChecked(true);
			ui.stitchingLowerJawBtn->setVisible(true);
		}
		//str = QString::fromLocal8Bit("可以通过工具栏修剪") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str()) + QString::fromLocal8Bit("数据");
		//ui.CutJawFinishPanelTips->setText(str);
	}
	else if (pNextTask && pCurrentTask) {
		if (pNextTask->Get_TaskType() == eScan && pNextTask->Get_ScanType() == etoothCrown
			|| pNextTask->Get_ScanType() == einlayScan) {
			showOralSubstitutePanel();
		}
		else if (pNextTask->Get_TaskType() == eScan) {
			ui.CutJawPanel->setVisible(false);
			ui.ScanJawBackStepBtn->setVisible(true);
			showScanJawGroup();
		}
		else if (pNextTask->Get_TaskType() == eUpperStitching || pNextTask->Get_TaskType() == eLowerStitching) {	//上下颌的合并
			ui.CutJawPanel->setVisible(false);
			//emit taskSititchingSignal();
			showSubstitutePanel();
		}
		else if (pNextTask->Get_TaskType() == eUpperTeethStit || pNextTask->Get_TaskType() == eLowerTeethStit) {
			//牙齿拼接
			ui.CutJawPanel->setVisible(false);
			//emit taskTeethSititSignal();
			showSubstitutePanel();
		}

	}
}
void ScanMainGUI::stitchingBackBtnClick() {
	ShowLastScanTask(true);
}
void ScanMainGUI::stitchingPanelBtnClick() {
#ifdef UITEST
	ui.StitchingPanel->setVisible(false);
	ui.StitchingFinishPanel->setVisible(true);
	return;
#endif
	hideAllPanel();
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask->Get_TaskType() == eUpperStitching ||
		pCurrentTask->Get_TaskType() == eLowerStitching) {
		emit taskSititchingSignal();
	}
	else if (pCurrentTask->Get_TaskType() == eUpperTeethStit|| pCurrentTask->Get_TaskType() == eLowerTeethStit) {
		emit taskTeethSititSignal();
	}
}

void ScanMainGUI::upperJawBtnBtnClick() {
	if (m_pupperTeethModel) {
		m_pupperTeethModel->Set_Visible(ui.upperJawBtn->isChecked());
		glWidget->update();
	}
}
void ScanMainGUI::lowJawBtnClick() {
	if (m_plowerTeethModel) {
		m_plowerTeethModel->Set_Visible(ui.lowJawBtn->isChecked());
		glWidget->update();
	}
}

void ScanMainGUI::progressBarSetSlot(int min, int max, bool bVisible)
{
	ui.progressBar->reset();
	ui.progressBar->setMinimum(min);
	ui.progressBar->setMaximum(max);
	//ui.progressBar->setOrientation(tation);
	ui.progressBar->setVisible(bVisible);
}

void ScanMainGUI::stitchingUpperJawBtnClick() {
	if (m_pupperTeethModel) {
		m_pupperTeethModel->Set_Visible(ui.stitchingUpperJawBtn->isChecked());
		glWidget->update();
	}
}
void ScanMainGUI::stitchingLowerJawBtnClick() {
	if (m_plowerTeethModel) {
		m_plowerTeethModel->Set_Visible(ui.stitchingLowerJawBtn->isChecked());
		glWidget->update();
	}
}

void ScanMainGUI::stitchingFNextBtnClick() {
#ifdef UITEST
	ui.StitchingFinishPanel->setVisible(false);
	ui.OralSubstitutePanel->setVisible(true);
	return;
#endif
	// 	if (m_plowerTeethModel)
	// 		m_plowerTeethModel->Set_Visible(glWidget->stitchingLowerJawBtn->isChecked());
	// 	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	// 	pCScanTask pNextTask = CTaskManager::getInstance()->getNextTask();
	// 	ui.StitchingFinishPanel->setVisible(false);
	// 	if (pCurrentTask->Get_ScanType() == etoothCrown) {		//代型扫描
	// 		showOralSubstitutePanel();
	// 	}
	// 	else if (pCurrentTask->Get_ScanType() == eUpperJawScan) {
	// // 		ui.ScanJawGroup->setVisible(true);
	// // 		QString str;
	// // 		str = QString::fromLocal8Bit("请插入") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str());
	// // 		ui.ScanJawTips->setText(str);
	// 		showScanJawGroup();
	// 	}
	if (m_pupperTeethModel) {
		pCScanTask ptask = CTaskManager::getInstance()->getTask(eUpperJawScan);
		if (ptask) {
			saveModelFile(ptask);
		}
	}
	if (m_plowerTeethModel) {
		pCScanTask ptask = CTaskManager::getInstance()->getTask(eLowerJawScan);
		if (ptask) {
			saveModelFile(ptask);
		}
	}
	tabMainPage->show();
	tabMainPage->showMaximized();
	this->hide();

}
void ScanMainGUI::OralSubstitutePanelNextBtnClick() {
#ifdef UITEST
	ui.OralSubstitutePanel->setVisible(false);
	ui.CutJawFinishPanel->setVisible(true);
	return;
#endif
	glWidget->m_ModelsVt.clear();
	glWidget->update();
	ui.OralSubstitutePanel->setVisible(false);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		if (isModelFileExit(pCurrentTask) && QMessageBox::information(NULL, QStringLiteral("提示"), QStringLiteral("模型已经存在是否重新扫描"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::No) {
			//加载模型
			loadModelFile(pCurrentTask);
			//cutPaneNextStepBtnClick();
			showCutJawPanel();
		}
		else {
			pCurrentTask->Set_TaskPro(eProgressScan);
			splitModelCalculatePointCloud(pCurrentTask);
		}
	}
}

void ScanMainGUI::teethStitchingPanelNextBtnClick() {
#ifdef UITEST
	ui.OralSubstitutePanel->setVisible(false);
	ui.CutJawFinishPanel->setVisible(true);
	return;
#endif
	hideAllPanel();
	ui.TeethStitchingPanel->setVisible(false);
	//ui.StitchingFinishPanel->setVisible(true);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	pCScanTask pNextTask = CTaskManager::getInstance()->getNextTask();
	//pCurrentTask->Set_TaskPro(eProgressScan);
	if (!pNextTask) {
		showStitchingFinishPanel();
	}
	else if (pNextTask->Get_TaskType() == eUpperStitching ||
		pNextTask->Get_TaskType() == eLowerStitching) {
		emit taskSititchingSignal();
	}
	else if (pNextTask->Get_TaskType() == eScan&& pNextTask->Get_ScanType() == etoothCrown
		|| pNextTask->Get_ScanType() == einlayScan) {
		showOralSubstitutePanel();
	}
	//splitModelCalculatePointCloud(pCurrentTask);
}


void ScanMainGUI::CutJawFinishPanelNextStepBtnClick() {
#ifdef UITEST
	ui.CutJawFinishPanel->setVisible(false);
	ui.ScanJawGroup->setVisible(true);
	return;
#endif
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	pCScanTask pNextTask = CTaskManager::getInstance()->getNextTask();
	if (!pCurrentTask)
		return;
	if (pCurrentTask->Get_TaskType() != eUpperStitching &&
		pCurrentTask->Get_TaskType() != eLowerStitching &&
		pCurrentTask->Get_TaskType() != eUpperTeethStit &&
		pCurrentTask->Get_TaskType() != eLowerTeethStit)
	{
		return;
	}
	pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pCurrentTask);
	if (!pTask)
		return;
	pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
	if (!pDstTask || !pSrcTask)
		return;
	if(pCurrentTask->Get_TaskType() == eUpperTeethStit || pCurrentTask->Get_TaskType() == eLowerTeethStit)
		saveModelFile(pDstTask);
	else if (pCurrentTask->Get_TaskType() == eUpperStitching || pCurrentTask->Get_TaskType() == eLowerStitching) {
		saveModelFile(pSrcTask);
	}
	hideAllPanel();
	if (!pNextTask) {
		showStitchingFinishPanel();
	}
	else if (pNextTask->Get_TaskType() == eUpperStitching ||
		pNextTask->Get_TaskType() == eLowerStitching) {
		//emit taskSititchingSignal();
		showSubstitutePanel();
	}
	else if (pNextTask->Get_TaskType() == eScan&& pNextTask->Get_ScanType() == etoothCrown
		|| pNextTask->Get_ScanType() == einlayScan) {
		showOralSubstitutePanel();
	}
}

void ScanMainGUI::updateTaskModel()
{
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {
		if(pCurrentTask->m_mModel.size()>0){
			int scan_index = pCurrentTask->m_mModel.size() - 1;
			glWidget->mm = pCurrentTask->m_mModel[scan_index];
			glWidget->makeObject();
			glWidget->update();
		}
	}
}

void ScanMainGUI::meshFinishSlot()
{
	hideAllPanel();
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pCurrentTask) {
		return;
	}

	glWidget->m_ModelsVt.clear();
	glWidget->mm = pCurrentTask->m_mAllModel;
	pCurrentTask->m_mModel.clear();
	pCurrentTask->pTeethModel = glWidget->makeObject();
	glWidget->update();
	pCurrentTask->Set_TaskPro(eProgressMesh);

	if (pCurrentTask->Get_ScanType() == eAllJawScan) {
		m_pallTeethModel = pCurrentTask->pTeethModel;
	}
	saveModelFile(pCurrentTask);
	if (pCurrentTask->Get_Gingiva() == false&& pCurrentTask->Get_DentalImplant() == true) {//种植体
		ui.DentalImplantPanel->setVisible(true);
		glWidget->m_cutBoxesMap.clear();
		ui.lowerJawGroupBox->setVisible(false);
		ui.upperJawGroupBox->setVisible(false);
		for (int i = 0; i < 32;i++) {
			QString strWidgetName = "teethBtn_" + QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10);
			QPushButton * pButton = findChild<QPushButton*>(strWidgetName);
			if (pButton) {
				pButton->setDisabled(true);
			}
		}
		if (pCurrentTask->Get_ScanType() == eUpperJawScan)	//上颌
		{
			ui.upperJawGroupBox->setVisible(true);
		}
		else if (pCurrentTask->Get_ScanType() == eLowerJawScan) {
			ui.lowerJawGroupBox->setVisible(true);
		}
		for (int i = 0; i < pCurrentTask->m_vtTeeth.size();i++) {
			QString strWidgetName = "teethBtn_" + QString::number((pCurrentTask->m_vtTeeth[i] / 8 + 1) * 10 + pCurrentTask->m_vtTeeth[i] % 8 + 1, 10);
			QPushButton * pButton = findChild<QPushButton*>(strWidgetName);
			if (pButton) {
				pButton->setEnabled(true);
			}
		}
		ui.DentalImplantNextBtn->setDisabled(true);
		return;
	}

	pCScanTask pNextTask = CTaskManager::getInstance()->getNextTask();

	if (!pNextTask&&pCurrentTask) {
		ui.CutJawPanel->setVisible(false);
		ui.StitchingFinishPanel->setVisible(true);
		QString str;
		ui.stitchingUpperJawBtn->setVisible(false);
		ui.stitchingLowerJawBtn->setVisible(false);
		if (pCurrentTask->Get_ScanType() == eUpperJawScan) {
			m_pupperTeethModel = pCurrentTask->pTeethModel;
			ui.stitchingUpperJawBtn->setChecked(true);
			ui.stitchingUpperJawBtn->setVisible(true);
		}
		else if (pCurrentTask->Get_ScanType() == eLowerJawScan) {
			m_plowerTeethModel = pCurrentTask->pTeethModel;
			ui.stitchingLowerJawBtn->setChecked(true);
			ui.stitchingLowerJawBtn->setVisible(true);
		}
		//str = QString::fromLocal8Bit("可以通过工具栏修剪") + QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str()) + QString::fromLocal8Bit("数据");
		//ui.CutJawFinishPanelTips->setText(str);
	}
	else if (pNextTask && pCurrentTask) {
		if (pNextTask->Get_TaskType() == eScan && pNextTask->Get_ScanType() == etoothCrown
			|| pNextTask->Get_ScanType() == einlayScan) {
			showOralSubstitutePanel();
		}
		else if (pNextTask->Get_TaskType() == eScan) {
			ui.CutJawPanel->setVisible(false);
			ui.ScanJawBackStepBtn->setVisible(true);
			showScanJawGroup();
		}
		else if (pNextTask->Get_TaskType() == eUpperStitching || pNextTask->Get_TaskType() == eLowerStitching) {	//上下颌的合并
			ui.CutJawPanel->setVisible(false);
			showSubstitutePanel();
// 			ui.StitchingPanel->setVisible(true);
// 			QString str;
// 			str = QString::fromLocal8Bit("拼接") + QString::fromLocal8Bit(pNextTask->Get_TaskName().c_str());
// 			ui.StitchingPanelTips->setText(str);
			//emit taskSititchingSignal();
		}
		else if (pNextTask->Get_TaskType() == eUpperTeethStit || pNextTask->Get_TaskType() == eLowerTeethStit) {
			//牙齿拼接
			ui.CutJawPanel->setVisible(false);
			showSubstitutePanel();
// 			ui.StitchingPanel->setVisible(true);
// 			QString str;
// 			str = QString::fromLocal8Bit("拼接") + QString::fromLocal8Bit(pNextTask->Get_TaskName().c_str());
// 			ui.StitchingPanelTips->setText(str);
			//emit taskTeethSititSignal();
		}

	}
}
void ScanMainGUI::StitchFinishSlot()
{
	//glWidget->showBkGround(false);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {

		pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pCurrentTask);
		if (!pTask)
			return;
		pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
		if (!pDstTask || !pSrcTask)
			return;

		if (pCurrentTask->Get_TaskType() == eUpperStitching || pCurrentTask->Get_TaskType() == eLowerStitching) {
			ui.CutJawFinishPanel->setVisible(true);
			QString str;
			str = QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str())+QString::fromLocal8Bit("拼接完成");
			ui.CutJawFinishPanelLabel->setText(str);
			glWidget->m_ModelsVt.clear();
			//glWidget->m_ModelsVt.push_back(m_pallTeethModel);
			//if (pTask->m_mAllModel.size() > 0) {
				glWidget->mm = pSrcTask->m_mAllModel;
				pSrcTask->pTeethModel = glWidget->makeObject();
				if(m_pallTeethModel){
					glWidget->m_ModelsVt.push_back(m_pallTeethModel);
					m_pallTeethModel->Set_Visible(true);
				}
				if (pCurrentTask->Get_TaskType() == eUpperStitching) {		//上颌拼接
					m_pupperTeethModel = pSrcTask->pTeethModel;//pDstTask->pTeethModel;
					ui.stitchingUpperJawBtn->setChecked(true);
					//m_pallTeethModel = pSrcTask->pTeethModel;
				}
				else if (pCurrentTask->Get_TaskType() == eLowerStitching) {	//下颌拼接
					m_plowerTeethModel = pSrcTask->pTeethModel; //pDstTask->pTeethModel;
					ui.stitchingLowerJawBtn->setChecked(true);
				}
				//saveModelFile(pSrcTask);
				glWidget->update();
			//}
		}
	}
}
void ScanMainGUI::taskTeethSititFinishSlot()
{
	//glWidget->showBkGround(false);
	pCScanTask pCurrentTask = CTaskManager::getInstance()->getCurrentTask();
	if (pCurrentTask) {

		pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pCurrentTask);
		if (!pTask)
			return;
		pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
		if (!pDstTask || !pSrcTask)
			return;

		if (pCurrentTask->Get_TaskType() == eUpperTeethStit || pCurrentTask->Get_TaskType() == eLowerTeethStit) {
			ui.CutJawFinishPanel->setVisible(true);
			QString str;
			str = QString::fromLocal8Bit(pCurrentTask->Get_TaskName().c_str()) + QString::fromLocal8Bit("拼接完成");
			ui.CutJawFinishPanelLabel->setText(str);
			glWidget->m_ModelsVt.clear();
			//glWidget->m_ModelsVt.push_back(m_pallTeethModel);
			glWidget->mm = pDstTask->m_mRegistrationModels.back();
			pDstTask->pTeethModel = glWidget->makeObject();
			if (pCurrentTask->Get_TaskType() == eUpperTeethStit) {		//上颌拼接
				//pDstTask->pTeethModel->makeObject();
				m_pupperTeethModel = pDstTask->pTeethModel;
				//glWidget->m_ModelsVt.push_back(m_pupperTeethModel);
			//	m_pupperTeethModel = pDstTask->pTeethModel;
			//	glWidget->m_ModelsVt.push_back(m_pupperTeethModel);
				//m_pallTeethModel = pSrcTask->pTeethModel;
			}
			else if (pCurrentTask->Get_TaskType() == eLowerTeethStit) {	//下颌拼接
			//	m_plowerTeethModel = pDstTask->pTeethModel;
			//	glWidget->m_ModelsVt.push_back(m_plowerTeethModel);
				//pDstTask->pTeethModel->makeObject();
				m_plowerTeethModel = pDstTask->pTeethModel;
				//glWidget->m_ModelsVt.push_back(m_plowerTeethModel);
			}
			//saveModelFile(pDstTask);
			glWidget->update();
		}
	}
}

void ScanMainGUI::recallWindow()
{
	if (this->isVisible()) {
		raise();
		activateWindow();
		//setWindowState((windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
	}
	else if (tabMainPage->isVisible()){
		tabMainPage->raise();
		tabMainPage->activateWindow();
		//tabMainPage->setWindowState((windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
	}
}

void ScanMainGUI::showOrderInfo(COrderInfo orderInfo)
{
	resetValue();
	glWidget->m_ModelsVt.clear();
	m_plowerTeethModel = nullptr;
	m_pupperTeethModel = nullptr;
	tabMainPage->hide();
	cameraWindow->setVisible(false);
	this->ui.orderInfoPanel->setVisible(true);
	this->showMaximized();
	ui.orderNoLabel->setText(QString::fromLocal8Bit(orderInfo.strOrderNumber.c_str()));
	ui.patientNameLabel->setText(QString::fromLocal8Bit(orderInfo.strPatientName.c_str()));
	ui.doctorNameLabel->setText(QString::fromLocal8Bit(orderInfo.strDoctorName.c_str()));
	ui.operatorNameLabel->setText(QString::fromLocal8Bit(orderInfo.strOperatorName.c_str()));
	ui.upperJawBtn->setVisible(false);
	ui.lowJawBtn->setVisible(false);
	ui.teethGroupBox->setVisible(false);
	ui.jawGroupBox->setVisible(false);
	if (orderInfo.eorderType == esplitModel) {//分模
		ui.teethGroupBox->setVisible(true);
		int chooseID = -1;
		for (int i = 0; i < 32;i++) {
			chooseID = orderInfo.eTeethScanType[i];
			QString dstpath = ":/MainWidget/Resources/images/tooth" + QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10) + QString::number(chooseID+1, 10) + ".png",
				srcPath = "./Resources/images/teeth/" + QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10) + ".png";;
			QImage resultImage, destinationImage, sourceImage;
			if(chooseID != eScanNULL){				
				destinationImage.load(dstpath);
				sourceImage.load(srcPath);
				resultImage = QImage(sourceImage.size(), QImage::Format_ARGB32_Premultiplied);
				QPainter painter(&resultImage);
				painter.setCompositionMode(QPainter::CompositionMode_Source);
				painter.fillRect(resultImage.rect(), QColor(255, 255, 255, 100));
				painter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
				painter.drawImage(QRect(0, 0, resultImage.width(), resultImage.height()), destinationImage);
				painter.setCompositionMode(QPainter::CompositionMode_DestinationAtop);
				painter.drawImage(0, 0, sourceImage);
				QTextOption option;
				option.setAlignment(Qt::AlignCenter);
				painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
				painter.drawText(resultImage.rect(), QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10), option);
				painter.end();
			}
			else {
				sourceImage.load(srcPath);
				resultImage = QImage(sourceImage.size(), QImage::Format_ARGB32_Premultiplied);
				QPainter painter(&resultImage);
				painter.setCompositionMode(QPainter::CompositionMode_Source);
				painter.fillRect(resultImage.rect(), QColor(255, 255, 255, 100));
				painter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
				painter.drawImage(QRect(0, 0, resultImage.width(), resultImage.height()), destinationImage);
				painter.setCompositionMode(QPainter::CompositionMode_DestinationAtop);
				painter.drawImage(0, 0, sourceImage);
				QTextOption option;
				option.setAlignment(Qt::AlignCenter);
				painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
				painter.drawText(resultImage.rect(), QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10), option);
				painter.end();
			}
			QString strLabelName = "label";
			strLabelName.sprintf("label_%d", (i / 8 + 1) * 10 + i % 8 + 1);
			QLabel * plabel = findChild<QLabel*>(strLabelName);
			if (plabel) {
				plabel->setPixmap(QPixmap::fromImage(resultImage));
			}
		}
	}
	else if (orderInfo.eorderType == eunMoulage) {
		ui.jawGroupBox->setVisible(true);
		bool bAllJaw = false, bUpperjaw = false, blowerJaw = false;
		std::string strUpperModelNameStr = orderInfo.strFilePath + orderInfo.strOderDate + orderInfo.strUpperJawModelName + ".ply";
		if (QFile::exists(QString::fromLocal8Bit(strUpperModelNameStr.c_str()))) {
			bUpperjaw = true;
		}
		string strLowerModelNameStr = orderInfo.strFilePath + orderInfo.strOderDate + orderInfo.strLowerJawModelName + ".ply";
		if (QFile::exists(QString::fromLocal8Bit(strLowerModelNameStr.c_str()))) {
			blowerJaw = true;
		}
		if (bUpperjaw&&blowerJaw) {		//全口
			QImage image("./Resources/images/alljaw.png");
			ui.jawImage->setPixmap(QPixmap::fromImage(image));
		}
		else if (blowerJaw) {				//下口
			QImage image("./Resources/images/lowerjaw_yes.png");
			ui.jawImage->setPixmap(QPixmap::fromImage(image));
		}	
		else if (bUpperjaw) {			//上口
			QImage image("./Resources/images/upperjaw_yes.png");
			ui.jawImage->setPixmap(QPixmap::fromImage(image));
		}
	}
	std::string strUpperModelNameStr = orderInfo.strFilePath + orderInfo.strOderDate + orderInfo.strUpperJawModelName + ".ply";
	if (QFile::exists(QString::fromLocal8Bit(strUpperModelNameStr.c_str()))) {
		orth::MeshModel meshModel;
		// 	orth::ModelIO finish_model_io(&meshModel);
		// 	finish_model_io.loadModel(modelNameStr);

//		orth::ModelIO finish_model_io(&meshModel);
		//finish_model_io.loadModel(strUpperModelNameStr);
 		tinyply::plyio io;
 		io.read_ply_file(strUpperModelNameStr, meshModel);
		glWidget->mm = meshModel;
		m_pupperTeethModel = glWidget->makeObject();
		ui.upperJawBtn->setVisible(true);
		ui.upperJawBtn->setChecked(true);
	}
	string strLowerModelNameStr = orderInfo.strFilePath + orderInfo.strOderDate + orderInfo.strLowerJawModelName + ".ply";
	if (QFile::exists(QString::fromLocal8Bit(strLowerModelNameStr.c_str()))) {
		orth::MeshModel meshModel;
		// 	orth::ModelIO finish_model_io(&meshModel);
		// 	finish_model_io.loadModel(modelNameStr);
//		orth::ModelIO finish_model_io(&meshModel);
		//finish_model_io.loadModel(strLowerModelNameStr);
 		tinyply::plyio io;
 		io.read_ply_file(strLowerModelNameStr, meshModel);
		glWidget->mm = meshModel;
		m_plowerTeethModel = glWidget->makeObject();
		ui.lowJawBtn->setVisible(true);
		ui.lowJawBtn->setChecked(true);
	}
}

void ScanMainGUI::calibImageCameraSlot(int endFlag)
{
	static int nnum = 0;
	cout << "calibImageCameraSlot num" << nnum++ << endl;
	QImage leftCalibCameraImage, rightCalibCameraImage;
	int CameraImageSize = ControlScanThread->calibImageCamera.size();
	cv::Mat leftImageMat, rightImageMat;
	leftImageMat = ControlScanThread->calibImageCamera[CameraImageSize - 2];
	rightImageMat = ControlScanThread->calibImageCamera[CameraImageSize - 1];

	leftCalibCameraImage = Mat2QImage(leftImageMat);
	tabMainPage->leftCameraLable->clear();
	tabMainPage->leftCameraLable->setPixmap(QPixmap::fromImage(leftCalibCameraImage));
	tabMainPage->leftCameraLable->setScaledContents(true);
	rightCalibCameraImage = Mat2QImage(rightImageMat);
	tabMainPage->rightCameraLable->clear();
	tabMainPage->rightCameraLable->setPixmap(QPixmap::fromImage(rightCalibCameraImage));;
	tabMainPage->rightCameraLable->setScaledContents(true);
	cout << "endFlag: " << endFlag << endl;
	if (endFlag == 1)
	{
		_sleep(3000);
		tabMainPage->leftCameraLable->clear();
		tabMainPage->rightCameraLable->clear();
		tabMainPage->leftCameraLable->setStyleSheet("background-color:rgb(0,0,0);");
		tabMainPage->rightCameraLable->setStyleSheet("background-color:rgb(0,0,0);");
		tabMainPage->leftCameraLable->update();
		tabMainPage->rightCameraLable->update();
		QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("标定结束!"));
		box.setStandardButtons(QMessageBox::Yes);
		box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
		box.exec();
	}
	tabMainPage->showMaximized();
}
void ScanMainGUI::updateRegMeshSlot()
{
	glWidget->m_ModelsVt.clear();

	if (ControlComputeThread->showUpperModelFlag == true && ControlComputeThread->showLowerModelFlag == true)
	{
		glWidget->mm = ControlComputeThread->allRegModel;
		glWidget->makeObject();
	}
	else if(ControlComputeThread->showUpperModelFlag == true)
	{
		int size = ControlComputeThread->upper_mModel.size();
		glWidget->mm = ControlComputeThread->upper_mModel[size - 1];
		glWidget->makeObject();
	}
	else if (ControlComputeThread->showLowerModelFlag == true)
	{
		int size = ControlComputeThread->lower_mModel.size();
		glWidget->mm = ControlComputeThread->lower_mModel[size - 1];
		glWidget->makeObject();
	}
	else
	{
		QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("拼接失败!"));
		box.setStandardButtons(QMessageBox::Yes);
		box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
		box.exec();
		glWidget->update();
	}

	this->showMaximized();
}

void ScanMainGUI::ShowHideUpperModel()
{
	if (ControlComputeThread->regUpperModelFlag == false)
	{
		return;
	}
	glWidget->m_ModelsVt.clear();
	if (ControlComputeThread->showUpperModelFlag == true)
	{
		ControlComputeThread->showUpperModelFlag = false;
		if (ControlComputeThread->showLowerModelFlag == true)
		{
			int size = ControlComputeThread->lower_mModel.size();
			glWidget->mm = ControlComputeThread->lower_mModel[size - 1];
			glWidget->makeObject();
		}
		else
		{
			glWidget->update();
		}
	}
	else
	{
		ControlComputeThread->showUpperModelFlag = true;
		if (ControlComputeThread->showLowerModelFlag == true)
		{
			glWidget->mm = ControlComputeThread->allRegModel;
			glWidget->makeObject();
		}
		else
		{
			int size = ControlComputeThread->upper_mModel.size();
			glWidget->mm = ControlComputeThread->upper_mModel[size - 1];
			glWidget->makeObject();
		}
	}

	this->showMaximized();
}

void ScanMainGUI::ShowHideLowerModel()
{
	if (ControlComputeThread->regLowerModelFlag == false)
	{
		return;
	}
	glWidget->m_ModelsVt.clear();
	if (ControlComputeThread->showLowerModelFlag == true)
	{
		ControlComputeThread->showLowerModelFlag = false;
		if (ControlComputeThread->showUpperModelFlag == true)
		{
			int size = ControlComputeThread->upper_mModel.size();
			glWidget->mm = ControlComputeThread->upper_mModel[size - 1];
			glWidget->makeObject();
		}
		else
		{
			glWidget->update();
		}
	}
	else
	{
		ControlComputeThread->showLowerModelFlag = true;
		if (ControlComputeThread->showUpperModelFlag == true)
		{
			glWidget->mm = ControlComputeThread->allRegModel;
			glWidget->makeObject();
		}
		else
		{
			int size = ControlComputeThread->lower_mModel.size();
			glWidget->mm = ControlComputeThread->lower_mModel[size - 1];
			glWidget->makeObject();
		}
	}


	this->showMaximized();
}

bool ScanMainGUI::nativeEventFilter(const QByteArray &eventType, void *message, long *)
{
	MSG* msg = reinterpret_cast<MSG*>(message);

	if (msg->message == WM_DEVICECHANGE)
	{
		/*if (msg->hwnd != WId()) {
		return false;
		}*/
		// Tracks DBT_DEVNODES_CHANGED followed by DBT_DEVICEREMOVECOMPLETE
		//cout << "nativeEventFilter" << endl;
		if (msg->wParam == DBT_DEVNODES_CHANGED)
		{
			bPnP_DevNodeChange = true;
			bPnP_Removal = false;
			//cout << "nativeEventFilter-------changed" << endl;
			//cout << "change = " << bPnP_DevNodeChange << "; arrival = " << bPnP_Arrival << "; Removal = " << bPnP_Removal << endl;
		}

		// Tracks DBT_DEVICEARRIVAL followed by DBT_DEVNODES_CHANGED
		if (msg->wParam == DBT_DEVICEARRIVAL)
		{
			bPnP_Arrival = true;
			bPnP_DevNodeChange = false;
			//cout << "nativeEventFilter-------arrival" << endl;
			//cout << "change = " << bPnP_DevNodeChange << "; arrival = " << bPnP_Arrival << "; Removal = " << bPnP_Removal << endl;
		}

		if (msg->wParam == DBT_DEVICEREMOVECOMPLETE)
		{
			bPnP_Removal = true;
			//cout << "nativeEventFilter-------remove" << endl;
			//cout << "change = " << bPnP_DevNodeChange << "; arrival = " << bPnP_Arrival << "; Removal = " << bPnP_Removal << endl;
		}

		//If DBT_DEVICEARRIVAL followed by DBT_DEVNODES_CHANGED
		if (bPnP_DevNodeChange && bPnP_Removal)
		{
			bPnP_Removal = false;
			bPnP_DevNodeChange = false;

			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("设备USB被拔出请重新连接!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();

			/*ControlScanThread->l_usbStream.AbortXferLoop();
			void *handle = (void *)winId();
			ControlScanThread->l_usbStream.InitCyUSBParameter(handle);*/
			ControlScanThread->l_usbStream.CloseUSB();
			void *handle = (void *)winId();
			ControlScanThread->l_usbStream.OpenUSB(handle);
		}

		// If DBT_DEVICEARRIVAL followed by DBT_DEVNODES_CHANGED
		if (bPnP_DevNodeChange && bPnP_Arrival)
		{
			bPnP_Arrival = false;
			bPnP_DevNodeChange = false;
			//bPnP_Removal = false;
			//	installNativeEventFilter();
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("设备USB已连接!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
			void *handle = (void *)winId();
			ControlScanThread->l_usbStream.OpenUSB(handle);
			if (!m_usbDeviceState)
			{
				ControlScanThread->l_usbStream.InitUSBBufferParameter();
				m_usbDeviceState = true;
			}
		}
	}
	return false;
};

void ScanMainGUI::initUSBDevice()
{
	void *handle = (void *)winId();
	int usbDeviceState = ControlScanThread->l_usbStream.InitCyUSBParameter(handle);//初始化
	if (usbDeviceState != 0)
	{
		m_usbDeviceState = false;
	}
	else
	{
		m_usbDeviceState = true;
		bPnP_DevNodeChange = 1;
	}
	emit usbDeviceSignal();
}

void ScanMainGUI::installNativeEventFilter()
{
	QApplication::instance()->installNativeEventFilter(this);
}

void ScanMainGUI::usbDeviceSlot()
{
	if (!m_usbDeviceState)
	{
		QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("设备未与电脑连接!"));
		box.setStandardButtons(QMessageBox::Yes);
		box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
		box.exec();
	}
}