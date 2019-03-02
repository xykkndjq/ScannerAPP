#include "ScanMainGUI.h"

ScanMainGUI::ScanMainGUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->initVariable();
	this->constructIHM();
	this->setConnections();
}

ScanMainGUI::~ScanMainGUI()
{
}


void ScanMainGUI::initVariable()
{
	glWidget = new GLWidget(this);

	//cameraWindow
	cameraWindow = new QDockWidget(QStringLiteral("相机显示"), this);
	//buttontoolButtons
	leftWatchButton = new QToolButton(this);
	rightWatchButton = new QToolButton(this);
	topWatchButton = new QToolButton(this);
	bottomWatchButton = new QToolButton(this);
	frontWatchButton = new QToolButton(this);
	backWatchButton = new QToolButton(this);

	spinCameraBox = new QSpinBox(this);
	sliderCamera = new QSlider(Qt::Horizontal);
	spinCameraBox->setRange(0, 130);
	sliderCamera->setRange(0, 130);
}

void ScanMainGUI::constructIHM()
{
	//glWidget->showMaximized();
	glWidget->setGeometry(0,0,1920,1080);
	glWidget->setWindowSize(QSize(1920, 1080));
	//glWidget->showFullScreen();
	//相机设置
	//cameraWindow->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable); // 设置停靠窗口特性，可移动,可关闭
	//cameraWindow->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);//设置可停靠区域为主窗口左边和右边
	//QTextEdit *textEdit = new QTextEdit("DockWindow First");

	//QWidget *bottomCameraWidget = new QWidget(this);
	//QHBoxLayout *bottomCameraHLayout = new QHBoxLayout(bottomCameraWidget);
	//bottomCameraHLayout->addWidget(spinCameraBox);
	//bottomCameraHLayout->addWidget(sliderCamera);
	//bottomCameraHLayout->setContentsMargins(0, 0, 0, 0);

	//QWidget *cameraWidget = new QWidget(this);
	//QVBoxLayout *cameraVLayout = new QVBoxLayout(cameraWidget);
	//cameraVLayout->addWidget(textEdit);
	//cameraVLayout->addWidget(bottomCameraWidget);
	//cameraVLayout->setContentsMargins(0, 0, 0, 0);

	//cameraWindow->setWidget(cameraWidget);
	//QWidget *totalCameraWidget = new QWidget(this);
	//totalCameraWidget->setFixedSize(400, 400);
	//QHBoxLayout *totalCameraHLayout = new QHBoxLayout(totalCameraWidget);
	//totalCameraHLayout->addWidget(cameraWindow);
	//totalCameraHLayout->addStretch();
	//totalCameraHLayout->setContentsMargins(0, 0, 0, 0);

	////MiddleDialog中间对话提示框


	////BottomTool底部工具栏
	////leftWatchButton->setFixedSize(20,20);
	//leftWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/LeftView.png"));
	//leftWatchButton->setIconSize(QSize(leftWatchButton->width(), leftWatchButton->height()));
	//leftWatchButton->setStyleSheet("border-style:flat");
	////connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	////rightWatchButton->setFixedSize(20, 20);
	//rightWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/RightView.png"));
	//rightWatchButton->setIconSize(QSize(rightWatchButton->width(), rightWatchButton->height()));
	//rightWatchButton->setStyleSheet("border-style:flat");
	////connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	////topWatchButton->setFixedSize(20, 20);
	//topWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/TopView.png"));
	//topWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	//topWatchButton->setStyleSheet("border-style:flat");
	////connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	////bottomWatchButton->setFixedSize(20, 20);
	//bottomWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BottomView.png"));
	//bottomWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	//bottomWatchButton->setStyleSheet("border-style:flat");
	////connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	//QWidget *bottomButtonWidget = new QWidget(this);
	//bottomButtonWidget->setFixedSize(300, 100);
	//QHBoxLayout *bottomButtonHLayout = new QHBoxLayout(bottomButtonWidget);

	//bottomButtonHLayout->addWidget(leftWatchButton);
	//bottomButtonHLayout->addWidget(rightWatchButton);
	//bottomButtonHLayout->addWidget(topWatchButton);
	//bottomButtonHLayout->addWidget(bottomWatchButton);
	//
	//QWidget *bottomWidget = new QWidget(this);
	//QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
	//bottomHLayout->addStretch();
	//bottomHLayout->addWidget(bottomButtonWidget);
	//bottomHLayout->addStretch();

	//QVBoxLayout *totalVLayout = new QVBoxLayout(this);
	//totalVLayout->addWidget(totalCameraWidget);
	////totalVLayout->addWidget(glWidget);
	//totalVLayout->addStretch();
	//totalVLayout->addWidget(bottomWidget);
}

void ScanMainGUI::setConnections()
{
	QObject::connect(sliderCamera, &QSlider::valueChanged, spinCameraBox, &QSpinBox::setValue);
	void (QSpinBox:: *spinBoxSignal)(int) = &QSpinBox::valueChanged;
	QObject::connect(spinCameraBox, spinBoxSignal, sliderCamera, &QSlider::setValue);
	spinCameraBox->setValue(35);
}