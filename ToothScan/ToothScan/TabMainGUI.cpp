#include "TabMainGUI.h"
#include <QPainter>
#include <QProxyStyle>
#include <QTextCodec>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QGraphicsDropShadowEffect>
#include "commonFun.h"
#include "SystemConfig.h"
QWidget *g_pCurrentWidget = nullptr;
string Qstring2String(QString qString) {
	QByteArray cdata = qString.toLocal8Bit();
	return string(cdata);
}
class CustomTabStyle : public QProxyStyle
{
public:
	QSize sizeFromContents(ContentsType type, const QStyleOption *option,
		const QSize &size, const QWidget *widget) const
	{
		QSize s = QProxyStyle::sizeFromContents(type, option, size, widget);
		if (type == QStyle::CT_TabBarTab) {
			s.transpose();
			s.rwidth() = 100; // 设置每个tabBar中item的大小
			s.rheight() = 60;
			if (widget->parent()->objectName() == "selftabControl") {
				s.transpose();
				s.rwidth() = 200; // 设置每个tabBar中item的大小
				s.rheight() = 60;
			}
		}
		return s;
	}

	void drawControl(ControlElement element, const QStyleOption *option, QPainter *painter, const QWidget *widget) const
	{
		if (element == CE_TabBarTab) {
			if (const QStyleOptionTab *tab = qstyleoption_cast<const QStyleOptionTab *>(option)) {
				if (widget->parent()->objectName() == "selftabControl") {
					QRect allRect = tab->rect;
					if (tab->state & QStyle::State_Selected) {
						painter->save();
						painter->setPen(QColor(255, 255, 255, 0));
						painter->setBrush(QBrush(QColor(255, 255, 255)));
						painter->drawRect(allRect);
						painter->setPen(0x89cfff);
						painter->setBrush(QBrush(0x89cfff));
						painter->drawRect(allRect.adjusted(0, allRect.height() - 5, 0, 0));
						painter->restore();
					}
					QTextOption option;
					option.setAlignment(Qt::AlignCenter);
					if (tab->state & QStyle::State_Selected) {
						painter->setPen(QColor(128, 128, 128));
					}
					else {
						painter->setPen(QColor(128, 128, 128));
					}

					painter->drawText(allRect, tab->text, option);
					return;
				}
			}
		}
		if (element == CE_TabBarTabLabel) {
			if (const QStyleOptionTab *tab = qstyleoption_cast<const QStyleOptionTab *>(option)) {
				if (widget->parent()->objectName() == "selftabControl") {
					QRect allRect = tab->rect;

					if (tab->state & QStyle::State_Selected) {
						painter->save();
						painter->setPen(0x89cfff);
						painter->setBrush(QBrush(0x89cfff));
						painter->drawRect(allRect.adjusted(6, 6, -6, -6));
						painter->drawRoundRect(allRect.adjusted(6, 6, -1, 0), 10, 10);
						painter->setBrush(QBrush(0xffffff));
						painter->setPen(QColor(128, 128, 128,0));
						painter->drawRect(allRect.adjusted(6, 10, 0, 0));
						painter->setPen(QColor(128, 128, 128, 255));
						painter->drawLine(allRect.x(), allRect.height()-1, allRect.x()+6,allRect.height()-1);
						painter->drawLine(allRect.x()+6, allRect.height()-1, allRect.x()+6, allRect.y()+10);
						painter->drawLine(allRect.right(), allRect.y()+10, allRect.right(), allRect.height() - 1);
						painter->restore();
					}
					QTextOption option;
					option.setAlignment(Qt::AlignCenter);
					if (tab->state & QStyle::State_Selected) {
						painter->setPen(QColor(128, 128, 128));
					}
					else {
						painter->setPen(QColor(128, 128, 128));
					}

					painter->drawText(allRect, tab->text, option);
					return;
				}
				QRect allRect = tab->rect;

				if (tab->state & QStyle::State_Selected) {
					painter->save();
					painter->setPen(0x89cfff);
					painter->setBrush(QBrush(0x89cfff));
					painter->drawRect(allRect.adjusted(6, 6, -6, -6));
					painter->restore();
				}
				QTextOption option;
				option.setAlignment(Qt::AlignCenter);
				if (tab->state & QStyle::State_Selected) {
					painter->setPen(0xf8fcff);
				}
				else {
					painter->setPen(0x5d5d5d);
				}

				painter->drawText(allRect, tab->text, option);
				return;
			}
			
		}
		else {
			painter->setPen(QColor(128, 128, 128, 255));
		}

		if (element == CE_TabBarTab) {
			QProxyStyle::drawControl(element, option, painter, widget);
		}
		QProxyStyle::drawControl(element, option, painter, widget);
	}
};

void addShadow(QWidget * pWidget) {
	if (pWidget == nullptr)
		return;
	QGraphicsDropShadowEffect *shadow_effect = new QGraphicsDropShadowEffect();

	shadow_effect->setOffset(0, 0);

	shadow_effect->setColor(/*Qt::gray*/QColor(243,243,243,255));

	shadow_effect->setBlurRadius(8);
	pWidget->setGraphicsEffect(shadow_effect);
}

void styleControl(QObject *obj) {
	//return;
	QObjectList list = obj->children();
	QWidget *b;
	QBoxLayout *pBoxLayout;
	foreach(QObject *obj, list)
	{
		pBoxLayout = static_cast<QBoxLayout*>(obj);
		if (obj->metaObject()->className() == QStringLiteral("QPushButton")
			|| obj->metaObject()->className() == QStringLiteral("QDateTimeEdit")
			|| obj->metaObject()->className() == QStringLiteral("QLineEdit")
			|| obj->metaObject()->className() == QStringLiteral("QTextEdit"))
		{
			b = static_cast<QWidget*>(obj);
			if(b){
				QString strStyleSheet = b->styleSheet();
				strStyleSheet += "border:0px groove gray;border-radius:5px;padding:2px 4px;border-color: rgb(128, 128, 128);background: rgb(255, 255, 255);";
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
			styleControl(pBoxLayout);
		}
	}
}
TabMainGUI::TabMainGUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->initVariable();
	this->constructIHM();
	this->setConnections();
	m_eScanType = eScanType::eScanNULL;
	//network_group_box->setGraphicsEffect(shadow_effect);
	//styleControl(this);
	setWindowFlags(Qt::FramelessWindowHint);
	qRegisterMetaType<COrderInfo>();

	CSystemConfig::shareInstance()->load();
	if (CSystemConfig::shareInstance()->getValue(STR_SCANDATAPATH) != "") {
		ui.scanPathLineEdit->setText(CSystemConfig::shareInstance()->getValue(STR_SCANDATAPATH).c_str());
		filePath = CSystemConfig::shareInstance()->getValue(STR_SCANDATAPATH).c_str();
		//ui.saveSpliteModelCheckBox->setChecked(true);
	}
	if (CSystemConfig::shareInstance()->getValue(B_GPU) != "") {
		if (CSystemConfig::shareInstance()->getValue(B_GPU) == "true") {
			ui.GPUCheckBox->click();
		}
	}
}

TabMainGUI::~TabMainGUI()
{
	CSystemConfig::shareInstance()->save();
}

void TabMainGUI::setConnections()
{
	/*-----------------------------------------------------------update tabMainGui:begin-------------------------------------------------*/
	/*----------------------关闭软件------------------------*/
	connect(ui.btnClose, SIGNAL(clicked()), this, SLOT(closeBtnClicked()));//关闭
	connect(ui.btnMin, SIGNAL(clicked()), this, SLOT(btnMinClicked()));//关闭
	
	/*----------------------打开订单管理子页面------------------------*/
	connect(m_orderMgrBtn, SIGNAL(clicked()), this, SLOT(showOrderInforGroupBox()));//打开设置子页面
	//订单的新建、导入、预览、保存、订单扫描
	connect(ui.newPushButton, SIGNAL(clicked()), this, SLOT(newButtonClickedSlot()));//新建
	connect(ui.openPushButton, SIGNAL(clicked()), this, SLOT(openFileDialogSlot()));//导入
	connect(ui.watchPushButton, SIGNAL(clicked()), this, SLOT(OrderPreview()));//预览
	connect(ui.savePushButton, SIGNAL(clicked()), this, SLOT(PatientInformationSave()));//保存订单信息
	connect(ui.scanPushButton, SIGNAL(clicked()), this, SLOT(PatientInformationSave()));//保存并扫描订单
	connect(this, SIGNAL(scanSignal()), this, SLOT(ScanDataPackagePress()));//扫描按钮连接

	//未分模
	connect(ui.unModelPushButton, SIGNAL(clicked()), this, SLOT(showUnModelGroupBox()));//选择未分模功能
	connect(ui.upperJawPushButton, SIGNAL(clicked()), this, SLOT(UpperJawPress()));
	connect(this, SIGNAL(exportUpperJawSignal()), this, SLOT(UpperJawPress()));
	connect(ui.lowerJawPushButton, SIGNAL(clicked()), this, SLOT(LowerJawPress()));
	connect(this, SIGNAL(exportLowerJawSignal()), this, SLOT(LowerJawPress()));

	//分模
	connect(ui.spliteModelPushButton, SIGNAL(clicked()), this, SLOT(showSpliteModelGroupBox()));//选择未分模功能
	connect(toothRadioButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(ToothGroupClicked(int)));
	connect(ui.clearAllButton, SIGNAL(clicked()), this, SLOT(clearAllButtonPress()));

	for (int i = 0; i < TOOTHNUM; i++)
	{
		connect(toothList[i], SIGNAL(clicked()), this, SLOT(ToothButtonListPress()));
	}

	//印模
	connect(ui.moulagePushButton, SIGNAL(clicked()), this, SLOT(showMoulageGroupBox()));//选择未分模功能
	connect(ui.moulagePushButton_1, SIGNAL(clicked()), this, SLOT(MoulagePress1()));
	connect(this, SIGNAL(exportFirstMoulageSignal()), this, SLOT(MoulagePress1()));
	connect(ui.moulagePushButton_2, SIGNAL(clicked()), this, SLOT(MoulagePress2()));
	connect(this, SIGNAL(exportSecondMoulageSignal()), this, SLOT(MoulagePress2()));
	connect(ui.moulagePushButton_3, SIGNAL(clicked()), this, SLOT(MoulagePress3()));
	connect(this, SIGNAL(exportThirdMoulageSignal()), this, SLOT(MoulagePress3()));

	/*----------------------打开标定子页面------------------------*/
	connect(m_calibrationMgrBtn, SIGNAL(clicked()), this, SLOT(showCalibrationGroupBox()));//打开标定子页面

	/*----------------------打开设置子页面------------------------*/
	connect(m_settingMgrBtn, SIGNAL(clicked()), this, SLOT(showSettingGroupBox()));//打开设置子页面
	//connect(ui.checkBoxGroupBox, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(settingButtonClicked(QAbstractButton*)));
	connect(ui.saveSpliteModelCheckBox, SIGNAL(clicked()), this, SLOT(saveSpliteModelCheckBoxClicked()));//保存单帧数据
	connect(ui.choosePathPushButton, SIGNAL(clicked()), this, SLOT(openDirectoryDialogSlot()));

	/*----------------------打开关于子页面------------------------*/
	connect(m_aboutMgrBtn, SIGNAL(clicked()), this, SLOT(showAboutGroupBox()));//打开设置子页面
	/*-------------------------------------------------------update tabMainGui:end---------------------------------------------------------*/
}



void TabMainGUI::initVariable()
{
	//totalGLayOut = new QGridLayout(this);

	//totalTabWidget = new QTabWidget(this);

	//orderInforWidget = new QWidget(this);
	//settingWidget = new QWidget(this);
	//calibrateWidget = new QWidget(this);
	//aboutWidget = new QWidget(this);

	//orderInformPage订单管理页面
	//topbutton
	//newButton = new QPushButton(QStringLiteral("新   建"), this);
	//newButton->setFixedSize(180, 30);

	//exportButton = new QPushButton(QStringLiteral("导   入"), this);
	//exportButton->setFixedSize(180, 30);
	//saveButton = new QPushButton(QStringLiteral("保存"), this);
	//watchButton = new QPushButton(QStringLiteral("预   览"), this);
	//watchButton->setFixedSize(180, 30);
	//saveScanButton = new QPushButton(QStringLiteral("扫描"), this);
	//saveScanButton->setFixedSize(180, 30);

	//saveButton = new QPushButton(QStringLiteral("保存"), this);
	//saveButton->setFixedSize(180, 30);

	//m_closeBtn = new CTeethImgBtn("./Resources/images/closebtn.png","",this);
	//m_closeBtn->setFixedSize(30, 30);
	
	addShadow(ui.mainGroupBox);
	/*订单管理页面*/
	
	//利用当前时间设置订单编号
	QDateTime currentDateTime = QDateTime::currentDateTime();
	QString curDateTimeStr = currentDateTime.toString("yyyy/MM/dd/hh:mm:ss");
	ui.orderDateLineEdit->setText(curDateTimeStr);
	ui.orderDateLineEdit->setReadOnly(true);

	curDateTimeStr = currentDateTime.toString("yyyyMMddhhmmss");
	
	lastDateTimeStr = curDateTimeStr;
	ui.orderNumLineEdit->setText(curDateTimeStr);
	QRegExp regx("[0-9]+$");
	QValidator* validator = new QRegExpValidator(regx, ui.orderNumLineEdit);
	ui.orderNumLineEdit->setValidator(validator);

	//顶部：订单管理、设置、标定、关于控件
	headerButtonGroup = new QButtonGroup();
	headerButtonGroup->setExclusive(true);
	m_orderMgrBtn = new CHeaderCheckBtn(":/MainWidget/Resources/images/orderIconCheck.png",
		":/MainWidget/Resources/images/orderIconUnCheck.png", QStringLiteral("订单管理"),
		QColor(255, 255, 255, 0), QColor(40, 138, 237, 255), QColor(160, 160, 160, 255),
		QColor(40, 138, 237, 255), ui.mainGroupBox);
	m_settingMgrBtn = new CHeaderCheckBtn(":/MainWidget/Resources/images/setIconChecked.png",
		":/MainWidget/Resources/images/setIconUnCheck.png", QStringLiteral("设置"),
		QColor(255, 255, 255, 0), QColor(40, 138, 237, 255), QColor(160, 160, 160, 255),
		QColor(40, 138, 237, 255), ui.mainGroupBox);;
	m_calibrationMgrBtn = new CHeaderCheckBtn(":/MainWidget/Resources/images/bdIconChecked.png",
		":/MainWidget/Resources/images/bdIconUnCheck.png", QStringLiteral("标定"),
		QColor(255, 255, 255, 0), QColor(40, 138, 237, 255), QColor(160, 160, 160, 255),
		QColor(40, 138, 237, 255), ui.mainGroupBox);;
	m_aboutMgrBtn = new CHeaderCheckBtn(":/MainWidget/Resources/images/aboutIconCheck.png",
		":/MainWidget/Resources/images/aboutIconUncheck.png", QStringLiteral("关于"),
		QColor(255, 255, 255, 0), QColor(40, 138, 237, 255), QColor(160, 160, 160, 255),
		QColor(40, 138, 237, 255), ui.mainGroupBox);
	m_orderMgrBtn->setGeometry(717, 40, 110, 60);
	m_settingMgrBtn->setGeometry(857, 40, 110, 60);
	m_calibrationMgrBtn->setGeometry(997, 40, 110, 60);
	m_aboutMgrBtn->setGeometry(1137, 40, 110, 60);
	m_orderMgrBtn->setCheckable(true);
	m_settingMgrBtn->setCheckable(true);
	m_calibrationMgrBtn->setCheckable(true);
	m_aboutMgrBtn->setCheckable(true);
	headerButtonGroup->addButton(m_orderMgrBtn);
	headerButtonGroup->addButton(m_settingMgrBtn);
	headerButtonGroup->addButton(m_calibrationMgrBtn);
	headerButtonGroup->addButton(m_aboutMgrBtn);

	m_orderMgrBtn->setChecked(true);

	//右侧：未分模、分模、印模
	rightButtonGroup = new QButtonGroup();
	rightButtonGroup->setExclusive(true);
	rightButtonGroup->addButton(ui.unModelPushButton);
	rightButtonGroup->addButton(ui.spliteModelPushButton);
	rightButtonGroup->addButton(ui.moulagePushButton);
	ui.unModelPushButton->setChecked(true);


	//分模
	//split
	for (int i = 0; i < TOOTHNUM; i++)
	{
		int row = i / 8;
		int col = i % 8;
		int value = (row + 1) * 10 + col + 1;
		QString path = ":/MainWidget/Resources/images/0/" + QString::number(value, 10) + ".png";
		//QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}";
		CTeethImgBtn *toothPushButton = new CTeethImgBtn(path, QString::number((i / 8 + 1) * 10 + i % 8 + 1, 10));
		toothList.append(toothPushButton);
		toothList[i]->setObjectName(QString::number(i, 10));
		toothFlagList.append(false);
		addShadow(toothPushButton);
		toothID.append(0);
		m_pTeethScanTaskArray[i] = make_shared<CScanTask>();
		m_pTeethScanTaskArray[i]->Set_TeethId(i);
	}

	//splitright
	toothRadioButtonGroup = new QButtonGroup();
	toothRadioButtonGroup->setExclusive(true);

	totalCrownButton = new CImageBtn("./Resources/images/facing.png",QColor(255,0,0,100),QStringLiteral("牙冠"));

	totalCrownButton->setCheckable(true);

	toothCrownButton = new CImageBtn("./Resources/images/facing.png", QColor(126, 90, 34, 100), QStringLiteral("嵌体"));
	toothCrownButton->setCheckable(true);

	lossToothButton = new CImageBtn("./Resources/images/loseTooth.png", QColor(0, 255, 0, 100), QStringLiteral("缺失牙"));
	lossToothButton->setCheckable(true);

	inlayButton = new CImageBtn("./Resources/images/inlay.png", QColor(0, 0, 255, 100), QStringLiteral("种植牙"));
	inlayButton->setCheckable(true);


	jawToothButton = new CImageBtn("./Resources/images/inlay.png", QColor(0, 253, 255, 100), QStringLiteral("对颌牙"));
	jawToothButton->setCheckable(true);

	facingButton = new QCheckBox(QStringLiteral("           贴面"));
	/*facingButton->setFixedSize(150, 30);
	facingButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	facingButton->setIcon(QIcon(":/MainWidget/Resources/images/facing.png"));*/
	waxTypeButton = new QCheckBox(QStringLiteral("               蜡型"));
	/*waxTypeButton->setFixedSize(150, 30);
	waxTypeButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");*/
	implantButton = new QCheckBox(QStringLiteral("             种植体"));
	/*implantButton->setFixedSize(150, 30);
	implantButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");*/

	/*设置初始页面*/
	ui.settingGroupBox->setVisible(false);
	ui.aboutGroupBox->setVisible(false);
	ui.calibrationGroupBox->setVisible(false);
	ui.orderGroupBox->setVisible(true);
	ui.unModelGroupBox->setVisible(true);
	ui.spliteModelGroupBox->setVisible(false);
	ui.moulageGroupBox->setVisible(false);




//	//关于页面
//	aboutTextLabel = new QLabel(this);
//	aboutImageLabel = new QLabel(this);
//
//	//calibatepage标定页面
//	calibratePushButton = new QPushButton(QStringLiteral("开始标定"), this);
//	calibratePushButton->setFixedSize(150, 30);
//	globalCaliPushButton = new QPushButton(QStringLiteral("全局标定"), this);
//	globalCaliPushButton->setFixedSize(150, 30);
//	leftCameraLable = new QLabel(this);
//	leftCameraLable->setStyleSheet("background-color:rgb(0,0,0);");
//	leftCameraLable->setScaledContents(true);
//	leftCameraLable->setFixedSize(800, 640);
//	rightCameraLable = new QLabel(this);
//	rightCameraLable->setStyleSheet("background-color:rgb(0,0,0);");
//	rightCameraLable->setScaledContents(true);
//	rightCameraLable->setFixedSize(800, 640);
//
//	//设置子页面
//	settingButtonGroup = new QButtonGroup(this);
//	textureCheckBox = new QCheckBox(this);
//	textureCheckBox->setText(QStringLiteral("纹理"));
//
//	textureCheckBox->setFixedSize(100, 60);
//	ACheckBox = new QCheckBox(this);
//	ACheckBox->setText(QStringLiteral("标志点"));
//	ACheckBox->setFixedSize(100, 60);
//	BCheckBox = new QCheckBox(this);
//	BCheckBox->setText(QStringLiteral("颌架"));
//	BCheckBox->setFixedSize(100, 60);
//	saveSpliteModelCheckBox = new QCheckBox(this);
//	saveSpliteModelCheckBox->setText(QStringLiteral("保存单帧模型"));
//	saveSpliteModelCheckBox->setFixedSize(150, 60);
//
//	settingButtonGroup->addButton(textureCheckBox);
//	settingButtonGroup->addButton(ACheckBox);
//	settingButtonGroup->addButton(BCheckBox);
//	//settingButtonGroup->addButton(saveSpliteModelCheckBox);
//	settingButtonGroup->setExclusive(false);
//
//	scanDataPathLabel = new QLabel(QStringLiteral("扫描路径:"), this);
//	scanDataPathEdit = new QLineEdit(this);
//	scanDataPathEdit->setText("./ScanData");
//	choosePathButton = new QPushButton(QStringLiteral("选择路径"), this);
//
//	{
//		facingButton->setVisible(false);
//		waxTypeButton->setVisible(false);
//		implantButton->setVisible(false);
//		//jawToothButton->setVisible(false);
//	}
	g_pCurrentWidget = this;
}


void TabMainGUI::constructIHM()
{
//	totalTabWidget->setTabPosition(QTabWidget::West);
//	totalTabWidget->setStyle(new CustomTabStyle);
//	totalTabWidget->setStyleSheet("QTabWidget::pane{ \border-left: 1px solid #eeeeee;}");
//
//	//订单管理子页面
//	totalOrderVLayout = new QVBoxLayout(orderInforWidget);
//
//	//topwidget
//	QWidget *topWidget = new QWidget();
//	QHBoxLayout *topHLayout = new QHBoxLayout(topWidget);
//	topHLayout->addStretch(1);
//	topHLayout->addWidget(newButton);
//	topHLayout->addStretch(1);
//	topHLayout->addWidget(exportButton);
//	topHLayout->addWidget(saveButton);
//	topHLayout->addWidget(watchButton);
//	topHLayout->addStretch(6);
//	topHLayout->addWidget(saveScanButton);
//	topHLayout->addStretch(8);
//	topHLayout->addWidget(m_closeBtn);
//
//	//leftwidget
//	QWidget *leftTopWidget = new QWidget();
//	QFormLayout *leftTopFLayout = new QFormLayout(leftTopWidget);
//
//	leftTopFLayout->addRow(QStringLiteral("订单日期:"), dateLineEdit);
//	leftTopFLayout->addRow(QStringLiteral("订单编号:"), orderLineEdit);
//	leftTopFLayout->addRow(QStringLiteral("患者姓名:"), patientLineEdit);
//	leftTopFLayout->addRow(QStringLiteral("医生姓名:"), doctorLineEdit);
//	leftTopFLayout->addRow(QStringLiteral("技师姓名:"), operatorLineEdit);
//
//	QWidget *leftBottomWidget = new QWidget();
//	QGridLayout *leftBottomFLayout = new QGridLayout(leftBottomWidget);
//	leftBottomFLayout->addWidget(additionLabel);
//	leftBottomFLayout->addWidget(additionTextEdit);
//
//	QWidget *leftWidget = new QWidget();
//	QGridLayout *leftGLayout = new QGridLayout(leftWidget);
//	leftGLayout->addWidget(leftTopWidget);
//	leftGLayout->addWidget(leftBottomWidget);
//	QWidget *totalleftWidget = new QWidget();
//	QHBoxLayout *leftHLayout = new QHBoxLayout(totalleftWidget);
//	leftHLayout->addWidget(leftWidget);
//	leftHLayout->addStretch();
//	//rightWidget
//	rightTabWidget = new QTabWidget();
//	rightTabWidget->setStyleSheet("border:0px");
//	rightTabWidget->setStyle(new CustomTabStyle);
//	//未分模
//	QWidget *rightTotalModelVWidget = new QWidget();
//	QVBoxLayout *rightTotalModelVLayout = new QVBoxLayout(rightTotalModelVWidget);
//	QWidget *rightTotalModelHWidget = new QWidget();
//	QHBoxLayout *rightTotalModelHLayout = new QHBoxLayout(rightTotalModelHWidget);
//	rightTotalModelVLayout->addStretch();
//	rightTotalModelVLayout->addWidget(upperJawButton);
//	rightTotalModelVLayout->addWidget(lowerJawButton);
//	rightTotalModelVLayout->addStretch();
//
//	rightTotalModelHLayout->addStretch();
//	rightTotalModelHLayout->addWidget(rightTotalModelVWidget);
//	rightTotalModelHLayout->addStretch();
//
//	//印模
//	QWidget *rightMoulageVWidget = new QWidget();
//	QVBoxLayout *rightMoulageVLayout = new QVBoxLayout(rightMoulageVWidget);
//	rightMoulageVLayout->addStretch();
//	rightMoulageVLayout->addWidget(MoulageButton1);
//	rightMoulageVLayout->addWidget(MoulageButton2);
//	rightMoulageVLayout->addWidget(MoulageButton3);
//	rightMoulageVLayout->addStretch();
//	QWidget *rightMoulageHWidget = new QWidget();
//	QHBoxLayout *rightMoulageHLayout = new QHBoxLayout(rightMoulageHWidget);
//	rightMoulageHLayout->addStretch();
//	rightMoulageHLayout->addWidget(rightMoulageVWidget);
//	rightMoulageHLayout->addStretch();
//
//	rightTabWidget->addTab(rightTotalModelHWidget, QStringLiteral("未分模"));
//	rightTabWidget->addTab(middleSplitModelWidget, QStringLiteral("分模"));
//	rightTabWidget->addTab(rightMoulageHWidget, QStringLiteral("印模"));
//	rightTabWidget->setObjectName("selftabControl");
////	rightTotalModelHWidget->setStyleSheet("border:0px");
//
//	QWidget *bottomWidget = new QWidget();
//	QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
//	bottomHLayout->setContentsMargins(0, 0, 0, 0);
//	bottomHLayout->setSpacing(0);
//	bottomHLayout->addWidget(totalleftWidget);
//	//bottomHLayout->setStretch();
//	bottomHLayout->addWidget(rightTabWidget);
//
//	totalOrderVLayout->addWidget(topWidget);
//	totalOrderVLayout->addWidget(bottomWidget);
//
//	//关于子页面
//	totalAboutGLayout = new QGridLayout(aboutWidget);
//
//	aboutImageLabel->setPixmap(QPixmap(":/MainWidget/Resources/images/Setting.png"));
//	aboutTextLabel->setStyleSheet(QString("color:rgb(128,128,128);"));
//	aboutTextLabel->setText(QStringLiteral("牙齿扫描仪软件是一款对牙齿模型进行扫描，取得相应三维模型的软件，该软件配套牙齿扫描仪器使用。"));
//
//	totalAboutGLayout->addWidget(aboutImageLabel, 1, 0, 1, 1);
//	totalAboutGLayout->addWidget(aboutTextLabel, 1, 3, 1, 2);
//
//	//标定子页面
//	QVBoxLayout *calibVLayout = new QVBoxLayout(calibrateWidget);
//
//	QWidget *topCalibHWidget = new QWidget();
//	QHBoxLayout *topCalibHLayout = new QHBoxLayout(topCalibHWidget);
//	topCalibHLayout->addStretch(1);
//	topCalibHLayout->addWidget(globalCaliPushButton);
//	topCalibHLayout->addStretch(2);
//	topCalibHLayout->addWidget(calibratePushButton);
//	topCalibHLayout->addStretch(1);
//
//	QWidget *bottomCalibHWidget = new QWidget();
//	QHBoxLayout *bottomCalibHLayout = new QHBoxLayout(bottomCalibHWidget);
//	bottomCalibHLayout->addStretch(2);
//	bottomCalibHLayout->addWidget(leftCameraLable);
//	bottomCalibHLayout->addStretch(3);
//	bottomCalibHLayout->addWidget(rightCameraLable);
//	bottomCalibHLayout->addStretch(2);
//	calibVLayout->addStretch(2);
//	calibVLayout->addWidget(topCalibHWidget);
//	calibVLayout->addStretch(1);
//	calibVLayout->addWidget(bottomCalibHWidget);
//	calibVLayout->addStretch(3);
//
//	//设置子页面
//	QVBoxLayout *settingVLayout = new QVBoxLayout(settingWidget);
//
//	QWidget *topSettingHWidget = new QWidget();
//	QHBoxLayout *topSettingHLayout = new QHBoxLayout(topSettingHWidget);
//	topSettingHLayout->addStretch();
//	topSettingHLayout->addWidget(textureCheckBox);
//	topSettingHLayout->addWidget(ACheckBox);
//	topSettingHLayout->addWidget(BCheckBox);
//	topSettingHLayout->addStretch();
//	topSettingHLayout->addWidget(saveSpliteModelCheckBox);
//	topSettingHLayout->addStretch();
//
//	QWidget *bottomSettingHWidget = new QWidget();
//	QHBoxLayout *bottomSettingHLayout = new QHBoxLayout(bottomSettingHWidget);
//	bottomSettingHLayout->addStretch();
//	bottomSettingHLayout->addWidget(scanDataPathLabel);
//	bottomSettingHLayout->addWidget(scanDataPathEdit);
//	bottomSettingHLayout->addWidget(choosePathButton);
//	bottomSettingHLayout->addStretch();
//
//	settingVLayout->addStretch(2);
//	settingVLayout->addWidget(topSettingHWidget);
//	settingVLayout->addStretch(1);
//	settingVLayout->addWidget(bottomSettingHWidget);
//	settingVLayout->addStretch(3);
//
//
//	//总TabWidget
//	totalTabWidget->addTab(orderInforWidget, QStringLiteral("订单管理"));
//	totalTabWidget->addTab(settingWidget, QStringLiteral("设置"));
//	totalTabWidget->addTab(calibrateWidget, QStringLiteral("标定"));
//	totalTabWidget->addTab(aboutWidget, QStringLiteral("关于"));
//	totalGLayOut->addWidget(totalTabWidget);
//	totalGLayOut->setContentsMargins(0, 0, 0, 0);
//	this->setLayout(totalGLayOut);



	//分模
	for (int i = 0; i < TOOTHNUM; i++)
	{
		toothList[i]->setParent(ui.spliteModelGroupBox);
	}

	toothList[0]->move(150, 23);
	toothList[1]->move(118, 33);
	toothList[2]->move(95, 55);
	toothList[3]->move(72, 85);
	toothList[4]->move(58, 117);
	toothList[5]->move(37, 148);
	toothList[6]->move(25, 195);
	toothList[7]->move(13, 241);

	toothList[8]->move(193, 23);
	toothList[9]->move(231, 33);
	toothList[10]->move(249, 55);
	toothList[11]->move(268, 85);
	toothList[12]->move(285, 117);
	toothList[13]->move(297, 148);
	toothList[14]->move(320, 195);
	toothList[15]->move(331, 240);

	toothList[16]->move(186, 530);
	toothList[17]->move(212, 526);
	toothList[18]->move(238, 512);
	toothList[19]->move(258, 492);
	toothList[20]->move(271, 461);
	toothList[21]->move(285, 412);
	toothList[22]->move(305, 366);
	toothList[23]->move(323, 322);

	toothList[24]->move(159, 530);
	toothList[25]->move(128, 526);
	toothList[26]->move(101, 512);
	toothList[27]->move(76, 492);
	toothList[28]->move(59, 461);
	toothList[29]->move(39, 412);
	toothList[30]->move(23, 366);
	toothList[31]->move(18, 322);

	/*toothList[0]->move(247, 23);
	toothList[1]->move(215, 33);
	toothList[2]->move(192, 55);
	toothList[3]->move(169, 85);
	toothList[4]->move(155, 117);
	toothList[5]->move(134, 148);
	toothList[6]->move(122, 195);
	toothList[7]->move(110, 241);

	toothList[8]->move(290, 23);
	toothList[9]->move(328, 33);
	toothList[10]->move(346, 55);
	toothList[11]->move(365, 85);
	toothList[12]->move(382, 117);
	toothList[13]->move(394, 148);
	toothList[14]->move(417, 195);
	toothList[15]->move(428, 240);

	toothList[16]->move(283,530);
	toothList[17]->move(309,526);
	toothList[18]->move(335, 512);
	toothList[19]->move(355, 492);
	toothList[20]->move(368, 461);
	toothList[21]->move(382, 412);
	toothList[22]->move(402, 366);
	toothList[23]->move(420, 322);

	toothList[24]->move(256, 530);
	toothList[25]->move(225, 526);
	toothList[26]->move(198, 512);
	toothList[27]->move(173, 492);
	toothList[28]->move(156, 461);
	toothList[29]->move(136, 412);
	toothList[30]->move(120, 366);
	toothList[31]->move(115, 322);*/


	toothRadioButtonGroup->setParent(ui.spliteModelGroupBox);
	toothRadioButtonGroup->addButton(totalCrownButton, 1);
	toothRadioButtonGroup->addButton(toothCrownButton, 2);
	toothRadioButtonGroup->addButton(lossToothButton, 3);
	toothRadioButtonGroup->addButton(inlayButton, 4);
	toothRadioButtonGroup->addButton(jawToothButton, 5);


	totalCrownButton->setParent(ui.spliteModelGroupBox);
	toothCrownButton->setParent(ui.spliteModelGroupBox);
	lossToothButton->setParent(ui.spliteModelGroupBox);
	inlayButton->setParent(ui.spliteModelGroupBox);
	jawToothButton->setParent(ui.spliteModelGroupBox);

	totalCrownButton->setGeometry(420, 100, 200, 50);
	toothCrownButton->setGeometry(420, 180, 200, 50);
	lossToothButton->setGeometry(420, 420, 200, 50);
	inlayButton->setGeometry(420, 260, 200, 50);
	jawToothButton->setGeometry(420, 340, 200, 50);


}

bool TabMainGUI::isDirExist(QString fullPath)
{
	QDir dir(fullPath);
	if (dir.exists())
	{
		return true;
	}
	else
	{
		bool ok = dir.mkpath(".");//创建多级目录
		return ok;
	}
}

QByteArray TabMainGUI::ToChineseStr(const QString &chineseString)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	QByteArray byteArray = codec->fromUnicode(chineseString);
	return byteArray;
}

void TabMainGUI::newButtonClickedSlot() {
	QDateTime currentDateTime = QDateTime::currentDateTime();
	QString curDateTimeStr = currentDateTime.toString("yyyy/MM/dd/hh:mm:ss");
	ui.orderDateLineEdit->setText(curDateTimeStr);
	ui.orderDateLineEdit->setReadOnly(true);

	curDateTimeStr = currentDateTime.toString("yyyyMMddhhmmss");
	lastDateTimeStr = curDateTimeStr;
	ui.orderNumLineEdit->setText(curDateTimeStr);
	//orderLineEdit->setReadOnly(true);
	ui.patientNameLineEdit->setText("");
	ui.doctorNameLineEdit->setText("");
	ui.operatorNameLineEdit->setText("");
	ui.additionTextEdit->setText("");
	CTaskManager::getInstance()->DelAllTasks();
	clearAllButtonPress();
}

bool TabMainGUI::judgePatientSaveFlag()
{
	coTimeCout ccTimeout;
	if ((orderDate != NULL && orderNumber != NULL && orderPatientName != NULL && orderDoctorName != NULL&& orderOperatorName != NULL) &&
		(unMoulageFlag == true || doMoulageFlag == true || splitModelFlag == true))
	{
		QString tempFilePath = filePath + "/" + orderNumber+"_"+ orderDoctorName+"_"+orderPatientName;
		std::cout << "tempFilePath is " << tempFilePath.toStdString() << std::endl;
		bool creatFlag = isDirExist(tempFilePath);
		fileQStr = tempFilePath + "/";
		std::cout << "fileQStr is " << fileQStr.toStdString() << std::endl;
		QString strOrderDate = orderDate;
		strOrderDate.replace(":", "").replace(" ", "").replace("/", "");
		QByteArray orderPN = ToChineseStr(strOrderDate);
		std::string filePathStr = Qstring2String(fileQStr)+ orderPN.data() + ".OI";
		std::cout << "string filepath is " << filePathStr << std::endl;
		cv::FileStorage fwrite(filePathStr.c_str(), cv::FileStorage::WRITE);
		fwrite << "Order Date" << orderDate.toStdString()
			<< "Order Number" << orderNumber.toStdString()
			<< "Patient Name" << ToChineseStr(orderPatientName).data()
			<< "Doctor Name" << ToChineseStr(orderDoctorName).data()
			<<"Operator Name"<< ToChineseStr(orderOperatorName).data()
			<< "Addition" << ToChineseStr(orderAddition).data()
			<< "splitModelFlag" << splitModelFlag;

		if (unMoulageFlag == true)
		{
			if (upperJawButtonFlag == true && lowerJawButtonFlag == false)
			{
				fwrite << "scanModel" << 1;
			}
			if (lowerJawButtonFlag == true && upperJawButtonFlag == false)
			{
				fwrite << "scanModel" << 2;
			}
			else if (upperJawButtonFlag == true && lowerJawButtonFlag == true)
			{
				fwrite << "scanModel" << 3;
			}
		}
		else if (doMoulageFlag == true)
		{
			if (MoulageFlag1 == true)
			{
				fwrite << "scanModel" << 4;
			}
			else if (MoulageFlag2 == true)
			{
				fwrite << "scanModel" << 5;
			}
			else if (MoulageFlag3 == true)
			{
				fwrite << "scanModel" << 6;
			}
		}
		else
		{
			fwrite << "scanModel" << 7;
			for (int j = 1; j < 9; j++)
			{
				switch (j)
				{
				case 1:
				{
					if (totalCrownList.size() != 0)
					{
						//fwrite << "splitModel" << 1;
						for (int i = 0; i < totalCrownList.size(); i++)
						{
							//fwrite << "totalCrown" << totalCrownList[i]->text().toStdString();
						}
					}
					break;
				}
				case 2:
				{
					if (toothCrownList.size() != 0)
					{
						//fwrite << "splitModel" << 2;
						for (int i = 0; i < toothCrownList.size(); i++)
						{
							//fwrite << "toothCrown" << toothCrownList[i]->text().toStdString();
						}
					}
					break;
				}
				case 3:
				{
					if (lossToothList.size() != 0)
					{
						for (int i = 0; i < lossToothList.size(); i++)
						{
							//fwrite << "lossTooth" << lossToothList[i]->text().toStdString();
						}
					}
					break;
				}
				case 4:
				{
					if (inlayList.size() != 0)
					{
						for (int i = 0; i < inlayList.size(); i++)
						{
							fwrite << "inlay" << inlayList[i]->text().toStdString();
						}
					}
					break;
				}
				case 5:
				{
					if (facingList.size() != 0)
					{
						for (int i = 0; i < facingList.size(); i++)
						{
							//fwrite << "facing" << facingList[i]->text().toStdString();
						}
					}
					break;
				}
				case 6:
				{
					if (waxTypeList.size() != 0)
					{
						for (int i = 0; i < waxTypeList.size(); i++)
						{
							//fwrite << "waxType" << waxTypeList[i]->text().toStdString();
						}
					}
					break;
				}
				case 7:
				{
					if (implantList.size() != 0)
					{
						for (int i = 0; i < implantList.size(); i++)
						{
							//fwrite << "implant" << implantList[i]->text().toStdString();
						}
					}
					break;
				}
				case 8:
				{
					if (jawToothList.size() != 0)
					{
						for (int i = 0; i < jawToothList.size(); i++)
						{
							//fwrite << "jawTooth" << jawToothList[i]->text().toStdString();
						}
					}
					break;
				}
				}
			}
		}
		ccTimeout.Print("fwrite.release();");
		fwrite.release();
		if (splitModelFlag) {
			if (CTaskManager::getInstance()->getTasks().size() > 0) {
				QJsonDocument document;
				QJsonArray jsonArray;
				list<pCScanTask>::iterator iter = CTaskManager::getInstance()->getTasks().begin();
				int i = 0;
				for (; iter != CTaskManager::getInstance()->getTasks().end(); iter++) {
					QJsonObject simp_ayjson;
					simp_ayjson.insert("class", (*iter)->getClassName());
					datastream kdataStream;
					(*iter)->StreamValue(kdataStream, true);
					QByteArray byteArray(kdataStream.data(), kdataStream.size());
					//simp_ayjson.insert("classdata", kdataStream.data());
					simp_ayjson.insert("classdata", byteArray.toBase64().data());
					//document.setObject(simp_ayjson);
					jsonArray.insert(i++, simp_ayjson);
				}
				document.setArray(jsonArray);
				QByteArray simpbyte_array = document.toJson(QJsonDocument::Compact);
				QString simpjson_str(simpbyte_array);
				std::string filePathStr2 = filePathStr + ".json";
				fstream f(filePathStr2, ios::out);//供写使用，文件不存在则创建，存在则清空原内容 
				qDebug() << QString::fromLocal8Bit("简单的QTJson数据：") << simpjson_str;
				f << simpjson_str.toStdString(); //写入数据
				f.close();
			}

		}
		ccTimeout.Print("return true;");
		return true;
	}
	else
	{
		QMessageBox::warning(NULL, QStringLiteral("警告"), QStringLiteral("订单信息不完整"), QMessageBox::Yes, QMessageBox::Yes);
		return false;
	}
}
void TabMainGUI::closeBtnClicked() {
	QApplication* app;
	app->quit();
}
void TabMainGUI::btnMinClicked() {
	this->showMinimized();
	g_pCurrentWidget = this;
}
void TabMainGUI::PatientInformationSave()
{
	std::cout << "storage a order information..." << std::endl;
	coTimeCout cctimeout;
	orderDate = ui.orderDateLineEdit->text();
	orderNumber = ui.orderNumLineEdit->text();
	orderPatientName = ui.patientNameLineEdit->text();
	orderDoctorName = ui.doctorNameLineEdit->text();
	orderAddition = ui.additionTextEdit->toPlainText();
	orderOperatorName = ui.operatorNameLineEdit->text();
	judgeSplitModelFlag();
	cctimeout.Print("judgeSplitModelFlag");
	if (splitModelFlag)//分模
	{
		CTaskManager::getInstance()->DelAllTasks();
		pCScanTask upperScanTask = nullptr, lowerJawScanTask = nullptr, allJawScanTask = nullptr,
			pUpperJawGingvaScanTask = nullptr, pLowerJawGingivaScanTask = nullptr;
		//pCOralSubstituteScan upperJawOralScanTask, lowJawOralScanTask;
		pCGroupScan einlayScanTask = nullptr;
		pCGroupScan pupperJawTotalCrownGroupScan = nullptr, plowJawTotalCrownGroupScan = nullptr;
		for (int j = 0; j < 32; j++) {
			eScanType l_eScanType = m_pTeethScanTaskArray[j]->Get_ScanType();

			if (l_eScanType != eScanNULL) {
				if (m_pTeethScanTaskArray[j]->Get_TeethId() > 15) {
					if (lowerJawScanTask == nullptr) {
						lowerJawScanTask = make_shared<CScanTask>();
						lowerJawScanTask->Set_ScanType(eLowerJawScan);
						lowerJawScanTask->Set_TaskName((g_strScanName[eLowerJawScan]));
						lowerJawScanTask->Set_TaskType(eScan);
					}
				}
				else {
					if (upperScanTask == nullptr) {
						upperScanTask = make_shared<CScanTask>();
						upperScanTask->Set_ScanType(eUpperJawScan);
						upperScanTask->Set_TaskName((g_strScanName[eUpperJawScan]));
						upperScanTask->Set_TaskType(eScan);
					}
				}
				switch (l_eScanType) {
				case etoothCrown:
					if (m_pTeethScanTaskArray[j]->Get_TeethId() > 15) {	//下颌
						if (plowJawTotalCrownGroupScan == nullptr || plowJawTotalCrownGroupScan->m_vtTeeth.size() > 8) {
							plowJawTotalCrownGroupScan = make_shared<CGroupScan>();
							plowJawTotalCrownGroupScan->Set_ScanType(l_eScanType);
							plowJawTotalCrownGroupScan->Set_TaskName((g_strScanName[l_eScanType]));
							plowJawTotalCrownGroupScan->Set_TaskType(eScan);
							CTaskManager::getInstance()->AddTask(plowJawTotalCrownGroupScan);
							pCStitchingTask pStitchingTask = make_shared<CStitchingTask>();
							pStitchingTask->Set_TaskName(g_strScanName[l_eScanType]);
							pStitchingTask->Set_TaskType(eLowerTeethStit);
							pStitchingTask->m_pDstTask = lowerJawScanTask;
							pStitchingTask->m_pSrcTask = plowJawTotalCrownGroupScan;
							CTaskManager::getInstance()->AddTask(pStitchingTask);
						}
						plowJawTotalCrownGroupScan->m_vtTeeth.push_back(m_pTeethScanTaskArray[j]->Get_TeethId());
					}
					else //上颌
					{
						if (pupperJawTotalCrownGroupScan == nullptr || pupperJawTotalCrownGroupScan->m_vtTeeth.size() > 8) {
							pupperJawTotalCrownGroupScan = make_shared<CGroupScan>();
							pupperJawTotalCrownGroupScan->Set_ScanType(l_eScanType);
							pupperJawTotalCrownGroupScan->Set_TaskName((g_strScanName[l_eScanType]));
							pupperJawTotalCrownGroupScan->Set_TaskType(eScan);
							CTaskManager::getInstance()->AddTask(pupperJawTotalCrownGroupScan);
							pCStitchingTask pStitchingTask = make_shared<CStitchingTask>();
							pStitchingTask->Set_TaskName(g_strScanName[l_eScanType]);
							pStitchingTask->Set_TaskType(eUpperTeethStit);
							pStitchingTask->m_pDstTask = upperScanTask;
							pStitchingTask->m_pSrcTask = pupperJawTotalCrownGroupScan;
							CTaskManager::getInstance()->AddTask(pStitchingTask);
						}
						pupperJawTotalCrownGroupScan->m_vtTeeth.push_back(m_pTeethScanTaskArray[j]->Get_TeethId());
					}
					break;
				case elossToothScan:
				case eJawToothScan:
				{
					pCScanTask plossToothScanTask = make_shared<CScanTask>();
					plossToothScanTask->Set_ScanType(l_eScanType);
					plossToothScanTask->Set_TaskName((g_strScanName[l_eScanType]));
					plossToothScanTask->Set_TaskType(eScan);
					plossToothScanTask->Set_TeethId(j);
					CTaskManager::getInstance()->AddTask(plossToothScanTask);
				}
					break;
				case einlayScan:		//嵌体
					if (m_pTeethScanTaskArray[j]->Get_TeethId() > 15) {	//下颌
					//	if (plowJawTotalCrownGroupScan == nullptr || plowJawTotalCrownGroupScan->m_vtTeeth.size() > 8) {
						einlayScanTask = make_shared<CGroupScan>();
						einlayScanTask->Set_ScanType(l_eScanType);
						einlayScanTask->Set_TaskName((g_strScanName[l_eScanType]));
						einlayScanTask->Set_TaskType(eScan);
						einlayScanTask->Set_TeethId(j);
						CTaskManager::getInstance()->AddTask(einlayScanTask);
						pCStitchingTask pStitchingTask = make_shared<CStitchingTask>();
						pStitchingTask->Set_TaskName(g_strScanName[l_eScanType]);
						pStitchingTask->Set_TaskType(eLowerTeethStit);
						pStitchingTask->m_pSrcTask = einlayScanTask;
						pStitchingTask->m_pDstTask = lowerJawScanTask;
						CTaskManager::getInstance()->AddTask(pStitchingTask);
						einlayScanTask->m_vtTeeth.push_back(j);
					}
					else //上颌
					{
						einlayScanTask = make_shared<CGroupScan>();
						einlayScanTask->Set_ScanType(l_eScanType);
						einlayScanTask->Set_TaskName((g_strScanName[l_eScanType]));
						einlayScanTask->Set_TaskType(eScan);
						einlayScanTask->Set_TeethId(j);
						CTaskManager::getInstance()->AddTask(einlayScanTask);
						pCStitchingTask pStitchingTask = make_shared<CStitchingTask>();
						pStitchingTask->Set_TaskName(g_strScanName[l_eScanType]);
						pStitchingTask->Set_TaskType(eUpperTeethStit);
						pStitchingTask->m_pSrcTask = einlayScanTask;
						pStitchingTask->m_pDstTask = upperScanTask;
						CTaskManager::getInstance()->AddTask(pStitchingTask);
						einlayScanTask->m_vtTeeth.push_back(j);
					}
					break;
				case eDentalImplantScan: 
				{
					if (m_pTeethScanTaskArray[j]->Get_TeethId() > 15) {	//下颌
						if (lowerJawScanTask == nullptr) {
							lowerJawScanTask = make_shared<CScanTask>();
							lowerJawScanTask->Set_ScanType(eLowerJawScan);
							lowerJawScanTask->Set_TaskName((g_strScanName[eLowerJawScan]));
							lowerJawScanTask->Set_TaskType(eScan);
						}
						lowerJawScanTask->Set_DentalImplant(true);
						lowerJawScanTask->m_vtTeeth.push_back(j);
						lowerJawScanTask->Set_Gingiva(false);
						if (pLowerJawGingivaScanTask == nullptr) {
							pLowerJawGingivaScanTask = make_shared<CScanTask>();
							pLowerJawGingivaScanTask->Set_ScanType(eLowerJawScan);
							pLowerJawGingivaScanTask->Set_TaskName((g_strScanName[eLowerJawScan]));
							pLowerJawGingivaScanTask->Set_TaskType(eScan);
							pLowerJawGingivaScanTask->Set_Gingiva(true);//有牙龈
						}
					}
					else {
						if (upperScanTask == nullptr) {
							upperScanTask = make_shared<CScanTask>();
							upperScanTask->Set_ScanType(eUpperJawScan);
							upperScanTask->Set_TaskName((g_strScanName[eUpperJawScan]));
							upperScanTask->Set_TaskType(eScan);
						}
						upperScanTask->Set_DentalImplant(true);
						upperScanTask->m_vtTeeth.push_back(j);
						upperScanTask->Set_Gingiva(false);
						if (pUpperJawGingvaScanTask == nullptr) {
							pUpperJawGingvaScanTask = make_shared<CScanTask>();
							pUpperJawGingvaScanTask->Set_ScanType(eUpperJawScan);
							pUpperJawGingvaScanTask->Set_TaskName((g_strScanName[eUpperJawScan]));
							pUpperJawGingvaScanTask->Set_TaskType(eScan);
							pUpperJawGingvaScanTask->Set_Gingiva(true);		//有牙龈
						}
					}
					break;
				}
				}

			}
		}
		if (upperScanTask && lowerJawScanTask) {
			if (allJawScanTask == nullptr) {
				allJawScanTask = make_shared<CScanTask>();
				allJawScanTask->Set_ScanType(eAllJawScan);
				allJawScanTask->Set_TaskName((g_strScanName[eAllJawScan]));
				if(upperScanTask->Get_DentalImplant() != true){
					pCStitchingTask pUpperStitcing = make_shared<CStitchingTask>();
					pUpperStitcing->Set_TaskType(eUpperStitching);
					pUpperStitcing->Set_TaskName((g_strScanName[eUpperJawScan]));
					pUpperStitcing->m_pSrcTask = upperScanTask;
					pUpperStitcing->m_pDstTask = allJawScanTask;
					CTaskManager::getInstance()->AddTask(pUpperStitcing, true);
				}
				if(lowerJawScanTask->Get_DentalImplant() != true){
					pCStitchingTask pLowerSitcing = make_shared<CStitchingTask>();
					pLowerSitcing->Set_TaskType(eLowerStitching);
					pLowerSitcing->Set_TaskName((g_strScanName[eLowerJawScan]));
					pLowerSitcing->m_pSrcTask = lowerJawScanTask;
					pLowerSitcing->m_pDstTask = allJawScanTask;
					CTaskManager::getInstance()->AddTask(pLowerSitcing, true);
				}
				if (pLowerJawGingivaScanTask) {
					pCStitchingTask pLowerSitcing = make_shared<CStitchingTask>();
					pLowerSitcing->Set_TaskType(eLowerStitching);
					pLowerSitcing->Set_TaskName((g_strScanName[eLowerJawScan]));
					pLowerSitcing->m_pSrcTask = pLowerJawGingivaScanTask;
					pLowerSitcing->m_pDstTask = allJawScanTask;
					CTaskManager::getInstance()->AddTask(pLowerSitcing, true);
				}
				if (pUpperJawGingvaScanTask) {
					pCStitchingTask pUpperStitcing = make_shared<CStitchingTask>();
					pUpperStitcing->Set_TaskType(eUpperStitching);
					pUpperStitcing->Set_TaskName((g_strScanName[eUpperJawScan]));
					pUpperStitcing->m_pSrcTask = pUpperJawGingvaScanTask;
					pUpperStitcing->m_pDstTask = allJawScanTask;
					CTaskManager::getInstance()->AddTask(pUpperStitcing, true);
				}
			}
		}
		if (upperScanTask) {
			CTaskManager::getInstance()->AddTask(upperScanTask, true);
			if (pUpperJawGingvaScanTask) {
				CTaskManager::getInstance()->AddTask(pUpperJawGingvaScanTask, true);
			}
		}
		if (lowerJawScanTask) {
			CTaskManager::getInstance()->AddTask(lowerJawScanTask, true);
			if (pLowerJawGingivaScanTask) {
				CTaskManager::getInstance()->AddTask(pLowerJawGingivaScanTask, true);
			}
		}
		if (allJawScanTask)
			CTaskManager::getInstance()->AddTask(allJawScanTask, true);
	}
	else if (unMoulageFlag) {
		CTaskManager::getInstance()->DelAllTasks();
		pCScanTask upperScanTask = nullptr, lowerJawScanTask = nullptr, allJawScanTask = nullptr;
		if (upperJawButtonFlag == true && lowerJawButtonFlag == true)
		{
			if (allJawScanTask == nullptr) {
				allJawScanTask = make_shared<CScanTask>();
				allJawScanTask->Set_ScanType(eAllJawScan);
				allJawScanTask->Set_TaskName((g_strScanName[eAllJawScan]));
			}
			if (upperScanTask == nullptr) {
				upperScanTask = make_shared<CScanTask>();
				upperScanTask->Set_ScanType(eUpperJawScan);
				upperScanTask->Set_TaskName((g_strScanName[eUpperJawScan]));
				upperScanTask->Set_TaskType(eScan);
			}
			if (lowerJawScanTask == nullptr) {
				lowerJawScanTask = make_shared<CScanTask>();
				lowerJawScanTask->Set_ScanType(eLowerJawScan);
				lowerJawScanTask->Set_TaskName((g_strScanName[eLowerJawScan]));
				lowerJawScanTask->Set_TaskType(eScan);
			}
		}
		else if (upperJawButtonFlag == true && lowerJawButtonFlag == false)
		{
			if (upperScanTask == nullptr) {
				upperScanTask = make_shared<CScanTask>();
				upperScanTask->Set_ScanType(eUpperJawScan);
				upperScanTask->Set_TaskName((g_strScanName[eUpperJawScan]));
				upperScanTask->Set_TaskType(eScan);
			}
		}
		else if (lowerJawButtonFlag == true && upperJawButtonFlag == false)
		{
			if (lowerJawScanTask == nullptr) {
				lowerJawScanTask = make_shared<CScanTask>();
				lowerJawScanTask->Set_ScanType(eLowerJawScan);
				lowerJawScanTask->Set_TaskName((g_strScanName[eLowerJawScan]));
				lowerJawScanTask->Set_TaskType(eScan);
			}
		}
		if (upperScanTask)
			CTaskManager::getInstance()->AddTask(upperScanTask);
		if (lowerJawScanTask)
			CTaskManager::getInstance()->AddTask(lowerJawScanTask);
		if (allJawScanTask) {
			CTaskManager::getInstance()->AddTask(allJawScanTask, true);
			pCStitchingTask pUpperStitcing = make_shared<CStitchingTask>();
			pUpperStitcing->Set_TaskType(eUpperStitching);
			pUpperStitcing->Set_TaskName((g_strScanName[eUpperJawScan]));
			pUpperStitcing->m_pSrcTask = upperScanTask;
			pUpperStitcing->m_pDstTask = allJawScanTask;
			CTaskManager::getInstance()->AddTask(pUpperStitcing);
			pCStitchingTask pLowerSitcing = make_shared<CStitchingTask>();
			pLowerSitcing->Set_TaskType(eLowerStitching);
			pLowerSitcing->Set_TaskName((g_strScanName[eLowerJawScan]));
			pLowerSitcing->m_pSrcTask = lowerJawScanTask;
			pLowerSitcing->m_pDstTask = allJawScanTask;
			CTaskManager::getInstance()->AddTask(pLowerSitcing);
		}
	}
	QPushButton * pBtn = static_cast<QPushButton *> (sender());
	if (!pBtn)
		return;
	cctimeout.Print("judgePatientSaveFlag");
	if (judgePatientSaveFlag() == true&& pBtn->text() == QStringLiteral("扫描"))
	{
		if (m_usbDeviceState)
		{
			emit scanSignal();
		}
		else
		{
			QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("设备未与电脑连接!"));
			box.setStandardButtons(QMessageBox::Yes);
			box.setButtonText(QMessageBox::Yes, QStringLiteral("确 定"));
			box.exec();
		}
		
	}
	cctimeout.Print("PatientInformationSave");
}

void TabMainGUI::OrderPreview()
{
	QString strOrderDate = ui.orderDateLineEdit->text();
	strOrderDate.replace(":", "").replace(" ", "").replace("/", "");
	QByteArray orderPN = ToChineseStr(strOrderDate);
	QString tempFilePath = filePath + "/" + orderNumber + "_" + orderDoctorName + "_" + orderPatientName;
	COrderInfo orderInfo;
	orderInfo.strOderDate = orderPN.data();
	orderInfo.strPatientName = Qstring2String(ui.patientNameLineEdit->text());
	orderInfo.strDoctorName = Qstring2String(ui.doctorNameLineEdit->text());
	orderInfo.strOrderAddition = Qstring2String(ui.additionTextEdit->toPlainText());
	orderInfo.strOrderNumber = Qstring2String(ui.orderNumLineEdit->text());
	orderInfo.strFilePath = Qstring2String(tempFilePath + "/");
	orderInfo.strOperatorName = Qstring2String(ui.operatorNameLineEdit->text());
	if (splitModelFlag)
		orderInfo.eorderType = esplitModel;
	else if (unMoulageFlag) {
		orderInfo.eorderType = eunMoulage;
	}
	else if (doMoulageFlag) {
		orderInfo.eorderType = eMoulage;
	}
	if (splitModelFlag) {
		for (int i = 0; i < 32;i++) {
			orderInfo.eTeethScanType[i] = m_pTeethScanTaskArray[i]->Get_ScanType();
		}
	}
	orderInfo.strUpperJawModelName = "toothUpperJaw";
	orderInfo.strLowerJawModelName = "toothLowerJaw";
	emit showOrderInfoSignal(orderInfo);
}

void TabMainGUI::UpperJawPress()
{

	if (upperJawButtonFlag == false)
	{
	//	ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}");
		upperJawButtonFlag = true;
		unMoulageFlag = true;

		//doMoulage
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
		MoulageFlag1 = false;
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
		MoulageFlag2 = false;
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
		MoulageFlag3 = false;
		doMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else
	{
	//	ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		upperJawButtonFlag = false;
		if (lowerJawButtonFlag == false)
		{
			unMoulageFlag = false;
		}
	}
}

void TabMainGUI::LowerJawPress()
{
	if (lowerJawButtonFlag == false)
	{
	//	ui.lowerJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}");
		lowerJawButtonFlag = true;
		unMoulageFlag = true;

		//doMoulage
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
		ui.moulagePushButton_1->setChecked(false);
		MoulageFlag1 = false;
	//	ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
		ui.moulagePushButton_2->setChecked(false);
		MoulageFlag2 = false;
//		ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
		ui.moulagePushButton_3->setChecked(false);
		MoulageFlag3 = false;

		doMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else
	{
	//	ui.lowerJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		ui.lowerJawPushButton->setChecked(false);
		lowerJawButtonFlag = false;
		if (upperJawButtonFlag == false)
		{
			unMoulageFlag = false;
		}
	}
}

void TabMainGUI::MoulagePress1()
{
	if (MoulageFlag1 == false)
	{
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_yes.png);}");
		ui.moulagePushButton_1->setChecked(false);
		MoulageFlag1 = true;
		doMoulageFlag = true;

		//doMoulage
	//	ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
		ui.moulagePushButton_2->setChecked(false);
		MoulageFlag2 = false;
	//	ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
		ui.moulagePushButton_3->setChecked(false);
		MoulageFlag3 = false;
		//unMoulage
	//	ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		ui.upperJawPushButton->setChecked(false);
		upperJawButtonFlag = false;
	//	ui.lowerJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		ui.lowerJawPushButton->setChecked(false);
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag1 == true && MoulageFlag2 == false && MoulageFlag3 == false)
	{
	//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
		ui.moulagePushButton_1->setChecked(false);
		MoulageFlag1 = false;
		doMoulageFlag = false;
	}
}

void TabMainGUI::MoulagePress2()
{
	if (MoulageFlag2 == false)
	{
		//ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_yes.png);}");
		ui.moulagePushButton_2->setChecked(false);
		MoulageFlag2 = true;
		doMoulageFlag = true;

		//doMoulage
		//ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
		ui.moulagePushButton_1->setChecked(false);
		MoulageFlag1 = false;
		//ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
		ui.moulagePushButton_3->setChecked(false);
		MoulageFlag3 = false;
		//unMoulage
		//ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		ui.upperJawPushButton->setChecked(false);
		upperJawButtonFlag = false;
		//ui.lowerJawPushButton ->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		ui.lowerJawPushButton->setChecked(false);
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag2 == true && MoulageFlag1 == false && MoulageFlag3 == false)
	{
		//ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
		ui.moulagePushButton_2->setChecked(false);
		MoulageFlag2 = false;
		doMoulageFlag = false;
	}
}

void TabMainGUI::MoulagePress3()
{
	if (MoulageFlag3 == false)
	{
		//ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_yes.png);}");
		ui.moulagePushButton_3->setChecked(false);
		MoulageFlag3 = true;
		doMoulageFlag = true;

		//doMoulage
		//ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
		ui.moulagePushButton_1->setChecked(false);
		MoulageFlag1 = false;
		//ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
		ui.moulagePushButton_2->setChecked(false);
		MoulageFlag2 = false;
		//unMoulage
		//ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		ui.upperJawPushButton->setChecked(false);
		upperJawButtonFlag = false;
		//ui.lowerJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		ui.lowerJawPushButton->setChecked(false);
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag3 == true && MoulageFlag1 == false && MoulageFlag2 == false)
	{
		//ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
		ui.moulagePushButton_3->setChecked(false);
		MoulageFlag3 = false;
		doMoulageFlag = false;
	}
}

QList<QPushButton*> *TabMainGUI::judgeToothList(int id)
{
	switch (id)
	{
	case 1:
	{
		return &totalCrownList;
		//break;
	}
	case 2:
	{
		return &toothCrownList;
		//break;
	}
	case 3:
	{
		return &lossToothList;
		break;
	}
	case 4:
	{
		return &inlayList;
		break;
	}
	case 5:
	{
		return &facingList;
		break;
	}
	case 6:
	{
		return &waxTypeList;
		break;
	}
	case 7:
	{
		return &implantList;
		break;
	}
	case 8:
	{
		return &jawToothList;
		break;
	}
	default:
		return nullptr;
	}
}

void TabMainGUI::ToothGroupClicked(int id)
{
	chooseID = id;
	m_eScanType = (eScanType)(id -1);
	return;
	switch (id)
	{
	case 1:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/11.png);}";
		totalCrownButton->setStyleSheet(path);
		if (totalCrownList.size() != 0)
		{
			foreach(QPushButton *chooseButton, totalCrownList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "1.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 2:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/22.png);}";
		toothCrownButton->setStyleSheet(path);
		if (toothCrownList.size() != 0)
		{
			foreach(QPushButton *chooseButton, toothCrownList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "2.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 3:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/33.png);}";
		lossToothButton->setStyleSheet(path);
		if (lossToothList.size() != 0)
		{
			foreach(QPushButton *chooseButton, lossToothList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "3.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 4:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/44.png);}";
		inlayButton->setStyleSheet(path);
		if (inlayList.size() != 0)
		{
			foreach(QPushButton *chooseButton, inlayList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "4.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 5:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/55.png);}";
		facingButton->setStyleSheet(path);
		if (facingList.size() != 0)
		{
			foreach(QPushButton *chooseButton, facingList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "5.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 6:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/66.png);}";
		waxTypeButton->setStyleSheet(path);
		if (waxTypeList.size() != 0)
		{
			foreach(QPushButton *chooseButton, waxTypeList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "6.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 7:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/77.png);}";
		implantButton->setStyleSheet(path);
		if (implantList.size() != 0)
		{
			foreach(QPushButton *chooseButton, implantList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "7.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	case 8:
	{
		QString path = "QRadioButton{border-image: url(:/MainWidget/Resources/images/88.png);}";
		jawToothButton->setStyleSheet(path);
		if (jawToothList.size() != 0)
		{
			foreach(QPushButton *chooseButton, jawToothList)
			{
				QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "8.png);}";
				chooseButton->setStyleSheet(path);
			}
		}
		break;
	}
	}
}

void TabMainGUI::addToothList(int id, int toothButtonIndex)
{
	if (id == 1)
	{
		totalCrownList.append(toothList[toothButtonIndex]);
	}
	else if (id == 2)
	{
		toothCrownList.append(toothList[toothButtonIndex]);
	}
	else if (id == 3)
	{
		lossToothList.append(toothList[toothButtonIndex]);
	}
	else if (id == 4)
	{
		inlayList.append(toothList[toothButtonIndex]);
	}
	else if (id == 5)
	{
		facingList.append(toothList[toothButtonIndex]);
	}
	else if (id == 6)
	{
		waxTypeList.append(toothList[toothButtonIndex]);
	}
	else if (id == 7)
	{
		implantList.append(toothList[toothButtonIndex]);
	}
	else if (id == 8)
	{
		jawToothList.append(toothList[toothButtonIndex]);
	}
}

void TabMainGUI::removeToothList(int id, int toothButtonIndex)
{
	if (id == 1)
	{
		totalCrownList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 2)
	{
		toothCrownList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 3)
	{
		lossToothList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 4)
	{
		inlayList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 5)
	{
		facingList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 6)
	{
		waxTypeList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 7)
	{
		implantList.removeOne(toothList[toothButtonIndex]);
	}
	else if (id == 8)
	{
		jawToothList.removeOne(toothList[toothButtonIndex]);
	}
}

void TabMainGUI::ToothButtonListPress()
{
	int toothButtonIndex = sender()->objectName().toInt();
	QPushButton * l_pTootPushButton = static_cast<QPushButton *> (sender());
	if (!l_pTootPushButton)
		return;
	if (m_pTeethScanTaskArray[toothButtonIndex]->Get_ScanType() == m_eScanType) {
		m_pTeethScanTaskArray[toothButtonIndex]->Set_ScanType(eScanNULL);
	}
	else if(m_eScanType != eScanNULL){
		m_pTeethScanTaskArray[toothButtonIndex]->Set_ScanType(m_eScanType);
		m_pTeethScanTaskArray[toothButtonIndex]->Set_TaskName((g_strScanName[m_eScanType]));
	}else {
		m_pTeethScanTaskArray[toothButtonIndex]->Set_ScanType(m_eScanType);
	}
	if (chooseID != -1)
	{
		if (toothFlagList[toothButtonIndex] == false)
		{
			//QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + QString::number(chooseID, 10) + ".png);}";
			//QString dstpath = ":/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + QString::number(chooseID, 10) + ".png",
			QString dstpath = ":/MainWidget/Resources/images/" + QString::number(chooseID, 10) + QString::number(chooseID, 10) + ".png",
				srcPath = ":/MainWidget/Resources/images/0/" + QString::number((toothButtonIndex / 8 + 1) * 10 + toothButtonIndex % 8 + 1, 10) + ".png";;

			QImage resultImage, destinationImage, sourceImage;
			destinationImage.load(dstpath);
			sourceImage.load(srcPath);
			resultImage = QImage(sourceImage.size(), QImage::Format_ARGB32_Premultiplied);
			QPainter painter(&resultImage);
			painter.setCompositionMode(QPainter::CompositionMode_Source);
			painter.fillRect(resultImage.rect(),QColor(255,255,255,100));
			painter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
			painter.drawImage(QRect(0, 0, resultImage.width(),resultImage.height()),destinationImage);
			painter.setCompositionMode(QPainter::CompositionMode_DestinationAtop);
			painter.drawImage(0, 0, sourceImage);
			QTextOption option;
			option.setAlignment(Qt::AlignCenter);
			painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
			painter.drawText(resultImage.rect(), QString::number((toothButtonIndex / 8 + 1) * 10 + toothButtonIndex % 8 + 1, 10), option);
			painter.end();
			
			//toothList[toothButtonIndex]->setStyleSheet(path);
			//toothList[toothButtonIndex]->setIcon(QPixmap::fromImage(resultImage));
			//toothList[toothButtonIndex]->setIconSize(QPixmap::fromImage(resultImage).rect().size());
			toothList[toothButtonIndex]->SetImage(resultImage);
			addToothList(chooseID, toothButtonIndex);
			qDebug() << totalCrownList.size();
			toothFlagList[toothButtonIndex] = true;
			toothID[toothButtonIndex] = chooseID;
			//doMoulage
		//	ui.moulagePushButton_1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}");
			ui.moulagePushButton_1->setChecked(false);
			MoulageFlag1 = false;
		//	ui.moulagePushButton_2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}");
			ui.moulagePushButton_2->setChecked(false);
			MoulageFlag2 = false;
		//	ui.moulagePushButton_3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}");
			ui.moulagePushButton_3->setChecked(false);
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
		//	ui.upperJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			ui.upperJawPushButton->setChecked(false);
			upperJawButtonFlag = false;
		//	ui.lowerJawPushButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			ui.lowerJawPushButton->setChecked(false);
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[toothButtonIndex] == true && chooseID == toothID[toothButtonIndex])
		{
			//QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + "0.png);}";
			//toothList[toothButtonIndex]->setStyleSheet(path);
			toothList[toothButtonIndex]->SetImage(QString(":/MainWidget/Resources/images/0/"+ toothList[toothButtonIndex]->text() + ".png"));
			removeToothList(chooseID, toothButtonIndex);
			toothFlagList[toothButtonIndex] = false;
		}
		else if (toothFlagList[toothButtonIndex] == true && chooseID != toothID[toothButtonIndex])
		{
			QString dstpath = ":/MainWidget/Resources/images/" + QString::number(chooseID, 10) + QString::number(chooseID, 10) + ".png",
				srcPath = ":/MainWidget/Resources/images/0/" + QString::number((toothButtonIndex / 8 + 1) * 10 + toothButtonIndex % 8 + 1, 10) + ".png";;

			QImage resultImage, destinationImage, sourceImage;
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
			painter.drawText(resultImage.rect(), QString::number((toothButtonIndex / 8 + 1) * 10 + toothButtonIndex % 8 + 1, 10), option);
			painter.end();

			//toothList[toothButtonIndex]->setStyleSheet(path);
			//toothList[toothButtonIndex]->setIcon(QPixmap::fromImage(resultImage));
			//toothList[toothButtonIndex]->setIconSize(QPixmap::fromImage(resultImage).rect().size());
			toothList[toothButtonIndex]->SetImage(resultImage);
			addToothList(chooseID, toothButtonIndex);
			removeToothList(toothID[toothButtonIndex], toothButtonIndex);
			toothID[toothButtonIndex] = chooseID;
			toothFlagList[toothButtonIndex] = true;
		}
	}
}

void TabMainGUI::setSplitToothFalse()
{
	for (int i = 0; i < TOOTHNUM; i++)
	{
		if (toothFlagList[i] == true)
		{
			toothFlagList[i] = false;
			int row = i / 8;
			int col = i % 8;
			int value = (row + 1) * 10 + col + 1;
			QString path = ":/MainWidget/Resources/images/0/" + QString::number(value, 10) + ".png";
			toothList[i]->SetImage(path);
			QList<QPushButton *> *toothPushButtonList = judgeToothList(toothID[i]);
			if(toothPushButtonList&& toothPushButtonList->size()>0)
				toothPushButtonList->removeOne(toothList[i]);

		}
	}

	totalCrownButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/0.png);}");
	if (totalCrownButton->isChecked())
	{
		totalCrownButton->setAutoExclusive(false);
		totalCrownButton->setChecked(false);

		totalCrownButton->setAutoExclusive(true);

		chooseID = -1;
	}

	toothCrownButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/20.png);}");//牙冠
	if (toothCrownButton->isChecked())
	{
		toothCrownButton->setAutoExclusive(false);
		toothCrownButton->setChecked(false);
		toothCrownButton->setAutoExclusive(true);
		chooseID = -1;
	}

	lossToothButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/30.png);}");//缺失牙
	if (lossToothButton->isChecked())
	{
		lossToothButton->setAutoExclusive(false);
		lossToothButton->setChecked(false);
		lossToothButton->setAutoExclusive(true);
		chooseID = -1;
	}

	inlayButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/40.png);}");//嵌体
	if (inlayButton->isChecked())
	{
		inlayButton->setAutoExclusive(false);
		inlayButton->setChecked(false);
		inlayButton->setAutoExclusive(true);
		chooseID = -1;
	}


	facingButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/50.png);}");//贴面
	if (facingButton->isChecked())
	{
		facingButton->setAutoExclusive(false);
		facingButton->setChecked(false);
		facingButton->setAutoExclusive(true);
		chooseID = -1;
	}

	waxTypeButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/60.png);}");//蜡型

	if (waxTypeButton->isChecked())
	{
		waxTypeButton->setAutoExclusive(false);
		waxTypeButton->setChecked(false);
		waxTypeButton->setAutoExclusive(true);
		chooseID = -1;
	}

	implantButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/70.png);}");//种植体
	if (implantButton->isChecked())
	{
		implantButton->setAutoExclusive(false);
		implantButton->setChecked(false);
		implantButton->setAutoExclusive(true);
		chooseID = -1;
	}


	jawToothButton->setStyleSheet("QRadioButton{border-image: url(:/MainWidget/Resources/images/80.png);}");//对颌牙
	if (jawToothButton->isChecked())
	{
		jawToothButton->setAutoExclusive(false);
		jawToothButton->setChecked(false);
		jawToothButton->setAutoExclusive(true);
		chooseID = -1;
	}
}

void TabMainGUI::clearAllButtonPress()
{
	for (int i = 0; i < TOOTHNUM; i++)
	{
		m_pTeethScanTaskArray[i]->Set_ScanType(eScanNULL);
		if (toothFlagList[i] == true)
		{			
			toothFlagList[i] = false;
			int row = i / 8;
			int col = i % 8;
			int value = (row + 1) * 10 + col + 1;
			//QString path = "QPushButton{border-image: url(:/MainWidget/Resources/images/tooth" + QString::number(value, 10) + "0.png);}";
			//toothList[i]->setStyleSheet(path);
			toothList[i]->SetImage(QString(":/MainWidget/Resources/images/0/" + toothList[i]->text() + ".png"));
			QList<QPushButton *>* toothPushButtonList = judgeToothList(toothID[i]);
			if (toothPushButtonList)
				toothPushButtonList->clear();
			//toothPushButtonList.removeOne(toothList[i]);
		}
	}
}

void TabMainGUI::judgeSplitModelFlag()
{
	splitModelFlag = false;
	for (int i = 0; i < TOOTHNUM; i++)
	{
		if (toothFlagList[i] == true)
		{
			splitModelFlag = true;
		}
	}
}

void TabMainGUI::ScanDataPackagePress()
{
	judgeSplitModelFlag();
	QJsonObject scanObj;
	if (unMoulageFlag == true)//未分模
	{
		if (upperJawButtonFlag == true && lowerJawButtonFlag == true)
		{
			scanObj.insert("allJaw", 1);
		}
		else if (upperJawButtonFlag == true && lowerJawButtonFlag == false)
		{
			scanObj.insert("upperJaw", 1);
		}
		else if (lowerJawButtonFlag == true && upperJawButtonFlag == false)
		{
			scanObj.insert("lowerJaw", 1);
		}
		scanObj.insert("caseType", 1);
	}
	else if (splitModelFlag == true)//分模
	{
		scanObj.insert("caseType", 2);
	}
	else if (doMoulageFlag == true)//印模
	{
		scanObj.insert("caseType", 3);
	}
	/*QString fileQstr = QString::fromStdString(fileStr);*/
	scanObj.insert("filePath", fileQStr);
	QString strOrderDate = orderDate;
	strOrderDate.replace(":", "").replace(" ", "").replace("/", "");
	scanObj.insert("patientName", strOrderDate);
	emit scanDataSignal(scanObj);
}

void TabMainGUI::readFileStorage(QString fPath)
{
	QByteArray cdata = fPath.toLocal8Bit();
	std::string fPathStr = string(cdata);
	cv::FileStorage fRead(fPathStr.c_str(), cv::FileStorage::READ);
	orderDate = QString::fromLocal8Bit(fRead["Order Date"].string().c_str());
	orderNumber = QString::fromLocal8Bit(fRead["Order Number"].string().c_str());
	orderPatientName = QString::fromLocal8Bit(fRead["Patient Name"].string().c_str());
	orderDoctorName = QString::fromLocal8Bit(fRead["Doctor Name"].string().c_str());
	orderAddition = QString::fromLocal8Bit(fRead["Addition"].string().c_str());
	orderOperatorName = QString::fromLocal8Bit(fRead["Operator Name"].string().c_str());
	bool bsplitModelFlag = int(fRead["splitModelFlag"]);
	//QDateTime orderDateTime =QDateTime::fromString(orderDate, "yyyy/MM/dd hh:mm:ss");
	//QString orderDateTimeStr = orderDateTime.toString("yyyy/MM/dd/hh:mm:ss");
	ui.orderDateLineEdit->setText(orderDate);
	ui.orderNumLineEdit->setText(orderNumber);
	ui.patientNameLineEdit->setText(orderPatientName);
	ui.doctorNameLineEdit->setText(orderDoctorName);
	ui.additionTextEdit->setText(orderAddition);
	ui.operatorNameLineEdit->setText(orderOperatorName);
	if (fRead["scanModel"].real() == 1)
	{
		ui.unModelPushButton->click();
		ui.upperJawPushButton->setChecked(true);
		emit exportUpperJawSignal();
	}
	else if (fRead["scanModel"].real() == 2)
	{
		ui.unModelPushButton->click();
		ui.lowerJawPushButton->setChecked(true);
		emit exportLowerJawSignal();
	}
	else if (fRead["scanModel"].real() == 3)
	{
		ui.unModelPushButton->click();
		ui.lowerJawPushButton->setChecked(true);
		ui.upperJawPushButton->setChecked(true);
		emit exportUpperJawSignal();
		emit exportLowerJawSignal();
	}
	else if (fRead["scanModel"].real() == 4)
	{
		emit exportFirstMoulageSignal();
	}
	else if (fRead["scanModel"].real() == 5)
	{
		emit exportSecondMoulageSignal();
	}
	else if (fRead["scanModel"].real() == 6)
	{
		emit exportThirdMoulageSignal();
	}
	else if (fRead["scanModel"].real() == 7)
	{

	}
	if (bsplitModelFlag) {
		CTaskManager::getInstance()->DelAllTasks();
		QFile file2(fPath + ".json");
		if (!file2.open(QIODevice::ReadOnly))
		{
			qDebug() << "读出失败！";
			return;
		}
		QByteArray ba = file2.readAll();
		QJsonParseError e;
		QJsonDocument jsonDoc = QJsonDocument::fromJson(ba, &e);
		if (e.error == QJsonParseError::NoError && !jsonDoc.isNull())
		{
			qDebug() << "doc=\n" << jsonDoc;
		}
		if (jsonDoc.isArray()) {
			QJsonArray jsonArray = jsonDoc.array();
			pCScanTask upperScanTask = nullptr, lowerJawScanTask = nullptr, allJawScanTask = nullptr,
				pCrownGroupScan = nullptr;
			m_eScanType = eScanNULL;
			chooseID = m_eScanType + 1;
			clearAllButtonPress();
// 			for (int i = 0; i < 32;i++) {
// 				m_eScanType = eScanNULL;
// 				chooseID = m_eScanType + 1;
// 				toothList[i]->clicked();
// 			}
			for (int i = 0; i < jsonArray.size(); i++) {
				QJsonObject simp_ayjson = jsonArray[i].toObject();
				QString strClassName = simp_ayjson.value("class").toString(),
					strData = simp_ayjson.value("classdata").toString();
				pCScanTask pScanTask;
				if (strClassName == "CScanTask") {
					pScanTask = make_shared<CScanTask>();
				}
				else if (strClassName == "CStitchingTask") {
					pScanTask = make_shared<CStitchingTask>();
				}
				else if (strClassName == "CGroupScan") {
					pScanTask = make_shared<CGroupScan>();
				}
				QByteArray byteArray = QByteArray::fromBase64(strData.toLatin1());
				datastream kdata(byteArray.data(), byteArray.size());
				pScanTask->StreamValue(kdata, false);
				switch (pScanTask->Get_ScanType()) {
				case etoothCrown:{
					pCGroupScan pGroupScanTask = static_pointer_cast<CGroupScan>(pScanTask);
					chooseID = pScanTask->Get_ScanType() + 1;
					if(pScanTask){
						m_eScanType = pScanTask->Get_ScanType();
						for (int i = 0; i < pGroupScanTask->m_vtTeeth.size();i++) {
							toothList[pGroupScanTask->m_vtTeeth[i]]->clicked();
						}
					}
				}
					break;
				case einlayScan:
					chooseID = pScanTask->Get_ScanType() + 1;
					m_eScanType = pScanTask->Get_ScanType();
					toothList[pScanTask->Get_TeethId()]->clicked();
					break;
				case elossToothScan:
				case eJawToothScan:	//对颌牙
					chooseID = pScanTask->Get_ScanType() + 1;
					m_eScanType = pScanTask->Get_ScanType();
					toothList[pScanTask->Get_TeethId()]->clicked();
					break;
				case eLowerJawScan:
				case eUpperJawScan:
					if (pScanTask->Get_DentalImplant()) {
						chooseID = eDentalImplantScan + 1;
						m_eScanType = eDentalImplantScan;
						for (int i = 0; i < pScanTask->m_vtTeeth.size();i++) {
							toothList[pScanTask->m_vtTeeth[i]]->clicked();
						}						
					}
					break;
				}
// 				switch (pScanTask->Get_TaskType())
// 				{
// 				case eUpperStitching: {
// 					pCStitchingTask pTask = static_pointer_cast<CStitchingTask>(pScanTask);
// 					pTask->m_pSrcTask = upperScanTask;
// 					pTask->m_pDstTask = allJawScanTask;
// 				}
// 									  break;
// 				case eLowerStitching: {
// 					pCStitchingTask pTask = static_pointer_cast<CStitchingTask>(pScanTask);
// 					pTask->m_pSrcTask = lowerJawScanTask;
// 					pTask->m_pDstTask = allJawScanTask;
// 				}
// 									  break;
// 				case eUpperTeethStit: {
// 					pCStitchingTask pTask = static_pointer_cast<CStitchingTask>(pScanTask);
// 					pTask->m_pSrcTask = pCrownGroupScan;
// 					pTask->m_pDstTask = upperScanTask;
// 					break;
// 				}
// 				case eLowerTeethStit: {
// 					pCStitchingTask pTask = static_pointer_cast<CStitchingTask>(pScanTask);
// 					pTask->m_pSrcTask = pCrownGroupScan;
// 					pTask->m_pDstTask = lowerJawScanTask;
// 					break;
// 				}
// 				default:
// 					break;
// 				}
				splitModelFlag = true;
			}
		}
		ui.spliteModelPushButton->click();
	//	rightTabWidget->setCurrentIndex(1);
	}
}

void TabMainGUI::openFileDialogSlot()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("打开文件"), "./ScanData/", tr("Text Files(*.OI)"));
	if (!path.isEmpty())
	{
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QMessageBox::warning(this, QStringLiteral("读取文件"),
				QStringLiteral("不能打开文件:\n%1").arg(path));
			return;
		}
		readFileStorage(path);
		CTaskManager::getInstance()->getCurrentTask();
	}
	else {
		QMessageBox::warning(this, QStringLiteral("路径"), QStringLiteral("您未选择任何路径。"));
	}

}

void TabMainGUI::openDirectoryDialogSlot()
{
	QString file_path = QFileDialog::getExistingDirectory(this, "请选择扫描数据保存路径...", "./");
	if (!file_path.isEmpty())
	{
		ui.scanPathLineEdit->setText(file_path);
		filePath = file_path;
		CSystemConfig::shareInstance()->setValue(STR_SCANDATAPATH, filePath.toStdString());
	}
	else {
		QMessageBox::warning(this, QStringLiteral("路径"), QStringLiteral("您未选择任何路径。"));
	}

}

//void TabMainGUI::settingButtonClicked(QAbstractButton *button)
//{
//	// 当前点击的按钮
//	qDebug() << QString("Clicked Button : %1").arg(button->text());
//
//	// 遍历按钮，获取选中状态
//	QList<QAbstractButton*> list = ui.checkBoxGroupBox->buttons();
//	foreach(QAbstractButton *pCheckBox, list)
//	{
//		QString strStatus = pCheckBox->isChecked() ? "Checked" : "Unchecked";
//		qDebug() << QString("Button : %1 is %2").arg(pCheckBox->text()).arg(strStatus);
//	}
//}

void TabMainGUI::saveSpliteModelCheckBoxClicked()
{
	QPushButton * pCheckBox = static_cast<QPushButton *> (sender());
	if (!pCheckBox)
		return;
	CSystemConfig::shareInstance()->setValue(B_SAVESPLITEMODEL,pCheckBox->isChecked()?"true":"false");
}



void TabMainGUI::showSettingGroupBox()
{
	ui.settingGroupBox->show();
	//ui.settingGroupBox->show();
}

void TabMainGUI::showOrderInforGroupBox()
{
	ui.settingGroupBox->hide();
	ui.calibrationGroupBox->hide();
	ui.aboutGroupBox->hide();
	ui.orderInforGroupBox->show();
	ui.orderInforGroupBox->update();
}

void TabMainGUI::showCalibrationGroupBox()
{
	ui.settingGroupBox->hide();
	ui.aboutGroupBox->hide();
	ui.orderInforGroupBox->hide();
	ui.calibrationGroupBox->show();
	ui.calibrationGroupBox->update();
}

void TabMainGUI::showAboutGroupBox()
{
	ui.settingGroupBox->hide();
	ui.calibrationGroupBox->hide();
	ui.orderInforGroupBox->hide();
	ui.aboutGroupBox->show();
	ui.aboutGroupBox->update();
}

void TabMainGUI::showUnModelGroupBox()
{
	ui.unModelGroupBox->show();
	ui.spliteModelGroupBox->hide();
	ui.moulageGroupBox->hide();
}
void TabMainGUI::showSpliteModelGroupBox()
{
	ui.unModelGroupBox->hide();
	ui.spliteModelGroupBox->show();
	ui.moulageGroupBox->hide();
}

void TabMainGUI::showMoulageGroupBox()
{
	ui.unModelGroupBox->hide();
	ui.spliteModelGroupBox->hide();
	ui.moulageGroupBox->show();
}
