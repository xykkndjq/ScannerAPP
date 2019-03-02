#include "TabMainGUI.h"
#include <QPainter>
#include <QProxyStyle>

#define TOOTHNUM 32

class CustomTabStyle : public QProxyStyle
{
public:
	QSize sizeFromContents(ContentsType type, const QStyleOption *option,
		const QSize &size, const QWidget *widget) const
	{
		QSize s = QProxyStyle::sizeFromContents(type, option, size, widget);
		if (type == QStyle::CT_TabBarTab) {
			s.transpose();
			s.rwidth() = 90; // 设置每个tabBar中item的大小
			s.rheight() = 44;
		}
		return s;
	}

	void drawControl(ControlElement element, const QStyleOption *option, QPainter *painter, const QWidget *widget) const
	{
		if (element == CE_TabBarTabLabel) {
			if (const QStyleOptionTab *tab = qstyleoption_cast<const QStyleOptionTab *>(option)) {
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

		if (element == CE_TabBarTab) {
			QProxyStyle::drawControl(element, option, painter, widget);
		}
		QProxyStyle::drawControl(element, option, painter, widget);
	}
};

TabMainGUI::TabMainGUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->initVariable();
	this->constructIHM();
	this->setConnections();
}

TabMainGUI::~TabMainGUI()
{
}

void TabMainGUI::setConnections()
{
	//保存
	connect(saveButton, SIGNAL(clicked()), this, SLOT(PatientInformationSave()));
	//未分模
	connect(upperJawButton, SIGNAL(clicked()), this, SLOT(UpperJawPress()));
	connect(lowerJawButton, SIGNAL(clicked()), this, SLOT(LowerJawPress()));
	//印模
	connect(MoulageButton1, SIGNAL(clicked()), this, SLOT(MoulagePress1()));
	connect(MoulageButton2, SIGNAL(clicked()), this, SLOT(MoulagePress2()));
	connect(MoulageButton3, SIGNAL(clicked()), this, SLOT(MoulagePress3()));
	//分模
	connect(toothRadioButtonGroup,SIGNAL(buttonClicked(int)), this,SLOT(ToothGroupClicked(int)));

	connect(clearAllButton, SIGNAL(clicked()), this, SLOT(clearAllButtonPress()));

	connect(toothList[0], SIGNAL(clicked()), this, SLOT(Tooth11Press()));
	connect(toothList[1], SIGNAL(clicked()), this, SLOT(Tooth12Press()));
	connect(toothList[2], SIGNAL(clicked()), this, SLOT(Tooth13Press()));
	connect(toothList[3], SIGNAL(clicked()), this, SLOT(Tooth14Press()));
	connect(toothList[4], SIGNAL(clicked()), this, SLOT(Tooth15Press()));
	connect(toothList[5], SIGNAL(clicked()), this, SLOT(Tooth16Press()));
	connect(toothList[6], SIGNAL(clicked()), this, SLOT(Tooth17Press()));
	connect(toothList[7], SIGNAL(clicked()), this, SLOT(Tooth18Press()));

	connect(toothList[8], SIGNAL(clicked()), this, SLOT(Tooth21Press()));
	connect(toothList[9], SIGNAL(clicked()), this, SLOT(Tooth22Press()));
	connect(toothList[10], SIGNAL(clicked()), this, SLOT(Tooth23Press()));
	connect(toothList[11], SIGNAL(clicked()), this, SLOT(Tooth24Press()));
	connect(toothList[12], SIGNAL(clicked()), this, SLOT(Tooth25Press()));
	connect(toothList[13], SIGNAL(clicked()), this, SLOT(Tooth26Press()));
	connect(toothList[14], SIGNAL(clicked()), this, SLOT(Tooth27Press()));
	connect(toothList[15], SIGNAL(clicked()), this, SLOT(Tooth28Press()));

	connect(toothList[16], SIGNAL(clicked()), this, SLOT(Tooth31Press()));
	connect(toothList[17], SIGNAL(clicked()), this, SLOT(Tooth32Press()));
	connect(toothList[18], SIGNAL(clicked()), this, SLOT(Tooth33Press()));
	connect(toothList[19], SIGNAL(clicked()), this, SLOT(Tooth34Press()));
	connect(toothList[20], SIGNAL(clicked()), this, SLOT(Tooth35Press()));
	connect(toothList[21], SIGNAL(clicked()), this, SLOT(Tooth36Press()));
	connect(toothList[22], SIGNAL(clicked()), this, SLOT(Tooth37Press()));
	connect(toothList[23], SIGNAL(clicked()), this, SLOT(Tooth38Press()));

	connect(toothList[24], SIGNAL(clicked()), this, SLOT(Tooth41Press()));
	connect(toothList[25], SIGNAL(clicked()), this, SLOT(Tooth42Press()));
	connect(toothList[26], SIGNAL(clicked()), this, SLOT(Tooth43Press()));
	connect(toothList[27], SIGNAL(clicked()), this, SLOT(Tooth44Press()));
	connect(toothList[28], SIGNAL(clicked()), this, SLOT(Tooth45Press()));
	connect(toothList[29], SIGNAL(clicked()), this, SLOT(Tooth46Press()));
	connect(toothList[30], SIGNAL(clicked()), this, SLOT(Tooth47Press()));
	connect(toothList[31], SIGNAL(clicked()), this, SLOT(Tooth48Press()));

}

void TabMainGUI::initVariable()
{
	totalGLayOut = new QGridLayout(this);

	totalTabWidget = new QTabWidget(this);
	
	orderInforWidget = new QWidget(this);
	settingWidget = new QWidget(this);
	calibrateWidget = new QWidget(this);
	aboutWidget = new QWidget(this);

	//topbutton
	newButton = new QPushButton(QStringLiteral("新建"), this);
	exportButton = new QPushButton(QStringLiteral("导入"), this);
	saveButton = new QPushButton(QStringLiteral("保存"), this);
	watchButton = new QPushButton(QStringLiteral("预览"), this);
	scanButton = new QPushButton(QStringLiteral("扫描"), this);
	//leftbutton
	dateLineEdit = new QDateTimeEdit(this);
	dateLineEdit->setDateTime(QDateTime::currentDateTime());
	orderLineEdit = new QLineEdit(this);
	patientLineEdit = new QLineEdit(this);
	doctorLineEdit = new QLineEdit(this);
	additionTextEdit = new QTextEdit(this);
	additionLabel = new QLabel(QStringLiteral("备注："), this);
	//未分模
	upperJawButton = new QPushButton(this);
	upperJawButton->setFixedSize(310,110);
	upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}"
		"QPushButton:hover{background-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}");
	lowerJawButton = new QPushButton(this);
	lowerJawButton->setFixedSize(310, 110);
	lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}"
		"QPushButton:hover{background-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}");
	//印模
	MoulageButton1 = new QPushButton(this);
	MoulageButton1->setFixedSize(362, 185);
	MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}"
		"QPushButton:hover{background-image: url(:/MainWidget/Resources/images/Moulage1_yes.png);}");
	MoulageButton2 = new QPushButton(this);
	MoulageButton2->setFixedSize(362, 185);
	MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}"
		"QPushButton:hover{background-image: url(:/MainWidget/Resources/images/Moulage2_yes.png);}");
	MoulageButton3 = new QPushButton(this);
	MoulageButton3->setFixedSize(362, 185);
	MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}"
		"QPushButton:hover{background-image: url(:/MainWidget/Resources/images/Moulage3_yes.png);}");
	//分模
	//splitleft
	for (int i = 0; i < TOOTHNUM; i++)
	{
		QPushButton *toothPushButton = new QPushButton();
		toothPushButton->setFixedSize(40, 40);
		toothPushButton->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
		toothList.append(toothPushButton);
		int row = i / 8;
		int col = i % 8;
		int value = (row + 1) * 10 + col + 1;
		QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + QString::number(value, 10) + "0.png);}";
		toothList[i]->setStyleSheet(path);
		toothFlagList.append(false);
		toothID.append(0);
	}
	toothList[0]->setText("11");
	toothList[1]->setText("12");
	toothList[2]->setText("13");
	toothList[3]->setText("14");
	toothList[4]->setText("15");
	toothList[5]->setText("16");
	toothList[6]->setText("17");
	toothList[7]->setText("18");

	toothList[8]->setText("21");
	toothList[9]->setText("22");
	toothList[10]->setText("23");
	toothList[11]->setText("24");
	toothList[12]->setText("25");
	toothList[13]->setText("26");
	toothList[14]->setText("27");
	toothList[15]->setText("28");

	toothList[16]->setText("31");
	toothList[17]->setText("32");
	toothList[18]->setText("33");
	toothList[19]->setText("34");
	toothList[20]->setText("35");
	toothList[21]->setText("36");
	toothList[22]->setText("37");
	toothList[23]->setText("38");

	toothList[24]->setText("41");
	toothList[25]->setText("42");
	toothList[26]->setText("43");
	toothList[27]->setText("44");
	toothList[28]->setText("45");
	toothList[29]->setText("46");
	toothList[30]->setText("47");
	toothList[31]->setText("48");
	
	clearAllButton = new QPushButton(QStringLiteral("清除所有"));
	clearAllButton->setFixedSize(60,20);
	
	//splitright
	toothRadioButtonGroup = new QButtonGroup();
	toothRadioButtonGroup->setExclusive(true);
	
	totalCrownButton = new QRadioButton(QStringLiteral("              全冠"));
	totalCrownButton->setFixedSize(150,30);
	totalCrownButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	toothCrownButton = new QRadioButton(QStringLiteral("              牙冠"));
	toothCrownButton->setFixedSize(150, 30);
	toothCrownButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	lossToothButton = new QRadioButton(QStringLiteral("          缺失牙"));
	lossToothButton->setFixedSize(150, 30);
	lossToothButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	lossToothButton->setIcon(QIcon(":/MainWidget/Resources/images/loseTooth.png"));
	inlayButton = new QRadioButton(QStringLiteral("           嵌体"));
	inlayButton->setFixedSize(150, 30);
	inlayButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	inlayButton->setIcon(QIcon(":/MainWidget/Resources/images/inlay.png"));
	facingButton = new QRadioButton(QStringLiteral("           贴面"));
	facingButton->setFixedSize(150, 30);
	facingButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	facingButton->setIcon(QIcon(":/MainWidget/Resources/images/facing.png"));
	waxTypeButton = new QRadioButton(QStringLiteral("               蜡型"));
	waxTypeButton->setFixedSize(150, 30);
	waxTypeButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	implantButton = new QRadioButton(QStringLiteral("             种植体"));
	implantButton->setFixedSize(150, 30);
	implantButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	jawToothButton = new QRadioButton(QStringLiteral("             对颌牙"));
	jawToothButton->setFixedSize(150, 30);
	jawToothButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");

	//关于页面
	aboutTextLabel = new QLabel(this);
	aboutImageLabel = new QLabel(this);
}

void TabMainGUI::constructIHM()
{
	totalTabWidget->setTabPosition(QTabWidget::West);
	totalTabWidget->setStyle(new CustomTabStyle);
	totalTabWidget->setStyleSheet("QTabWidget::pane{ \border-left: 1px solid #eeeeee;}");
	
	//订单管理子页面
	totalOrderVLayout = new QVBoxLayout(orderInforWidget);

	//topwidget
	QWidget *topWidget = new QWidget();
	QHBoxLayout *topHLayout = new QHBoxLayout(topWidget);
	topHLayout->addStretch(1);
	topHLayout->addWidget(newButton);
	topHLayout->addStretch(1);
	topHLayout->addWidget(exportButton);
	topHLayout->addStretch(1);
	topHLayout->addWidget(saveButton);
	topHLayout->addStretch(5);
	topHLayout->addWidget(watchButton);
	topHLayout->addStretch(1);
	topHLayout->addWidget(scanButton);
	
	//leftwidget
	QWidget *leftTopWidget = new QWidget();
	QFormLayout *leftTopFLayout = new QFormLayout(leftTopWidget);

	leftTopFLayout->addRow(QStringLiteral("订单日期:"), dateLineEdit);
	leftTopFLayout->addRow(QStringLiteral("订单编号:"), orderLineEdit);
	leftTopFLayout->addRow(QStringLiteral("患者姓名:"), patientLineEdit);
	leftTopFLayout->addRow(QStringLiteral("医生姓名:"), doctorLineEdit);
	
	QWidget *leftBottomWidget = new QWidget();
	QGridLayout *leftBottomFLayout = new QGridLayout(leftBottomWidget);
	leftBottomFLayout->addWidget(additionLabel);
	leftBottomFLayout->addWidget(additionTextEdit);

	QWidget *leftWidget = new QWidget();
	QGridLayout *leftGLayout = new QGridLayout(leftWidget);
	leftGLayout->addWidget(leftTopWidget);
	leftGLayout->addWidget(leftBottomWidget);
	QWidget *totalleftWidget = new QWidget();
	QHBoxLayout *leftHLayout = new QHBoxLayout(totalleftWidget);
	leftHLayout->addWidget(leftWidget);
	leftHLayout->addStretch();
	//rightWidget
	QTabWidget *rightTabWidget = new QTabWidget();
	//未分模
	QWidget *rightTotalModelVWidget = new QWidget();
	QVBoxLayout *rightTotalModelVLayout = new QVBoxLayout(rightTotalModelVWidget);
	QWidget *rightTotalModelHWidget = new QWidget();
	QHBoxLayout *rightTotalModelHLayout = new QHBoxLayout(rightTotalModelHWidget);
	rightTotalModelVLayout->addStretch();
	rightTotalModelVLayout->addWidget(upperJawButton);
	rightTotalModelVLayout->addWidget(lowerJawButton);
	rightTotalModelVLayout->addStretch();

	rightTotalModelHLayout->addStretch();
	rightTotalModelHLayout->addWidget(rightTotalModelVWidget);
	rightTotalModelHLayout->addStretch();
	//分模
	QWidget *middleSplitModelWidget = new QWidget(rightTabWidget);
	middleSplitModelWidget->setGeometry(0,0,1000,1000);
	for (int i = 0; i < TOOTHNUM; i++)
	{
		toothList[i]->setParent(middleSplitModelWidget);
	}
	toothList[0]->setGeometry(200, 50, 40, 40);
	toothList[1]->setGeometry(180, 91, 40, 40);
	toothList[2]->setGeometry(160, 132, 40, 40);
	toothList[3]->setGeometry(140, 173, 40, 40);
	toothList[4]->setGeometry(120, 214, 40, 40);
	toothList[5]->setGeometry(100, 255, 40, 40);
	toothList[6]->setGeometry(80, 296, 40, 40);
	toothList[7]->setGeometry(60, 337, 40, 40);

	toothList[8]->setGeometry(250, 50, 40, 40);
	toothList[9]->setGeometry(270, 91, 40, 40);
	toothList[10]->setGeometry(290, 132, 40, 40);
	toothList[11]->setGeometry(310, 173, 40, 40);
	toothList[12]->setGeometry(330, 214, 40, 40);
	toothList[13]->setGeometry(350, 255, 40, 40);
	toothList[14]->setGeometry(370, 296, 40, 40);
	toothList[15]->setGeometry(390, 337, 40, 40);

	clearAllButton->setParent(middleSplitModelWidget);
	clearAllButton->setGeometry(220,390,60,20);

	toothList[23]->setGeometry(390, 437, 40, 40);
	toothList[22]->setGeometry(370, 478, 40, 40);
	toothList[21]->setGeometry(350, 519, 40, 40);
	toothList[20]->setGeometry(330, 560, 40, 40);
	toothList[19]->setGeometry(310, 601, 40, 40);
	toothList[18]->setGeometry(290, 642, 40, 40);
	toothList[17]->setGeometry(270, 683, 40, 40);
	toothList[16]->setGeometry(250, 724, 40, 40);

	toothList[31]->setGeometry(60, 437, 40, 40);
	toothList[30]->setGeometry(80, 478, 40, 40);
	toothList[29]->setGeometry(100, 519, 40, 40);
	toothList[28]->setGeometry(120, 560, 40, 40);
	toothList[27]->setGeometry(140, 601, 40, 40);
	toothList[26]->setGeometry(160, 642, 40, 40);
	toothList[25]->setGeometry(180, 683, 40, 40);
	toothList[24]->setGeometry(200, 724, 40, 40);

	toothRadioButtonGroup->setParent(middleSplitModelWidget);
	toothRadioButtonGroup->addButton(totalCrownButton, 1);
	toothRadioButtonGroup->addButton(toothCrownButton, 2);
	toothRadioButtonGroup->addButton(lossToothButton, 3);
	toothRadioButtonGroup->addButton(inlayButton, 4);
	toothRadioButtonGroup->addButton(facingButton, 5);
	toothRadioButtonGroup->addButton(waxTypeButton, 6);
	toothRadioButtonGroup->addButton(implantButton, 7);
	toothRadioButtonGroup->addButton(jawToothButton, 8);

	totalCrownButton->setParent(middleSplitModelWidget);
	toothCrownButton->setParent(middleSplitModelWidget);
	lossToothButton->setParent(middleSplitModelWidget);
	inlayButton->setParent(middleSplitModelWidget);
	facingButton->setParent(middleSplitModelWidget);
	waxTypeButton->setParent(middleSplitModelWidget);
	implantButton->setParent(middleSplitModelWidget);
	jawToothButton->setParent(middleSplitModelWidget);

	totalCrownButton->setGeometry(550,100,150,30);
	toothCrownButton->setGeometry(550, 150, 150, 30);
	lossToothButton->setGeometry(550, 200, 150, 30);
	inlayButton->setGeometry(550, 250, 150, 30);
	facingButton->setGeometry(550, 300, 150, 30);
	waxTypeButton->setGeometry(550, 350, 150, 30);
	implantButton->setGeometry(550, 400, 150, 30);
	jawToothButton->setGeometry(550, 450, 150, 30);

	//印模
	QWidget *rightMoulageVWidget = new QWidget();
	QVBoxLayout *rightMoulageVLayout = new QVBoxLayout(rightMoulageVWidget);
	rightMoulageVLayout->addStretch();
	rightMoulageVLayout->addWidget(MoulageButton1);
	rightMoulageVLayout->addWidget(MoulageButton2);
	rightMoulageVLayout->addWidget(MoulageButton3);
	rightMoulageVLayout->addStretch();
	QWidget *rightMoulageHWidget = new QWidget();
	QHBoxLayout *rightMoulageHLayout = new QHBoxLayout(rightMoulageHWidget);
	rightMoulageHLayout->addStretch();
	rightMoulageHLayout->addWidget(rightMoulageVWidget);
	rightMoulageHLayout->addStretch();

	rightTabWidget->addTab(rightTotalModelHWidget, QStringLiteral("未分模"));
	rightTabWidget->addTab(middleSplitModelWidget, QStringLiteral("分模"));
	rightTabWidget->addTab(rightMoulageHWidget, QStringLiteral("印模"));

	QWidget *bottomWidget = new QWidget();
	QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
	bottomHLayout->addWidget(totalleftWidget);
	bottomHLayout->addWidget(rightTabWidget);

	totalOrderVLayout->addWidget(topWidget);
	totalOrderVLayout->addWidget(bottomWidget);

	//关于子页面
	totalAboutGLayout = new QGridLayout(aboutWidget);
	
	aboutImageLabel->setPixmap(QPixmap(":/MainWidget/Resources/images/Setting.png"));
	aboutTextLabel->setStyleSheet(QString("color:rgb(128,128,128);"));
	aboutTextLabel->setText(QStringLiteral("牙齿扫描仪软件是一款对牙齿模型进行扫描，取得相应三维模型的软件，该软件配套牙齿扫描仪器使用。"));
	
	totalAboutGLayout->addWidget(aboutImageLabel, 1, 0, 1, 1);
	totalAboutGLayout->addWidget(aboutTextLabel, 1, 3, 1, 2);
	

	totalTabWidget->addTab(orderInforWidget, QStringLiteral("订单管理"));
	totalTabWidget->addTab(settingWidget, QStringLiteral("设置"));
	totalTabWidget->addTab(calibrateWidget, QStringLiteral("标定"));
	totalTabWidget->addTab(aboutWidget, QStringLiteral("关于"));
	totalGLayOut->addWidget(totalTabWidget);
	totalGLayOut->setContentsMargins(0, 0, 0, 0);
	this->setLayout(totalGLayOut);
}

void TabMainGUI::PatientInformationSave()
{
	std::cout << "storage a order information..." << std::endl;
	orderDate = dateLineEdit->text();
	orderNumber = orderLineEdit->text();
	orderPatientName = patientLineEdit->text();
	orderDoctorName = doctorLineEdit->text();
	orderAddition = additionTextEdit->toPlainText();

	//QByteArray cdata = orderPatientName.toLocal8Bit();
	//std::string pn(cdata);
	std::string fileStr = "./data/" + orderNumber.toStdString() + "_" + orderPatientName.toStdString() + ".orderInformation";
	cv::FileStorage fwrite(fileStr.c_str(), cv::FileStorage::WRITE);

	fwrite << "Date" << orderDate.toStdString()
		<< "Number" << orderNumber.toStdString()
		<< "Patient Name" << orderPatientName.toStdString()
		<< "Doctor Name" << orderDoctorName.toStdString()
		<< "Addition" << orderAddition.toStdString();

	if (unMoulageFlag == true)
	{
		if (upperJawButtonFlag == true)
		{
			fwrite << "upperJawButtonFlag" << true;
		}
		if (lowerJawButtonFlag == true)
		{
			fwrite << "lowerJawButtonFlag" << true;
		}
	}
	else if (doMoulageFlag == true)
	{
		if (MoulageFlag1 == true)
		{
			fwrite << "MoulageFlag1" << true;
		}
		else if (MoulageFlag2 == true)
		{
			fwrite << "MoulageFlag2" << true;
		}
		else if (MoulageFlag3 == true)
		{
			fwrite << "MoulageFlag3" << true;
		}
	}
	for (int i = 0; i < TOOTHNUM; i++)
	{
		if (toothFlagList[i] == true)
		{
			int row = i / 8;
			int col = i % 8;
			int value = (row + 1) * 10 + col + 1;
			QString path = "tooth" + QString::number(value, 10);
			fwrite << path.toStdString() << true;
		}
	}
}

void TabMainGUI::UpperJawPress()
{
	
	if (upperJawButtonFlag == false)
	{
		upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}");
		upperJawButtonFlag = true;
		unMoulageFlag = true;

		//doMoulage
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
		MoulageFlag1 = false;
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
		MoulageFlag2 = false;
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
		MoulageFlag3 = false;
		doMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else
	{
		upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
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
		lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}");
		lowerJawButtonFlag = true;
		unMoulageFlag = true;

		//doMoulage
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
		MoulageFlag1 = false;
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
		MoulageFlag2 = false;
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
		MoulageFlag3 = false;

		doMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else
	{
		lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
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
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_yes.png);}");
		MoulageFlag1 = true;
		doMoulageFlag = true;

		//doMoulage
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
		MoulageFlag2 = false;
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
		MoulageFlag3 = false;
		//unMoulage
		upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		upperJawButtonFlag = false;
		lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag1 == true && MoulageFlag2 == false && MoulageFlag3 == false)
	{
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
		MoulageFlag1 = false;
		doMoulageFlag = false;
	}
}

void TabMainGUI::MoulagePress2()
{
	if (MoulageFlag2 == false)
	{
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_yes.png);}");
		MoulageFlag2 = true;
		doMoulageFlag = true;

		//doMoulage
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
		MoulageFlag1 = false;
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
		MoulageFlag3 = false;
		//unMoulage
		upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		upperJawButtonFlag = false;
		lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag2 == true && MoulageFlag1 == false && MoulageFlag3 == false)
	{
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
		MoulageFlag2 = false;
		doMoulageFlag = false;
	}
}

void TabMainGUI::MoulagePress3()
{
	if (MoulageFlag3 == false)
	{
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_yes.png);}");
		MoulageFlag3 = true;
		doMoulageFlag = true;

		//doMoulage
		MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
		MoulageFlag1 = false;
		MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
		MoulageFlag2 = false;
		//unMoulage
		upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
		upperJawButtonFlag = false;
		lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
		lowerJawButtonFlag = false;

		unMoulageFlag = false;
		//spllitModel
		setSplitToothFalse();
	}
	else if (MoulageFlag3 == true && MoulageFlag1 == false && MoulageFlag2 == false)
	{
		MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
		MoulageFlag3 = false;
		doMoulageFlag = false;
	}
}

void TabMainGUI::judgeToothList(int id, QList<QPushButton*> &toothButtonList)
{
	switch (id)
	{
		case 1:
		{
			toothButtonList = totalCrownList;
			break;
		}
		case 2:
		{
			toothButtonList = toothCrownList;
			break;
		}
		case 3:
		{
			toothButtonList = lossToothList;
			break;
		}
		case 4:
		{
			toothButtonList = inlayList;
			break;
		}
		case 5:
		{
			toothButtonList = facingList;
			break;
		}
		case 6:
		{
			toothButtonList = waxTypeList;
			break;
		}
		case 7:
		{
			toothButtonList = implantList;
			break;
		}
		case 8:
		{
			toothButtonList = jawToothList;
			break;
		}
	}
}

void TabMainGUI::ToothGroupClicked(int id)
{
	chooseID = id;
	switch (id)
	{
		case 1:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/11.png);}";
			totalCrownButton->setStyleSheet(path);
			if (totalCrownList.size() != 0)
			{
				foreach(QPushButton *chooseButton, totalCrownList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "1.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 2:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/22.png);}";
			toothCrownButton->setStyleSheet(path);
			if (toothCrownList.size() != 0)
			{
				foreach(QPushButton *chooseButton, toothCrownList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "2.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 3:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/33.png);}";
			lossToothButton->setStyleSheet(path);
			if (lossToothList.size() != 0)
			{
				foreach(QPushButton *chooseButton, lossToothList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "3.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 4:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/44.png);}";
			inlayButton->setStyleSheet(path);
			if (inlayList.size() != 0)
			{
				foreach(QPushButton *chooseButton, inlayList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "4.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 5:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/55.png);}";
			facingButton->setStyleSheet(path);
			if (facingList.size() != 0)
			{
				foreach(QPushButton *chooseButton, facingList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "5.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 6:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/66.png);}";
			waxTypeButton->setStyleSheet(path);
			if (waxTypeList.size() != 0)
			{
				foreach(QPushButton *chooseButton, waxTypeList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "6.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 7:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/77.png);}";
			implantButton->setStyleSheet(path);
			if (implantList.size() != 0)
			{
				foreach(QPushButton *chooseButton, implantList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "7.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 8:
		{
			QString path = "QRadioButton{background-image: url(:/MainWidget/Resources/images/88.png);}";
			jawToothButton->setStyleSheet(path);
			if (jawToothList.size() != 0)
			{
				foreach(QPushButton *chooseButton, jawToothList)
				{
					QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "8.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
	}
}

/*----------------1--------------------*/
void TabMainGUI::Tooth11Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[0] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth11" + QString::number(chooseID, 10) + ".png);}";
			toothList[0]->setStyleSheet(path);
			toothPushButtonList.append(toothList[0]);
			toothFlagList[0] = true;
			toothID[0] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[0] == true && chooseID == toothID[0])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth110.png);}";
			toothList[0]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[0]);
			toothFlagList[0] = false;
		}
		else if (toothFlagList[0] == true && chooseID != toothID[0])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth11" + QString::number(chooseID, 10) + ".png);}";
			toothList[0]->setStyleSheet(path);
			toothPushButtonList.append(toothList[0]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[0], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[0]);
			toothID[0] = chooseID;
		}
	}
}

void TabMainGUI::Tooth12Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[1] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth12" + QString::number(chooseID, 10) + ".png);}";
			toothList[1]->setStyleSheet(path);
			toothPushButtonList.append(toothList[1]);
			toothFlagList[1] = true;
			toothID[1] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[1] == true && chooseID == toothID[1])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth120.png);}";
			toothList[1]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[1]);
			toothFlagList[1] = false;
			toothID[1] = 0;
		}
		else if (toothFlagList[1] == true && chooseID != toothID[1])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth12" + QString::number(chooseID, 10) + ".png);}";
			toothList[1]->setStyleSheet(path);
			toothPushButtonList.append(toothList[1]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[1], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[1]);
			toothID[1] = chooseID;
		}
	}
}

void TabMainGUI::Tooth13Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[2] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth13" + QString::number(chooseID, 10) + ".png);}";
			toothList[2]->setStyleSheet(path);
			toothPushButtonList.append(toothList[2]);
			toothFlagList[2] = true;
			toothID[2] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[2] == true && chooseID == toothID[2])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth130.png);}";
			toothList[2]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[2]);
			toothFlagList[2] = false;
			toothID[2] = 0;
		}
		else if (toothFlagList[2] == true && chooseID != toothID[2])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth13" + QString::number(chooseID, 10) + ".png);}";
			toothList[2]->setStyleSheet(path);
			toothPushButtonList.append(toothList[2]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[2], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[2]);
			toothID[2] = chooseID;
		}
	}
}

void TabMainGUI::Tooth14Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[3] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth14" + QString::number(chooseID, 10) + ".png);}";
			toothList[3]->setStyleSheet(path);
			toothPushButtonList.append(toothList[3]);
			toothFlagList[3] = true;
			toothID[3] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[3] == true && chooseID == toothID[3])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth140.png);}";
			toothList[3]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[3]);
			toothFlagList[3] = false;
			toothID[3] = 0;
		}
		else if (toothFlagList[3] == true && chooseID != toothID[3])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth14" + QString::number(chooseID, 10) + ".png);}";
			toothList[3]->setStyleSheet(path);
			toothPushButtonList.append(toothList[3]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[3], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[3]);
			toothID[3] = chooseID;
		}
	}
}

void TabMainGUI::Tooth15Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[4] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth15" + QString::number(chooseID, 10) + ".png);}";
			toothList[4]->setStyleSheet(path);
			toothPushButtonList.append(toothList[4]);
			toothFlagList[4] = true;
			toothID[4] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[4] == true && chooseID == toothID[4])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth150.png);}";
			toothList[4]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[4]);
			toothFlagList[4] = false;
			toothID[4] = 0;
		}
		else if (toothFlagList[4] == true && chooseID != toothID[4])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth15" + QString::number(chooseID, 10) + ".png);}";
			toothList[4]->setStyleSheet(path);
			toothPushButtonList.append(toothList[4]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[4], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[4]);
			toothID[4] = chooseID;
		}
	}
}

void TabMainGUI::Tooth16Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[5] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth16" + QString::number(chooseID, 10) + ".png);}";
			toothList[5]->setStyleSheet(path);
			toothPushButtonList.append(toothList[5]);
			toothFlagList[5] = true;
			toothID[5] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[5] == true && chooseID == toothID[5])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth160.png);}";
			toothList[5]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[5]);
			toothFlagList[5] = false;
			toothID[5] = 0;
		}
		else if (toothFlagList[5] == true && chooseID != toothID[5])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth16" + QString::number(chooseID, 10) + ".png);}";
			toothList[5]->setStyleSheet(path);
			toothPushButtonList.append(toothList[5]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[5], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[5]);
			toothID[5] = chooseID;
		}
	}
}

void TabMainGUI::Tooth17Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[6] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth17" + QString::number(chooseID, 10) + ".png);}";
			toothList[6]->setStyleSheet(path);
			toothPushButtonList.append(toothList[6]);
			toothFlagList[6] = true;
			toothID[6] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[6] == true && chooseID == toothID[6])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth170.png);}";
			toothList[6]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[6]);
			toothFlagList[6] = false;
			toothID[6] = 0;
		}
		else if (toothFlagList[6] == true && chooseID != toothID[6])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth17" + QString::number(chooseID, 10) + ".png);}";
			toothList[6]->setStyleSheet(path);
			toothPushButtonList.append(toothList[6]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[6], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[6]);
			toothID[6] = chooseID;
		}
	}
}

void TabMainGUI::Tooth18Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[7] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth18" + QString::number(chooseID, 10) + ".png);}";
			toothList[7]->setStyleSheet(path);
			toothPushButtonList.append(toothList[7]);
			toothFlagList[7] = true;
			toothID[7] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[7] == true && chooseID == toothID[7])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth180.png);}";
			toothList[7]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[7]);
			toothFlagList[7] = false;
			toothID[7] = 0;
		}
		else if (toothFlagList[7] == true && chooseID != toothID[7])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth18" + QString::number(chooseID, 10) + ".png);}";
			toothList[7]->setStyleSheet(path);
			toothPushButtonList.append(toothList[7]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[7], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[7]);
			toothID[7] = chooseID;
		}
	}
}

/*----------------2--------------------*/
void TabMainGUI::Tooth21Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[8] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth21" + QString::number(chooseID, 10) + ".png);}";
			toothList[8]->setStyleSheet(path);
			toothPushButtonList.append(toothList[8]);
			toothFlagList[8] = true;
			toothID[8] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[8] == true && chooseID == toothID[8])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth210.png);}";
			toothList[8]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[8]);
			toothFlagList[8] = false;
			toothID[8] = 0;
		}
		else if (toothFlagList[8] == true && chooseID != toothID[8])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth21" + QString::number(chooseID, 10) + ".png);}";
			toothList[8]->setStyleSheet(path);
			toothPushButtonList.append(toothList[8]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[8], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[8]);
			toothID[8] = chooseID;
		}
	}
}

void TabMainGUI::Tooth22Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[9] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth22" + QString::number(chooseID, 10) + ".png);}";
			toothList[9]->setStyleSheet(path);
			toothPushButtonList.append(toothList[9]);
			toothFlagList[9] = true;
			toothID[9] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[9] == true && chooseID == toothID[9])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth220.png);}";
			toothList[9]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[9]);
			toothFlagList[9] = false;
			toothID[9] = 0;
		}
		else if (toothFlagList[9] == true && chooseID != toothID[9])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth22" + QString::number(chooseID, 10) + ".png);}";
			toothList[9]->setStyleSheet(path);
			toothPushButtonList.append(toothList[9]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[9], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[9]);
			toothID[9] = chooseID;
		}
	}
}

void TabMainGUI::Tooth23Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[10] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth23" + QString::number(chooseID, 10) + ".png);}";
			toothList[10]->setStyleSheet(path);
			toothPushButtonList.append(toothList[10]);
			toothFlagList[10] = true;
			toothID[10] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[10] == true && chooseID == toothID[10])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth230.png);}";
			toothList[10]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[10]);
			toothFlagList[10] = false;
			toothID[10] = 0;
		}
		else if (toothFlagList[10] == true && chooseID != toothID[10])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth23" + QString::number(chooseID, 10) + ".png);}";
			toothList[10]->setStyleSheet(path);
			toothPushButtonList.append(toothList[10]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[10], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[10]);
			toothID[10] = chooseID;
		}
	}
}

void TabMainGUI::Tooth24Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[11] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth24" + QString::number(chooseID, 10) + ".png);}";
			toothList[11]->setStyleSheet(path);
			toothPushButtonList.append(toothList[11]);
			toothFlagList[11] = true;
			toothID[11] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[11] == true && chooseID == toothID[11])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth240.png);}";
			toothList[11]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[11]);
			toothFlagList[11] = false;
			toothID[11] = 0;
		}
		else if (toothFlagList[11] == true && chooseID != toothID[11])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth24" + QString::number(chooseID, 10) + ".png);}";
			toothList[11]->setStyleSheet(path);
			toothPushButtonList.append(toothList[11]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[11], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[11]);
			toothID[11] = chooseID;
		}
	}
}

void TabMainGUI::Tooth25Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[12] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth25" + QString::number(chooseID, 10) + ".png);}";
			toothList[12]->setStyleSheet(path);
			toothPushButtonList.append(toothList[12]);
			toothFlagList[12] = true;
			toothID[12] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[12] == true && chooseID == toothID[12])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth250.png);}";
			toothList[12]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[12]);
			toothFlagList[12] = false;
			toothID[12] = 0;
		}
		else if (toothFlagList[12] == true && chooseID != toothID[12])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth25" + QString::number(chooseID, 10) + ".png);}";
			toothList[12]->setStyleSheet(path);
			toothPushButtonList.append(toothList[12]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[12], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[12]);
			toothID[12] = chooseID;
		}
	}
}

void TabMainGUI::Tooth26Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[13] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth26" + QString::number(chooseID, 10) + ".png);}";
			toothList[13]->setStyleSheet(path);
			toothPushButtonList.append(toothList[13]);
			toothFlagList[13] = true;
			toothID[13] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[13] == true && chooseID == toothID[13])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth260.png);}";
			toothList[13]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[13]);
			toothFlagList[13] = false;
			toothID[13] = 0;
		}
		else if (toothFlagList[13] == true && chooseID != toothID[13])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth26" + QString::number(chooseID, 10) + ".png);}";
			toothList[13]->setStyleSheet(path);
			toothPushButtonList.append(toothList[13]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[13], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[13]);
			toothID[13] = chooseID;
		}
	}
}

void TabMainGUI::Tooth27Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[14] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth27" + QString::number(chooseID, 10) + ".png);}";
			toothList[14]->setStyleSheet(path);
			toothPushButtonList.append(toothList[14]);
			toothFlagList[14] = true;
			toothID[14] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[14] == true && chooseID == toothID[14])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth270.png);}";
			toothList[14]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[14]);
			toothFlagList[14] = false;
			toothID[14] = 0;
		}
		else if (toothFlagList[14] == true && chooseID != toothID[14])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth27" + QString::number(chooseID, 10) + ".png);}";
			toothList[14]->setStyleSheet(path);
			toothPushButtonList.append(toothList[14]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[14], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[14]);
			toothID[14] = chooseID;
		}
	}
}

void TabMainGUI::Tooth28Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[15] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth28" + QString::number(chooseID, 10) + ".png);}";
			toothList[15]->setStyleSheet(path);
			toothPushButtonList.append(toothList[15]);
			toothFlagList[15] = true;
			toothID[15] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[15] == true && chooseID == toothID[15])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth280.png);}";
			toothList[15]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[15]);
			toothFlagList[15] = false;
			toothID[15] = 0;
		}
		else if (toothFlagList[15] == true && chooseID != toothID[15])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth28" + QString::number(chooseID, 10) + ".png);}";
			toothList[15]->setStyleSheet(path);
			toothPushButtonList.append(toothList[15]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[15], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[15]);
			toothID[15] = chooseID;
		}
	}
}

/*----------------3--------------------*/
void TabMainGUI::Tooth31Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[16] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth31" + QString::number(chooseID, 10) + ".png);}";
			toothList[16]->setStyleSheet(path);
			toothPushButtonList.append(toothList[16]);
			toothFlagList[16] = true;
			toothID[16] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[16] == true && chooseID == toothID[16])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth310.png);}";
			toothList[16]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[16]);
			toothFlagList[16] = false;
			toothID[16] = 0;
		}
		else if (toothFlagList[16] == true && chooseID != toothID[16])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth31" + QString::number(chooseID, 10) + ".png);}";
			toothList[16]->setStyleSheet(path);
			toothPushButtonList.append(toothList[16]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[16], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[16]);
			toothID[16] = chooseID;
		}
	}
}

void TabMainGUI::Tooth32Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[17] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth32" + QString::number(chooseID, 10) + ".png);}";
			toothList[17]->setStyleSheet(path);
			toothPushButtonList.append(toothList[17]);
			toothFlagList[17] = true;
			toothID[17] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[17] == true && chooseID == toothID[17])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth320.png);}";
			toothList[17]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[17]);
			toothFlagList[17] = false;
			toothID[17] = 0;
		}
		else if (toothFlagList[17] == true && chooseID != toothID[17])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth32" + QString::number(chooseID, 10) + ".png);}";
			toothList[17]->setStyleSheet(path);
			toothPushButtonList.append(toothList[17]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[17], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[17]);
			toothID[17] = chooseID;
		}
	}
}

void TabMainGUI::Tooth33Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[18] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth33" + QString::number(chooseID, 10) + ".png);}";
			toothList[18]->setStyleSheet(path);
			toothPushButtonList.append(toothList[18]);
			toothFlagList[18] = true;
			toothID[18] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[18] == true && chooseID == toothID[18])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth330.png);}";
			toothList[18]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[18]);
			toothFlagList[18] = false;
			toothID[18] = 0;
		}
		else if (toothFlagList[18] == true && chooseID != toothID[18])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth33" + QString::number(chooseID, 10) + ".png);}";
			toothList[18]->setStyleSheet(path);
			toothPushButtonList.append(toothList[18]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[18], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[18]);
			toothID[18] = chooseID;
		}
	}
}

void TabMainGUI::Tooth34Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[19] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth34" + QString::number(chooseID, 10) + ".png);}";
			toothList[19]->setStyleSheet(path);
			toothPushButtonList.append(toothList[19]);
			toothFlagList[19] = true;
			toothID[19] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[19] == true && chooseID == toothID[19])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth340.png);}";
			toothList[19]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[19]);
			toothFlagList[19] = false;
			toothID[19] = 0;
		}
		else if (toothFlagList[19] == true && chooseID != toothID[19])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth34" + QString::number(chooseID, 10) + ".png);}";
			toothList[19]->setStyleSheet(path);
			toothPushButtonList.append(toothList[19]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[19], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[19]);
			toothID[19] = chooseID;
		}
	}
}

void TabMainGUI::Tooth35Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[20] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth35" + QString::number(chooseID, 10) + ".png);}";
			toothList[20]->setStyleSheet(path);
			toothPushButtonList.append(toothList[20]);
			toothFlagList[20] = true;
			toothID[20] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[20] == true && chooseID == toothID[20])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth350.png);}";
			toothList[20]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[20]);
			toothFlagList[20] = false;
			toothID[20] = 0;
		}
		else if (toothFlagList[20] == true && chooseID != toothID[20])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth35" + QString::number(chooseID, 10) + ".png);}";
			toothList[20]->setStyleSheet(path);
			toothPushButtonList.append(toothList[20]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[20], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[20]);
			toothID[20] = chooseID;
		}
	}
}

void TabMainGUI::Tooth36Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[21] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth36" + QString::number(chooseID, 10) + ".png);}";
			toothList[21]->setStyleSheet(path);
			toothPushButtonList.append(toothList[21]);
			toothFlagList[21] = true;
			toothID[21] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[21] == true && chooseID == toothID[21])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth360.png);}";
			toothList[21]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[21]);
			toothFlagList[21] = false;
			toothID[21] = 0;
		}
		else if (toothFlagList[21] == true && chooseID != toothID[21])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth36" + QString::number(chooseID, 10) + ".png);}";
			toothList[21]->setStyleSheet(path);
			toothPushButtonList.append(toothList[21]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[21], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[21]);
			toothID[21] = chooseID;
		}
	}
}

void TabMainGUI::Tooth37Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[22] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth37" + QString::number(chooseID, 10) + ".png);}";
			toothList[22]->setStyleSheet(path);
			toothPushButtonList.append(toothList[22]);
			toothFlagList[22] = true;
			toothID[22] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[22] == true && chooseID == toothID[22])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth370.png);}";
			toothList[22]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[22]);
			toothFlagList[22] = false;
			toothID[22] = 0;
		}
		else if (toothFlagList[22] == true && chooseID != toothID[22])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth37" + QString::number(chooseID, 10) + ".png);}";
			toothList[22]->setStyleSheet(path);
			toothPushButtonList.append(toothList[22]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[22], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[22]);
			toothID[22] = chooseID;
		}
	}
}

void TabMainGUI::Tooth38Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[23] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth38" + QString::number(chooseID, 10) + ".png);}";
			toothList[23]->setStyleSheet(path);
			toothPushButtonList.append(toothList[23]);
			toothFlagList[23] = true;
			toothID[23] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[23] == true && chooseID == toothID[23])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth380.png);}";
			toothList[23]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[23]);
			toothFlagList[23] = false;
			toothID[23] = 0;
		}
		else if (toothFlagList[23] == true && chooseID != toothID[23])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth38" + QString::number(chooseID, 10) + ".png);}";
			toothList[23]->setStyleSheet(path);
			toothPushButtonList.append(toothList[23]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[23], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[23]);
			toothID[23] = chooseID;
		}
	}
}

/*----------------4--------------------*/
void TabMainGUI::Tooth41Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[24] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth41" + QString::number(chooseID, 10) + ".png);}";
			toothList[24]->setStyleSheet(path);
			toothPushButtonList.append(toothList[24]);
			toothFlagList[24] = true;
			toothID[24] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[24] == true && chooseID == toothID[24])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth410.png);}";
			toothList[24]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[24]);
			toothFlagList[24] = false;
			toothID[24] = 0;
		}
		else if (toothFlagList[24] == true && chooseID != toothID[24])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth41" + QString::number(chooseID, 10) + ".png);}";
			toothList[24]->setStyleSheet(path);
			toothPushButtonList.append(toothList[24]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[24], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[24]);
			toothID[24] = chooseID;
		}
	}
}

void TabMainGUI::Tooth42Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[25] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth42" + QString::number(chooseID, 10) + ".png);}";
			toothList[25]->setStyleSheet(path);
			toothPushButtonList.append(toothList[25]);
			toothFlagList[25] = true;
			toothID[25] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[25] == true && chooseID == toothID[25])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth420.png);}";
			toothList[25]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[25]);
			toothFlagList[25] = false;
			toothID[25] = 0;
		}
		else if (toothFlagList[25] == true && chooseID != toothID[25])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth42" + QString::number(chooseID, 10) + ".png);}";
			toothList[25]->setStyleSheet(path);
			toothPushButtonList.append(toothList[25]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[25], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[25]);
			toothID[25] = chooseID;
		}
	}
}

void TabMainGUI::Tooth43Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[26] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth43" + QString::number(chooseID, 10) + ".png);}";
			toothList[26]->setStyleSheet(path);
			toothPushButtonList.append(toothList[26]);
			toothFlagList[26] = true;
			toothID[26] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[26] == true && chooseID == toothID[26])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth430.png);}";
			toothList[26]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[26]);
			toothFlagList[26] = false;
			toothID[26] = 0;
		}
		else if (toothFlagList[26] == true && chooseID != toothID[26])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth43" + QString::number(chooseID, 10) + ".png);}";
			toothList[26]->setStyleSheet(path);
			toothPushButtonList.append(toothList[26]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[26], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[26]);
			toothID[26] = chooseID;
		}
	}
}

void TabMainGUI::Tooth44Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[27] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth44" + QString::number(chooseID, 10) + ".png);}";
			toothList[27]->setStyleSheet(path);
			toothPushButtonList.append(toothList[27]);
			toothFlagList[27] = true;
			toothID[27] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[27] == true && chooseID == toothID[27])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth440.png);}";
			toothList[27]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[27]);
			toothFlagList[27] = false;
			toothID[27] = 0;
		}
		else if (toothFlagList[27] == true && chooseID != toothID[28])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth44" + QString::number(chooseID, 10) + ".png);}";
			toothList[27]->setStyleSheet(path);
			toothPushButtonList.append(toothList[27]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[27], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[27]);
			toothID[27] = chooseID;
		}
	}
}

void TabMainGUI::Tooth45Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[28] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth45" + QString::number(chooseID, 10) + ".png);}";
			toothList[28]->setStyleSheet(path);
			toothPushButtonList.append(toothList[28]);
			toothFlagList[28] = true;
			toothID[28] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[28] == true && chooseID == toothID[28])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth450.png);}";
			toothList[28]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[28]);
			toothFlagList[28] = false;
			toothID[28] = 0;
		}
		else if (toothFlagList[28] == true && chooseID != toothID[28])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth45" + QString::number(chooseID, 10) + ".png);}";
			toothList[28]->setStyleSheet(path);
			toothPushButtonList.append(toothList[28]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[28], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[28]);
			toothID[28] = chooseID;
		}
	}
}

void TabMainGUI::Tooth46Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[29] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth46" + QString::number(chooseID, 10) + ".png);}";
			toothList[29]->setStyleSheet(path);
			toothPushButtonList.append(toothList[29]);
			toothFlagList[29] = true;
			toothID[29] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[29] == true && chooseID == toothID[29])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth460.png);}";
			toothList[29]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[29]);
			toothFlagList[29] = false;
			toothID[29] = 0;
		}
		else if (toothFlagList[29] == true && chooseID != toothID[29])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth46" + QString::number(chooseID, 10) + ".png);}";
			toothList[29]->setStyleSheet(path);
			toothPushButtonList.append(toothList[29]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[29], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[29]);
			toothID[29] = chooseID;
		}
	}
}

void TabMainGUI::Tooth47Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[30] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth47" + QString::number(chooseID, 10) + ".png);}";
			toothList[30]->setStyleSheet(path);
			toothPushButtonList.append(toothList[30]);
			toothFlagList[30] = true;
			toothID[30] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[30] == true && chooseID == toothID[30])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth470.png);}";
			toothList[30]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[30]);
			toothFlagList[30] = false;
			toothID[30] = 0;
		}
		else if (toothFlagList[30] == true && chooseID != toothID[30])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth47" + QString::number(chooseID, 10) + ".png);}";
			toothList[30]->setStyleSheet(path);
			toothPushButtonList.append(toothList[30]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[30], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[30]);
			toothID[30] = chooseID;
		}
	}
}

void TabMainGUI::Tooth48Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (toothFlagList[31] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth48" + QString::number(chooseID, 10) + ".png);}";
			toothList[31]->setStyleSheet(path);
			toothPushButtonList.append(toothList[31]);
			toothFlagList[31] = true;
			toothID[31] = chooseID;
			//doMoulage
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;

			doMoulageFlag = false;
			//unMoulage
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;

			unMoulageFlag = false;
		}
		else if (toothFlagList[31] == true && chooseID==toothID[31])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth480.png);}";
			toothList[31]->setStyleSheet(path);
			toothPushButtonList.removeOne(toothList[31]);
			toothFlagList[31] = false;
			toothID[31] = 0;
		}
		else if (toothFlagList[31] == true && chooseID != toothID[31])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth48"+ QString::number(chooseID, 10) +".png);}";
			toothList[31]->setStyleSheet(path);
			toothPushButtonList.append(toothList[31]);
			QList<QPushButton *> toothIDButtonList;
			judgeToothList(toothID[31], toothIDButtonList);
			toothIDButtonList.removeOne(toothList[31]);
			toothID[31] = chooseID;
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
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth"+ QString::number(value, 10)+"0.png);}";
			toothList[i]->setStyleSheet(path);
			QList<QPushButton *> toothPushButtonList;
			judgeToothList(toothID[i], toothPushButtonList);
			toothPushButtonList.removeOne(toothList[i]);

		}
	}
	
	totalCrownButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/10.png);}");
	if (totalCrownButton->isChecked())
	{
		totalCrownButton->setAutoExclusive(false);
		totalCrownButton->setChecked(false);
		
		totalCrownButton->setAutoExclusive(true);

		chooseID = -1;
	}

	toothCrownButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/20.png);}");//牙冠
	if (toothCrownButton->isChecked())
	{
		toothCrownButton->setAutoExclusive(false);
		toothCrownButton->setChecked(false);
		toothCrownButton->setAutoExclusive(true);
		chooseID = -1;
	}

	lossToothButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/30.png);}");//缺失牙
	if (lossToothButton->isChecked())
	{
		lossToothButton->setAutoExclusive(false);
		lossToothButton->setChecked(false);
		lossToothButton->setAutoExclusive(true);
		chooseID = -1;
	}
	
	inlayButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/40.png);}");//嵌体
	if (inlayButton->isChecked())
	{
		inlayButton->setAutoExclusive(false);
		inlayButton->setChecked(false);
		inlayButton->setAutoExclusive(true);
		chooseID = -1;
	}
	
	
	facingButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/50.png);}");//贴面
	if (facingButton->isChecked())
	{
		facingButton->setAutoExclusive(false);
		facingButton->setChecked(false);
		facingButton->setAutoExclusive(true);
		chooseID = -1;
	}
	
	waxTypeButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/60.png);}");//蜡型
	
	if (waxTypeButton->isChecked())
	{
		waxTypeButton->setAutoExclusive(false);
		waxTypeButton->setChecked(false);
		waxTypeButton->setAutoExclusive(true);
		chooseID = -1;
	}
	
	implantButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/70.png);}");//种植体
	if (implantButton->isChecked())
	{
		implantButton->setAutoExclusive(false);
		implantButton->setChecked(false);
		implantButton->setAutoExclusive(true);
		chooseID = -1;
	}
	
	
	jawToothButton->setStyleSheet("QRadioButton{background-image: url(:/MainWidget/Resources/images/80.png);}");//对颌牙
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
		if (toothFlagList[i] == true)
		{
			toothFlagList[i] = false;
			int row = i / 8;
			int col = i % 8;
			int value = (row + 1) * 10 + col + 1;
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + QString::number(value, 10) + "0.png);}";
			toothList[i]->setStyleSheet(path);
			QList<QPushButton *> toothPushButtonList;
			judgeToothList(toothID[i], toothPushButtonList);
			toothPushButtonList.removeOne(toothList[i]);
		}
	}
}