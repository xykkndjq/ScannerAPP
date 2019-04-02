#include "TabMainGUI.h"
#include <QPainter>
#include <QProxyStyle>
#include <QTextCodec>

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
	//订单信息子页面
	//导入
	connect(exportButton, SIGNAL(clicked()), this, SLOT(openFileDialogSlot()));

	//保存
	connect(saveScanButton, SIGNAL(clicked()), this, SLOT(PatientInformationSave()));
	connect(this, SIGNAL(scanSignal()), this, SLOT(ScanDataPackagePress()));//扫描按钮连接
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

	for (int i = 0; i < 32; i++)
	{
		connect(toothList[i], SIGNAL(clicked()), this, SLOT(ToothButtonListPress()));
	}
}

void TabMainGUI::initVariable()
{
	totalGLayOut = new QGridLayout(this);

	totalTabWidget = new QTabWidget(this);
	
	orderInforWidget = new QWidget(this);
	settingWidget = new QWidget(this);
	calibrateWidget = new QWidget(this);
	aboutWidget = new QWidget(this);
	 
	//orderInformPage订单管理页面
	//topbutton
	newButton = new QPushButton(QStringLiteral("新建"), this);
	exportButton = new QPushButton(QStringLiteral("导入"), this);
	//saveButton = new QPushButton(QStringLiteral("保存"), this);
	watchButton = new QPushButton(QStringLiteral("预览"), this);
	saveScanButton = new QPushButton(QStringLiteral("保存并扫描"), this);

	//leftbutton
	dateLineEdit = new QDateTimeEdit(this);
	QDateTime currentDateTime = QDateTime::currentDateTime();
	dateLineEdit->setDateTime(currentDateTime);
	dateLineEdit->setReadOnly(true);

	orderLineEdit = new QLineEdit(this);
	QString curDateTimeStr = currentDateTime.toString("yyyyMMddhhmmss");
	lastDateTimeStr = curDateTimeStr;
	orderLineEdit->setText(curDateTimeStr);
	orderLineEdit->setReadOnly(true);

	patientLineEdit = new QLineEdit(this);
	doctorLineEdit = new QLineEdit(this);
	additionTextEdit = new QTextEdit(this);
	additionLabel = new QLabel(QStringLiteral("备注："), this);
	//未分模
	upperJawButton = new QPushButton(this);
	upperJawButton->setFixedSize(310,110);
	upperJawButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}"
		"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}");
	lowerJawButton = new QPushButton(this);
	lowerJawButton->setFixedSize(310, 110);
	lowerJawButton->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}"
		"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}");
	//印模
	MoulageButton1 = new QPushButton(this);
	MoulageButton1->setFixedSize(362, 185);
	MoulageButton1->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}"
		"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/Moulage1_yes.png);}");
	MoulageButton2 = new QPushButton(this);
	MoulageButton2->setFixedSize(362, 185);
	MoulageButton2->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}"
		"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/Moulage2_yes.png);}");
	MoulageButton3 = new QPushButton(this);
	MoulageButton3->setFixedSize(362, 185);
	MoulageButton3->setStyleSheet("QPushButton{border-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}"
		"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/Moulage3_yes.png);}");
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
		toothList[i]->setObjectName(QString::number(i,10));
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

	//calibatepage标定页面
	calibratePushButton = new QPushButton(QStringLiteral("开始标定"),this);
	calibratePushButton->setFixedSize(150, 30);
	calibratePushButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");

	globalCaliPushButton = new QPushButton(QStringLiteral("全局标定"));
	globalCaliPushButton->setFixedSize(150, 30);
	globalCaliPushButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");

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
	//topHLayout->addWidget(saveButton);
	topHLayout->addWidget(watchButton);
	topHLayout->addStretch(8);
	topHLayout->addWidget(saveScanButton);
	
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
	
	//标定子页面
	calibratePushButton->setParent(calibrateWidget);
	calibratePushButton->setGeometry(1400, 50, 40, 40);
	globalCaliPushButton->setParent(calibrateWidget);
	globalCaliPushButton->setGeometry(500, 50, 40, 40);

	totalTabWidget->addTab(orderInforWidget, QStringLiteral("订单管理"));
	totalTabWidget->addTab(settingWidget, QStringLiteral("设置"));
	totalTabWidget->addTab(calibrateWidget, QStringLiteral("标定"));
	totalTabWidget->addTab(aboutWidget, QStringLiteral("关于"));
	totalGLayOut->addWidget(totalTabWidget);
	totalGLayOut->setContentsMargins(0, 0, 0, 0);
	this->setLayout(totalGLayOut);
}

bool TabMainGUI::isDirExist(QString fullPath)
{
	QDir dir(fullPath);
	std::cout << fullPath.toStdString() << std::endl;
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

bool TabMainGUI::judgePatientSaveFlag()
{
	if ((orderDate != NULL && orderNumber != NULL && orderPatientName != NULL && orderDoctorName != NULL)&&
		(unMoulageFlag == true|| doMoulageFlag == true|| splitModelFlag == true))
	{
		QString fullPath = "./data/" + orderNumber;
		bool creatFlag = isDirExist(fullPath);
		fileQStr = "./data/" + orderNumber + "/";
		QByteArray orderPN = ToChineseStr(orderPatientName);
		std::string filePath = "./data/" + orderNumber.toStdString() + "/" + orderPN.data() + ".OI";
		cv::FileStorage fwrite(filePath.c_str(), cv::FileStorage::WRITE);
		fwrite << "Order Date" << orderDate.toStdString()
			<< "Order Number" << orderNumber.toStdString()
			<< "Patient Name" << orderPN.data()
			<< "Doctor Name" << ToChineseStr(orderDoctorName).data()
			<< "Addition" << ToChineseStr(orderAddition).data();

		if (unMoulageFlag == true)
		{
			if (upperJawButtonFlag == true && lowerJawButtonFlag == false)
			{
				fwrite << "onlyUpperJaw" << true;
			}
			if (lowerJawButtonFlag == true && upperJawButtonFlag == false)
			{
				fwrite << "onlyLowerJaw" << true;
			}
			else if (upperJawButtonFlag == true && lowerJawButtonFlag == true)
			{
				fwrite << "allJaw" << true;
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

		for (int j = 1; j < 9; j++)
		{
			switch (j)
			{
			case 1:
			{
				if (totalCrownList.size() != 0)
				{
					for (int i = 0; i < totalCrownList.size(); i++)
					{
						fwrite << "totalCrown" << totalCrownList[i]->text().toStdString();
					}
				}
				break;
			}
			case 2:
			{
				if (toothCrownList.size() != 0)
				{
					for (int i = 0; i < toothCrownList.size(); i++)
					{
						fwrite << "toothCrown" << toothCrownList[i]->text().toStdString();
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
						fwrite << "lossTooth" << lossToothList[i]->text().toStdString();
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
						fwrite << "facing" << facingList[i]->text().toStdString();
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
						fwrite << "waxType" << waxTypeList[i]->text().toStdString();
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
						fwrite << "implant" << implantList[i]->text().toStdString();
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
						fwrite << "jawTooth" << jawToothList[i]->text().toStdString();
					}
				}
				break;
			}
			}
		}
		return true;
	}
	else
	{
		QMessageBox::warning(NULL, QStringLiteral("警告"), QStringLiteral("订单信息不完整"), QMessageBox::Yes, QMessageBox::Yes);
		return false;
	}
}

void TabMainGUI::PatientInformationSave()
{

	std::cout << "storage a order information..." << std::endl;
	orderDate = dateLineEdit->text();
	orderNumber = orderLineEdit->text();
	orderPatientName = patientLineEdit->text();
	orderDoctorName = doctorLineEdit->text();
	orderAddition = additionTextEdit->toPlainText();
	judgeSplitModelFlag();
	
	if (judgePatientSaveFlag() == true)
	{
		emit scanSignal();
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

QList<QPushButton*> TabMainGUI::judgeToothList(int id)
{
	switch (id)
	{
		case 1:
		{
			return totalCrownList;
			//break;
		}
		case 2:
		{
			return toothCrownList;
			//break;
		}
		case 3:
		{
			return lossToothList;
			break;
		}
		case 4:
		{
			return inlayList;
			break;
		}
		case 5:
		{
			return facingList;
			break;
		}
		case 6:
		{
			return waxTypeList;
			break;
		}
		case 7:
		{
			return implantList;
			break;
		}
		case 8:
		{
			return jawToothList;
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

void TabMainGUI::addToothList(int id,int toothButtonIndex)
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
	if (chooseID != -1)
	{
		if (toothFlagList[toothButtonIndex] == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + QString::number(chooseID, 10) + ".png);}";
			toothList[toothButtonIndex]->setStyleSheet(path);
			addToothList(chooseID, toothButtonIndex);
			qDebug() << totalCrownList.size();
			toothFlagList[toothButtonIndex] = true;
			toothID[toothButtonIndex] = chooseID;
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
		else if (toothFlagList[toothButtonIndex] == true && chooseID == toothID[toothButtonIndex])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + "0.png);}";
			toothList[toothButtonIndex]->setStyleSheet(path);
			removeToothList(chooseID, toothButtonIndex);
			toothFlagList[toothButtonIndex] = false;
		}
		else if (toothFlagList[toothButtonIndex] == true && chooseID != toothID[0])
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth" + toothList[toothButtonIndex]->text() + QString::number(chooseID, 10) + ".png);}";
			toothList[toothButtonIndex]->setStyleSheet(path);
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
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth"+ QString::number(value, 10)+"0.png);}";
			toothList[i]->setStyleSheet(path);
			QList<QPushButton *> toothPushButtonList = judgeToothList(toothID[i]);
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
			QList<QPushButton *> toothPushButtonList = judgeToothList(toothID[i]);
			toothPushButtonList.removeOne(toothList[i]);
		}
	}
}

void TabMainGUI::judgeSplitModelFlag()
{
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
		scanObj.insert("caseType", 1);
		if (upperJawButtonFlag == true && lowerJawButtonFlag == true)
		{
			scanObj.insert("allJaw", 1);
		}
		else if(upperJawButtonFlag == true && lowerJawButtonFlag == false)
		{
			scanObj.insert("upperJaw", 1);
		}
		else if (lowerJawButtonFlag == true && upperJawButtonFlag == false)
		{
			scanObj.insert("lowerJaw", 1);
		}
	}
	else if (splitModelFlag == true)//分模
	{
		scanObj.insert("caseType", 2);
	}
	else if(doMoulageFlag == true)//印模
	{
		scanObj.insert("caseType", 3);
	}
	/*QString fileQstr = QString::fromStdString(fileStr);*/
	scanObj.insert("filePath", fileQStr);
	scanObj.insert("patientName", orderPatientName);
	emit scanDataSignal(scanObj);
}

void TabMainGUI::openFileDialogSlot()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("打开文件"),".",tr("Text Files(*.OI)"));
	if (!path.isEmpty()) 
	{
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) 
		{
			QMessageBox::warning(this, QStringLiteral("读取文件"),
				QStringLiteral("不能打开文件:\n%1").arg(path));
			return;
		}
		
	}
	else {
		QMessageBox::warning(this, QStringLiteral("路径"),QStringLiteral("您未选择任何路径。"));
	}

}