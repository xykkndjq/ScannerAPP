#include "TabMainGUI.h"
#include <QPainter>
#include <QProxyStyle>

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

	connect(tooth11, SIGNAL(clicked()), this, SLOT(Tooth11Press()));
	connect(tooth12, SIGNAL(clicked()), this, SLOT(Tooth12Press()));
	connect(tooth13, SIGNAL(clicked()), this, SLOT(Tooth13Press()));
	connect(tooth14, SIGNAL(clicked()), this, SLOT(Tooth14Press()));
	connect(tooth15, SIGNAL(clicked()), this, SLOT(Tooth15Press()));
	connect(tooth16, SIGNAL(clicked()), this, SLOT(Tooth16Press()));
	connect(tooth17, SIGNAL(clicked()), this, SLOT(Tooth17Press()));
	connect(tooth18, SIGNAL(clicked()), this, SLOT(Tooth18Press()));

	connect(tooth21, SIGNAL(clicked()), this, SLOT(Tooth21Press()));
	connect(tooth22, SIGNAL(clicked()), this, SLOT(Tooth22Press()));
	connect(tooth23, SIGNAL(clicked()), this, SLOT(Tooth23Press()));
	connect(tooth24, SIGNAL(clicked()), this, SLOT(Tooth24Press()));
	connect(tooth25, SIGNAL(clicked()), this, SLOT(Tooth25Press()));
	connect(tooth26, SIGNAL(clicked()), this, SLOT(Tooth26Press()));
	connect(tooth27, SIGNAL(clicked()), this, SLOT(Tooth27Press()));
	connect(tooth28, SIGNAL(clicked()), this, SLOT(Tooth28Press()));

	connect(tooth31, SIGNAL(clicked()), this, SLOT(Tooth31Press()));
	connect(tooth32, SIGNAL(clicked()), this, SLOT(Tooth32Press()));
	connect(tooth33, SIGNAL(clicked()), this, SLOT(Tooth33Press()));
	connect(tooth34, SIGNAL(clicked()), this, SLOT(Tooth34Press()));
	connect(tooth35, SIGNAL(clicked()), this, SLOT(Tooth35Press()));
	connect(tooth36, SIGNAL(clicked()), this, SLOT(Tooth36Press()));
	connect(tooth37, SIGNAL(clicked()), this, SLOT(Tooth37Press()));
	connect(tooth38, SIGNAL(clicked()), this, SLOT(Tooth38Press()));

	connect(tooth41, SIGNAL(clicked()), this, SLOT(Tooth41Press()));
	connect(tooth42, SIGNAL(clicked()), this, SLOT(Tooth42Press()));
	connect(tooth43, SIGNAL(clicked()), this, SLOT(Tooth43Press()));
	connect(tooth44, SIGNAL(clicked()), this, SLOT(Tooth44Press()));
	connect(tooth45, SIGNAL(clicked()), this, SLOT(Tooth45Press()));
	connect(tooth46, SIGNAL(clicked()), this, SLOT(Tooth46Press()));
	connect(tooth47, SIGNAL(clicked()), this, SLOT(Tooth47Press()));
	connect(tooth48, SIGNAL(clicked()), this, SLOT(Tooth48Press()));

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
	tooth11 = new QPushButton("11", this);
	tooth11->setFixedSize(33,33);
	tooth11->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth11->setContentsMargins(0, 0, 0, 0);
	tooth12 = new QPushButton("12", this);
	tooth12->setFixedSize(33, 33);
	tooth12->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth12->setContentsMargins(0, 0, 0, 0);
	tooth13 = new QPushButton("13", this);
	tooth13->setFixedSize(33, 33);
	tooth13->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth13->setContentsMargins(0, 0, 0, 0);
	tooth14 = new QPushButton("14", this);
	tooth14->setFixedSize(33, 33);
	tooth14->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth14->setContentsMargins(0, 0, 0, 0);
	tooth15 = new QPushButton("15", this);
	tooth15->setFixedSize(33, 33);
	tooth15->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth14->setContentsMargins(0, 0, 0, 0);
	tooth16 = new QPushButton("16", this);
	tooth16->setFixedSize(33, 33);
	tooth16->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth16->setContentsMargins(0, 0, 0, 0);
	tooth17 = new QPushButton("17", this);
	tooth17->setFixedSize(33, 33);
	tooth17->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth17->setContentsMargins(0, 0, 0, 0);
	tooth18 = new QPushButton("18", this);
	tooth18->setFixedSize(33, 33);
	tooth18->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth18->setContentsMargins(0, 0, 0, 0);

	tooth21 = new QPushButton("21", this);
	tooth21->setFixedSize(33, 33);
	tooth21->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth21->setContentsMargins(0, 0, 0, 0);
	tooth22 = new QPushButton("22", this);
	tooth22->setFixedSize(33, 33);
	tooth22->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth22->setContentsMargins(0, 0, 0, 0);
	tooth23 = new QPushButton("23", this);
	tooth23->setFixedSize(33, 33);
	tooth23->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth23->setContentsMargins(0, 0, 0, 0);
	tooth24 = new QPushButton("24", this);
	tooth24->setFixedSize(33, 33);
	tooth24->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth24->setContentsMargins(0, 0, 0, 0);
	tooth25 = new QPushButton("25", this);
	tooth25->setFixedSize(33, 33);
	tooth25->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth25->setContentsMargins(0, 0, 0, 0);
	tooth26 = new QPushButton("26", this);
	tooth26->setFixedSize(33, 33);
	tooth26->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth26->setContentsMargins(0, 0, 0, 0);
	tooth27 = new QPushButton("27", this);
	tooth27->setFixedSize(33, 33);
	tooth27->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth27->setContentsMargins(0, 0, 0, 0);
	tooth28 = new QPushButton("28", this);
	tooth28->setFixedSize(33, 33);
	tooth28->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth28->setContentsMargins(0, 0, 0, 0);

	tooth31 = new QPushButton("31", this);
	tooth31->setFixedSize(33, 33);
	tooth31->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth31->setContentsMargins(0, 0, 0, 0);
	tooth32 = new QPushButton("32", this);
	tooth32->setFixedSize(33, 33);
	tooth32->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth32->setContentsMargins(0, 0, 0, 0);
	tooth33 = new QPushButton("33", this);
	tooth33->setFixedSize(33, 33);
	tooth33->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth33->setContentsMargins(0, 0, 0, 0);
	tooth34 = new QPushButton("34", this);
	tooth34->setFixedSize(33, 33);
	tooth34->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth34->setContentsMargins(0, 0, 0, 0);
	tooth35 = new QPushButton("35", this);
	tooth35->setFixedSize(33, 33);
	tooth35->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth35->setContentsMargins(0, 0, 0, 0);
	tooth36 = new QPushButton("36", this);
	tooth36->setFixedSize(33, 33);
	tooth36->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth36->setContentsMargins(0, 0, 0, 0);
	tooth37 = new QPushButton("37", this);
	tooth37->setFixedSize(33, 33);
	tooth37->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth37->setContentsMargins(0, 0, 0, 0);
	tooth38 = new QPushButton("38", this);
	tooth38->setFixedSize(33, 33);
	tooth38->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth38->setContentsMargins(0, 0, 0, 0);

	tooth41 = new QPushButton("41", this);
	tooth41->setFixedSize(33, 33);
	tooth41->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth41->setContentsMargins(0, 0, 0, 0);
	tooth42 = new QPushButton("42", this);
	tooth42->setFixedSize(33, 33);
	tooth42->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth42->setContentsMargins(0, 0, 0, 0);
	tooth43 = new QPushButton("43", this);
	tooth43->setFixedSize(33, 33);
	tooth43->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth43->setContentsMargins(0, 0, 0, 0);
	tooth44 = new QPushButton("44", this);
	tooth44->setFixedSize(33, 33);
	tooth44->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth44->setContentsMargins(0, 0, 0, 0);
	tooth45 = new QPushButton("45", this);
	tooth45->setFixedSize(33, 33);
	tooth45->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth45->setContentsMargins(0, 0, 0, 0);
	tooth46 = new QPushButton("46", this);
	tooth46->setFixedSize(33, 33);
	tooth46->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth46->setContentsMargins(0, 0, 0, 0);
	tooth47 = new QPushButton("47", this);
	tooth47->setFixedSize(33, 33);
	tooth47->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth47->setContentsMargins(0, 0, 0, 0);
	tooth48 = new QPushButton("48", this);
	tooth48->setFixedSize(33, 33);
	tooth48->setStyleSheet("border-width: 1px;border-style: solid;border-color: rgb(128, 128, 128);");
	tooth48->setContentsMargins(0, 0, 0, 0);

	clearAllButton = new QPushButton(QStringLiteral("清除所有"), this);
	clearAllButton->setFixedSize(60,20);
	
	//splitright
	toothRadioButtonGroup = new QButtonGroup(this);
	toothRadioButtonGroup->setExclusive(true);
	
	totalCrownButton = new QRadioButton(QStringLiteral("              全冠"),this);
	totalCrownButton->setFixedSize(150,30);
	totalCrownButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	toothCrownButton = new QRadioButton(QStringLiteral("              牙冠"), this);
	toothCrownButton->setFixedSize(150, 30);
	toothCrownButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	lossToothButton = new QRadioButton(QStringLiteral("          缺失牙"), this);
	lossToothButton->setFixedSize(150, 30);
	lossToothButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	lossToothButton->setIcon(QIcon(":/MainWidget/Resources/images/loseTooth.png"));
	inlayButton = new QRadioButton(QStringLiteral("           嵌体"), this);
	inlayButton->setFixedSize(150, 30);
	inlayButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	inlayButton->setIcon(QIcon(":/MainWidget/Resources/images/inlay.png"));
	facingButton = new QRadioButton(QStringLiteral("           贴面"), this);
	facingButton->setFixedSize(150, 30);
	facingButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	facingButton->setIcon(QIcon(":/MainWidget/Resources/images/facing.png"));
	waxTypeButton = new QRadioButton(QStringLiteral("               蜡型"), this);
	waxTypeButton->setFixedSize(150, 30);
	waxTypeButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	implantButton = new QRadioButton(QStringLiteral("             种植体"), this);
	implantButton->setFixedSize(150, 30);
	implantButton->setStyleSheet("border-width: 2px;border-style: solid;border-color: rgb(128, 128, 128);");
	jawToothButton = new QRadioButton(QStringLiteral("             对颌牙"), this);
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
	QWidget *oneRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *oneRowTopLayout = new QHBoxLayout(oneRowTopWidget);
	oneRowTopLayout->addStretch(8);
	oneRowTopLayout->addWidget(tooth11);
	oneRowTopLayout->addStretch(1);
	oneRowTopLayout->addWidget(tooth21);
	oneRowTopLayout->addStretch(8);
	QWidget *twoRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *twoRowTopLayout = new QHBoxLayout(twoRowTopWidget);
	twoRowTopLayout->addStretch(7);
	twoRowTopLayout->addWidget(tooth12);
	twoRowTopLayout->addStretch(4);
	twoRowTopLayout->addWidget(tooth22);
	twoRowTopLayout->addStretch(7);
	QWidget *threeRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *threeRowTopLayout = new QHBoxLayout(threeRowTopWidget);
	threeRowTopLayout->addStretch(6);
	threeRowTopLayout->addWidget(tooth13);
	threeRowTopLayout->addStretch(7);
	threeRowTopLayout->addWidget(tooth23);
	threeRowTopLayout->addStretch(6);
	QWidget *fourRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *fourRowTopLayout = new QHBoxLayout(fourRowTopWidget);
	fourRowTopLayout->addStretch(5);
	fourRowTopLayout->addWidget(tooth14);
	fourRowTopLayout->addStretch(10);
	fourRowTopLayout->addWidget(tooth24);
	fourRowTopLayout->addStretch(5);
	QWidget *fiveRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *fiveRowTopLayout = new QHBoxLayout(fiveRowTopWidget);
	fiveRowTopLayout->addStretch(4);
	fiveRowTopLayout->addWidget(tooth15);
	fiveRowTopLayout->addStretch(11);
	fiveRowTopLayout->addWidget(tooth25);
	fiveRowTopLayout->addStretch(4);
	QWidget *sixRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *sixRowTopLayout = new QHBoxLayout(sixRowTopWidget);
	sixRowTopLayout->addStretch(3);
	sixRowTopLayout->addWidget(tooth16);
	sixRowTopLayout->addStretch(12);
	sixRowTopLayout->addWidget(tooth26);
	sixRowTopLayout->addStretch(3);
	QWidget *sevenRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *sevenRowTopLayout = new QHBoxLayout(sevenRowTopWidget);
	sevenRowTopLayout->addStretch(2);
	sevenRowTopLayout->addWidget(tooth17);
	sevenRowTopLayout->addStretch(12);
	sevenRowTopLayout->addWidget(tooth27);
	sevenRowTopLayout->addStretch(2);
	QWidget *eightRowTopWidget = new QWidget(rightTabWidget);
	QHBoxLayout *eightRowTopLayout = new QHBoxLayout(eightRowTopWidget);
	eightRowTopLayout->addStretch(1);
	eightRowTopLayout->addWidget(tooth18);
	eightRowTopLayout->addStretch(12);
	eightRowTopLayout->addWidget(tooth28);
	eightRowTopLayout->addStretch(1);

	QWidget *totalRowTopWidget = new QWidget(rightTabWidget);
	QVBoxLayout *totalRowTopVLayout = new QVBoxLayout(totalRowTopWidget);
	totalRowTopVLayout->addStretch(5);
	totalRowTopVLayout->addWidget(oneRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(twoRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(threeRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(fourRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(fiveRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(sixRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(sevenRowTopWidget);
	totalRowTopVLayout->addStretch(1);
	totalRowTopVLayout->addWidget(eightRowTopWidget);
	//totalRowTopVLayout->addStretch(5);

	QWidget *oneRowBottomWidget = new QWidget();
	QHBoxLayout *oneRowBottomLayout = new QHBoxLayout(oneRowBottomWidget);
	oneRowBottomLayout->addStretch(1);
	oneRowBottomLayout->addWidget(tooth48);
	oneRowBottomLayout->addStretch(12);
	oneRowBottomLayout->addWidget(tooth38);
	oneRowBottomLayout->addStretch(1);
	QWidget *twoRowBottomWidget = new QWidget();
	QHBoxLayout *twoRowBottomLayout = new QHBoxLayout(twoRowBottomWidget);
	twoRowBottomLayout->addStretch(2);
	twoRowBottomLayout->addWidget(tooth47);
	twoRowBottomLayout->addStretch(12);
	twoRowBottomLayout->addWidget(tooth37);
	twoRowBottomLayout->addStretch(2);
	QWidget *threeRowBottomWidget = new QWidget();
	QHBoxLayout *threeRowBottomLayout = new QHBoxLayout(threeRowBottomWidget);
	threeRowBottomLayout->addStretch(3);
	threeRowBottomLayout->addWidget(tooth46);
	threeRowBottomLayout->addStretch(12);
	threeRowBottomLayout->addWidget(tooth36);
	threeRowBottomLayout->addStretch(3);
	QWidget *fourRowBottomWidget = new QWidget();
	QHBoxLayout *fourRowBottomLayout = new QHBoxLayout(fourRowBottomWidget);
	fourRowBottomLayout->addStretch(4);
	fourRowBottomLayout->addWidget(tooth45);
	fourRowBottomLayout->addStretch(11);
	fourRowBottomLayout->addWidget(tooth35);
	fourRowBottomLayout->addStretch(4);
	QWidget *fiveRowBottomWidget = new QWidget();
	QHBoxLayout *fiveRowBottomLayout = new QHBoxLayout(fiveRowBottomWidget);
	fiveRowBottomLayout->addStretch(5);
	fiveRowBottomLayout->addWidget(tooth44);
	fiveRowBottomLayout->addStretch(10);
	fiveRowBottomLayout->addWidget(tooth34);
	fiveRowBottomLayout->addStretch(5);
	QWidget *sixRowBottomWidget = new QWidget();
	QHBoxLayout *sixRowBottomLayout = new QHBoxLayout(sixRowBottomWidget);
	sixRowBottomLayout->addStretch(6);
	sixRowBottomLayout->addWidget(tooth43);
	sixRowBottomLayout->addStretch(7);
	sixRowBottomLayout->addWidget(tooth33);
	sixRowBottomLayout->addStretch(6);
	QWidget *sevenRowBottomWidget = new QWidget();
	QHBoxLayout *sevenRowBottomLayout = new QHBoxLayout(sevenRowBottomWidget);
	sevenRowBottomLayout->addStretch(7);
	sevenRowBottomLayout->addWidget(tooth42);
	sevenRowBottomLayout->addStretch(4);
	sevenRowBottomLayout->addWidget(tooth32);
	sevenRowBottomLayout->addStretch(7);
	QWidget *eightRowBottomWidget = new QWidget();
	QHBoxLayout *eightRowBottomLayout = new QHBoxLayout(eightRowBottomWidget);
	eightRowBottomLayout->addStretch(8);
	eightRowBottomLayout->addWidget(tooth41);
	eightRowBottomLayout->addStretch(1);
	eightRowBottomLayout->addWidget(tooth31);
	eightRowBottomLayout->addStretch(8);

	QWidget *totalRowBottomWidget = new QWidget();
	QVBoxLayout *totalRowBottomVLayout = new QVBoxLayout(totalRowBottomWidget);
	totalRowBottomVLayout->addWidget(oneRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(twoRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(threeRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(fourRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(fiveRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(sixRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(sevenRowBottomWidget);
	totalRowBottomVLayout->addStretch(1);
	totalRowBottomVLayout->addWidget(eightRowBottomWidget);
	totalRowBottomVLayout->addStretch(5);

	QWidget *clearAllWidget = new QWidget();
	QHBoxLayout *clearAllLayout = new QHBoxLayout(clearAllWidget);
	clearAllLayout->addStretch();
	clearAllLayout->addWidget(clearAllButton);
	clearAllLayout->addStretch();

	QWidget *splitLeftModelWidget = new QWidget(rightTabWidget);
	splitLeftModelWidget->setFixedSize(450,860);
	QVBoxLayout *splitLeftModelLayout = new QVBoxLayout(splitLeftModelWidget);
	splitLeftModelLayout->addStretch(3);
	splitLeftModelLayout->addWidget(totalRowTopWidget);
	splitLeftModelLayout->addWidget(clearAllWidget);
	splitLeftModelLayout->addWidget(totalRowBottomWidget);
	splitLeftModelLayout->addStretch(3);

	//toothRadioBox
	QWidget *toothRadioButtonWidget = new QWidget();
	QVBoxLayout *toothRadioButtonVLayout = new QVBoxLayout(toothRadioButtonWidget);
	toothRadioButtonVLayout->addStretch(5);
	toothRadioButtonVLayout->addWidget(totalCrownButton); 
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(toothCrownButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(lossToothButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(inlayButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(facingButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(waxTypeButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(implantButton);
	toothRadioButtonVLayout->addStretch(1);
	toothRadioButtonVLayout->addWidget(jawToothButton);
	toothRadioButtonVLayout->addStretch(5);

	toothRadioButtonGroup->addButton(totalCrownButton,1);
	toothRadioButtonGroup->addButton(toothCrownButton,2);
	toothRadioButtonGroup->addButton(lossToothButton,3);
	toothRadioButtonGroup->addButton(inlayButton,4);
	toothRadioButtonGroup->addButton(facingButton,5);
	toothRadioButtonGroup->addButton(waxTypeButton,6);
	toothRadioButtonGroup->addButton(implantButton,7);
	toothRadioButtonGroup->addButton(jawToothButton,8);
	
	//splitLeftModelWidget->show();

	QWidget *splitRightModelWidget = new QWidget();
	QVBoxLayout *splitRightModelLayout = new QVBoxLayout(splitRightModelWidget);
	splitRightModelLayout->addWidget(toothRadioButtonWidget);

	QWidget *rightSplitModelWidget = new QWidget(rightTabWidget);
	QHBoxLayout *rightSplitModelLayout = new QHBoxLayout(rightSplitModelWidget);
	//rightSplitModelLayout->addStretch(1);
	rightSplitModelLayout->addWidget(splitLeftModelWidget);
	rightSplitModelLayout->addStretch(1);
	rightSplitModelLayout->addWidget(splitRightModelWidget);
	rightSplitModelLayout->addStretch(1);

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
	rightTabWidget->addTab(rightSplitModelWidget, QStringLiteral("分模"));
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
}

void TabMainGUI::UpperJawPress()
{
	if (unMoulageFlag == true)
	{
		if (upperJawButtonFlag == false)
		{
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}");
			upperJawButtonFlag = true;
			doMoulageFlag = false;
		}
		else
		{
			upperJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}");
			upperJawButtonFlag = false;
			if (lowerJawButtonFlag == false)
			{
				doMoulageFlag = true;
			}
		}
	}
}

void TabMainGUI::LowerJawPress()
{
	if (unMoulageFlag == true)
	{
		if (lowerJawButtonFlag == false)
		{
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}");
			lowerJawButtonFlag = true;
			doMoulageFlag = false;
		}
		else
		{
			lowerJawButton->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}");
			lowerJawButtonFlag = false;
			if (upperJawButtonFlag == false)
			{
				doMoulageFlag = true;
			}
		}
	}
}

void TabMainGUI::MoulagePress1()
{
	if (doMoulageFlag == true)
	{
		if (MoulageFlag1 == false && MoulageFlag2 == false && MoulageFlag3 == false)
		{
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_yes.png);}");
			MoulageFlag1 = true;
			unMoulageFlag = false;
		}
		else if (MoulageFlag1 == true && MoulageFlag2 == false && MoulageFlag3 == false)
		{
			MoulageButton1->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage1_no.png);}");
			MoulageFlag1 = false;
			unMoulageFlag = true;

		}
	}

}

void TabMainGUI::MoulagePress2()
{
	if (doMoulageFlag == true)
	{
		if (MoulageFlag2 == false && MoulageFlag1 == false && MoulageFlag3 == false)
		{
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_yes.png);}");
			MoulageFlag2 = true;
			unMoulageFlag = false;
		}
		else if (MoulageFlag2 == true && MoulageFlag1 == false && MoulageFlag3 == false)
		{
			MoulageButton2->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage2_no.png);}");
			MoulageFlag2 = false;
			unMoulageFlag = true;
		}
	}
}

void TabMainGUI::MoulagePress3()
{
	if (doMoulageFlag == true)
	{
		if (MoulageFlag3 == false && MoulageFlag1 == false && MoulageFlag2 == false)
		{
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_yes.png);}");
			MoulageFlag3 = true;
			unMoulageFlag = false;
		}
		else if (MoulageFlag3 == true && MoulageFlag1 == false && MoulageFlag2 == false)
		{
			MoulageButton3->setStyleSheet("QPushButton{background-image: url(:/MainWidget/Resources/images/Moulage3_no.png);}");
			MoulageFlag3 = false;
			unMoulageFlag = true;
		}
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
			if (totalCrownList.size() != 0)
			{
				foreach(QPushButton *chooseButton, totalCrownList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "1.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 2:
		{
			if (toothCrownList.size() != 0)
			{
				foreach(QPushButton *chooseButton, toothCrownList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "2.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 3:
		{
			if (lossToothList.size() != 0)
			{
				foreach(QPushButton *chooseButton, lossToothList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "3.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 4:
		{
			if (inlayList.size() != 0)
			{
				foreach(QPushButton *chooseButton, inlayList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "4.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 5:
		{
			if (facingList.size() != 0)
			{
				foreach(QPushButton *chooseButton, facingList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "5.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 6:
		{
			if (waxTypeList.size() != 0)
			{
				foreach(QPushButton *chooseButton, waxTypeList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "6.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 7:
		{
			if (implantList.size() != 0)
			{
				foreach(QPushButton *chooseButton, implantList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "7.png);}";
					chooseButton->setStyleSheet(path);
				}
			}
			break;
		}
		case 8:
		{
			if (jawToothList.size() != 0)
			{
				foreach(QPushButton *chooseButton, jawToothList)
				{
					QString path = "QLabel{background-image: url(:/MainWidget/Resources/images/" + chooseButton->objectName() + "8.png);}";
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
		if (tooth11Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth11" + QString::number(chooseID, 10) + ".png);}";
			tooth11->setStyleSheet(path);
			toothPushButtonList.append(tooth11);
			tooth11Flag = true;
		}
		else if (tooth11Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth110.png);}";
			tooth11->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth11);
			tooth11Flag = false;
		}
	}
}

void TabMainGUI::Tooth12Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth12Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth12" + QString::number(chooseID, 10) + ".png);}";
			tooth12->setStyleSheet(path);
			toothPushButtonList.append(tooth12);
			tooth12Flag = true;
		}
		else if (tooth12Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth120.png);}";
			tooth12->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth12);
			tooth12Flag = false;
		}
	}
}

void TabMainGUI::Tooth13Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth13Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth13" + QString::number(chooseID, 10) + ".png);}";
			tooth13->setStyleSheet(path);
			toothPushButtonList.append(tooth13);
			tooth13Flag = true;
		}
		else if (tooth13Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth130.png);}";
			tooth13->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth13);
			tooth13Flag = false;
		}
	}
}

void TabMainGUI::Tooth14Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth14Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth14" + QString::number(chooseID, 10) + ".png);}";
			tooth14->setStyleSheet(path);
			toothPushButtonList.append(tooth14);
			tooth14Flag = true;
		}
		else if (tooth14Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth140.png);}";
			tooth14->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth14);
			tooth14Flag = false;
		}
	}
}

void TabMainGUI::Tooth15Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth15Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth15" + QString::number(chooseID, 10) + ".png);}";
			tooth15->setStyleSheet(path);
			toothPushButtonList.append(tooth15);
			tooth15Flag = true;
		}
		else if (tooth15Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth150.png);}";
			tooth15->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth15);
			tooth15Flag = false;
		}
	}
}

void TabMainGUI::Tooth16Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth16Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth16" + QString::number(chooseID, 10) + ".png);}";
			tooth16->setStyleSheet(path);
			toothPushButtonList.append(tooth16);
			tooth16Flag = true;
		}
		else if (tooth16Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth160.png);}";
			tooth16->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth16);
			tooth16Flag = false;
		}
	}
}

void TabMainGUI::Tooth17Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth17Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth17" + QString::number(chooseID, 10) + ".png);}";
			tooth17->setStyleSheet(path);
			toothPushButtonList.append(tooth17);
			tooth17Flag = true;
		}
		else if (tooth17Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth170.png);}";
			tooth17->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth17);
			tooth17Flag = false;
		}
	}
}

void TabMainGUI::Tooth18Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth18Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth18" + QString::number(chooseID, 10) + ".png);}";
			tooth18->setStyleSheet(path);
			toothPushButtonList.append(tooth18);
			tooth18Flag = true;
		}
		else if (tooth18Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth180.png);}";
			tooth18->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth18);
			tooth18Flag = false;
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
		if (tooth21Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth21" + QString::number(chooseID, 10) + ".png);}";
			tooth21->setStyleSheet(path);
			toothPushButtonList.append(tooth21);
			tooth21Flag = true;
		}
		else if (tooth21Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth210.png);}";
			tooth21->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth21);
			tooth21Flag = false;
		}
	}
}

void TabMainGUI::Tooth22Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth22Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth22" + QString::number(chooseID, 10) + ".png);}";
			tooth22->setStyleSheet(path);
			toothPushButtonList.append(tooth22);
			tooth22Flag = true;
		}
		else if (tooth22Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth220.png);}";
			tooth22->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth22);
			tooth22Flag = false;
		}
	}
}

void TabMainGUI::Tooth23Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth23Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth23" + QString::number(chooseID, 10) + ".png);}";
			tooth23->setStyleSheet(path);
			toothPushButtonList.append(tooth23);
			tooth23Flag = true;
		}
		else if (tooth23Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth230.png);}";
			tooth23->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth23);
			tooth23Flag = false;
		}
	}
}

void TabMainGUI::Tooth24Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth24Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth24" + QString::number(chooseID, 10) + ".png);}";
			tooth24->setStyleSheet(path);
			toothPushButtonList.append(tooth24);
			tooth24Flag = true;
		}
		else if (tooth24Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth240.png);}";
			tooth24->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth24);
			tooth24Flag = false;
		}
	}
}

void TabMainGUI::Tooth25Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth25Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth25" + QString::number(chooseID, 10) + ".png);}";
			tooth25->setStyleSheet(path);
			toothPushButtonList.append(tooth25);
			tooth25Flag = true;
		}
		else if (tooth25Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth250.png);}";
			tooth25->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth25);
			tooth25Flag = false;
		}
	}
}

void TabMainGUI::Tooth26Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth26Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth26" + QString::number(chooseID, 10) + ".png);}";
			tooth26->setStyleSheet(path);
			toothPushButtonList.append(tooth26);
			tooth26Flag = true;
		}
		else if (tooth26Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth260.png);}";
			tooth26->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth26);
			tooth26Flag = false;
		}
	}
}

void TabMainGUI::Tooth27Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth27Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth27" + QString::number(chooseID, 10) + ".png);}";
			tooth27->setStyleSheet(path);
			toothPushButtonList.append(tooth27);
			tooth27Flag = true;
		}
		else if (tooth27Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth270.png);}";
			tooth27->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth27);
			tooth27Flag = false;
		}
	}
}

void TabMainGUI::Tooth28Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth28Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth28" + QString::number(chooseID, 10) + ".png);}";
			tooth28->setStyleSheet(path);
			toothPushButtonList.append(tooth28);
			tooth28Flag = true;
		}
		else if (tooth28Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth280.png);}";
			tooth28->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth28);
			tooth28Flag = false;
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
		if (tooth31Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth31" + QString::number(chooseID, 10) + ".png);}";
			tooth31->setStyleSheet(path);
			toothPushButtonList.append(tooth31);
			tooth31Flag = true;
		}
		else if (tooth31Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth310.png);}";
			tooth31->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth31);
			tooth31Flag = false;
		}
	}
}

void TabMainGUI::Tooth32Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth32Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth32" + QString::number(chooseID, 10) + ".png);}";
			tooth32->setStyleSheet(path);
			toothPushButtonList.append(tooth32);
			tooth32Flag = true;
		}
		else if (tooth32Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth320.png);}";
			tooth32->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth32);
			tooth32Flag = false;
		}
	}
}

void TabMainGUI::Tooth33Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth33Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth33" + QString::number(chooseID, 10) + ".png);}";
			tooth33->setStyleSheet(path);
			toothPushButtonList.append(tooth33);
			tooth33Flag = true;
		}
		else if (tooth33Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth330.png);}";
			tooth33->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth33);
			tooth33Flag = false;
		}
	}
}

void TabMainGUI::Tooth34Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth34Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth34" + QString::number(chooseID, 10) + ".png);}";
			tooth34->setStyleSheet(path);
			toothPushButtonList.append(tooth34);
			tooth34Flag = true;
		}
		else if (tooth34Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth340.png);}";
			tooth34->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth34);
			tooth34Flag = false;
		}
	}
}

void TabMainGUI::Tooth35Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth35Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth35" + QString::number(chooseID, 10) + ".png);}";
			tooth35->setStyleSheet(path);
			toothPushButtonList.append(tooth35);
			tooth35Flag = true;
		}
		else if (tooth35Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth350.png);}";
			tooth35->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth35);
			tooth35Flag = false;
		}
	}
}

void TabMainGUI::Tooth36Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth36Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth36" + QString::number(chooseID, 10) + ".png);}";
			tooth36->setStyleSheet(path);
			toothPushButtonList.append(tooth36);
			tooth36Flag = true;
		}
		else if (tooth36Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth360.png);}";
			tooth36->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth36);
			tooth36Flag = false;
		}
	}
}

void TabMainGUI::Tooth37Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth37Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth37" + QString::number(chooseID, 10) + ".png);}";
			tooth37->setStyleSheet(path);
			toothPushButtonList.append(tooth37);
			tooth37Flag = true;
		}
		else if (tooth37Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth370.png);}";
			tooth37->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth37);
			tooth37Flag = false;
		}
	}
}

void TabMainGUI::Tooth38Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth38Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth38" + QString::number(chooseID, 10) + ".png);}";
			tooth38->setStyleSheet(path);
			toothPushButtonList.append(tooth38);
			tooth38Flag = true;
		}
		else if (tooth38Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth380.png);}";
			tooth38->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth38);
			tooth38Flag = false;
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
		if (tooth41Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth41" + QString::number(chooseID, 10) + ".png);}";
			tooth41->setStyleSheet(path);
			toothPushButtonList.append(tooth41);
			tooth41Flag = true;
		}
		else if (tooth41Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth410.png);}";
			tooth41->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth41);
			tooth41Flag = false;
		}
	}
}

void TabMainGUI::Tooth42Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth42Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth42" + QString::number(chooseID, 10) + ".png);}";
			tooth42->setStyleSheet(path);
			toothPushButtonList.append(tooth42);
			tooth42Flag = true;
		}
		else if (tooth42Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth420.png);}";
			tooth42->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth42);
			tooth42Flag = false;
		}
	}
}

void TabMainGUI::Tooth43Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth43Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth43" + QString::number(chooseID, 10) + ".png);}";
			tooth43->setStyleSheet(path);
			toothPushButtonList.append(tooth43);
			tooth43Flag = true;
		}
		else if (tooth43Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth430.png);}";
			tooth43->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth43);
			tooth43Flag = false;
		}
	}
}

void TabMainGUI::Tooth44Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth44Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth44" + QString::number(chooseID, 10) + ".png);}";
			tooth44->setStyleSheet(path);
			toothPushButtonList.append(tooth44);
			tooth44Flag = true;
		}
		else if (tooth44Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth440.png);}";
			tooth44->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth44);
			tooth44Flag = false;
		}
	}
}

void TabMainGUI::Tooth45Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth45Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth45" + QString::number(chooseID, 10) + ".png);}";
			tooth45->setStyleSheet(path);
			toothPushButtonList.append(tooth45);
			tooth45Flag = true;
		}
		else if (tooth45Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth450.png);}";
			tooth45->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth45);
			tooth45Flag = false;
		}
	}
}

void TabMainGUI::Tooth46Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth46Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth46" + QString::number(chooseID, 10) + ".png);}";
			tooth46->setStyleSheet(path);
			toothPushButtonList.append(tooth46);
			tooth46Flag = true;
		}
		else if (tooth46Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth460.png);}";
			tooth46->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth46);
			tooth46Flag = false;
		}
	}
}

void TabMainGUI::Tooth47Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth47Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth47" + QString::number(chooseID, 10) + ".png);}";
			tooth47->setStyleSheet(path);
			toothPushButtonList.append(tooth47);
			tooth47Flag = true;
		}
		else if (tooth47Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth470.png);}";
			tooth47->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth47);
			tooth47Flag = false;
		}
	}
}

void TabMainGUI::Tooth48Press()
{
	if (chooseID != -1)
	{
		QList<QPushButton *> toothPushButtonList;
		judgeToothList(chooseID, toothPushButtonList);
		if (tooth48Flag == false)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth48" + QString::number(chooseID, 10) + ".png);}";
			tooth48->setStyleSheet(path);
			toothPushButtonList.append(tooth48);
			tooth48Flag = true;
		}
		else if (tooth48Flag == true)
		{
			QString path = "QPushButton{background-image: url(:/MainWidget/Resources/images/tooth480.png);}";
			tooth48->setStyleSheet(path);
			toothPushButtonList.removeOne(tooth48);
			tooth48Flag = false;
		}
	}
}