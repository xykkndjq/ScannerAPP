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
}

TabMainGUI::~TabMainGUI()
{
}

void TabMainGUI::PatientInformationSave()
{
	std:: cout << "storage a order information..." << std::endl;
	orderDate = dateLineEdit->text();
	orderNumber = orderLineEdit->text();
	orderPatientName = patientLineEdit->text();
	orderDoctorName = doctorLineEdit->text();
	orderAddition = additionTextEdit;
	
	std::string orderPatientNameString = orderPatientName.toStdString();
	cv::FileStorage fs("./data/" + pn + ".patient_name", cv::FileStorage::WRITE);
}

void TabMainGUI::initVariable()
{
	totalGLayOut = new QGridLayout(this);

	totalTabWidget = new QTabWidget();
	
	orderInforWidget = new QWidget();
	settingWidget = new QWidget();
	calibrateWidget = new QWidget();
	aboutWidget = new QWidget();

	//topbutton
	newButton = new QPushButton(QStringLiteral("新建"));
	exportButton = new QPushButton(QStringLiteral("导入"));
	saveButton = new QPushButton(QStringLiteral("保存"));
	watchButton = new QPushButton(QStringLiteral("预览"));
	scanButton = new QPushButton(QStringLiteral("扫描"));
	//leftbutton
	dateLineEdit = new QDateTimeEdit();
	dateLineEdit->setDateTime(QDateTime::currentDateTime());
	orderLineEdit = new QLineEdit();
	patientLineEdit = new QLineEdit();
	doctorLineEdit = new QLineEdit();
	additionTextEdit = new QTextEdit();
	additionLabel = new QLabel(QStringLiteral("备注："));
	//未分模
	upperJawButton = new QPushButton();
	lowerJawButton = new QPushButton();
	//印模
	MoulageButton = new QPushButton();
	
	//关于页面
	aboutTextLabel = new QLabel();
	aboutImageLabel = new QLabel();
}

void TabMainGUI::constructIHM()
{
	totalTabWidget->setTabPosition(QTabWidget::West);
	totalTabWidget->setStyle(new CustomTabStyle);
	totalTabWidget->setStyleSheet("QTabWidget::pane{ \border-left: 1px solid #eeeeee;\}");
	
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
	QWidget *rightTotalModelWidget = new QWidget();
	QVBoxLayout *rightTotalModelLayout = new QVBoxLayout(rightTotalModelWidget);
	rightTotalModelLayout->addWidget(upperJawButton);
	rightTotalModelLayout->addWidget(lowerJawButton);
	//分模
	/*QWidget *rightSplitModelWidget = new QWidget();
	QVBoxLayout *rightSplitModelLayout = new QVBoxLayout(rightSplitModelWidget);
	rightSplitModelLayout->addWidget();*/
	//印模
	QWidget *rightMoulageWidget = new QWidget();
	QVBoxLayout *rightMoulageLayout = new QVBoxLayout(rightMoulageWidget);
	rightMoulageLayout->addWidget(MoulageButton);

	rightTabWidget->addTab(rightTotalModelWidget, QStringLiteral("未分模"));
	rightTabWidget->addTab(rightMoulageWidget, QStringLiteral("印模"));

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

