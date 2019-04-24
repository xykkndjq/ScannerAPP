#ifndef TABMAINGUI_H
#define TABMAINGUI_H

#include <QWidget>
#include "ui_TabMainGUI.h"
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFormLayout>
#include <QLineEdit>
#include <QTextEdit>
#include <QLabel>
#include <QDateTimeEdit>
#include <QDateTime>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <QButtonGroup>
#include <QCheckBox>
#include <QRadioButton>
#include <QGroupBox>
#include <QDebug>
#include <qdir.h>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QMessageBox>
#include <iostream>
#include <QFileDialog>
#include <QDockWidget>
#include "TaskManager.h"
#include "SplitModelBtn.h"

#define TOOTHNUM 32

class TabMainGUI : public QWidget
{
	Q_OBJECT

public:
	TabMainGUI(QWidget *parent = Q_NULLPTR);
	~TabMainGUI();

	void initVariable();
	void constructIHM();
	void setConnections();

	QList<QPushButton*> judgeToothList(int id);
	void setSplitToothFalse();

	//判断文件夹是否存在并创建文件夹
	bool isDirExist(QString fullPath);
	void saveSplitModel(int id, cv::FileStorage fWrite);

	void addToothList(int id, int toothButtonIndex);
	void removeToothList(int id, int toothButtonIndex);

	void judgeSplitModelFlag();
	 
	//判断是否满足保存订单信息条件
	bool judgePatientSaveFlag();

	//转化为中文字符串
	QByteArray ToChineseStr(const QString &chineseString);

	QString lastDateTimeStr;//最新订单时间字符串
	QString fileQStr;//保存文件路径
	QString filePath = "./ScanData/";//文件目录路径

	//读取订单信息文件
	void readFileStorage(QString fPath);

public:
	Ui::TabMainGUI ui;
	
	QGridLayout *totalGLayOut;
	QTabWidget *totalTabWidget;

	QWidget *orderInforWidget;
	QWidget *settingWidget;
	QWidget *calibrateWidget;
	QWidget *aboutWidget;

	QVBoxLayout *totalOrderVLayout;
	QGridLayout *totalAboutGLayout;
	//orderInfor订单管理控件
	//topbutton
	QPushButton *newButton;
	QPushButton *exportButton;
	//QPushButton *saveButton;
	QPushButton *watchButton;
	QPushButton *saveScanButton;
	
	QDateTimeEdit *dateLineEdit;
	QLineEdit *orderLineEdit;
	QLineEdit *patientLineEdit;
	QLineEdit *doctorLineEdit;
	QTextEdit *additionTextEdit;
	QLabel *additionLabel;
	//未分模
	bool unMoulageFlag = false;
	QPushButton *upperJawButton;
	bool upperJawButtonFlag = false;
	QPushButton *lowerJawButton;
	bool lowerJawButtonFlag = false;
	
	//分模
	//splitleft
	bool splitModelFlag = false;
	QList<bool> toothFlagList;
	int chooseID = -1;
	QList<QPushButton *> toothList;
	
	
	QList<int> toothID;
	QPushButton *clearAllButton;
	//splitright
	QButtonGroup *toothRadioButtonGroup;
	
	QCheckBox *totalCrownButton;//全冠
	QCheckBox*toothCrownButton;//牙冠
	QCheckBox *lossToothButton;//缺失牙
	QCheckBox *inlayButton;//嵌体
	QCheckBox *facingButton;//贴面
	QCheckBox *waxTypeButton;//蜡型
	QCheckBox *implantButton;//种植体
	QCheckBox *jawToothButton;//对颌牙

	QList<QPushButton*> totalCrownList;
	QList<QPushButton*> toothCrownList;
	QList<QPushButton*> lossToothList;
	QList<QPushButton*> inlayList;
	QList<QPushButton*> facingList;
	QList<QPushButton*> waxTypeList;
	QList<QPushButton*> implantList;
	QList<QPushButton*> jawToothList;

	//印模
	bool doMoulageFlag = false;
	QPushButton *MoulageButton1;
	bool MoulageFlag1 = false;
	QPushButton *MoulageButton2;
	bool MoulageFlag2 = false;
	QPushButton *MoulageButton3;
	bool MoulageFlag3 = false;
	//orderInfor订单管理信息
	QString orderDate;
	QString orderNumber;
	QString orderPatientName;
	QString orderDoctorName;
	QString orderAddition;
	//QString upperjaw;
	//About关于页面控件
	QLabel *aboutTextLabel;
	QLabel *aboutImageLabel;

	//Calibrate标定页面
	QPushButton *calibratePushButton;
	QPushButton *globalCaliPushButton;
	QLabel *leftCameraLable;
	QLabel *rightCameraLable;

	//设置子页面
	QButtonGroup *settingButtonGroup;
	QCheckBox *textureCheckBox;
	QCheckBox *ACheckBox;
	QCheckBox *BCheckBox;
	QLabel *scanDataPathLabel;
	QLineEdit *scanDataPathEdit;
	QPushButton *choosePathButton;
	pCScanTask m_pTeethScanTaskArray[32];
	eScanType m_eScanType;

signals:
	void scanDataSignal(QJsonObject scanObj);
	void scanSignal();
	//导入上颌
	void exportUpperJawSignal();
	//导入上颌
	void exportLowerJawSignal();
	//导入印模1
	void exportFirstMoulageSignal();
	//导入印模2
	void exportSecondMoulageSignal();
	//导入印模3
	void exportThirdMoulageSignal();

public slots:
	//订单管理信息槽
	void PatientInformationSave();

	//打开文件
	void openFileDialogSlot();

	//打开路径
	void openDirectoryDialogSlot();

    //未分模
	void UpperJawPress();
	void LowerJawPress();
	//印模
	void MoulagePress1();
	void MoulagePress2();
	void MoulagePress3();
	//分模
	void ToothGroupClicked(int id);

	void ToothButtonListPress();
	
	void clearAllButtonPress();

	void ScanDataPackagePress();

	//
	void settingButtonClicked(QAbstractButton *button);
};
#endif