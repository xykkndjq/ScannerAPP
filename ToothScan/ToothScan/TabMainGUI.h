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
	
	QRadioButton *totalCrownButton;//全冠
	QRadioButton *toothCrownButton;//牙冠
	QRadioButton *lossToothButton;//缺失牙
	QRadioButton *inlayButton;//嵌体
	QRadioButton *facingButton;//贴面
	QRadioButton *waxTypeButton;//蜡型
	QRadioButton *implantButton;//种植体
	QRadioButton *jawToothButton;//对颌牙

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

signals:
	void scanDataSignal(QJsonObject scanObj);
	void scanSignal();

public slots:
	//订单管理信息槽
	void PatientInformationSave();
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
};
#endif