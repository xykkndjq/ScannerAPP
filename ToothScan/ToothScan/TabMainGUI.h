#pragma once

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
class TabMainGUI : public QWidget
{
	Q_OBJECT

public:
	TabMainGUI(QWidget *parent = Q_NULLPTR);
	~TabMainGUI();

	void initVariable();
	void constructIHM();
	void setConnections();

	void judgeToothList(int id, QList<QLabel*> &toothQLabelList);
private:
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
	QPushButton *saveButton;
	QPushButton *watchButton;
	QPushButton *scanButton;
	//PatientInformation *patientInformation;
	QDateTimeEdit *dateLineEdit;
	QLineEdit *orderLineEdit;
	QLineEdit *patientLineEdit;
	QLineEdit *doctorLineEdit;
	QTextEdit *additionTextEdit;
	QLabel *additionLabel;
	//未分模
	bool unMoulageFlag = true;
	QPushButton *upperJawButton;
	bool upperJawButtonFlag = false;
	QPushButton *lowerJawButton;
	bool lowerJawButtonFlag = false;
	//分模
	//splitleft
	int chooseID = -1;
	QLabel *tooth11;
	QLabel *tooth12;
	QLabel *tooth13;
	QLabel *tooth14;
	QLabel *tooth15;
	QLabel *tooth16;
	QLabel *tooth17;
	QLabel *tooth18;

	bool tooth11Flag = false;
	bool tooth12Flag = false;
	bool tooth13Flag = false;
	bool tooth14Flag = false;
	bool tooth15Flag = false;
	bool tooth16Flag = false;
	bool tooth17Flag = false;
	bool tooth18Flag = false;

	QLabel *tooth21;
	QLabel *tooth22;
	QLabel *tooth23;
	QLabel *tooth24;
	QLabel *tooth25;
	QLabel *tooth26;
	QLabel *tooth27;
	QLabel *tooth28;

	bool tooth21Flag = false;
	bool tooth22Flag = false;
	bool tooth23Flag = false;
	bool tooth24Flag = false;
	bool tooth25Flag = false;
	bool tooth26Flag = false;
	bool tooth27Flag = false;
	bool tooth28Flag = false;

	QLabel *tooth31;
	QLabel *tooth32;
	QLabel *tooth33;
	QLabel *tooth34;
	QLabel *tooth35;
	QLabel *tooth36;
	QLabel *tooth37;
	QLabel *tooth38;

	bool tooth31Flag = false;
	bool tooth32Flag = false;
	bool tooth33Flag = false;
	bool tooth34Flag = false;
	bool tooth35Flag = false;
	bool tooth36Flag = false;
	bool tooth37Flag = false;
	bool tooth38Flag = false;

	QLabel *tooth41;
	QLabel *tooth42;
	QLabel *tooth43;
	QLabel *tooth44;
	QLabel *tooth45;
	QLabel *tooth46;
	QLabel *tooth47;
	QLabel *tooth48;

	bool tooth41Flag = false;
	bool tooth42Flag = false;
	bool tooth43Flag = false;
	bool tooth44Flag = false;
	bool tooth45Flag = false;
	bool tooth46Flag = false;
	bool tooth47Flag = false;
	bool tooth48Flag = false;

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

	QList<QLabel*> totalCrownList;
	QList<QLabel*> toothCrownList;
	QList<QLabel*> lossToothList;
	QList<QLabel*> inlayList;
	QList<QLabel*> facingList;
	QList<QLabel*> waxTypeList;
	QList<QLabel*> implantList;
	QList<QLabel*> jawToothList;

	//印模
	bool doMoulageFlag = true;
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

	void Tooth11Press();
	void Tooth12Press();
	void Tooth13Press();
	void Tooth14Press();
	void Tooth15Press();
	void Tooth16Press();
	void Tooth17Press();
	void Tooth18Press();

	void Tooth21Press();
	void Tooth22Press();
	void Tooth23Press();
	void Tooth24Press();
	void Tooth25Press();
	void Tooth26Press();
	void Tooth27Press();
	void Tooth28Press();

	void Tooth31Press();
	void Tooth32Press();
	void Tooth33Press();
	void Tooth34Press();
	void Tooth35Press();
	void Tooth36Press();
	void Tooth37Press();
	void Tooth38Press();

	void Tooth41Press();
	void Tooth42Press();
	void Tooth43Press();
	void Tooth44Press();
	void Tooth45Press();
	void Tooth46Press();
	void Tooth47Press();
	void Tooth48Press();
};
