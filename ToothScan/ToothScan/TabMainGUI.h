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

class TabMainGUI : public QWidget
{
	Q_OBJECT

public:
	TabMainGUI(QWidget *parent = Q_NULLPTR);
	~TabMainGUI();

	void initVariable();
	void constructIHM();
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
	QPushButton *upperJawButton;
	QPushButton *lowerJawButton;
	//分模

	//印模
	QPushButton *MoulageButton;

	//orderInfor订单管理信息
	QString orderDate;
	QString orderNumber;
	QString orderPatientName;
	QString orderDoctorName;
	QString orderAddition;

	//About关于页面控件
	QLabel *aboutTextLabel;
	QLabel *aboutImageLabel;

public slots:
	//订单管理信息槽
void PatientInformationSave();
};
