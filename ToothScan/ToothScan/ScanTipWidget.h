#ifndef SCANTIPWIDGET_H
#define SCANTIPWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include "ui_ScanTipWidget.h"

class ScanTipWidget : public QWidget
{
	Q_OBJECT

public:
	ScanTipWidget(QWidget *parent = Q_NULLPTR);
	~ScanTipWidget();
	
	//top
	QLabel *primaryTipLabel;
	QLabel *secondTipLabel;
	
	//middle
	//插入
	QLabel *imageTipLabel;
	//补扫
	QPushButton *compensationButton;//增加补扫
	QPushButton *discardButton;//丢弃补扫
	QLabel *rotationLabel;//旋转角度
	QLineEdit *rotationLineEdit;//旋转角度
	QLabel *waverLabel;//摆动角度
	QLineEdit *waverLineEdit;//摆动角度
	QLabel *cutHeightLabel;//切割高度
	QLineEdit *cutHeightLineEdit;//切割高度
	QSpinBox *spinCutBox;//切割控制
	QSlider *sliderCut;
	QPushButton *cutHeightButton;//切割高度应用
	QPushButton *discardCutHeightButton;//丢弃切割高度应用
	QPushButton *saveCutHeightButton;//保存当前切割高度

	QPushButton *upperShowButton;//完成
	QPushButton *lowerShowButton;

	//bottom
	QPushButton *backStepButton;
	QPushButton *forwardStepButton;

	//alljaw
	void InitVariable();//变量初始化

	void placeVariable1();//插入
	void compenVariable2();//补扫
	void finishVariable3();//完成
	void splitScanVariable4();//完成

	void placeConstruct1();//插入
	void compenConstruct2();//补扫
	void finishConstruct3();//完成

	void allPlaceConstructIHM1();//插入全颌
	
	void allCompenConstructIHM2();//补扫全颌

	void allPlaceConstructIHM3();//插入下颌
	
	void allCompenConstructIHM4();//补扫下颌

	void allPlaceConstructIHM5();//插入上颌

	void allCompenConstructIHM6();//补扫上颌

	void allFinishConstructIHM7();//完成

	void upperPlaceConstructIHM1();

	void upperCompenConstructIHM2();

	void lowerPlaceConstructIHM1();

	void lowerCompenConstructIHM2();

	//split分模 
	//代型
	QList<QLabel *> toothList;

	void splitScanConstruct1();

	void upperSplitScanConstructIHM5();
	void upperSplitRemoveConstructIHM6();
	void upperSplitFinishConstructIHM7();
	void lowerSplitScanConstructIHM5();
	void lowerSplitRemoveConstructIHM6();
	void lowerSplitFinishConstructIHM7();
	void allSplitFinishConstructIHM8();
private:
	Ui::ScanTipWidget ui;
};
#endif