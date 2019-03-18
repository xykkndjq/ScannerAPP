#ifndef SCANMAINGUI_H
#define SCANMAINGUI_H

#include <QWidget>
#include "ui_ScanMainGUI.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QToolButton>
#include <QDockWidget>
#include <QSpinBox>
#include <QSlider>
#include <QTextEdit>
#include "glwidget.h"
#include "ControlThread.h"
#include "ComputeThread.h"
#include "TabMainGUI.h"
#include "ScanTipWidget.h"

class ScanMainGUI : public QWidget
{
	Q_OBJECT

public:
	ScanMainGUI(QWidget *parent = Q_NULLPTR);
	~ScanMainGUI();

	int globalTipIndex = 0;
	/*globalTipStep解释
	globalTipIndex = 1；"上颌"
	globalTipIndex = 2；"下颌"
	globalTipIndex = 3；"全颌"*/

	int forwardIndex = 0;
	/*forwardSteButton解释
	forwardIndex = 1；"扫描"
	forwardIndex = 2；"下一步"
	forwardIndex = 3；"完成"*/

	int allJawIndex = 1;
	
	int chooseJawIndex = 0;//1代表是只扫上颌；2代表只扫下颌；3代表扫全颌；

	bool compensationFlag = false;

	TabMainGUI *tabMainPage;
	ControlThread *ControlScanThread;
	ComputeThread *ControlComputeThread;
	QThread *controlScanQThread, *controlComputeQThread;

	ScanTipWidget *scanTipWidget;
	
	void initVariable();
	void constructIHM();
	void setConnections();
	//扫描模型
	void CalculatePointCloud();

signals:
	void startControlNormalScan(int chooseJawIndex);//控制正常扫描信号signals
	void compensationSignal(int chooseJawIndex);//补充扫描信号
	void doScanSignal();
	

private:
	Ui::ScanMainGUI ui;

	GLWidget *glWidget;

	QToolButton *leftWatchButton;
	QToolButton *rightWatchButton;
	QToolButton *topWatchButton;
	QToolButton *bottomWatchButton;
	QToolButton *frontWatchButton;
	QToolButton *backWatchButton;
	//相机显示窗口
	QDockWidget *cameraWindow;
	QPushButton *autoTunePushButton;
	QLabel *cameraImageLabel;
	QSpinBox *spinCameraBox;
	QSlider *sliderCamera;

private slots:
	
	//标定
	void ToothCalibrateSlot();
	void GlobalCalibrateSlot();

	void updateMeshModel(int refreshIndex, int scanIndex);
	void doScanDialogSlot(QJsonObject scanObj);

	//扫描
	void judgeForwardStep();
	void JawScan();
	void compensationScan();
};

#endif