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

	vector<shared_ptr<BaseModel>> upper_ModelsVt;
	vector<shared_ptr<BaseModel>> lower_ModelsVt;
	vector<shared_ptr<BaseModel>> all_ModelsVt;

	pCTeethModel upperTeethModel;
	pCTeethModel lowerTeethModel;
	pCTeethModel allTeethModel;

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

	int upperJawIndex = 1;

	int lowerJawIndex = 1;

	int allJawIndex = 1;
	
	int chooseJawIndex = 0;//1代表是只扫上颌；2代表只扫下颌；3代表扫全颌；

	bool compensationFlag = false;

	float ax = 0, ay = 0;

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
	//Mat2QImage
	QImage Mat2QImage(const cv::Mat &InputMat);
	QString filePath;//保存文件路径
	QString patientNameQStr;//患者姓名

	//水平切割数量
	int upperCutModelNum = 0;
	int lowerCutModelNum = 0;
	int allCutModelNum = 0;
	//删除所有切割数据
	void deleteAllCutModel();
	//增加补扫数量
	int addCompensationNum = 0;
	
signals:
	void startControlNormalScan(int chooseJawIndex);//控制正常扫描信号signals
	void compensationSignal(int chooseJawIndex);//补充扫描信号
	void doScanSignal();
	void cutSurfaceSignal(bool bShow);

	void gpaMeshSignal(int chooseJawIndex);
	void saveCutModelSignal();//保存切割后的模型

	void updateMeshModelSingel(int refreshIndex);//刷新模型显示信号
	void updateModelsVtSingle();
	void saveModeltoFileSignal();
private:
	Ui::ScanMainGUI ui;

	GLWidget *glWidget;
	//顶部模型操作栏
	QToolButton *leftWatchButton;
	QToolButton *rightWatchButton;
	QToolButton *topWatchButton;
	QToolButton *bottomWatchButton;
	QToolButton *frontWatchButton;
	QToolButton *backWatchButton;
	QToolButton *enlargeButton;
	QToolButton *shrinkButton;
	//底部模型操作栏
	bool bSelected = false;
	QToolButton *selectRegionButton;
	QToolButton *deleteModelButton;

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
	//展示模型
	void updateMeshModel(int refreshIndex);
	void updateModelsVtSlot();
	void updateCamera();

	void doScanDialogSlot(QJsonObject scanObj);

	//扫描
	void judgeForwardStep();
	void JawScan();
	//补扫
	void compensationScan();
	//撤销补扫
	void discardCompensationSlot();
	void judgeBackStep();

	//操作模型
	void topModelWatchSlot();
	void bottomModelWatchSlot();
	void leftModelWatchSlot();
	void rightModelWatchSlot();
	void frontModelWatchSlot();
	void backModelWatchSlot();
	void enlargeModelSlot();
	void shrinkModelSlot();

	void selectRegionSlot();
	void deleteSelectedRegionSlot();

	//cutSurface
	void showCutSurfaceSlot(bool bShow);//切割模型

	void moveCutSurfaceSpinSlot();//调整切割水平面
	void moveCutSurfaceSliderSlot();
	void cutModelSlot();//切割模型
	void setRotationWaverSlot();//显示旋转和摆动角度
	void saveCutModelSlot();//保存切割后模型
	void discardCutModelSlot();//撤销切割模型

	//保存所有模型到文件
	void saveModeltoFileSlot();
	public slots:
	void fontviewBtnClicked();
	void pushBtnClicked();
	void rightViewBtnClicked();
	void leftViewBtnClicked();
	void backViewBtnClicked();
	void jawViewBtnClicked();
	void narrowViewBtnClicked();
	void enlargeViewBtnClicked();
	void modelMoveStateSetBtnClicked();
	void delSelectedBtnClicked();
	void confirmSelectedBtnClicked();
	void bgGroundmoveDownBtnClicked();
	void bgGroundmoveUpBtnClicked();
	void bgGroundShowBtnClicked();
	void cutModelUnderBgBtnClicked();
	void changeBgColorBtnClicked();
};

#endif