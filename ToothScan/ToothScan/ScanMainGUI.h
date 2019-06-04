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
//#include "ScanTipWidget.h"

class ScanMainGUI : public QWidget, public QAbstractNativeEventFilter,public IGlWidget
{
	Q_OBJECT

public:
	ScanMainGUI(QWidget *parent = Q_NULLPTR);
	~ScanMainGUI();

	QTimer *timer;

	int globalSpinCutValue = 0;

	bool	bPnP_Arrival = false;
	bool	bPnP_Removal = false;
	bool	bPnP_DevNodeChange = false;
	bool    m_usbDeviceState = false;//判断开机是否连接了USB

	virtual bool nativeEventFilter(const QByteArray &eventType, void *message, long *) Q_DECL_OVERRIDE;

	void initUSBDevice();
	void installNativeEventFilter();

	vector<shared_ptr<BaseModel>> upper_ModelsVt;
	vector<shared_ptr<BaseModel>> lower_ModelsVt;
	vector<shared_ptr<BaseModel>> all_ModelsVt;

	pCTeethModel upperTeethModel;
	pCTeethModel lowerTeethModel;
	pCTeethModel allTeethModel;

	bool m_bShowOrderInfo;
	std::string m_strOrderInfoPath;

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

	//int upperJawIndex = 1;

	//int lowerJawIndex = 1;

	//int allJawIndex = 1;

	//int chooseJawIndex = 0;//1代表是只扫上颌；2代表只扫下颌；3代表扫全颌；

	bool compensationFlag = false;

	float ax = 0, ay = 0;

	TabMainGUI *tabMainPage;
	ControlThread *ControlScanThread;
	ComputeThread *ControlComputeThread;
	QThread *controlScanQThread, *controlComputeQThread;

	//ScanTipWidget *scanTipWidget;

	void initVariable();
	void constructIHM();
	void setConnections();
	//扫描模型
	//void CalculatePointCloud();
	void splitModelCalculatePointCloud(pCScanTask pScanTask);
	//Mat2QImage
	QImage Mat2QImage(const cv::Mat &InputMat);
	QString filePath;//保存文件路径
	QString patientNameQStr;//患者姓名

	//水平切割数量
	int upperCutModelNum = 0;
	int lowerCutModelNum = 0;
	int allCutModelNum = 0;
	//删除所有切割数据
	//void deleteAllCutModel();

	pCTeethModel m_pupperTeethModel;
	pCTeethModel m_plowerTeethModel;
	pCTeethModel m_pallTeethModel;
	pCTeethModel m_pLowerDentalImplantModel;
	pCTeethModel m_pUpperDentalImplantModel;
	pCTeethModel m_pUpperJawGingvaModel;
	pCTeethModel m_pLowJawGingvaModel;

	void resetValue();
	void setDentalImplantNextBtnEnable(bool bEnable) {
		ui.DentalImplantNextBtn->setEnabled(true);
	}
signals:
	//void startControlNormalScan(int chooseJawIndex);//控制正常扫描信号signals
	//void startAllJawNormalScan();
	void startControlCalibrationSignal();//开始标定信号
	//void compensationSignal(int chooseJawIndex);//补充扫描信号
	void doScanSignal();
	void cutSurfaceSignal(bool bShow);

	//void gpaMeshSignal(int chooseJawIndex);
	void farRegistrationSignal();

	//void updateMeshModelSingel(int refreshIndex);//刷新模型显示信号
	void updateModelsVtSingle();
	void saveModeltoFileSignal();

	void startNormalScan();//控制正常扫描信号signals
	void compensationScanSignal();//补充扫描信号
	void gpaTaskMeshSignal();
	void taskSititchingSignal();
	void taskTeethSititSignal();
	void startAllJawScan();//控制正常扫描信号signals
	void showOrderInfoSignal(COrderInfo);
	
	void usbDeviceSignal();//开机判断usbDevice是否成功信号
private:
	Ui::ScanMainGUI ui;

	GLWidget *glWidget;
	//顶部模型操作栏
	//QToolButton *leftWatchButton;
	//QToolButton *rightWatchButton;
	//QToolButton *topWatchButton;
	//QToolButton *bottomWatchButton;
	//QToolButton *frontWatchButton;
	//QToolButton *backWatchButton;
	//QToolButton *enlargeButton;
	//QToolButton *shrinkButton;
	//CTeethImgBtn * m_closeBtn;
	//底部模型操作栏
	bool bSelected = false;
	//QToolButton *selectRegionButton;
	//QToolButton *deleteModelButton;

	//相机显示窗口
	//QDockWidget *cameraWindow;
	//QPushButton *autoTunePushButton;
	//QLabel *cameraImageLabel;
	//QSpinBox *spinCameraBox;
	//QSlider *sliderCamera;
	bool m_bsplitModelFlag;	//分模
	private slots:
	void closeBtnClicked();
	//标定
	void ToothCalibrateSlot();
	//void GlobalCalibrateSlot();
	void calibImageCameraSlot(int endFlag);//更新标定相机照片

	//展示模型
	//void updateMeshModel(int refreshIndex);
	//void updateModelsVtSlot();
	void updateCamera();
	void updateRegMeshSlot();

	//完成后，显示或隐藏模型按钮
	//void ShowHideUpperModel();//显示或者隐藏上颌
	//void ShowHideLowerModel();//显示或者隐藏下颌

	void doScanDialogSlot(QJsonObject scanObj);

	
	//补扫
	//void compensationScan();
	//撤销补扫
	//void discardCompensationSlot();

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
	//void cutModelSlot();//切割模型
	void setRotationWaverSlot();//显示旋转和摆动角度
	//void saveCutModelSlot();//保存切割后模型
	//void discardCutModelSlot();//撤销切割模型

	//保存所有模型到文件
	//void saveModeltoFileSlot();

	void scanJawScanBtnClick();
	void ScanJawBackStepBtnClick();
	void ShowLastScanTask(bool bCurrentTaskSame = false);
	
	void compensationBtnClick();
	void discardBtnClick();		//撤销补扫
	void compensationScanPanelNextBtnClick();
	void compensationScanPanelBackBtnClick();
	void cutModelBtnClick();
	void unDoCutBtnClick();
	void saveCutHeightCutBtnClick();
	void cutPaneNextStepBtnClick();
	void cutPanelBackBtnClick();
	void CutJawFinishPanelNextStepBtnClick();
	void stitchingPanelBtnClick();
	void stitchingBackBtnClick();
	void stitchingUpperJawBtnClick();
	void stitchingUpperJawDenBtnClick();
	void stitchingUpperJawGingivaBtnClick();
	void stitchingLowerJawBtnClick();
	void stitchingLowerJawDenBtnClick();
	void stitchingLowerJawGingivaBtnClick();
	void stitchingFNextBtnClick();
	void OralSubstitutePanelNextBtnClick();
	void oralSubstitutePanelBackBtnClick();
	void teethStitchingPanelBackBtnClick();
	void cutJawFinishPanelBackBtnClick();
	void stitchingFinishPanelBackBtnClick();	
	void teethStitchingPanelNextBtnClick();
	void saveModelFile(pCScanTask pTask);
	void saveModelFile(std::string strPath, pCTeethModel pTask);
	void saveDenModelFile(pCScanTask pTask);//种植体
	bool isModelFileExit(pCScanTask &pTask);
	void loadModelFile(pCScanTask &pTask);
	void hideAllPanel();
	void showScanJawGroup(bool bBack = false);
	void showcompensationScanPanel(bool bBack = false);
	void showCutJawPanel(bool bBack = false);
	void showOralSubstitutePanel(bool bBack = false);
	void showStitchingFinishPanel(bool bBack = false);
	void showSubstitutePanel(bool bBack = false);
	void showDentalImplantPanel(bool bBack = false);
	void movecutHeightSpinBoxSlot();//调整切割水平面
	void movecutHeightSliderSlot();
	void ToothButtonListPress();
	public slots:
	void updateTaskModel();
	void meshFinishSlot();
	void StitchFinishSlot();
	void taskTeethSititFinishSlot();
	void recallWindow();

	void showOrderInfo(COrderInfo orderInfo);
	void upperJawBtnBtnClick();
	void upperJawDenBtnClick();
	void upperJawGinBtnClick();
	void lowJawBtnClick();
	void lowJawDenBtnClick();
	void lowJawGinBtnClick();
	void DentalImplantNextBtnClick();
	void DentalImplantPanelBackBtnClick();
	void DentalImplantFinishNextBtnClick();
	void DentalImplantFinishBackBtnClick();
	

	void progressBarSetSlot(int min, int max, bool bVisible);
	
	void usbDeviceSlot();//软件打开时，提示是否USB连接成功
						 //扫描
	void JawScanFinish();
};

#endif