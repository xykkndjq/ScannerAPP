#ifndef ComputeThread_H
#define ComputeThread_H

#include <QObject>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include "ConTrolThread.h"
#include "3DScan_cuda.h"
#include "registration.h"
#include "GPA.h"
#include <pcl/io/ply_io.h>
#include <Windows.h>
#include <time.h>
#include "TaskManager.h"


//#define SCAN_ROTATE_POS_CNT2 18    //定义扫描过程的位置数 11
// #define IMG_ROW 1024
// #define IMG_COL 1280

class ComputeThread : public QObject
{
	Q_OBJECT

public:
	ComputeThread(QObject *parent = 0);
	~ComputeThread();

	//是否做GPU运算标志
	bool doGPUFlag = false;

	//???,????????
	bool showUpperModelFlag = false;
	bool showLowerModelFlag = false;
	//????????????????
	bool regUpperModelFlag = false;
	bool regLowerModelFlag = false;
	GPA gpa;

	//??????
	int addUpperCompensationNum = 0;
	int addLowerCompensationNum = 0;
	int addAllCompensationNum = 0;

	int oldJawIndex = 0;
	vector<orth::MeshModel> all_mModel;//??
	//vector<vector<double>> all_points_cloud_globle2;
	//vector<double> all_points_cloud_end2;
	vector<orth::MeshModel> upper_mModel;//??
	//vector<vector<double>> upper_points_cloud_globle2;
	//vector<double> upper_points_cloud_end2;
	vector<orth::MeshModel> lower_mModel;//??
	//vector<vector<double>> lower_points_cloud_globle2;
	//vector<double> lower_points_cloud_end2;
	
	//??????
	//orth::MeshModel upperRegModel;
	//orth::MeshModel lowerRegModel;
	orth::MeshModel allRegModel;

	cv::Mat camera_image;//?????

	/*vector<vector<double>> all_points_cloud_globle;
	vector<vector<double>> all_points_cloud_globle2;
	vector<vector<uint32_t>> all_vertexIndicies;
	vector<vector<unsigned char>> all_rgb_image;
	vector<double> all_points_cloud_end2;
	vector<vector<float>> all_global_normals;
	
	vector<vector<double>> upper_points_cloud_globle;
	vector<vector<double>> upper_points_cloud_globle2;
	vector<vector<uint32_t>> upper_vertexIndicies;
	vector<vector<unsigned char>> upper_rgb_image;
	vector<double> upper_points_cloud_end2;
	vector<vector<float>> upper_global_normals;
	
	vector<vector<double>> lower_points_cloud_globle;
	vector<vector<double>> lower_points_cloud_globle2;
	vector<vector<uint32_t>> lower_vertexIndicies;
	vector<vector<unsigned char>> lower_rgb_image;
	vector<double> lower_points_cloud_end2;
	vector<vector<float>> lower_global_normals;*/

	bool  isStop;
	float color_red_parameter = 4.0;
	float color_green_parameter = 2.0;
	float color_blue_parameter = 3.2;

	int	current_pointcloud_count = 0;

	void InitParameters();
	void setFlage(bool flag = true);  //?????,???????

	orth::MeshModel curTotalModel;//合并出当前整个MeshModel便于后续用于补扫配准

	
public:
	inline double  ToRad(const double &a) { return M_PI*a / 180.0; }
	bool isTriangle(const cv::Vec6f &triangle, const cv::Mat &point_cloud);
	float DistanceCalculate(float x1, float y1, float z1, float x2, float y2, float z2);
	void pointcloudrotation(orth::PointCloudD &pointcloud, orth::PointNormal &PointNormal, cv::Mat &RT);
	void pointcloudrotationandtotalmesh(orth::PointCloudD &pointCloud, orth::PointNormal &pointNormal, orth::PointColor &pointColor, cv::Mat &RT, orth::MeshModel &totalMeshModel);
	void pointcloudrotation(vector<double> &pointcloud, cv::Mat &RT);
	void Motor2Rot(const float pitch, const float yaw, cv::Mat &Rot);
	//bool chooseJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex, int scan_index, scan::Registration & reg);
	//bool chooseCompenJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex, scan::Registration & reg);
	
	void writefile(orth::MeshModel totalMeshModel, string name);
	void writefile(vector<double> cloud, string name);
	void ReductMesh(orth::MeshModel &model_target, orth::MeshModel &model_source);

signals:
	void showModeltoGlSingel(int refreshIndex);
	void cameraShowSignal();
	void computeFinish();
	void showTaskModel();
	void meshFinish();
	void StitchFinish();
	void taskTeethSititFinish();
	void finishFarRegSignal();//???????????
	void progressBarResetSignal();
	void progressBarSetMinSignal(int min);
	void progressBarSetMaxSignal(int max);
	void progressBarSetValueSignal(int value);
	void progressBarsetOrientation(Qt::Orientation);
	void progressBarVisibleSignal(bool visible);
	void progressBarSetSignal(int min,int max, bool bVisible);
	void toolsGroupBoxSetVisible(bool bVisible);
public slots:
	void FarRegistrationSlot();//??????????
	//void normalAllJawComputeScan();
	//void GPAMeshing(int chooseJawIndex);//全局配准和Meshing

	void normalComputeScan();
	void allJawComputeScan();
	void compensationCompute();
	void GPAMeshing();//全局配准和Meshing

	void Stitching();
	void taskTeethSitit();
public:
	bool chooseJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp,
		int scan_index, scan::Registration & reg, pCScanTask pScanTask);

	bool chooseCompenJawAndIcp(cv::Mat matched_pixel_image, 
		vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, scan::Registration & reg, pCScanTask pScanTask);
};

#endif // ComputeThread_H