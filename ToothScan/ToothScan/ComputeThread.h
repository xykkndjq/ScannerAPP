#ifndef ComputeThread_H
#define ComputeThread_H

#include <QObject>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include "features.hpp"
#include "octree.hpp"
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

	GPA gpa;

	//增加补扫数量
	int addUpperCompensationNum = 0;
	int addLowerCompensationNum = 0;
	int addAllCompensationNum = 0;

	int oldJawIndex = 0;
	vector<orth::MeshModel> all_mModel;//全颌
	vector<vector<double>> all_points_cloud_globle2;
	vector<double> all_points_cloud_end2;
	vector<orth::MeshModel> upper_mModel;//上颌
	vector<vector<double>> upper_points_cloud_globle2;
	vector<double> upper_points_cloud_end2;
	vector<orth::MeshModel> lower_mModel;//下颌
	vector<vector<double>> lower_points_cloud_globle2;
	vector<double> lower_points_cloud_end2;
	
	cv::Mat camera_image;//相机上图像

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
	void setFlage(bool flag = true);  //设置标志位，何时关闭子线程


	
public:
	inline double  ToRad(const double &a) { return M_PI*a / 180.0; }
	bool isTriangle(const cv::Vec6f &triangle, const cv::Mat &point_cloud);
	float DistanceCalculate(float x1, float y1, float z1, float x2, float y2, float z2);
	void pointcloudrotation(orth::PointCloudD &pointcloud, orth::PointNormal &PointNormal, cv::Mat &RT);
	void pointcloudrotationandtotalmesh(orth::PointCloudD &pointCloud, orth::PointNormal &pointNormal, orth::PointColor &pointColor, cv::Mat &RT, orth::MeshModel &totalMeshModel);
	void pointcloudrotation(vector<double> &pointcloud, cv::Mat &RT);
	double MatchCalculate(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud2);
	bool pointcloudICP(vector<double> &cloud1, vector<double> &cloud2, int sample_number1, int sample_number2, cv::Mat &Rt);
	void NormalCalculate(vector<double> &pointcloud, vector<float> &pointnormal, cv::Mat &rt);
	void Motor2Rot(const float pitch, const float yaw, cv::Mat &Rot);
	bool chooseJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex, int scan_index);
	bool chooseCompenJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex);
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
public slots:
	void controlComputeScan(int chooseJawIndex);
	void compensationComputeScan(int chooseJawIndex);
	void GPAMeshing(int chooseJawIndex);//全局配准和Meshing

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