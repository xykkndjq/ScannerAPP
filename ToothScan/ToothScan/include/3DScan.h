#ifndef SCAN_H
#define SCAN_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/features2d.hpp>
#include <math.h>
#include "basetype.h"
#include <iostream>
#include <windows.h>

#define EXPOSURE_L -6
#define EXPOSURE_R -1
#define color_threshold 60
#define IsoValue 0.f
#define cali_width 11.0f
#define cali_height 9.0f
#define cali_size 10.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846   // pi
#endif


using cv::Mat;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::cin;
using std::vector;

static const uint32_t c_countOfImagesToGrab = 10;
static const size_t c_maxCamerasToUse = 2;


namespace scan
{

	struct Voxel
	{
		float distance_;
		float weight_;

		//Eigen::Vector3f normal_;
		orth::Point3f normal_;

		//float weight_color_;
		//int color_r_;
		//int color_g_;
		//int color_b_;
	};

	struct Voxel2
	{
		vector<int> point_index;
	};

	struct PointEdges
	{
		//Eigen::VectorXd point_coor_;
		orth::Point3d point_coor_;
		vector<double> Edges;
	};

	struct MeshEdgeNode
	{
		int edge1_node_ = -1;
		int edge2_node_ = -1;
		int edge3_node_ = -1;
	};

	struct PointAndIndex
	{
		orth::Point3d point_coor_;
		int	point_index_;
	};

	typedef std::vector<std::vector<std::vector<Voxel>>> Volume;
	typedef std::vector<std::vector<std::vector<Voxel2>>> Volume2;

	//struct MaskEdge
	//{
	//	double length = 0;
	//	int p1 = 0;
	//	int p2 = 0;
	//};
	//
	//struct MastPoint
	//{
	//	int currentP_ = -1;
	//	int candidateP1_ = -1;
	//	int candidateP2_ = -1;
	//	int matchP_ = -1;
	//};


	extern "C"
		class RasterScan
	{
	public:
		__declspec (dllexport) RasterScan();
		__declspec (dllexport) ~RasterScan();

		void __declspec (dllexport) RasterScan::InitRasterScan(const std::string out_put_name);

		void RasterScan::ReadScanData(const string dir, vector<Mat> &image_data, bool flag);

		void __declspec (dllexport) RasterScan::ChosePoints(const float point1_x, const float point1_y, const float point2_x, const float point2_y, const int screen_width, const int screen_height, float *model_matrix, float *view_matrix, float *projection_matrix, orth::MeshModel &mm);

		void __declspec (dllexport) delaunayAlgorithm(const cv::Mat &point_cloud, vector<cv::Mat> &image_rgb, cv::Mat &Rtmatrix, const float color_red_parameter, const float color_green_parameter, const float color_blue_parameter,const float max_edge_length, orth::MeshModel &mm, const float sample_rate, vector<double>& points2);

		void RasterScan::CircleCenterCalculate(Mat &image, std::vector<cv::Point2d> &imagePoints, bool flag);

		void RasterScan::FindCorners(const Mat &Image, vector<cv::Point2f>& imagePoints, const cv::Size &boardSize);

		//Eigen::Vector3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2);
		cv::Point3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2);

		//Eigen::Vector3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2, float x3, float y3);
		cv::Point3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2, float x3, float y3);

		void RasterScan::mergeImg(Mat & dst, Mat &src1, Mat &src2);

		//����ƥ���õ�ƥ��㷽����Ϊͨ�÷���
		//
		void RasterScan::EpiLinesFindPoint(Mat &img1, Mat &img2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2);

		//����λͼ����ƥ��ͼ���
		//kΪ����ͼ��tΪ��λͼ��F�����������׵�ֱ�Ϊ����ͼ�ж�Ӧ��ƥ���
		void __declspec (dllexport) RasterScan::MatchPoint(Mat &img_k_1, Mat &img_k_2, Mat &img_t_1, Mat &img_t_2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &dis_);

		void RasterScan::CornerDistanceTest(Mat &img_k_1, Mat &img_k_2, Mat &img_t_1, Mat &img_t_2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<cv::Point2f> &point_corner);

		void DrawEpiLines(vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<float> &points);

		void RasterScan::Calculate3DPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2,
			vector<double> &points, vector<double> &new_points);

		void RasterScan::Calculate3DPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, Mat &right_depth_map);

		//void RasterScan::Calculate3DMaskPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, vector<Eigen::VectorXd> &points_end2);
		void RasterScan::Calculate3DMaskPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, orth::PointCloudD &points_end2);

		void RasterScan::Calculate3DPointsEvaluate(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2,
			vector<double> &points, vector<double> &new_points);

		std::vector<cv::Point2f> RasterScan::Generate2DPoints();

		std::vector<cv::Point3f> RasterScan::Generate3DPoints();

		Mat __declspec (dllexport) RasterScan::Distortion(Mat& image, bool flag);

		void __declspec(dllexport) RasterScan::UndistortionPoint(double u, double v, cv::Mat &intr, cv::Mat &dist, double &ut, double &vt);

		void RasterScan::FindLaserPoints(const Mat &LaserImage, vector<cv::Point2f> &points);

		//bool RasterScan::EdgeSearch(vector<Eigen::VectorXd> &points_mask, Eigen::Matrix4d &Rt);
		//bool RasterScan::EdgeSearch(orth::PointCloudD &points_mask, cv::Mat &Rt);
		bool RasterScan::EdgeSearch(orth::PointCloudD &globel_points, orth::PointCloudD &points_mask, cv::Mat &Rt);

		double RasterScan::Distance(double x1, double y1, double z1, double x2, double y2, double z2);

		void  __declspec (dllexport) XY_TsdfVolume(const double* size, const int* resolution, const double &truncation, const int width_, const int height_);

		void  __declspec (dllexport) XY_TsdfVolume(double* tsdf_info, float* tsdf_data);

		void __declspec (dllexport) XY_PointFilterVolume(const double* size, const int* resolution);

		void __declspec (dllexport)  CloseFilter(cv::Mat &depth);

		void __declspec (dllexport)  TSDF(const Mat &depth, cv::Mat &RT2 /*const Eigen::Matrix4d &RT2*/);

		void TSDFGaussFilter();

		void __declspec (dllexport)  MarchingCube();
		void __declspec (dllexport)  MarchingCube2();

		std::vector<float> verts;

		std::vector<float> norms;

		std::vector<int32_t> vertexIndicies;

		void __declspec (dllexport) SystemCalibration(std::string out_put_name, orth::Point3d &error_);

		void __declspec (dllexport) PreCalibration(const int projector_width_, const int projector_height_, const int image_number_, std::vector<std::vector<cv::Mat>> &image_groups_);
		
		void __declspec(dllexport) CalibrationImagePrint(cv::Mat &image_show_l, cv::Mat &image_show_r, int image_index, bool &left_flag, bool &right_flag);
		
		void CalculateProjectorCameraParameter(const vector<double> &Points, const vector<double> &k, const vector<double> &t, cv::Mat &X);

		void OneCameraPointCloudCalculate(cv::Mat &periodic_, cv::Mat &phase_, vector<double> &point_cloud, const bool camera_flag);

		void __declspec (dllexport) CompareCameraAndProjectorPointCloud(cv::Mat &periodic_, cv::Mat &phase_, cv::Mat &depth_map, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &point_cloud, double threshold_value);

		void __declspec (dllexport) UnwrapComp(const vector<Mat> &cimg, Mat &phiO, Mat &periodO);

		void __declspec (dllexport) PointCloudAddIn(vector<double> &point_cloud_, vector<float> &point_normal_, cv::Mat &Rt_);

		void __declspec (dllexport) ReductionWholePoints(vector<double> &point_cloud_, vector<float> &point_normal_);

		void __declspec (dllexport) PlaneRTCalculate(vector<cv::Mat> &left_circle_images, vector<cv::Mat> &right_circle_images, const string RT_file_name, vector<double> &totle_mask_point);

		void __declspec(dllexport) Motor2Rot(const float pitch, const float yaw, cv::Mat &Rot);

		void __declspec(dllexport) SystemCompensation(static int* scan_x, static int* scan_y, int scan_number, vector<cv::Mat> &RT_matrixs);

		void __declspec(dllexport) PointCloudCalculateCuda2(unsigned char *left_images, unsigned char *right_images, int image_height, int image_width, double *F_matrix, double* rot_l, double* rot_r, double* trans_l, double* trans_r, double* cameraMatrix_l, double* cameraMatrix_r, double* distortion_l, double* distortion_r, double* c_p_system_r, double *matched_pixel_image, double *normal_image, double* depth_image, double dis_threshold);

		void __declspec(dllexport) PointCloudCalculateCuda16(unsigned char *left_images, unsigned char *right_images, int image_height, int image_width, double *F_matrix, double* rot_l, double* rot_r, double* trans_l, double* trans_r, double* cameraMatrix_l, double* cameraMatrix_r, double* distortion_l, double* distortion_r, double* c_p_system_r, double *matched_pixel_image, double *normal_image, double* depth_image, double dis_threshold);


		void PatternsComp(vector<Mat> &pimg);

		void CompensatePhase(const Mat &phi1, const Mat &phi2, Mat &phi_out);

		Mat PatternPlain(int n, float f, int N, float init_phase);

		Mat intr1, intr2, distCoeffs[3], intrp;

		Mat R;

		Mat T;

		Mat F;

		Mat F2;

		Mat Rot_l;

		Mat tvec_l;

		Mat Rot_r;

		Mat tvec_r;

		Mat Rot_p;

		Mat tvec_p;


		Mat c_p_system_l;

		Mat c_p_system_r;

		vector<double> point_cloud_whole;
		vector<float> point_normal_whole;


	private:

		inline double  ToRad(const double &a) { return M_PI*a / 180.0; }
		//const int kN = 6;
		//const float kF = 16;
		const cv::Size kPatternSize = cv::Size(1024, 768);
		const int kN_comp = 3;
		const int kN_comp_last = 3;
		const float kF_comp = 4;
		const int kOrder = 4;

		//Eigen::VectorXd A_l;
		//Eigen::VectorXd A_r;
		vector<double> A_l;
		vector<double> A_r;

		double tranc_dist_;

		Volume volume_Data;
		Volume2 volume_point;
		vector<float> volume_Data2;



		int resolution_[3];

		int image_width;

		int image_height;

		double cell_size[3];

		double start_x, start_y, start_z, end_x, end_y, end_z;

		std::vector<std::vector<std::vector<MeshEdgeNode>>> meshedge;
		//std::vector<std::vector<std::vector<float> > > GaussTemplate;
		std::vector<std::vector<float> > GaussTemplate;

		std::vector<std::vector<cv::Point3f>> objectPoints_l;
		std::vector<std::vector<cv::Point3f>> objectPoints_r;
		std::vector<std::vector<cv::Point3f>> objectPoints_projector;
		std::vector<std::vector<cv::Point2f>> imagePoints_l;
		std::vector<std::vector<cv::Point2f>> imagePoints_r;
		std::vector<std::vector<cv::Point2f>> imagePoints_projector;
		std::vector<std::vector<int>> CCPoints_index_l;
		std::vector<std::vector<int>> CCPoints_index_r;

		std::vector<std::vector<double>> projector_phase_whole_l;
		std::vector<std::vector<double>> projector_period_whole_l;
		std::vector<std::vector<double>> projector_phase_whole_r;
		std::vector<std::vector<double>> projector_period_whole_r;

		std::vector<std::vector<cv::Mat>> image_groups;

		vector<vector<Mat>> images_h_l, images_v_l;
		vector<vector<Mat>> images_h_r, images_v_r;

		int projector_width, projector_height;

		//vector<Eigen::VectorXd> mask_points_list;
		//vector<MaskEdge> mask_edge_list;
		vector<PointEdges> mask_points_edge;

		float RasterScan::Distance(float x1, float y1, float x2, float y2);

		//void RasterScan::CalculatePlane(const vector<Eigen::VectorXd> &Points, Eigen::Vector4d &Plane);
		void RasterScan::CalculatePlane(const orth::PointCloudD &Points, cv::Mat &Plane);

		//double RasterScan::Function(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, int matched_number);
		double RasterScan::Function(cv::Mat &X_, orth::PointCloudD &points_mask, vector<int> &match_list, int matched_number);

		//void RasterScan::computF(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, Eigen::VectorXd &f, int matched_number);
		void RasterScan::computF(cv::Mat &X_, orth::PointCloudD &points_mask, vector<int> &match_list, cv::Mat &f, int matched_number);

		//void RasterScan::computJ(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, Eigen::MatrixXd &j, int matched_number);
		void RasterScan::computJ(cv::Mat &X_, orth::PointCloudD &points_mask, vector<int> &match_list, cv::Mat &j, int matched_number);

		//void RasterScan::GaussNewTon(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, int matched_number);
		void RasterScan::GaussNewTon(cv::Mat &X_, orth::PointCloudD &points_mask, vector<int> &match_list, int matched_number);

		//void RasterScan::CalculatePlane_XY_theta(const vector<Eigen::VectorXd> &Points, Eigen::Vector4d &Plane);
		void RasterScan::CalculatePlane_XY_theta(const orth::PointCloudD &Points, cv::Mat &Plane);

		void RasterScan::CircleCenterCalculate2(Mat &image, std::vector<cv::Point2d> &imagePoints);

		void RasterScan::bilateralKernel(const Mat &depth, Mat &depth_filterd, float sigma_space2_inv_half, float sigma_color2_inv_half);

		void RasterScan::bilateralKernelScan(const Mat &depth, const Mat &period, Mat &depth_filterd, float sigma_space2_inv_half, float sigma_color2_inv_half);

		void RasterScan::scaleDepth(const Mat &depth, Mat &scaled_depth);

		int RasterScan::computeCubeIndex(int x, int y, int z, float f[8]) const;

		void RasterScan::readTsdf(int x, int y, int z, float& tsdf, int& weight) const;
		void RasterScan::readTsdf(int x, int y, int z, float& tsdf) const;

		//Eigen::Vector3f RasterScan::getNodeCoo(int x, int y, int z);
		orth::Point3f RasterScan::getNodeCoo(int x, int y, int z);

		//Eigen::Vector3f RasterScan::vertex_interp(Eigen::Vector3f &p0, Eigen::Vector3f &p1, float f0, float f1, float &t2);
		orth::Point3f RasterScan::vertex_interp(orth::Point3f &p0, orth::Point3f &p1, float f0, float f1, float &t2);

		//float RasterScan::interpolateTrilineary(const Eigen::Vector3f &point);
		float RasterScan::interpolateTrilineary(const orth::Point3f &point);

		//Eigen::Vector3f RasterScan::MeshNormalCalculate(const Eigen::Vector3f &point);
		orth::Point3f RasterScan::MeshNormalCalculate(const orth::Point3f &point);

		//Eigen::Vector3i RasterScan::getVoxel(const Eigen::Vector3f point)const;
		orth::Point3i RasterScan::getVoxel(const orth::Point3f point)const;

		//float RasterScan::TSDFGaussCore(int x, int y, int z);

		double RasterScan::TSDFGaussCore(const cv::Mat &image_intput, const cv::Mat &image_intput2, const int x, const int y);

		/*------------------------------- marching cubes ------------------------------*/


		/** \brief Edge table for marching cubes  */
		// edge table maps 8-bit flag representing which cube vertices are inside
		// the isosurface to 12-bit number indicating which edges are intersected
		const int edgeTable_[256] =
		{
			0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
			0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
			0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
			0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
			0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
			0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
			0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
			0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
			0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
			0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
			0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
			0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
			0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
			0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
			0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
			0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
			0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
			0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
			0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
			0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
			0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
			0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
			0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
			0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
			0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
			0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
			0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
			0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
			0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
			0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
			0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
			0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
		};

		// number of vertices for each case above
		const int numVertsTable_[256] =
		{
			0,
			3,
			3,
			6,
			3,
			6,
			6,
			9,
			3,
			6,
			6,
			9,
			6,
			9,
			9,
			6,
			3,
			6,
			6,
			9,
			6,
			9,
			9,
			12,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			9,
			3,
			6,
			6,
			9,
			6,
			9,
			9,
			12,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			9,
			6,
			9,
			9,
			6,
			9,
			12,
			12,
			9,
			9,
			12,
			12,
			9,
			12,
			15,
			15,
			6,
			3,
			6,
			6,
			9,
			6,
			9,
			9,
			12,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			9,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			15,
			9,
			12,
			12,
			15,
			12,
			15,
			15,
			12,
			6,
			9,
			9,
			12,
			9,
			12,
			6,
			9,
			9,
			12,
			12,
			15,
			12,
			15,
			9,
			6,
			9,
			12,
			12,
			9,
			12,
			15,
			9,
			6,
			12,
			15,
			15,
			12,
			15,
			6,
			12,
			3,
			3,
			6,
			6,
			9,
			6,
			9,
			9,
			12,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			9,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			15,
			9,
			6,
			12,
			9,
			12,
			9,
			15,
			6,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			15,
			9,
			12,
			12,
			15,
			12,
			15,
			15,
			12,
			9,
			12,
			12,
			9,
			12,
			15,
			15,
			12,
			12,
			9,
			15,
			6,
			15,
			12,
			6,
			3,
			6,
			9,
			9,
			12,
			9,
			12,
			12,
			15,
			9,
			12,
			12,
			15,
			6,
			9,
			9,
			6,
			9,
			12,
			12,
			15,
			12,
			15,
			15,
			6,
			12,
			9,
			15,
			12,
			9,
			6,
			12,
			3,
			9,
			12,
			12,
			15,
			12,
			15,
			9,
			12,
			12,
			15,
			15,
			6,
			9,
			12,
			6,
			3,
			6,
			9,
			9,
			6,
			9,
			12,
			6,
			3,
			9,
			6,
			12,
			3,
			6,
			3,
			3,
			0,
		};

		// triangle table maps same cube vertex index to a list of up to 5 triangles
		// which are built from the interpolated edge vertices
		const int triTable_[256][16] =
		{
			{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
			{ 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 },
			{ 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 },
			{ 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
			{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 },
			{ 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 },
			{ 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 },
			{ 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
			{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 },
			{ 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
			{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
			{ 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
			{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
			{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 },
			{ 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
			{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 },
			{ 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 },
			{ 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
			{ 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
			{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
			{ 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 },
			{ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
			{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 },
			{ 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
			{ 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
			{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 },
			{ 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
			{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 },
			{ 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
			{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 },
			{ 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
			{ 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 },
			{ 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 },
			{ 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
			{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 },
			{ 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
			{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 },
			{ 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 },
			{ 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
			{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
			{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
			{ 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
			{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 },
			{ 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
			{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 },
			{ 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
			{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
			{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 },
			{ 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 },
			{ 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 },
			{ 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
			{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 },
			{ 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 },
			{ 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
			{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 },
			{ 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
			{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
			{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
			{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 },
			{ 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 },
			{ 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
			{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 },
			{ 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 },
			{ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
			{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 },
			{ 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
			{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 },
			{ 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
			{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 },
			{ 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 },
			{ 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
			{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
			{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 },
			{ 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
			{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 },
			{ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
			{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
			{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
			{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 },
			{ 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 },
			{ 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 },
			{ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
			{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 },
			{ 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 },
			{ 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
			{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 },
			{ 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 },
			{ 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
			{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 },
			{ 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
			{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 },
			{ 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
			{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
			{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
			{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
			{ 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
			{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 },
			{ 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
			{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 },
			{ 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
			{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 },
			{ 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
			{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 },
			{ 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
			{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 },
			{ 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
			{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 },
			{ 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 },
			{ 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
			{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
			{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
			{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }
		};
	};

}

#endif


