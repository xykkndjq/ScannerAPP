#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <opencv2/opencv.hpp>
#include <3DScan_cuda.h>
#include <basetype.h>
#include <iostream>
#include <utility>

namespace scan
{
	extern "C"
	class Registration
	{
	public:
		//初始化配准参数
		__declspec (dllexport) Registration( const float MaxCorrespondenceDistance_, const float RANSACOutlier_,const int MaxIteration_);
		__declspec (dllexport) Registration();
		__declspec (dllexport) ~Registration();

		//正常扫描：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) NearRegist(std::vector<orth::MeshModel> &v_M_Models, orth::MeshModel &add_model);

		//补扫配准函数：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) CompenNearRegist(orth::MeshModel &v_M_Model, orth::MeshModel &add_model);

		//上下颌与全颌配准 或 带型与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) FarRegist(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//单颗牙与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) ToothFarRegist(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//设定配准误差阈值
		void __declspec (dllexport) SetRegistError(const float mesh_regist_error_);

		//设定查找深度
		void __declspec (dllexport) SetSearchDepth(const int serach_depth_);


	private:

		double MatchCalculate(orth::MeshModel &pointcloud1, orth::MeshModel &pointcloud2);

		std::vector<orth::MeshModel> *M_Models;
		std::vector<cv::Mat> M_RotMatrix;

		float mesh_regist_error = 0.4;
		float MaxCorrespondenceDistance = 0.7;
		float RANSACOutlier = 15.0;
		int MaxIteration = 50;
		int search_depth = 50;

		/*void getRansacCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_source, vector<pair<int, int>> &curCorrespondences, vector<pair<int, int>> &newCorrespondences);
		void getSamples(vector<int> &source_indices, int &iterations, std::vector<int> &samples);
		void drawIndexSample(vector<int> &source_indices, std::vector<int> &sample);
		bool isSampleGood(std::vector<int> &samples);
		void computeSampleDistanceThreshold(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &indices, double &sample_dist_thresh);
		void computeMeanAndCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices, float *covariance_matrix, float *centroid);*/
	};


}

#endif // !REGISTRATION_H
