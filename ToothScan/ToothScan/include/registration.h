#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <opencv2/opencv.hpp>
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
		__declspec (dllexport) Registration(const float RANSACOutlier_,const int MaxIteration_);
		__declspec (dllexport) Registration();
		__declspec (dllexport) ~Registration();

		/*--------------------------------------CPU----------------------------------------*/
		//正常扫描：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) NearRegistCPU(std::vector<orth::MeshModel> &v_M_Models, orth::MeshModel &add_model);

		//补扫配准函数：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) CompenNearRegistCPU(orth::MeshModel &v_M_Model, orth::MeshModel &add_model);

		//上下颌与全颌配准 或 带型与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) FarRegistCPU(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//单颗牙与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) ToothFarRegistCPU(orth::MeshModel &model_target, orth::MeshModel &model_source);


		/*--------------------------------------GPU----------------------------------------*/
		//正常扫描：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) NearRegistGPU(std::vector<orth::MeshModel> &v_M_Models, orth::MeshModel &add_model);

		//补扫配准函数：将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool __declspec (dllexport) CompenNearRegistGPU(orth::MeshModel &v_M_Model, orth::MeshModel &add_model);

		//上下颌与全颌配准 或 带型与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) FarRegistGPU(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//单颗牙与上下颌配准时采用的配准方式，返回是否配准成功
		bool __declspec (dllexport) ToothFarRegistGPU(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//设定配准误差阈值
		void __declspec (dllexport) SetRegistError(const float mesh_regist_error_);

		//设定查找深度
		void __declspec (dllexport) SetSearchDepth(const int serach_depth_);


	private:
		double MatchCalculateCPU(orth::MeshModel &pointcloud1, orth::MeshModel &pointcloud2);
		double MatchCalculateGPU(orth::MeshModel &pointcloud1, orth::MeshModel &pointcloud2);

		void MeshRot(double *RT_src, orth::MeshModel *cloud_src, orth::MeshModel *cloud_dst);
		void MeshRotGPU(double *RT, orth::MeshModel *cloudMeshRot);

		bool NPSGPU(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int tree_depth, vector<int> &query_indexV, vector<double> &nearest_distanceV);//求最近点
		void MeshModeltoVectorDouble(orth::MeshModel *mModel, vector<float> &FloatCloud);

		std::vector<orth::MeshModel> *M_Models;
		//std::vector<cv::Mat> M_RotMatrix;

		float mesh_regist_error = 0.4;
		//float MaxCorrespondenceDistance = 0.7;
		float RANSACOutlier = 15.0;
		int MaxIteration = 50;
		int search_depth = 50;
	};


}

#endif // !REGISTRATION_H
