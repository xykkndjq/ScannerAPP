#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <opencv2/opencv.hpp>
#include <3DScan_cuda.h>
#include <orthio.h>
#include <iostream>

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


	};


}

#endif // !REGISTRATION_H
