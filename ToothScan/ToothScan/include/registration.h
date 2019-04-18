#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <opencv2/opencv.hpp>
#include <3DScan_cuda.h>
#include <orthio.h>
#include <iostream>

namespace scan
{

	class Registration
	{
	public:
		//初始化配准参数
		Registration(const float MaxCorrespondenceDistance_, const float RANSACOutlier_,const int MaxIteration_);
		Registration();
		~Registration();

		//将模型加入配准队列进行配准，配准成功后模型被矫正到准确位置，返回是否配准成功
		bool PushIn(orth::MeshModel &add_model);

		//上下颌与全颌配准 或 带型与上下颌配准时采用的配准方式，返回是否配准成功
		bool FarRegist(orth::MeshModel &model_target, orth::MeshModel &model_source);

		//设定配准误差阈值
		void SetRegistError(const float mesh_regist_error_);

	private:

		double MatchCalculate(orth::MeshModel &pointcloud1, orth::MeshModel &pointcloud2);

		std::vector<orth::MeshModel> M_Models;
		std::vector<cv::Mat> M_RotMatrix;

		float mesh_regist_error = 0.4;
		float MaxCorrespondenceDistance = 1.0;
		float RANSACOutlier = 10.0;
		int MaxIteration = 50;

	};


}

#endif // !REGISTRATION_H
