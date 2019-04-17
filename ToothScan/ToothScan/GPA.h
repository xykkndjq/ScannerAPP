#include <stdio.h>
#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include "GPAGPU.cuh"
#include <vector>
#include <fstream>
#include "orthio.h"
#include "basetype.h"
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using pcl::PointXYZ;
using pcl::PointXYZRGB;
using pcl::PointCloud;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Matrix3d;
using cv::Mat;

class GPA {
public:
	__declspec(dllexport) GPA();
	__declspec(dllexport) ~GPA();

	GPAGPU *GpaGpu;
	//GPU
	__declspec(dllexport) void GpaRegistrationGPU(vector<orth::MeshModel> CloudSet_source, vector<cv::Mat> &RTVec, int TotalIterNum);

private:
	int CloudNum = 0;

	void RTSolve(vector<double> &points_original, vector<double> &points_target, cv::Mat &RTmatrix);
	void GaussNewTon(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target);
	void computJ(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target, cv::Mat &j);
	void computF(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target, cv::Mat &f);
	double Function(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target);
	//MeshModel×ª»¯Îªvectordouble
	void MeshModeltoVectorDouble(vector<orth::MeshModel> &mModelSet, vector<vector<double>> &DoubleCloudSet);
};