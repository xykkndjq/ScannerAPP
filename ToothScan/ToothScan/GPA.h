#include <stdio.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "GPAGPU.cuh"
#include <vector>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Matrix3d;
using cv::Mat;

class GPA {
public:
	__declspec(dllexport) GPA();
	__declspec(dllexport) ~GPA();

	GPAGPU *GpaGpu;
	__declspec(dllexport) void GpaRegistrationGPU(vector<vector<double>> DoubleCloudSet_source, vector<vector<double>> &DoubleCloudSet_target, vector<cv::Mat> &RTVec, int TotalIterNum);

private:
	int CloudNum = 0;

	void RTSolve(vector<double> &points_original, vector<double> &points_target, cv::Mat &RTmatrix);
	void GaussNewTon(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target);
	void computJ(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target, cv::Mat &j);
	void computF(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target, cv::Mat &f);
	double Function(cv::Mat &X_, vector<double> &points_original, vector<double> &points_target);
};