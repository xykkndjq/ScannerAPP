#ifndef SCAN_CUDA_CUH
#define SCAN_CUDA_CUH

#include "cuda_runtime.h"  
#include "device_launch_parameters.h"  
#include <iostream>
#include <cmath>
#include <thrust\host_vector.h>

#define MY_PI 3.1415926535897932384626433832795

namespace scan
{


	extern "C"
	class Unwarp
	{
	public:
		__declspec (dllexport) Unwarp();
		__declspec (dllexport) ~Unwarp();

		cudaError_t __declspec (dllexport) addWithCuda(double *k1_image, double *k2_image, double *t1_image, double *t2_image, int image_height, int image_width, double *F_matrix, double *matched_pixel_image);

		cudaError_t __declspec (dllexport) PointCloudCalculateCuda(double *k1_image, double *k2_image, double *t1_image, double *t2_image, int image_height, int image_width, double *F_matrix, double* rot_l, double* rot_r, double* trans_l, double* trans_r, double* cameraMatrix_l, double* cameraMatrix_r, double* c_p_system_r, double *matched_pixel_image, double dis_threshold);

		//cudaError_t __declspec (dllexport) MeshCalculateCuda(double *k1_image, double *k2_image, double *t1_image, double *t2_image, int image_height, int image_width, double *F_matrix, double* rot_l, double* rot_r, double* trans_l, double* trans_r, double* cameraMatrix_l, double* cameraMatrix_r, double* c_p_system_r, double *matched_pixel_image, double* TSDF_info, float* TSDF_, double dis_threshold);

		cudaError_t __declspec (dllexport) PointCloudCalculateCuda2(unsigned char *left_images, unsigned char *right_images, int image_height, int image_width, double *F_matrix, double* rot_l, double* rot_r, double* trans_l, double* trans_r, double* cameraMatrix_l, double* cameraMatrix_r, double* distortion_l, double* distortion_r, double* c_p_system_r, double *matched_pixel_image,double *normal_image, double dis_threshold);



	};


}



#endif