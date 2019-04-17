#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include "device_functions.h"
#include "device_launch_parameters.h"  
#include <thrust/sort.h>
#include <opencv2\opencv.hpp>

extern "C" void newDcloudtoPcloudCaller(double *DoubleCloud_src, float *Cloud_dst, int PointsSize);
extern "C" void newPcloudtoDcloudCaller(float *Cloud_src, double *DoubleCloud_dst, int PointsSize);
extern "C" void newTakeDimCaller(double *CloudSet_src, int CloudsPointsSize, double *XDim_dst, double *YDim_dst, double *ZDim_dst, double *MinMax_p);
//extern "C" void newTakeDimCaller(double *CloudSet_src, int CloudsPointsSize, double *XDim_dst, double *YDim_dst, double *ZDim_dst);
//ClosestPoint
extern "C" void newCloudHashCaller(double *CloudSet_src, int CloudsPointsSize, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst);
extern "C" void newClosestSearchCaller(double *CloudSet_src, int *CloudSize_src, int *CloudSize_p, int *StartIndex_src, char *CellSize_src, double *MinMax_src, int CloudsPointsSize, int CloudNum, double *ClosestPointMatrixV_dst, int *IndexValue_dst);
//MutulClosest
extern "C" void newMutulClosestCaller(double *ClosestPointMatrixV_src, int *CloudSize_src, int *CloudSize_p, int CloudNum);
//solve AiVector and KiVector
extern "C" void DistributeAiKiCaller(double *CloudSet_src, double *ClosestPointMatrixV_src, int *CloudSize_src, int CloudsPointsSize, int CloudNum, double *AiV_dst, double *KiV_dst, char *JwV_dst);
//solve Tw and Rw
extern "C" void newAsvdCaller(double *AiV_dst, double *KiV_dst, int *ValidRowIndex_src, double *ValidAij_dst, double *MultAijKi_dst, double *Asvd, int ValidRowSize, int bias);
extern "C" void newRwTwCaller(double *AiV_dst, double *KiV_dst, double *Mu_src, double *Mv_src, int *ValidRowIndex_src, double *AiRw_dst, double *MultAiRwKi_dst, double *Rw_dst, double *Tw, int ValidRowSize, int bias);
extern "C" void newUpdataCaller(double *CloudSet_src, double *Rw_src, double *Tw_src, double *TransCloud_dst, int CloudSize, int sourcebias);

extern "C" int divUp(int total, int grain);

__global__ void newDcloudtoPcloudKernel(double *DoubleCloud_src, float *Cloud_dst, int PointsSize);
__global__ void newPcloudtoDcloudKernel(float *Cloud_src, double *DoubleCloud_dst, int PointsSize);
__global__ void newTakeDimKernel(double *CloudSet_src, double *XDim_dst, double *YDim_dst, double *ZDim_dst, int CloudsPointsSize);
//ClosestPoint
__global__ void newCloudHashKernel(double *CloudSet_src, int CloudsPointsSize, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst);
__global__ void newClosestSearchKernel(double *CloudSet_src, int *CloudSize_src, int *StartIndex_src, char *CellSize_src, double *MinMax_src, int CloudsPointsSize, int CloudMark, int CloudNum, double *ClosestPointMatrixV_dst, int *IndexValue_dst, int sourcebias);
//MutulClosest
__global__ void newMutulClosestKernel(double *ClosestPointMatrixV_src, int *CloudSize_src, int CloudMark, int CloudRow, int CloudCol);
//solve AiVector and KiVector
__global__ void newDistributeAiKiKernel(double *CloudSet_src, double *ClosestPointMatrixV_src, int *CloudSize_src, int CloudsPointsSize, int CloudNum, double *AiV_dst, double *KiV_dst, char *JwV_dst);
//transforam
__global__ void newMultiplyAiandIJKernel(double *AiV_dst, int *ValidRowIndex_src, double *ValidAij_dst, int ValidCols, int ValidRowSize, int bias);
__global__ void newMultiplyAijandKiKernel(double *ValidAij_dst, double *KiV_dst, int *ValidRowIndex_src, double *MultAijKi_dst, int ColNum, int ValidRowSize, int bias);
__global__ void newRowMultiplyKernel(double *Matrix1, double *Matrix2, double *ResultMatrix, int ValidRows, int ValidCols);
__global__ void newAiRwMultiplyKernel(double *AiV_dst, double *Rw_dst, int *ValidRowIndex_src, double *AiRw_dst, int ValidRowSize, int ValidColSize, int bias);
__global__ void newKisubAiRwKernel(double *KiV_dst, double *AiRw_dst, int *ValidRowIndex_src, double *Tw_dst, int ValidRows, int ValidCols, int ColMark, int bias);
//updata
__global__ void  newUpdataKernel(double *CloudSet_src, double *Rw_src, double *Tw_src, double *TransCloud_dst, int CloudSize, int sourcebias);

class GPAGPU {
public:
	GPAGPU();
	~GPAGPU();
	
	void HANDLE_ERROR(cudaError_t err);
	
	void DcloudtoPcloudCuda(double *DoubleCloud_p, float *Cloud_p, int PointsSize);
	void PcloudtoDcloudCuda(float *Cloud_p, double *DoubleCloud_p, int PointsSize);
	void GpaRegistrationCuda(double **CloudSet_p, int *CloudSize_p, int CloudsPointsSize, int CloudNum, int TotalIterNum);
	void MinMaxCuda(double *CloudSet_src, int CloudsPointsSize, double *MinMax_p);

	void chooseMaxMin(int CloudsPointsSize, double *XDim_src, double *YDim_src, double *ZDim_src, double *MaxMin);

	//ClosestMatrixVector
	void CloudClosestCuda(double *CloudSet_src, double *MinMax_src, int *CloudSize_src, int *CloudSize_p, int CloudsPointsSize, int CloudNum, int *CloudsHashCode_dst, int *IndexValue_dst, double *ClosestPointMatrixV_dst, int *StartIndex_src, char *CellSize_src);
	void MutulClosestCuda(double *ClosestPointMatrixV_src, int *CloudSize_src, int *CloudSize_p, int CloudNum);
	void DistributeAiKiCuda(double *CloudSet_src, double *ClosestPointMatrixV_src, int *CloudSize_src, int CloudsPointsSize, int CloudNum, double *AiV_dst, double *KiV_dst, char *JwV_dst);
	//transform
	void SolveValidRow(char *JwV_dst, int *ValidRowIndex_src, int *ValidRowSizeV, int CloudsPointsSize, int *CloudSize_p, int CloudNum);
	void TransformCuda(double *AiV_dst, double *KiV_dst, int *ValidRowIndex_src, double *ValidMulTemp_dst, double *MultTemp_dst, double *Mu_dst, double *Mv_dst, double *Rw_dst, double *Tw_dst, int ValidRowSize, int bias);
	void UpdataCuda(double *CloudSet_src, double *Rw_dst, double *Tw_dst, double *ValidMulTemp_dst, int CloudSize, int sourcebias);
};