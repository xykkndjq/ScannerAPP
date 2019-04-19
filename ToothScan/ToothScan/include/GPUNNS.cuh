#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include "device_functions.h"
#include "device_launch_parameters.h"  
#include <thrust/sort.h>


class GPUNNS {
public:
	GPUNNS();
	~GPUNNS();
	
	void HANDLE_ERROR(cudaError_t err);
	
	void GpuNnsCuda(double **CloudSet_p, int *CloudSize_p, int CloudNum, int CloudsPointsSize, double *ClosestPointMatrix_p, const int treeSize);
	void MinMaxCuda(double *CloudSet_src, int CloudsPointsSize, double *MinMax_p);

	void chooseMaxMin(int CloudsPointsSize, double *XDim_src, double *YDim_src, double *ZDim_src, double *MaxMin);

	// solve ClosestMatrixVector
	void CloudClosestCuda(double *CloudSet_src, double *MinMax_src, int *CloudSize_src, int *CloudSize_p, int CloudsPointsSize, int CloudNum, int *CloudsHashCode_dst, int *IndexValue_dst, double *ClosestPointMatrixV_dst, int *StartIndex_src, char *CellSize_src, const int SIZE);
};