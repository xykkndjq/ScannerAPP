#ifndef NEARESTNEIGHBORSEARCHES_H
#define NEARESTNEIGHBORSEARCHES_H

#include <stdio.h>
#include "basetype.h"
//#include <cuda.h>
//#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include "device_functions.h"
#include "device_launch_parameters.h"  
#include <thrust/sort.h>

void newTakeDimNnsCaller(double *CloudSet_src, int CloudsPointsSize, double *XDim_dst, double *YDim_dst, double *ZDim_dst, double *MinMax_p);

void newCloudHashNnsCaller(double *CloudSet_src, int *CloudSize_p, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst, const int treeSize);
void newClosestSearchNnsCaller(double *CloudSet_src, int *CloudSize_src, int *CloudSize_p, int *StartIndex_src, int *CellSize_src, double *MinMax_src, int CloudsPointsSize, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *radius_dst, int *IndexValue_dst, const int treeSize, double corr_dist_threshold_);

int divup(int total, int grain);

__global__ void newTakeDimNnsKernel(double *CloudSet_src, double *XDim_dst, double *YDim_dst, double *ZDim_dst, int CloudsPointsSize);
__global__ void newCloudHashNnsKernel(double *CloudSet_src, int CloudPointSize, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst, const int treeSize, int dataBias);
__global__ void newClosestSearchNnsKernel(double *CloudSet_src, int *CloudSize_src, int *StartIndex_src, int *CellSize_src, double *MinMax_src, int CloudsPointsSize, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *radius_dst, int *IndexValue_dst, const int treeSize, double corr_dist_threshold_);

class GPUNNS {
public:
	GPUNNS();
	~GPUNNS();

	double corr_dist_threshold = 2.0;

	void HANDLE_ERROR(cudaError_t err);
	void GpuNnsCuda(double **CloudSet_p, int *CloudSize_p, int CloudNum, int CloudsPointsSize, int *ClosestPointIndexV_p, double *ClosestPointDistanceV_p, const int treeSize);
	void MinMaxNnsCuda(double *CloudSet_src, int CloudsPointsSize, double *MinMax_p);

	//ClosestMatrixVector
	void NearestPointsSearchCuda(double *CloudSet_src, double *MinMax_src, int *CloudSize_src, int *CloudSize_p, int CloudsPointsSize, int CloudNum, int *CloudsHashCode_dst, int *IndexValue_dst, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *radius_dst, int *StartIndex_src, int *CellSize_src, const int treeSize, double corr_dist_threshold_);
};

class NearestNeighborSearches
{
public:
	__declspec(dllexport) NearestNeighborSearches();
	__declspec(dllexport) ~NearestNeighborSearches();

	bool __declspec(dllexport) NearestPointsSearchGPU(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int tree_depth, vector<int> &query_indexV, vector<double> &nearest_distanceV);

private:
	GPUNNS *GpuNns;
	vector<orth::MeshModel> CloudSet;
	void MeshModeltoVectorDouble(orth::MeshModel *mModel, vector<double> &DoubleCloud);
	
};



#endif //NEARESTNEIGHBORSEARCHES_H