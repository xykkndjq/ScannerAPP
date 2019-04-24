#include <stdio.h>
#include "basetype.h"
#include "orthio.h"
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include "device_functions.h"
#include "device_launch_parameters.h"  
#include <thrust/sort.h>

using std::cout;
using std::endl;

void newTakeDimCaller(double *CloudSet_src, int CloudsPointsSize, double *XDim_dst, double *YDim_dst, double *ZDim_dst, double *MinMax_p);

void newCloudHashCaller(double *CloudSet_src, int CloudsPointsSize, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst, const int SIZE);
void newClosestSearchCaller(double *CloudSet_src, int *CloudSize_src, int *CloudSize_p, int *StartIndex_src, char *CellSize_src, double *MinMax_src, int CloudsPointsSize, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *IndexValue_dst, const int SIZE);

int divUp(int total, int grain);

__global__ void newTakeDimKernel(double *CloudSet_src, double *XDim_dst, double *YDim_dst, double *ZDim_dst, int CloudsPointsSize);
__global__ void newCloudHashKernel(double *CloudSet_src, int CloudsPointsSize, double *MinMax_src, int *CloudsHashCode_dst, int *IndexValue_dst, const int SIZE);
__global__ void newClosestSearchKernel(double *CloudSet_src, int *CloudSize_src, int *StartIndex_src, char *CellSize_src, double *MinMax_src, int CloudsPointsSize, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *IndexValue_dst, const int SIZE);

class GPUNNS {
public:
	GPUNNS();
	~GPUNNS();

	void HANDLE_ERROR(cudaError_t err);

	void GpuNnsCuda(double **CloudSet_p, int *CloudSize_p, int CloudNum, int CloudsPointsSize, int *ClosestPointIndexV_p, double *ClosestPointDistanceV_p, const int treeSize);
	void MinMaxCuda(double *CloudSet_src, int CloudsPointsSize, double *MinMax_p);

	void chooseMaxMin(int CloudsPointsSize, double *XDim_src, double *YDim_src, double *ZDim_src, double *MaxMin);

	//ClosestMatrixVector
	void CloudClosestCuda(double *CloudSet_src, double *MinMax_src, int *CloudSize_src, int *CloudSize_p, int CloudsPointsSize, int CloudNum, int *CloudsHashCode_dst, int *IndexValue_dst, int *ClosestPointIndexV_dst, double *ClosestPointDistanceV_dst, int *StartIndex_src, char *CellSize_src, const int SIZE);
};


class NearestNeighborSearches
{
public:
	__declspec(dllexport) NearestNeighborSearches();
	__declspec(dllexport) ~NearestNeighborSearches();
	__declspec(dllexport) bool NearestPointsSearchGPU(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int tree_depth, vector<int> &query_indexV, vector<double> &nearest_distanceV);
	
private:
	GPUNNS *GpuNns;
	vector<orth::MeshModel> CloudSet;
	void MeshModeltoVectorDouble(orth::MeshModel *mModel, vector<double> &DoubleCloud);
};