#include <stdio.h>
#include "basetype.h"
#include "orthio.h"
#include "GPUNNS.cuh"

using std::cout;
using std::endl;

class NearestNeighborSearches
{
public:
	__declspec(dllexport) NearestNeighborSearches();
	__declspec(dllexport) ~NearestNeighborSearches();
	__declspec(dllexport) bool NearestPointsSearchGPU(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int tree_depth, vector<unsigned int> &query_index, vector<double> &nearest_distance);
	
private:
	GPUNNS *GpuNns;
	vector<orth::MeshModel> CloudSet;
	void MeshModeltoVectorDouble(orth::MeshModel *mModel, vector<double> &DoubleCloud);
};