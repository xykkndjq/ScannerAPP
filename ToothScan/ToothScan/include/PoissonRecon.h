

#ifndef POISSONRECONSTRUCTION_H
#define POISSONRECONSTRUCTION_H


#include "basetype.h"
extern "C"
class PoissonReconstruction 
{

public:

	__declspec (dllexport) PoissonReconstruction();

	bool __declspec (dllexport) run(orth::MeshModel& mm,const int depth, const float error_threshold);

	bool __declspec (dllexport) run(vector<orth::MeshModel>& mms, orth::MeshModel& mm, const int depth, const float error_threshold);

	void __declspec (dllexport) RedundancyFilter(orth::MeshModel &model_target, orth::MeshModel &model_to_filt, double error_threshold);

	bool __declspec (dllexport) MergeWithoutRedundancy(orth::MeshModel &model_target, vector<orth::MeshModel> &models_to_merge, orth::MeshModel &model_merged, double error_threshold);
};


#endif
