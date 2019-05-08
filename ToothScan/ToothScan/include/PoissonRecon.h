

#ifndef POISSONRECONSTRUCTION_H
#define POISSONRECONSTRUCTION_H


#include "basetype.h"
extern "C"
class PoissonReconstruction 
{

public:

	__declspec (dllexport) PoissonReconstruction();

	bool __declspec (dllexport) run(orth::MeshModel& mm,const int depth);

	bool __declspec (dllexport) run(vector<orth::MeshModel>& mms, orth::MeshModel& mm, const int depth);
};


#endif
