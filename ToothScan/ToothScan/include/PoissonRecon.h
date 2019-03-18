
#ifndef POISSON_RECON
#define POISSON_RECON
/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#endif // _WIN32
#include "recon/prTime.h"
#include "recon/MarchingCubes.h"
#include "recon/Octree.h"
#include "recon/SparseMatrix.h"
#include "recon/CmdLineParser.h"
#include "recon/PPolynomial.h"
#include "recon/Ply.h"
#include "recon/MemoryUsage.h"
#include "orthio.h"
#ifdef _OPENMP
#include "omp.h"

#endif // _OPENMP

#include "recon/MultiGridOctreeData.h"

#define DEFAULT_FULL_DEPTH 5

#define XSTR(x) STR(x)
#define STR(x) #x
#if DEFAULT_FULL_DEPTH
#pragma message ( "[WARNING] Setting default full depth to " XSTR(DEFAULT_FULL_DEPTH) )
#endif // DEFAULT_FULL_DEPTH

#include <stdarg.h>

char* outputFile = NULL;
int echoStdout = 0;
void DumpOutput(const char* format, ...)
{
	if (outputFile)
	{
		FILE* fp = fopen(outputFile, "a");
		va_list args;
		va_start(args, format);
		vfprintf(fp, format, args);
		fclose(fp);
		va_end(args);
	}
	if (echoStdout)
	{
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
}
void DumpOutput2(char* str, const char* format, ...)
{
	if (outputFile)
	{
		FILE* fp = fopen(outputFile, "a");
		va_list args;
		va_start(args, format);
		vfprintf(fp, format, args);
		fclose(fp);
		va_end(args);
	}
	if (echoStdout)
	{
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	va_end(args);
	if (str[strlen(str) - 1] == '\n') str[strlen(str) - 1] = 0;
}

namespace recon
{

	cmdLineString
		In("in"),
		Out("out"),
		VoxelGrid("voxel"),
		XForm("xForm");

	cmdLineReadable
#ifdef _WIN32
		Performance("performance"),
#endif // _WIN32
		Complete("complete"),
		ShowResidual("showResidual"),
		NoComments("noComments"),
		PolygonMesh("polygonMesh"),
		Confidence("confidence"),
		NormalWeights("nWeights"),
		NonManifold("nonManifold"),
		ASCII("ascii"),
		Density("density"),
		Verbose("verbose"),
		Double("double");

	cmdLineInt
		Depth("depth", 8),
		CGDepth("cgDepth", 0),
		KernelDepth("kernelDepth"),
		AdaptiveExponent("adaptiveExp", 1),
		Iters("iters", 8),
		VoxelDepth("voxelDepth", -1),
		FullDepth("fullDepth", DEFAULT_FULL_DEPTH),
		MinDepth("minDepth", 0),
		MaxSolveDepth("maxSolveDepth"),
		BoundaryType("boundary", 1),
		Threads("threads", omp_get_num_procs());

	cmdLineFloat
		SamplesPerNode("samplesPerNode", 1.f),
		Scale("scale", 1.1f),
		CSSolverAccuracy("cgAccuracy", float(1e-3)),
		PointWeight("pointWeight", 4.f);


	cmdLineReadable* params[] =
	{
		&In , &Depth , &Out , &XForm ,
		&Scale , &Verbose , &CSSolverAccuracy , &NoComments , &Double ,
		&KernelDepth , &SamplesPerNode , &Confidence , &NormalWeights , &NonManifold , &PolygonMesh , &ASCII , &ShowResidual , &VoxelDepth ,
		&PointWeight , &VoxelGrid , &Threads , &MaxSolveDepth ,
		&AdaptiveExponent , &BoundaryType ,
		&Density ,
		&FullDepth ,
		&MinDepth ,
		&CGDepth , &Iters ,
		&Complete ,
#ifdef _WIN32
		&Performance ,
#endif // _WIN32
	};

	template< class Real, class Vertex >
	int Execute(orth::MeshModel &mm)
	{
		Reset< Real >();
		int i;
		int paramNum = sizeof(params) / sizeof(cmdLineReadable*);
		int commentNum = 0;
		char **comments;

		comments = new char*[paramNum + 7];
		for (i = 0; i<paramNum + 7; i++) comments[i] = new char[1024];

		if (Verbose.set) echoStdout = 1;

		XForm4x4< Real > xForm, iXForm;
		if (XForm.set)
		{
			FILE* fp = fopen(XForm.value, "r");
			if (!fp)
			{
				fprintf(stderr, "[WARNING] Could not read x-form from: %s\n", XForm.value);
				xForm = XForm4x4< Real >::Identity();
			}
			else
			{
				for (int i = 0; i<4; i++) for (int j = 0; j<4; j++)
				{
					float f;
					fscanf(fp, " %f ", &f);
					xForm(i, j) = (Real)f;
				}
				fclose(fp);
			}
		}
		else xForm = XForm4x4< Real >::Identity();
		iXForm = xForm.inverse();

		DumpOutput2(comments[commentNum++], "Running Screened Poisson Reconstruction (Version 6.13)\n");
		char str[1024];
		for (int i = 0; i<paramNum; i++)
			if (params[i]->set)
			{
				params[i]->writeValue(str);
				if (strlen(str)) DumpOutput2(comments[commentNum++], "\t--%s %s\n", params[i]->name, str);
				else                DumpOutput2(comments[commentNum++], "\t--%s\n", params[i]->name);
			}
		
		double t;
		double tt = Time();
		Real isoValue_ = 0;

		Octree< Real > tree;
		tree.threads = Threads.value;
		//if (!In.set)
		//{
		//	cout << " Poisson Wrong !!" << endl;
		//	return 0;
		//}
		if (!MaxSolveDepth.set) MaxSolveDepth.value = Depth.value;

		OctNode< TreeNodeData >::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

		t = Time();
		int kernelDepth = KernelDepth.set ? KernelDepth.value : Depth.value - 2;
		if (kernelDepth>Depth.value)
		{
			fprintf(stderr, "[ERROR] %s can't be greater than %s: %d <= %d\n", KernelDepth.name, Depth.name, KernelDepth.value, Depth.value);
			return EXIT_FAILURE;
		}
		
		double maxMemoryUsage;
		t = Time(), tree.maxMemoryUsage = 0;
		typename Octree< Real >::PointInfo* pointInfo = new typename Octree< Real >::PointInfo();
		typename Octree< Real >::NormalInfo* normalInfo = new typename Octree< Real >::NormalInfo();

		std::vector< Real >* kernelDensityWeights = new std::vector< Real >();
		std::vector< Real >* centerWeights = new std::vector< Real >();
		PointStream< float >* pointStream;

		//char* ext = GetFileExtension(In.value);
		//if     ( !strcasecmp( ext , "bnpts" ) ) pointStream = new BinaryPointStream< float >( In.value );
		//else if( !strcasecmp( ext , "ply"   ) ) pointStream = new    PLYPointStream< float >( In.value );
		//else                                    pointStream = new  ASCIIPointStream< float >( In.value );

		
		//orth::ModelRead mr("./0016.ply", mm);
		int all_points_number = 0;
		std::vector<std::pair<Point3D<float>, Point3D<float>>> pn;
		for (size_t i = 0; i < mm.P.size(); i++)
		{
			std::pair<Point3D<float>, Point3D<float>> pp;
			//pp.first = Point3D<float>(mm.P[i].x, mm.P[i].y, mm.P[i].z);
			//pp.second = Point3D<float>(mm.N[i].x, mm.N[i].y, mm.N[i].z);
			pp.first[0] = (float)mm.P[i].x;
			pp.first[1] = (float)mm.P[i].y;
			pp.first[2] = (float)mm.P[i].z;
			pp.second[0] = (float)mm.N[i].x;
			pp.second[1] = (float)mm.N[i].y;
			pp.second[2] = (float)mm.N[i].z;
			pn.push_back(pp);
		}
		pointStream = new MemoryPointStream< float >(mm.P.size(), &(pn[0]));

		
		int pointCount = tree.template SetTree< float >(pointStream, MinDepth.value, Depth.value, FullDepth.value, kernelDepth, Real(SamplesPerNode.value), Scale.value, Confidence.set, NormalWeights.set, PointWeight.value, AdaptiveExponent.value, *pointInfo, *normalInfo, *kernelDensityWeights, *centerWeights, BoundaryType.value, xForm, Complete.set);
		if (!Density.set) delete kernelDensityWeights, kernelDensityWeights = NULL;

		//DumpOutput2(comments[commentNum++], "#             Tree set in: %9.1f (s), %9.1f (MB)\n", Time() - t, tree.maxMemoryUsage);
		//DumpOutput("Input Points: %d\n", pointCount);
		//DumpOutput("Leaves/Nodes: %d/%d\n", tree.tree.leaves(), tree.tree.nodes());
		//DumpOutput("Memory Usage: %.3f MB\n", float(MemoryInfo::Usage()) / (1 << 20));

		maxMemoryUsage = tree.maxMemoryUsage;
		t = Time(), tree.maxMemoryUsage = 0;
		Pointer(Real) constraints = tree.SetLaplacianConstraints(*normalInfo);
		delete normalInfo;
		//DumpOutput2(comments[commentNum++], "#      Constraints set in: %9.1f (s), %9.1f (MB)\n", Time() - t, tree.maxMemoryUsage);
		//DumpOutput("Memory Usage: %.3f MB\n", float(MemoryInfo::Usage()) / (1 << 20));
		maxMemoryUsage = std::max< double >(maxMemoryUsage, tree.maxMemoryUsage);

		t = Time(), tree.maxMemoryUsage = 0;
		Pointer(Real) solution = tree.SolveSystem(*pointInfo, constraints, ShowResidual.set, Iters.value, MaxSolveDepth.value, CGDepth.value, CSSolverAccuracy.value);
		delete pointInfo;
		FreePointer(constraints);
		/*	DumpOutput2(comments[commentNum++], "# Linear system solved in: %9.1f (s), %9.1f (MB)\n", Time() - t, tree.maxMemoryUsage);
		DumpOutput("Memory Usage: %.3f MB\n", float(MemoryInfo::Usage()) / (1 << 20));*/
		maxMemoryUsage = std::max< double >(maxMemoryUsage, tree.maxMemoryUsage);
		//CoredFileMeshData< Vertex > mesh;
		CoredVectorMeshData< Vertex > mesh;

		if (Verbose.set) tree.maxMemoryUsage = 0;
		t = Time();
		isoValue_ = tree.GetIsoValue(solution, *centerWeights);
		delete centerWeights;
		//DumpOutput("Got average in: %f\n", Time() - t);
		//DumpOutput("Iso-Value: %e\n", isoValue);
		if (VoxelGrid.set)
		{
			double t = Time();
			FILE* fp = fopen(VoxelGrid.value, "wb");
			if (!fp) fprintf(stderr, "Failed to open voxel file for writing: %s\n", VoxelGrid.value);
			else
			{
				int res;
				Pointer(Real) values = tree.Evaluate(solution, res, isoValue_, VoxelDepth.value);
				fwrite(&res, sizeof(int), 1, fp);
				if (sizeof(Real) == sizeof(float)) fwrite(values, sizeof(float), res*res*res, fp);
				else
				{
					float *fValues = new float[res*res*res];
					for (int i = 0; i<res*res*res; i++) fValues[i] = float(values[i]);
					fwrite(fValues, sizeof(float), res*res*res, fp);
					delete[] fValues;
				}
				fclose(fp);
				DeletePointer(values);
			}
			//DumpOutput("Got voxel grid in: %f\n", Time() - t);
		}
		//if (Out.set)
		{
			t = Time(), tree.maxMemoryUsage = 0;
			tree.GetMCIsoSurface(kernelDensityWeights ? GetPointer(*kernelDensityWeights) : NullPointer< Real >(), solution, isoValue_, mesh, true, !NonManifold.set, PolygonMesh.set);
			if (PolygonMesh.set) DumpOutput2(comments[commentNum++], "#         Got polygons in: %9.1f (s), %9.1f (MB)\n", Time() - t, tree.maxMemoryUsage);
			else                  DumpOutput2(comments[commentNum++], "#        Got triangles in: %9.1f (s), %9.1f (MB)\n", Time() - t, tree.maxMemoryUsage);
			maxMemoryUsage = std::max< double >(maxMemoryUsage, tree.maxMemoryUsage);
			//DumpOutput2(comments[commentNum++], "#             Total Solve: %9.1f (s), %9.1f (MB)\n", Time() - tt, maxMemoryUsage);
			mm.P.clear();
			mm.N.clear();
			mm.F.clear();
			for (int point_index = 0; point_index < mesh.oocPoints.size(); point_index++)
			{
				PlyValueVertex<Real> pv = mesh.oocPoints[point_index];
				
				orth::Vectord vec(pv.point[0], pv.point[1], pv.point[2]);
				mm.P.push_back(vec);
			}
			for (int face_index = 0; face_index < mesh.polygons.size(); face_index++)
			{
				orth::face ff(mesh.polygons[face_index][0], mesh.polygons[face_index][1], mesh.polygons[face_index][2]);
				mm.F.push_back(ff);

			}
			//for (int point_index = 0; point_index < length; point_index++)
			//{

			//}

			//if( NoComments.set )
			//{
			//	if( ASCII.set ) PlyWritePolygons( Out.value , &mesh , PLY_ASCII         , NULL , 0 , iXForm );
			//	else            PlyWritePolygons( Out.value , &mesh , PLY_BINARY_NATIVE , NULL , 0 , iXForm );
			//}
			//else
			//{
			//	if( ASCII.set ) PlyWritePolygons( Out.value , &mesh , PLY_ASCII         , comments , commentNum , iXForm );
			//	else            PlyWritePolygons( Out.value , &mesh , PLY_BINARY_NATIVE , comments , commentNum , iXForm );
			//}
			//DumpOutput("Vertices / Polygons: %d / %d\n", mesh.outOfCorePointCount() + mesh.inCorePoints.size(), mesh.polygonCount());
		}

		FreePointer(solution);

		return 1;


	}

#ifdef _WIN32
	inline double to_seconds(const FILETIME& ft)
	{
		const double low_to_sec = 100e-9; // 100 nanoseconds
		const double high_to_sec = low_to_sec*4294967296.0;
		return ft.dwLowDateTime*low_to_sec + ft.dwHighDateTime*high_to_sec;
	}
#endif // _WIN32

	//template< class Real>
	//int PoissonReconstruction(orth::MeshModel mm)
	//{
	//	return Execute< Real, PlyValueVertex< Real > >(mm);
	//}

}


#endif // POISSON_RECON
