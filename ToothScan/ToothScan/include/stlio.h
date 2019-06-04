

#ifndef TINYSTL_
#define TINYSTL_

#include <stdio.h>
#include <algorithm>
#include <basetype.h>


namespace tinystl {

	/**
	This class encapsulate a filter for importing stl (stereolitograpy) meshes.
	The stl format is quite simple and rather un-flexible. It just stores, in ascii or binary the, unindexed, geometry of the faces.
	Warning: this code assume little endian (PC) architecture!!!
	*/
	//template <class IOMeshType>


	extern "C"
	class stlio
	{
	public:
		// if it is binary there are 80 char of comment, the number fn of faces and then exactly fn*4*3 bytes.

		enum { STL_LABEL_SIZE = 80 };

		class STLFacet
		{
		public:
			orth::Point3f n;
			orth::Point3f v[3];
			//  short attr;
		};

		enum STLError {
			E_NOERROR,       // 0
			E_CANTOPEN,      // 1
			E_UNESPECTEDEOF, // 2
			E_MALFORMED,     // 3 
			E_LAST
		};

		/* Try to guess if a stl has color
		*
		* rules:
		* - It has to be binary
		* - The per face attribute should be not zero
		*
		* return false in case of malformed files
		*/
		bool IsSTLColored(const char * filename, bool &coloredFlag, bool &magicsMode);

		/* Try to guess if a stl is in binary format
		*
		* return false in case of malformed files
		*/
		bool IsSTLBinary(const char * filename, bool &binaryFlag);

		int __declspec (dllexport) read_stl_file(const char * filename,orth::MeshModel &mm);

		int OpenBinary(orth::MeshModel &mm, const char * filename);

		int OpenAscii(orth::MeshModel &mm, const char * filename);

		int __declspec (dllexport) Save(orth::MeshModel &mm, const char * filename);

		int __declspec (dllexport) write_stl_file(const char * filename,orth::MeshModel &mm,  bool binary = true);



		int RemoveDuplicateVertex(orth::MeshModel *mm, int Qctree_depth);   // V1.0

		/*
		returns mask of capability one define with what are the saveable information of the format.
		*/
		//static int GetExportMaskCapability()
		//{
		//	int capability = 0;
		//	capability |= vcg::tri::io::Mask::IOM_VERTCOORD;
		//	capability |= vcg::tri::io::Mask::IOM_FACEINDEX;
		//	capability |= vcg::tri::io::Mask::IOM_FACECOLOR;
		//	return capability;
		//}
	}; // end class

} // end Namespace TINYSTL


#endif
