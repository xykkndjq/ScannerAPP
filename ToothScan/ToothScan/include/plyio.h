#pragma once
#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#include <iterator>
#include "basetype.h"


namespace tinyply
{
	struct float2 { float x, y; };
	struct float3 { float x, y, z; };
	struct double3 { double x, y, z; };
	struct uint3 { uint32_t x, y, z; };
	struct uint4 { uint8_t x, y, z, w; };
	struct uintc3 { uint8_t x, y, z; };
	struct uintf3 { int32_t x, y, z; };
	extern "C"
		class plyio
	{
	public:
		__declspec (dllexport) plyio();
		__declspec (dllexport) ~plyio();

		void __declspec (dllexport) write_ply_file(const std::string & filename, orth::MeshModel &mm, const bool is_binary);

		void __declspec (dllexport) read_ply_file(const std::string & filepath, orth::MeshModel &mm);

	private:

	};






}
