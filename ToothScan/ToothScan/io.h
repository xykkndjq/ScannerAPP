#ifndef ORTH_IO_H
#define ORTH_IO_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

using namespace std;


namespace orth
{
	class ModelRead
	{
	public:
		/*  Model Data */
		string directory;
		bool gammaCorrection;

		/*  Functions   */
		// constructor, expects a filepath to a 3D model.
		ModelRead(string const &path, vector<float> &mm, bool gamma = false) : gammaCorrection(gamma)
		{
			loadModel(path, mm);
		}




	private:
		/*  Functions   */
		// loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
		void loadModel(string const &path, vector<float> &mm)
		{
			// read file via ASSIMP
			Assimp::Importer importer;
			const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
			// check for errors
			if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
			{
				cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
				return;
			}
			// retrieve the directory path of the filepath
			directory = path.substr(0, path.find_last_of('/'));

			// process ASSIMP's root node recursively
			processNode(scene->mRootNode, mm, scene);
		}

		// processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
		void processNode(aiNode *node, vector<float> &mm, const aiScene *scene)
		{
			aiMesh* mesh = scene->mMeshes[0];
			processMesh(mesh, mm, scene);

		}

		void processMesh(aiMesh *mesh, vector<float> &mm, const aiScene *scene)
		{
			// data to fill
			for (int face_index = 0; face_index < mesh->mNumFaces; face_index++)
			{
				aiFace *face_ = &(mesh->mFaces[face_index]);
				for (int point_index = 0; point_index < 3; point_index++)
				{
					aiVector3D *point_ = &(mesh->mVertices[face_->mIndices[point_index]]);
					mm.push_back(point_->x);
					mm.push_back(point_->y);
					mm.push_back(point_->z);
				}
			}
			//orth::PointCloudD P;
			//orth::Faces F;
			//P.resize(mesh->mNumVertices);
			//F.resize(mesh->mNumFaces);
			// Walk through each of the mesh's vertices
			//for (unsigned int i = 0; i < mesh->mNumVertices; i++)
			//{
			//	P[i].x = mesh->mVertices[i].x;
			//	P[i].y = mesh->mVertices[i].y;
			//	P[i].z = mesh->mVertices[i].z;
			//}
			//// now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
			//for (unsigned int i = 0; i < mesh->mNumFaces; i++)
			//{
			//	F[i].x = mesh->mFaces[i].mIndices[0];
			//	F[i].y = mesh->mFaces[i].mIndices[1];
			//	F[i].z = mesh->mFaces[i].mIndices[2];

			//}
			//mm.P.swap(P);
			//mm.F.swap(F);
			return;
		}

	};
}




#endif // !ORTH_IO_H

