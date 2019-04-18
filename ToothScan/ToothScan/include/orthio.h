#ifndef ORTH_IO_H
#define ORTH_IO_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Exporter.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "basetype.h"

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
		ModelRead(string const &path, MeshModel &mm, bool gamma = false) : gammaCorrection(gamma)
		{
			loadModel(path, mm);
		}




	private:
		/*  Functions   */
		// loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
		void loadModel(string const &path, MeshModel &mm)
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
		void processNode(aiNode *node, MeshModel &mm, const aiScene *scene)
		{
			aiMesh* mesh = scene->mMeshes[0];
			processMesh(mesh, mm, scene);

		}

		void processMesh(aiMesh *mesh, MeshModel &mm, const aiScene *scene)
		{
			// data to fill

			orth::PointCloudD P;
			orth::Faces F;
			orth::PointNormal facenormal;
			orth::PointNormal pointsnormal;

			P.resize(mesh->mNumVertices);
			F.resize(mesh->mNumFaces);
			facenormal.resize(mesh->mNumFaces);
			pointsnormal.resize(mesh->mNumVertices);

			// Walk through each of the mesh's vertices
			for (unsigned int i = 0; i < mesh->mNumVertices; i++)
			{
				P[i].x = mesh->mVertices[i].x;
				P[i].y = mesh->mVertices[i].y;
				P[i].z = mesh->mVertices[i].z;
			}

			// now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
			for (unsigned int i = 0; i < mesh->mNumFaces; i++)
			{
				F[i].x = mesh->mFaces[i].mIndices[0];
				F[i].y = mesh->mFaces[i].mIndices[1];
				F[i].z = mesh->mFaces[i].mIndices[2];
				//facenormal[i] = TriangleNormal(P[F[i].x], P[F[i].y], P[F[i].z]);
				//pointsnormal[F[i].x] = facenormal[i];
				//pointsnormal[F[i].y] = facenormal[i];
				//pointsnormal[F[i].z] = facenormal[i];
			}



			//for (unsigned int i = 0; i < mesh->mNumVertices; i++)
			//{
			//	pointsnormal[i].x = mesh->mNormals[i].x;
			//	pointsnormal[i].y = mesh->mNormals[i].y;
			//	pointsnormal[i].z = mesh->mNormals[i].z;
			//}



			mm.P.swap(P);
			mm.F.swap(F);
			mm.N.swap(pointsnormal);

			mm.NormalUpdate();

			return;
		}

	};

	class ModelIO
	{
	public:
		/*  Model Data */
		string directory;
		bool gammaCorrection;

		/*  Functions   */
		// constructor, expects a filepath to a 3D model.
		ModelIO(MeshModel *mesh_model, bool gamma = false) : gammaCorrection(gamma)
		{
			mm = mesh_model;
		}

		/*  Functions   */
		// loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
		void loadModel(string const &path)
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
			processNode(scene->mRootNode, *mm, scene);
		}

		void writeModel(const std::string&path, const std::string&Format)
		{
			// read file via ASSIMP

			if (mm->P.size() == 0)
			{
				std::cout << " MeshModel have not any data!! " << endl;
				return;
			}
			Assimp::Exporter exporter;

			aiScene* scene = new aiScene;

			scene->mRootNode = new aiNode();

			scene->mMaterials = new aiMaterial*[1];
			scene->mMaterials[0] = nullptr;
			scene->mNumMaterials = 1;

			scene->mMaterials[0] = new aiMaterial();

			scene->mMeshes = new aiMesh*[1];
			scene->mMeshes[0] = nullptr;
			scene->mNumMeshes = 1;

			scene->mMeshes[0] = new aiMesh();
			scene->mMeshes[0]->mMaterialIndex = 0;

			scene->mRootNode->mMeshes = new unsigned int[1];
			scene->mRootNode->mMeshes[0] = 0;
			scene->mRootNode->mNumMeshes = 1;

			aiMesh* pMesh = scene->mMeshes[0];

			pMesh->mVertices = new aiVector3D[mm->P.size()];
			pMesh->mNormals = new aiVector3D[mm->N.size()];
			pMesh->mColors[0] = new aiColor4D[mm->C.size()];
			pMesh->mNumVertices = mm->P.size();
			pMesh->mNumFaces = mm->F.size();

			//pMesh->mTextureCoords[0] = new aiVector3D[vVertices.size()];
			//pMesh->mNumUVComponents[0] = vVertices.size();
			//int j = 0;
			//for (auto itr = mm->P.begin(); itr != mm->P.end(); ++itr)
			//{
			//	pMesh->mVertices[itr - vVertices.begin()] = aiVector3D(vVertices[j].x, vVertices[j].y, vVertices[j].z);
			//	pMesh->mNormals[itr - vVertices.begin()] = aiVector3D(normals[j].x, normals[j].y, normals[j].z);
			//	pMesh->mTextureCoords[0][itr - vVertices.begin()] = aiVector3D(uvs[j].x, uvs[j].y, 0);
			//	j++;
			//}

			for (int point_index = 0; point_index < mm->P.size(); point_index++)
			{
				pMesh->mVertices[point_index] = aiVector3D(mm->P[point_index].x, mm->P[point_index].y, mm->P[point_index].z);
				pMesh->mNormals[point_index] = aiVector3D(mm->N[point_index].x, mm->N[point_index].y, mm->N[point_index].z);
				pMesh->mColors[0][point_index].r = mm->C[point_index].x / 255.0;
				pMesh->mColors[0][point_index].g = mm->C[point_index].y / 255.0;
				pMesh->mColors[0][point_index].b = mm->C[point_index].z / 255.0;
				pMesh->mColors[0][point_index].a = 1.0f;
				//color.r = mm->C[point_index].x / 255.0;
				//color.g = mm->C[point_index].y / 255.0;
				//color.b = mm->C[point_index].z / 255.0;
				//color.a = 1;

			}

			pMesh->mFaces = new aiFace[mm->F.size()];
			pMesh->mNumFaces = (mm->F.size());

			for (int face_index = 0; face_index < mm->F.size(); face_index++)
			{
				aiFace &face = pMesh->mFaces[face_index];
				face.mIndices = new unsigned int[3];
				face.mNumIndices = 3;

				face.mIndices[0] = (unsigned int)mm->F[face_index].x;
				face.mIndices[1] = (unsigned int)mm->F[face_index].y;
				face.mIndices[2] = (unsigned int)mm->F[face_index].z;
			}

			//mExportFormatDesc->id is "collada"  and mFilePath is "C:/Users/kevin/Desktop/myColladaFile.dae"

			exporter.Export(scene, Format, path);

		}



	private:

		orth::MeshModel *mm;

		// processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
		void processNode(aiNode *node, MeshModel &mm, const aiScene *scene)
		{
			aiMesh* mesh = scene->mMeshes[0];
			processMesh(mesh, mm, scene);

		}

		void processMesh(aiMesh *mesh, MeshModel &mm, const aiScene *scene)
		{
			// data to fill

			orth::PointCloudD P;

			P.resize(mesh->mNumVertices);

			// Walk through each of the mesh's vertices
			for (unsigned int i = 0; i < mesh->mNumVertices; i++)
			{
				P[i].x = mesh->mVertices[i].x;
				P[i].y = mesh->mVertices[i].y;
				P[i].z = mesh->mVertices[i].z;
			}
			mm.P.swap(P);

			///face read
			if (mesh->HasFaces())
			{
				orth::Faces F;
				F.resize(mesh->mNumFaces);
				orth::PointNormal facenormal;
				facenormal.resize(mesh->mNumFaces);
				// now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
				for (unsigned int i = 0; i < mesh->mNumFaces; i++)
				{
					F[i].x = mesh->mFaces[i].mIndices[0];
					F[i].y = mesh->mFaces[i].mIndices[1];
					F[i].z = mesh->mFaces[i].mIndices[2];
					//facenormal[i] = TriangleNormal(P[F[i].x], P[F[i].y], P[F[i].z]);
					//pointsnormal[F[i].x] = facenormal[i];
					//pointsnormal[F[i].y] = facenormal[i];
					//pointsnormal[F[i].z] = facenormal[i];
				}
				mm.F.swap(F);
				mm.NormalUpdate();
			}

			if (mesh->HasNormals())
			{

				orth::PointNormal pointsnormal;
				pointsnormal.resize(mesh->mNumVertices);

				// Walk through each of the mesh's vertices
				for (unsigned int i = 0; i < mesh->mNumVertices; i++)
				{
					pointsnormal[i].x = mesh->mNormals[i].x;
					pointsnormal[i].y = mesh->mNormals[i].y;
					pointsnormal[i].z = mesh->mNormals[i].z;
				}
				mm.N.swap(pointsnormal);
			}


			if (mesh->HasVertexColors(0))
			{
				orth::PointColor pointcolors(mesh->mNumVertices);

				for (unsigned int i = 0; i < mesh->mNumVertices; i++)
				{
					aiColor4D color = mesh->mColors[0][i];
					pointcolors[i].x = color.r*255.0;
					pointcolors[i].y = color.g*255.0;
					pointcolors[i].z = color.b*255.0;

				}
				mm.C.swap(pointcolors);
			}



			return;
		}

	};
}




#endif // !ORTH_IO_H

