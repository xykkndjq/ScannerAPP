#include "basetype.h"


namespace orth
{


	MeshModel::MeshModel()
	{
		//original_rt = (double*)malloc(16*sizeof(double));
		//current_rt = (double*)malloc(16 * sizeof(double));;
	}

	MeshModel::~MeshModel()
	{
		//free(original_rt);
		//original_rt = NULL;
		//free(current_rt);
		//current_rt = NULL;
	}

	bool MeshModel::HaveData()
	{
		return (P.size() > 0);
	}

	void MeshModel::Clear()
	{
		P.clear();
		F.clear();
		C.clear();
		N.clear();
		L.clear();
		Cur.clear();
		Selected.clear();


		Point3d original_center = orth::Point3d(0, 0, 0);
		//double* original_rt;

		Point3d current_center = orth::Point3d(0, 0, 0);
		//double* current_rt;

		motion_path.clear();
		rot_path.clear();

		Box box2;
		box = box2;


	}

	void MeshModel::resize(int s)
	{

		size_ = s;
	}

	bool MeshModel::NormalUpdate()
	{
		N.resize(P.size());
		FN.resize(F.size());
		for (unsigned int i = 0; i < F.size(); i++)
		{
			FN[i] = TriangleNormal(P[F[i].x], P[F[i].y], P[F[i].z]);
		}
		vector<int> point_face_number(P.size());
		for (unsigned int i = 0; i < F.size(); i++)
		{
			N[F[i].x] += FN[i]; point_face_number[F[i].x]++;
			N[F[i].y] += FN[i]; point_face_number[F[i].y]++;
			N[F[i].z] += FN[i]; point_face_number[F[i].z]++;
		}
		for (unsigned int i = 0; i < P.size(); i++)
		{
			N[i] /= (float)point_face_number[i];

			double e = sqrt(N[i].x*N[i].x + N[i].y*N[i].y + N[i].z*N[i].z);

			N[i] /= e;
		}

		return true;
	}

	bool MeshModel::EdgeUpdate(const bool PSTypeChoes)
	{
		if (P.size() == 0)
		{
			return false;
		}

		if (PSTypeChoes)
		{
			P2Edge.clear();
			Edge_P.clear();

			P2Edge.resize(P.size());
			Edge_P.resize(this->F.size() * 3);
			//HalfEdge_Parallel edge_current;

			for (size_t face_index = 0; face_index < F.size(); face_index++)
			{
				vector<Index_ui> point_index(4);
				point_index[0] = F[face_index].x;
				point_index[1] = F[face_index].y;
				point_index[2] = F[face_index].z;
				point_index[3] = F[face_index].x;

				for (size_t point_ = 0; point_ < 3; point_++)
				{
					Edge_P[face_index * 3 + point_].CurrentPoint = point_index[point_];
					Edge_P[face_index * 3 + point_].EndPoint = point_index[point_ + 1];
					Edge_P[face_index * 3 + point_].CurrentFace = face_index;
					if (point_ == 2)
					{
						Edge_P[face_index * 3 + point_].NextEdge = face_index * 3;

					}
					else
					{
						Edge_P[face_index * 3 + point_].NextEdge = face_index * 3 + point_ + 1;

					}

					P2Edge[point_index[point_]].push_back(face_index * 3 + point_);
				}
			}

			for (size_t edge_index = 0; edge_index < Edge_P.size(); edge_index++)
			{
				Edge_P[edge_index].OppoEdge = -1;
				Index_ui current_point_index = Edge_P[edge_index].CurrentPoint;
				Index_ui End_point_index = Edge_P[edge_index].EndPoint;
				for (size_t p2edge_index = 0; p2edge_index < P2Edge[End_point_index].size(); p2edge_index++)
				{

					if (Edge_P[P2Edge[End_point_index][p2edge_index]].EndPoint == current_point_index)
					{
						Edge_P[edge_index].OppoEdge = P2Edge[End_point_index][p2edge_index];
						break;
					}
				}

			}

		}
		else
		{
			//P2Edge.resize(P.size());
			//Edge_S.resize(this->F.size() * 3);
			////HalfEdge_Parallel edge_current;

			//for (size_t face_index = 0; face_index < F.size(); face_index++)
			//{
			//	//vector<Index_ui> point_index(4);
			//	//point_index[0] = F[face_index].x;
			//	//point_index[1] = F[face_index].y;
			//	//point_index[2] = F[face_index].z;
			//	//point_index[3] = F[face_index].x;

			//	for (size_t point_ = 0; point_ < 3; point_++)
			//	{
			//		Edge_P[face_index * 3 + point_].CurrentPoint = point_index[point_];
			//		Edge_P[face_index * 3 + point_].EndPoint = point_index[point_ + 1];
			//		Edge_P[face_index * 3 + point_].CurrentFace = face_index;
			//		if (point_ == 3)
			//		{
			//			Edge_P[face_index * 3 + point_].NextEdge = face_index * 3;

			//		}
			//		else
			//		{
			//			Edge_P[face_index * 3 + point_].NextEdge = face_index * 3 + point_ + 1;

			//		}

			//		P2Edge[point_index[point_]].push_back(face_index * 3 + point_);
			//	}
			//}

			//for (size_t edge_index = 0; edge_index < Edge_P.size(); edge_index++)
			//{
			//	Index_ui current_point_index = Edge_P[edge_index].CurrentPoint;
			//	Index_ui End_point_index = Edge_P[edge_index].EndPoint;
			//	for (size_t p2edge_index = 0; p2edge_index < P2Edge[End_point_index].size(); p2edge_index++)
			//	{

			//		if (Edge_P[P2Edge[End_point_index][p2edge_index]].EndPoint == current_point_index)
			//		{
			//			Edge_P[edge_index].OppoEdge = P2Edge[End_point_index][p2edge_index];
			//			break;
			//		}
			//	}
			//}
		}


	}

	bool MeshModel::ModelSplit(vector<orth::MeshModel> &models)
	{
		if (P2Edge.size() == 0)
		{
			EdgeUpdate(1);
		}

		if (FN.size() == 0)
		{
			this->NormalUpdate();
		}

		L.clear();
		L.resize(P.size(), -1);

		Index_ui label_index = 0;
		Index_ui candidate_scan_index = 0;
		vector<Index_ui> candidate_points_index;

		//从第一个面开始扫描，逐个对点进行标记
		for (size_t face_index = 0; face_index < F.size(); face_index++)
		{
			if (L[F[face_index].x] == -1)
			{
				candidate_points_index.push_back(F[face_index].x); L[F[face_index].x] = label_index; //std::cout << candidate_points_index[candidate_points_index.size()-1] << std::endl;
			}
			else
			{
				continue;
			}


			while (true)
			{
				if (candidate_points_index.size() == candidate_scan_index)
				{
					candidate_points_index.clear();
					candidate_scan_index = 0;
					label_index++;
					break;
				}

				L[candidate_points_index[candidate_scan_index]] = label_index;

				//逐个加入候选点
				for (size_t p2e_index = 0; p2e_index < P2Edge[candidate_points_index[candidate_scan_index]].size(); p2e_index++)
				{
					if (L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] == -1)
					{
						candidate_points_index.push_back(Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint); L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] = label_index; //std::cout << candidate_points_index[candidate_points_index.size() - 1] << std::endl;
					}
				}

				//std::cout <<"									"<< candidate_scan_index << std::endl;
				candidate_scan_index++;

			}

		}


		//将标记点按照索引分解成单独模型
		models.resize(label_index);
		vector<Index_ui> new_point_index(P.size());
		for (size_t point_index = 0; point_index < P.size(); point_index++)
		{
			if (L[point_index]<0 || L[point_index] >= label_index)
			{
				continue;
			}
			models[L[point_index]].P.push_back(P[point_index]);
			models[L[point_index]].N.push_back(N[point_index]);
			models[L[point_index]].C.push_back(C[point_index]);
			models[L[point_index]].L.push_back(L[point_index]);
			//models[L[point_index]].Cur.push_back(Cur[point_index]);
			new_point_index[point_index] = (models[L[point_index]].P.size() - 1);
		}
		for (size_t face_index = 0; face_index < F.size(); face_index++)
		{

			Index_ui l_point1 = F[face_index].x;
			Index_ui l_point2 = F[face_index].y;
			Index_ui l_point3 = F[face_index].z;
			Label l_label = L[l_point1];
			orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
			models[l_label].F.push_back(l_face);
			models[l_label].FN.push_back(FN[face_index]);
		}


		P2Edge.clear();
		Edge_P.clear();

		return true;

	}

	bool MeshModel::SmallModelFilter(const int point_number_threshold)
	{
		if (P2Edge.size() == 0)
		{
			EdgeUpdate(1);
		}

		if (N.size() == 0)
		{
			this->NormalUpdate();
		}

		L.clear();
		L.resize(P.size(), -1);

		vector<Index_ui> label_size;
		Index_ui label_index = 0;
		Index_ui candidate_scan_index = 0;
		vector<Index_ui> candidate_points_index;

		//从第一个面开始扫描，逐个对点进行标记
		for (size_t face_index = 0; face_index < F.size(); face_index++)
		{
			if (L[F[face_index].x] == -1)
			{
				candidate_points_index.push_back(F[face_index].x); L[F[face_index].x] = label_index; //std::cout << candidate_points_index[candidate_points_index.size()-1] << std::endl;
			}
			else
			{
				continue;
			}


			while (true)
			{
				if (candidate_points_index.size() == candidate_scan_index)
				{
					candidate_points_index.clear();
					label_size.push_back(candidate_scan_index);
					candidate_scan_index = 0;
					label_index++;
					break;
				}

				L[candidate_points_index[candidate_scan_index]] = label_index;

				//逐个加入候选点
				for (size_t p2e_index = 0; p2e_index < P2Edge[candidate_points_index[candidate_scan_index]].size(); p2e_index++)
				{
					if (L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] == -1)
					{
						candidate_points_index.push_back(Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint);
						L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] = label_index; //std::cout << candidate_points_index[candidate_points_index.size() - 1] << std::endl;
					}
				}

				//std::cout <<"									"<< candidate_scan_index << std::endl;
				candidate_scan_index++;

			}

		}

		//搜索小于100个点的小mesh并记录其label
		label_size.push_back(candidate_scan_index);
		vector<int> index_filted;
		for (int label_index_ = 0; label_index_ < label_size.size(); label_index_++)
		{
			if (label_size[label_index_]<point_number_threshold)
			{
				index_filted.push_back(label_index_);
			}
		}

		//重新标记需要过滤掉的mesh
		PointLabel label_filter(P.size(), 1);
		for (int point_index = 0; point_index < P.size(); point_index++)
		{
			for (int label_index_ = 0; label_index_ < index_filted.size(); label_index_++)
			{

				if (index_filted[label_index_] == L[point_index])
				{
					label_filter[point_index] = -1;
					break;
				}

			}
		}

		//按照标记对模型进行过滤
		PointCloudD new_points;
		PointNormal new_normals;
		PointColor new_colors;
		vector<Index_ui> new_point_index(P.size());
		for (size_t point_index = 0; point_index < P.size(); point_index++)
		{
			if (label_filter[point_index]<0)
			{
				continue;
			}

			new_points.push_back(P[point_index]);
			new_normals.push_back(N[point_index]);
			if (C.size() > 0)
			{
				new_colors.push_back(C[point_index]);
			}
			new_point_index[point_index] = (new_points.size() - 1);
		}
		Faces new_faces;
		FacesNormal new_face_normal;
		for (size_t face_index = 0; face_index < F.size(); face_index++)
		{

			Index_ui l_point1 = F[face_index].x;
			Index_ui l_point2 = F[face_index].y;
			Index_ui l_point3 = F[face_index].z;
			orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
			new_faces.push_back(l_face);
			new_face_normal.push_back(FN[face_index]);
		}

		//新数据替换
		F.swap(new_faces);
		P.swap(new_points);
		N.swap(new_normals);
		if (C.size()>0)
		{
			C.swap(new_colors);
		}
		FN.swap(new_face_normal);
		L.clear();

		P2Edge.clear();
		Edge_P.clear();

		return true;
	}

	void MeshModel::NormalSmooth(const int iteration_times)
	{
		if (P2Edge.size() == 0)
		{
			EdgeUpdate(1);
		}

		for (int iter_times = 0; iter_times < iteration_times; iter_times++)
		{
			orth::PointNormal new_normal;
			for (int point_index = 0; point_index < P.size(); point_index++)
			{
				orth::Normal L_normal;
				for (int neighbour_index = 0; neighbour_index < P2Edge[point_index].size(); neighbour_index++)
				{
					L_normal += N[Edge_P[P2Edge[point_index][neighbour_index]].EndPoint];
				}
				L_normal.normalize();
				new_normal.push_back(L_normal);
			}
			N.swap(new_normal);
			new_normal.clear();
		}

		P2Edge.clear();
		Edge_P.clear();
	}

	void MeshModel::ModelSample(const int rate)
	{
		int jump_number = (int)(P.size() / rate);
		if (rate > P.size())
		{
			jump_number = 1;
		}

		for (int point_index = 0; point_index < P.size(); point_index += jump_number)
		{
			S.push_back(point_index);
		}
	}

	//void MeshModel::DateDownload(Eigen::MatrixXd &Verts, Eigen::MatrixXi &Faces)
	//{
	//	if (P.size()==0)
	//	{
	//		std::cout << " Wrong PointCloud Size !!" << std::endl;
	//		getchar();
	//		return;
	//	}
	//	if (F.size() == 0)
	//	{
	//		std::cout << " Wrong Faces Size !!" << std::endl;
	//		getchar();
	//		return;
	//	}
	//	Verts = Eigen::MatrixXd::Zero(P.size(), 3);
	//	for (size_t point_index = 0; point_index < P.size(); point_index++)
	//	{
	//		Verts(point_index, 0) = P[point_index].x;
	//		Verts(point_index, 1) = P[point_index].y;
	//		Verts(point_index, 2) = P[point_index].z;
	//	}
	//	Faces = Eigen::MatrixXi::Zero(F.size(), 3);
	//	for (size_t face_index = 0; face_index < F.size(); face_index++)
	//	{
	//		Faces(face_index, 0) = F[face_index].x;
	//		Faces(face_index, 1) = F[face_index].y;
	//		Faces(face_index, 2) = F[face_index].z;
	//	}
	//	std::cout << " DownLoad Done !!" << std::endl;
	//}

	void Teeth::Rotation(double *rt_matrix)
	{
		//旋转顶点
		if (P.size())
		{

			for (int point_index = 0; point_index < P.size(); point_index++)
			{
				PointRot(rt_matrix, &P[point_index]);
			}
		}

		//旋转法向
		if (N.size())
		{

			for (int normal_index = 0; normal_index < N.size(); normal_index++)
			{
				PointRot(rt_matrix, &N[normal_index]);
			}
		}

		//旋转中心点


		//if (motion_path.size())
		//{
		//	PointRot(rt_matrix, &current_center);
		//	motion_path.push_back(current_center);
		//	for (size_t i = 0; i < 16; i++)
		//	{
		//		current_rt[i] = rt_matrix[i];
		//	}
		//}
		//else
		//{

		//}

		////旋转包围盒
		//if (box.u_0.x)
		//{

		//	for (int normal_index = 0; normal_index < N.size(); normal_index++)
		//	{
		//		PointRot(rt_matrix, &N[normal_index]);
		//	}
		//}

		//旋转尖点

		if (cusp.size() > 0)
		{
			for (int cusp_index = 0; cusp_index < cusp.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp[cusp_index]);
			}
		}
		if (cusp_b.size() > 0)
		{
			for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp_b[cusp_index]);
			}
		}
		if (cusp_l.size() > 0)
		{
			for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp_l[cusp_index]);
			}
		}
		if (incisal_edges.size() > 0)
		{
			for (int cusp_index = 0; cusp_index < incisal_edges.size(); cusp_index++)
			{
				PointRot(rt_matrix, &incisal_edges[cusp_index]);
			}
		}

	}

	void MeshModel::PointRot(double *rt_matrix, Point3d *point)
	{
		double x = point->x, y = point->y, z = point->z;
		double v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z + rt_matrix[3];
		double v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z + rt_matrix[7];
		double v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z + rt_matrix[11];
		if (v_c_x == NULL || v_c_y == NULL || v_c_z == NULL)
		{
			*point = Point3d(0, 0, 0);
			return;
		}
		point->x = v_c_x;
		point->y = v_c_y;
		point->z = v_c_z;

	}

	void MeshModel::PointRot(double *rt_matrix, Point3f *point)
	{
		double x = point->x, y = point->y, z = point->z;
		double v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z + rt_matrix[3];
		double v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z + rt_matrix[7];
		double v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z + rt_matrix[11];
		point->x = v_c_x;
		point->y = v_c_y;
		point->z = v_c_z;
	}

	Teeth::Teeth()
	{
	}

	Teeth::~Teeth()
	{
	}

	MaxillaryTeeth::MaxillaryTeeth()
	{
	}

	MaxillaryTeeth::~MaxillaryTeeth()
	{
	}

	InferiorTeeth::InferiorTeeth()
	{
	}

	InferiorTeeth::~InferiorTeeth()
	{
	}

}

