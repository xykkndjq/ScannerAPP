#ifndef ORTH_BASETYPE_H
#define ORTH_BASETYPE_H

#include <iostream>
#include <vector>
#include "Point.hpp"
#include <iomanip> 

using std::vector;

// Orthodontics library
namespace orth
{
	//***************************************//
	//			    基础数据类型              //
	//***************************************//

	typedef Point3f Vectorf;
	typedef Point3d Vectord;
	typedef Point3d Normal;
	typedef Point3ui Face;
	typedef Point3uc Color;
	typedef double Curvature;
	typedef short Label;
	typedef long Index_l;
	typedef unsigned int Index_ui;
	typedef vector<Index_ui> Point2Edge;

	struct HalfEdge_Parallel
	{
		Index_ui CurrentPoint;
		Index_ui EndPoint;
		Index_ui OppoEdge;
		Index_ui CurrentFace;
		Index_ui NextEdge;
		bool SearchLabel;
	};

	struct HalfEdge_Serial
	{
		Vectord* CurrentPoint;
		Vectord* EndPoint;
		HalfEdge_Serial* OppoEdge;
		Face* CurrentFace;
		HalfEdge_Serial* NextEdge;
		bool SearchLabel;
	};

	//基础图形类型
	typedef vector<Vectord> PointCloudD;
	typedef vector<Vectorf> PointCloudF;
	typedef vector<Normal> PointNormal;
	typedef vector<Color> PointColor;
	typedef vector<Face> Faces;
	typedef vector<Normal> FacesNormal;
	typedef vector<Label> PointLabel;
	typedef vector<Index_ui> SamplePoints;
	typedef vector<Curvature> PointCurs;
	typedef vector<HalfEdge_Parallel> HalfEdgeCloud_P;
	typedef vector<HalfEdge_Serial> HalfEdgeCloud_S;
	typedef vector<Point2Edge> HalfPointCloud_P;

	//外接包围盒
	struct Box {
		Vectorf u_0;
		Vectorf u_1;
		Vectorf u_2;
		Vectorf u_3;
		Vectorf d_0;
		Vectorf d_1;
		Vectorf d_2;
		Vectorf d_3;
	};

	struct Plane
	{
		double A;
		double B;
		double C;
		double D;

		Point3d Center;
	};

#define ACROSS 0
#define COPLANE 1
#define AEDGE 2
#define AVERTEX 3
#define NONINTERSECT 4


	//class H_type
	//{
	//public:
	//	H_type();
	//	~H_type();

	//private:

	//};


	//struct H_vert
	//{

	//	orth::Point3d coord;

	//	H_edge* edge;  // one of the half-edges emantating from the vertex

	//};

	//struct H_face
	//{

	//	H_edge* edge;  // one of the half-edges bordering the face

	//};

	//struct H_edge
	//{

	//	H_vert* vert;   // vertex at the end of the half-edge
	//	H_edge* pair;   // oppositely oriented adjacent half-edge 
	//	H_face* face;   // face the half-edge borders
	//	H_edge* next;   // next half-edge around the face

	//};

	//***************************************//
	//			      基础计算               //
	//**************************************//

	//三角形法相计算
	inline Normal TriangleNormal(Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		return ((point_b - point_a).cross(point_c - point_a));
	}

	//点到点距离
	inline double Point2PointDistance(Point3d &p1, Point3d &p2)
	{
		//double dis = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z);
		double dis = (p1 - p2).dot(p1 - p2);
		return sqrt(dis);
	}

	//点到平面距离
	inline double Point2PlaneDistance(Point3d &point, Plane &target_plane)
	{
		double a = target_plane.A, b = target_plane.B, c = target_plane.C, d = target_plane.D;
		double pqdot = a * point.x + b * point.y + c * point.z + d;
		double n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}

	//点到平面距离
	inline double Point2PlaneDistance(Point3d &point, Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		Point3d ab = point_b - point_a;
		Point3d ac = point_c - point_a;
		Point3d normal_vector = ab.cross(ac);
		double a = normal_vector.x, b = normal_vector.y, c = normal_vector.z; double d = -a * point_a.x - b * point_a.y - c * point_a.z;
		double pqdot = a * point.x + b * point.y + c * point.z + d;
		double n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}

	//plucker坐标计算
	inline void plucker(Point3d &a, Point3d &b, double* l)
	{
		l[0] = a.x*b.y - b.x*a.y;
		l[1] = a.x*b.z - b.x*a.z;
		l[2] = a.x - b.x;
		l[3] = a.y*b.z - b.y*a.z;
		l[4] = a.z - b.z;
		l[5] = b.y - a.y;
	}

	//plucker方向计算
	inline double sideOp(double *a, double *b)
	{
		double res = a[0] * b[4] + a[1] * b[5] + a[2] * b[3] + a[3] * b[2] + a[4] * b[0] + a[5] * b[1];
		return res;
	}

	//线面相交判断
	inline int LineFaceIntersect(Point3d &l1, Point3d &l2, Point3d &a, Point3d &b, Point3d &c)
	{
		double e1[6] = { 0 }, e2[6] = { 0 }, e3[6] = { 0 }, L[6] = { 0 };
		plucker(b, a, e1);
		plucker(c, b, e2);
		plucker(a, c, e3);
		plucker(l1, l2, L);

		double s1 = sideOp(L, e1);
		double s2 = sideOp(L, e2);
		double s3 = sideOp(L, e3);

		//cout << s1<<" --- " << s2 << " --- " << s3 << endl;

		if (s1 == 0 && s2 == 0 && s3 == 0)
		{
			return COPLANE;
		}
		else if ((s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0))
		{
			return ACROSS;
		}
		else if ((s1 == 0 && s2*s3 > 0) || (s2 == 0 && s1*s3 > 0) || (s3 == 0 && s1*s2 > 0))
		{
			return AEDGE;
		}
		else if ((s1 == 0 && s2 == 0) || (s1 == 0 && s3 == 0) || (s3 == 0 && s2 == 0))
		{
			return AVERTEX;
		}
		else
		{
			return NONINTERSECT;
		}
	}

	//面相交
	inline bool FaceIntersect(Point3d &a1, Point3d &b1, Point3d &c1, Point3d &a2, Point3d &b2, Point3d &c2)
	{

		if (LineFaceIntersect(a1, b1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(b1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}

		}
		if (LineFaceIntersect(a1, c1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		if (LineFaceIntersect(b1, c1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(b1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return false;
	}


	//***************************************//
	//			    基本模型类型              //
	//***************************************//

	//model base class
	class MeshModel
	{
	public:
		MeshModel();
		~MeshModel();
		void resize(int s);

		void Clear();

		inline int size() { return size_; }

		PointCloudD P;
		PointNormal N;
		PointColor C;
		Faces F;
		FacesNormal FN;
		PointLabel L;
		SamplePoints S;
		PointCurs Cur;
		PointLabel Selected;

		HalfEdgeCloud_P Edge_P;
		HalfEdgeCloud_S Edge_S;
		HalfPointCloud_P P2Edge;

		Point3d original_center;
		double* original_rt;

		Point3d current_center;
		double* current_rt;

		vector<Point3d> motion_path;
		vector<double*> rot_path;

		Box box;

		bool HaveData();

		///void DateDownload(Eigen::MatrixXd &Verts, Eigen::MatrixXi &Faces);

		void PointRot(double *rt_matrix, Point3d *point);

		void PointRot(double *rt_matrix, Point3f *point);

		bool NormalUpdate();

		///PSTypeChoes : 计算类型串行或并行
		bool EdgeUpdate(const bool PSTypeChoes = 1);

		///模型分割函数，对当前模型进行分解，不连续的mesh被分为独立的个体并用Label进行标记
		bool ModelSplit(vector<orth::MeshModel> &models);

		///计算采样点
		///rate : 采样率，最终采样后剩余数量
		void ModelSample(const int rate);
	private:
		int size_ = 0;


	};

	//***************************************//
	//			    模型基本运算              //
	//***************************************//

	//最邻近点查询
	inline bool NearestPointSearch(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int Qctree_depth, vector<unsigned int> &query_index, vector<double> &nearest_distance)
	{
		if (mm_query->P.size() == 0)
		{
			return false;
		}
		query_index.resize(mm_query->P.size());
		nearest_distance.resize(mm_query->P.size());

		//求外接矩形
		double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000, z_min = 1000, z_max = -1000;
		for (size_t point_index = 0; point_index < mm_target->P.size(); point_index++)
		{
			double x = mm_target->P[point_index].x;
			double y = mm_target->P[point_index].y;
			double z = mm_target->P[point_index].z;
			if (x < x_min)
			{
				x_min = x;
			}
			if (x > x_max)
			{
				x_max = x;
			}
			if (y < y_min)
			{
				y_min = y;
			}
			if (y > y_max)
			{
				y_max = y;
			}
			if (z < z_min)
			{
				z_min = z;
			}
			if (z > z_max)
			{
				z_max = z;
			}
		}

		//cout << " x_min = " << x_min << " y_min = " << y_min << " z_min = " << z_min << " x_max = " << x_max << " y_max = " << y_max << " z_max = " << z_max << endl;

		x_min -= 5; y_min -= 5; z_min -= 5;
		x_max += 5; y_max += 5; z_max += 5;

		double size = (pow(2, Qctree_depth));

		//求目标点云的key
		vector<unsigned __int32> target_key(mm_target->P.size());

		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			//for (size_t tree_depth = 1; tree_depth <= Qctree_depth; ++tree_depth)
			//{
			//	double size = (pow(2, tree_depth - 1));
			//	double cell_size_x = (x_max - x_min) / (size);
			//	double cell_size_y = (y_max - y_min) / (size);
			//	double cell_size_z = (z_max - z_min) / (size);

			//	double x = mm_target->P[points_index].x - x_min;
			//	double y = mm_target->P[points_index].y - y_min;
			//	double z = mm_target->P[points_index].z - z_min;

			//double temp_x = x/cell_size_x;
			//double temp_y = y/cell_size_y;
			//double temp_z = z/cell_size_z;

			//double temp2_x = temp_x - floor(temp_x);
			//double temp2_y = temp_y - floor(temp_y);
			//double temp2_z = temp_z - floor(temp_z);

			//if (temp2_x>=0.5)
			//{
			//	key += 4;
			//}
			//if (temp2_y >= 0.5)
			//{
			//	key += 2;
			//}
			//if (temp2_z >= 0.5)
			//{
			//	key += 1;
			//}

			//cout << std::bitset<32>(key)  << endl;

			//if (tree_depth!= Qctree_depth)
			//{
			//	key = key << 3;
			//}

			//}


			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_target->P[points_index].x - x_min;
			double y = mm_target->P[points_index].y - y_min;
			double z = mm_target->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			target_key[points_index] = key;
		}

		//按照node来统计点
		vector<vector<unsigned __int32>> target_key_sort(size*size*size);
		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			target_key_sort[target_key[points_index]].push_back(points_index);
		}

		//求查询点云的key
		vector<unsigned int> Qctree_key(mm_query->P.size());

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			//for (size_t tree_depth = 1; tree_depth <= Qctree_depth; ++tree_depth)
			//{
			//	double size = (pow(2, tree_depth - 1));
			//	double cell_size_x = (x_max - x_min) / (size);
			//	double cell_size_y = (y_max - y_min) / (size);
			//	double cell_size_z = (z_max - z_min) / (size);

			//	double x = mm_query->P[points_index].x - x_min;
			//	double y = mm_query->P[points_index].y - y_min;
			//	double z = mm_query->P[points_index].z - z_min;

			//	double temp_x = x / cell_size_x;
			//	double temp_y = y / cell_size_y;
			//	double temp_z = z / cell_size_z;

			//	double temp2_x = temp_x - floor(temp_x);
			//	double temp2_y = temp_y - floor(temp_y);
			//	double temp2_z = temp_z - floor(temp_z);

			//	if (temp2_x >= 0.5)
			//	{
			//		key += 4;
			//	}
			//	if (temp2_y >= 0.5)
			//	{
			//		key += 2;
			//	}
			//	if (temp2_z >= 0.5)
			//	{
			//		key += 1;
			//	}

			//	cout << std::bitset<32>(key) << endl;

			//	if (tree_depth != Qctree_depth)
			//	{
			//		key = key << 3;
			//	}

			//}

			if (mm_query->P[points_index].x<x_min || mm_query->P[points_index].x>x_max || mm_query->P[points_index].y<y_min || mm_query->P[points_index].y>y_max || mm_query->P[points_index].z<z_min || mm_query->P[points_index].z>z_max)
			{
				nearest_distance[points_index] = -1;
				continue;
			}

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;



			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			Qctree_key[points_index] = key;
		}

		//求查询点云的最近点

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			if (nearest_distance[points_index] == -1)
			{
				continue;
			}

			orth::Point3d query_point = mm_query->P[points_index];
			vector<unsigned __int32> searched_points;
			int break_label = 2;

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			for (size_t cycle_index = 0; cycle_index < Qctree_depth - 1; ++cycle_index)
			{
				if (searched_points.size() > 0)
				{
					break_label--;
				}
				if (!break_label)
				{
					break;
				}
				for (size_t x_index = temp_x - cycle_index; x_index <= temp_x + cycle_index; ++x_index)
				{
					for (size_t y_index = temp_y - cycle_index; y_index <= temp_y + cycle_index; ++y_index)
					{

						for (size_t z_index = temp_z - cycle_index; z_index <= temp_z + cycle_index; ++z_index)
						{
							if (target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size() > 0)
							{

								for (size_t point_index = 0; point_index < target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size(); point_index++)
								{
									searched_points.push_back(target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index][point_index]);
								}

							}
						}
					}
				}

			}

			nearest_distance[points_index] = 100;
			for (size_t search_index = 0; search_index < searched_points.size(); search_index++)
			{
				double dis = orth::Point2PointDistance(query_point, mm_target->P[searched_points[search_index]]);
				if (dis < nearest_distance[points_index])
				{
					query_index[points_index] = searched_points[search_index];
					nearest_distance[points_index] = dis;
				}
			}
		}

		return true;
	}

	//模型合并
	inline bool MergeModels(vector<orth::MeshModel> &mm_group_input,orth::MeshModel &mm_output)
	{
		if (mm_group_input.size()<=0)
		{
			return false;
		}

		mm_output.Clear();

		for (size_t group_index = 0; group_index < mm_group_input.size(); group_index++)
		{
			vector<Index_ui> new_point_index(mm_group_input[group_index].P.size());
			for (size_t point_index = 0; point_index < mm_group_input[group_index].P.size(); point_index++)
			{

				mm_output.P.push_back(mm_group_input[group_index].P[point_index]);
				mm_output.N.push_back(mm_group_input[group_index].N[point_index]);
				mm_output.C.push_back(mm_group_input[group_index].C[point_index]);
				mm_output.L.push_back(mm_group_input[group_index].L[point_index]);
				//models[L[point_index]].Cur.push_back(Cur[point_index]);
				new_point_index[point_index] = (mm_output.P.size() - 1);
			}
			for (size_t face_index = 0; face_index < (mm_group_input[group_index].F.size()); face_index++)
			{

				Index_ui l_point1 = mm_group_input[group_index].F[face_index].x;
				Index_ui l_point2 = mm_group_input[group_index].F[face_index].y;
				Index_ui l_point3 = mm_group_input[group_index].F[face_index].z;
				orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
				mm_output.F.push_back(l_face);
				mm_output.FN.push_back(mm_group_input[group_index].FN[face_index]);
			}
		}

		return true;
	}




	// teeth model
	class Teeth :public MeshModel
	{
	public:
		Teeth();
		~Teeth();

		// cusp of a teeth
		vector<Point3d> cusp;

		// b_cusp of a teeth
		vector<Point3d> cusp_b;

		// l_cusp of a teeth
		vector<Point3d> cusp_l;

		// ridge of a teeth
		vector<Point3d> ridge;

		// groove of a teeth
		vector<Point3d> groove;

		// groove of a teeth
		vector<Point3d> incisal_edges;

		//buccolingual inc
		double premolar_theta;

		//buccolingual inc
		double molar_theta1;
		double molar_theta2;

		void Rotation(double *rt_matrix);
		//
		//	add feature of the teeth
		//
		//

	private:

	};

	class MaxillaryTeeth :public MeshModel
	{
	public:
		MaxillaryTeeth();
		~MaxillaryTeeth();

		vector<Point3d> arch;

		Plane occlusion;

		vector<Teeth> teeths;

		vector<Plane> division_plane;
		//
		//	add feature of the teeth
		//
		//

	private:

	};

	class InferiorTeeth :public MeshModel
	{
	public:
		InferiorTeeth();
		~InferiorTeeth();

		vector<Point3d> arch;

		Plane occlusion;

		vector<Teeth> teeths;

		vector<Plane> division_plane;

		//
		//	add feature of the teeth
		//
		//

	private:

	};


}

#endif // !BASETYPE_H

