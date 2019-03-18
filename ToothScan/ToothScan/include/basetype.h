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
	typedef Point3f Vectorf;
	typedef Point3d Vectord;
	typedef Point3d Normal;
	typedef Point3ui face;
	typedef Point3uc color;
	typedef double curvature;
	typedef short label;


	typedef vector<Vectord> PointCloudD;
	typedef vector<Vectorf> PointCloudF;
	typedef vector<Normal> PointNormal;
	typedef vector<color> PointColor;
	typedef vector<face> Faces;
	typedef vector<label> PointLabel;
	typedef vector<curvature> PointCurs;

#define ACROSS 0
#define COPLANE 1
#define AEDGE 2
#define AVERTEX 3
#define NONINTERSECT 4


	struct Box{
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

		Point3d center;
	};



	inline Normal TriangleNormal(Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		return ((point_b - point_a).cross (point_c - point_a));
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
		double pqdot = a*point.x + b*point.y + c*point.z + d;
		double n = sqrt(a*a + b*b + c*c);
		return pqdot / n;
	}

	//点到平面距离
	inline double Point2PlaneDistance(Point3d &point, Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		Point3d ab = point_b - point_a;
		Point3d ac = point_c - point_a;
		Point3d normal_vector = ab.cross(ac);
		double a = normal_vector.x, b = normal_vector.y, c = normal_vector.z; double d = -a*point_a.x - b*point_a.y - c*point_a.z;
		double pqdot = a*point.x + b*point.y + c*point.z + d;
		double n = sqrt(a*a + b*b + c*c);
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
			if (dis1*dis2<0)
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
			if (dis1*dis2<0)
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
			if (dis1*dis2<0)
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

	//model base class
	class MeshModel
	{
	public:
		MeshModel();
		~MeshModel();
		void resize(int s);
		inline int size() { return size_; }

		PointCloudD P;
		PointNormal N;
		PointColor C;
		Faces F;
		PointLabel L;
		PointCurs Cur;
		PointLabel Selected;
		

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

		void NormalUpdate();


	private:
		int size_ = 0;

		
	};






	// teeth model
	class Teeth:public MeshModel
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

	class MaxillaryTeeth:public MeshModel
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

	class InferiorTeeth:public MeshModel 
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

