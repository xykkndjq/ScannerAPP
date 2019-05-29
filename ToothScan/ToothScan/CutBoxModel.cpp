#include "CutBoxModel.h"

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_COLOR_ATTRIBUTE 1
CCutBoxObject::CCutBoxObject(shared_ptr<QOpenGLShaderProgram> v_Program,
	shared_ptr<QOpenGLBuffer> v_Vbo) :BaseModel(v_Program, v_Vbo),
	m_bkGroundColor(0.65f, 0.79f, 1.0f, 0.7f)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->bindAttributeLocation("aColor", PROGRAM_COLOR_ATTRIBUTE);
	m_program->link();

	x_color = QVector4D(1.0f, 0.35f, 0.35f, 0.8f);
	y_color = QVector4D(0.35f, 1.0f, 0.35f, 0.8f);
	z_color = QVector4D(0.35f, 0.35f, 1.0f, 0.8f);
	press_color_x_color = QVector4D(1.0f, 0.15f, 0.15f, 1.0f);
	press_color_y_color = QVector4D(0.15f, 1.0f, 0.15f, 1.0f);
	press_color_z_color = QVector4D(0.15f, 0.15f, 1.0f, 1.0f);

	ColorUpdate();

	v_zoom_x = 1; v_zoom_y = 1; v_zoom_z = 1;
	v_trans_x = 0; v_trans_y = 0; v_trans_z = 0;
	movement_type = -1;
	face_index = -1;
	move_direction = -1;

}


CCutBoxObject::CCutBoxObject(const char * v_vsFile, const char * v_fsfile, QObject *parent)
	:BaseModel(v_vsFile, v_fsfile, parent)
	, m_bkGroundColor(0.65f, 0.79f, 1.0f, 0.7f)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->bindAttributeLocation("aColor", PROGRAM_COLOR_ATTRIBUTE);
	m_program->link();

	x_color = QVector4D(1.0f, 0.35f, 0.35f, 0.8f);
	y_color = QVector4D(0.35f, 1.0f, 0.35f, 0.8f);
	z_color = QVector4D(0.35f, 0.35f, 1.0f, 0.8f);
	press_color_x_color = QVector4D(1.0f, 0.15f, 0.15f, 1.0f);
	press_color_y_color = QVector4D(0.15f, 1.0f, 0.15f, 1.0f);
	press_color_z_color = QVector4D(0.15f, 0.15f, 1.0f, 1.0f);

	ColorUpdate();

	v_zoom_x = 1; v_zoom_y = 1; v_zoom_z = 1;
	v_trans_x = 0; v_trans_y = 0; v_trans_z = 0;
	movement_type = -1;
	face_index = -1;
	move_direction = -1;
}

CCutBoxObject::~CCutBoxObject()
{
}

void CCutBoxObject::ChosePoints2(const float point_x, const float point_y, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm)
{
	vector<orth::Point3d> point_rander(mm.P.size());
	if (mm.N.size() == 0)
	{
		mm.NormalUpdate();
	}

	//if (mm.Selected.size() == 0)
	//{
	//	mm.Selected.resize(mm.P.size());
	//}

	//float min_x = max(point_x - 10.0, 0.0);
	//float min_y = max(point_y - 10.0, 0.0);
	//float max_x = min(point_x + 10.0, (double)screen_width);
	//float max_y = min(point_y + 10.0, (double)screen_height);


	float min_x = point_x - 3.0;
	float min_y = point_y - 3.0;
	float max_x = point_x + 3.0;
	float max_y = point_y + 3.0;

	min_x /= screen_width / 2;
	max_x /= screen_width / 2;
	min_y /= screen_height / 2;
	max_y /= screen_height / 2;

	mm.L.clear();
	mm.L.resize(mm.P.size());
	orth::PointLabel Selected(mm.P.size());

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	//model_matrix = model_matrix.t();
	model_matrix = cv::Mat::eye(4, 4, CV_32FC1);

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.t();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.t();

	cv::Mat final_matrix = projection_matrix*view_matrix*model_matrix;


	int point_number_ = 0;
#pragma omp parallel for num_threads(4)
	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		if (Selected[point_index])
		{
			continue;
		}

		orth::Point3d point_ = mm.P[point_index];
		double x_ = final_matrix.at<float>(0, 0)*point_.x + final_matrix.at<float>(0, 1)*point_.y + final_matrix.at<float>(0, 2)*point_.z + final_matrix.at<float>(0, 3);
		double y_ = final_matrix.at<float>(1, 0)*point_.x + final_matrix.at<float>(1, 1)*point_.y + final_matrix.at<float>(1, 2)*point_.z + final_matrix.at<float>(1, 3);
		double z_ = final_matrix.at<float>(2, 0)*point_.x + final_matrix.at<float>(2, 1)*point_.y + final_matrix.at<float>(2, 2)*point_.z + final_matrix.at<float>(2, 3);
		double w_ = final_matrix.at<float>(3, 0)*point_.x + final_matrix.at<float>(3, 1)*point_.y + final_matrix.at<float>(3, 2)*point_.z + final_matrix.at<float>(3, 3);
		//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
		x_ /= w_;
		y_ /= w_;
		z_ /= w_;

		point_rander[point_index].x = x_;
		point_rander[point_index].y = y_;
		point_rander[point_index].z = z_;

		//int u = 1000 + x_*1000.0;
		//int v = 1000 + y_*1000.0;
		//if (u>0&&u<1999&&v>0&&v<1999)
		//{
		//	depthimage.at<float>(v, u) = z_;
		//}
		//cout << u << "; " << v << "; " << z_ << endl;
		if (x_>min_x&&x_<max_x&&y_>min_y&&y_<max_y)
			//if (x_>0&&z_>0&&y_<0)
		{
			mm.L[point_index] = 1;
			Selected[point_index] = 1;
			point_number_++;
			continue;
		}

	}
	cout << "point_number_ = " << point_number_ << endl;



	int selected_point_index = -1;
	double min_z = 1000;
	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		if (mm.L[point_index] == 1)
		{
			if (point_rander[point_index].z<min_z)
			{
				min_z = point_rander[point_index].z;
				selected_point_index = point_index;
			}

		}
	}


	if (mm.P2Edge.size() == 0)
	{
		mm.EdgeUpdate(1);
	}
	mm.L.clear();
	mm.L.resize(mm.P.size(), 0);

	vector<int> label_size;
	int label_index = 1;
	int candidate_scan_index = 0;
	vector<int> candidate_points_index;
	//从第一个面开始扫描，逐个对点进行标记
	//for (size_t face_index = 0; face_index < F.size(); face_index++)
	{

		if (mm.L[selected_point_index] == 0)
		{
			candidate_points_index.push_back(selected_point_index); mm.L[selected_point_index] = label_index; //std::cout << candidate_points_index[candidate_points_index.size()-1] << std::endl;
		}
		else
		{
			return;
		}


		//while (true)
		for (int while_index = 0; while_index < 5000; while_index++)
		{
			if (candidate_points_index.size() == candidate_scan_index)
			{
				candidate_points_index.clear();
				label_size.push_back(candidate_scan_index);
				candidate_scan_index = 0;
				label_index++;
				break;
			}

			mm.L[candidate_points_index[candidate_scan_index]] = label_index;
			//逐个加入候选点
			for (size_t p2e_index = 0; p2e_index < mm.P2Edge[candidate_points_index[candidate_scan_index]].size(); p2e_index++)
			{
				if (mm.L[mm.Edge_P[mm.P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] == 0)
				{
					candidate_points_index.push_back(mm.Edge_P[mm.P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint);
					mm.L[mm.Edge_P[mm.P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] = label_index; //std::cout << candidate_points_index[candidate_points_index.size() - 1] << std::endl;
				}
			}

			//std::cout <<"									"<< candidate_scan_index << std::endl;
			candidate_scan_index++;

		}

	}

	orth::PointCloudD label_points;
	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		if (mm.L[point_index] == 1)
		{
			label_points.push_back(mm.P[point_index]);
		}
	}

	box_center = orth::Point3d(0, 0, 0);
	cv::Mat A(label_points.size(), 3, CV_32FC1);
	for (int point_index = 0; point_index < label_points.size(); point_index++)
	{
		box_center += label_points[point_index];
	}
	box_center /= (double)label_points.size();
	cout << "BOX center = " << box_center << endl;
	for (int point_index = 0; point_index < label_points.size(); point_index++)
	{
		orth::Point3d p = label_points[point_index] - box_center;
		//for (int i = 0; i < 3; i++)
		{
			A.at<float>(point_index, 0) = p.x;
			A.at<float>(point_index, 1) = p.y;
			A.at<float>(point_index, 2) = p.z;
		}
	}
	cv::Mat W, U, V;
	cv::SVD::compute(A, W, U, V);

	direct_y = orth::Point3d(V.at<float>(0, 0), V.at<float>(1, 0), V.at<float>(2, 0));
	direct_z = orth::Point3d(V.at<float>(0, 1), V.at<float>(1, 1), V.at<float>(2, 1));
	direct_x = orth::Point3d(V.at<float>(0, 2), V.at<float>(1, 2), V.at<float>(2, 2));
	double box_min_x = 1000.0, box_min_y = 1000.0, box_min_z = 1000.0, box_max_x = -1000.0, box_max_y = -1000.0, box_max_z = -1000.0;

	for (int point_index = 0; point_index < label_points.size(); point_index++)
	{

		double dis_x = direct_x.dot(label_points[point_index] - box_center);
		double dis_y = direct_y.dot(label_points[point_index] - box_center);
		double dis_z = direct_z.dot(label_points[point_index] - box_center);
		if (dis_x<box_min_x)
		{
			box_min_x = dis_x;
		}
		if (dis_x>box_max_x)
		{
			box_max_x = dis_x;
		}

		if (dis_y<box_min_y)
		{
			box_min_y = dis_y;
		}
		if (dis_y>box_max_y)
		{
			box_max_y = dis_y;
		}

		if (dis_z<box_min_z)
		{
			box_min_z = dis_z;
		}
		if (dis_z>box_max_z)
		{
			box_max_z = dis_z;
		}
	}

	min_x_dir = direct_x*box_min_x, max_x_dir = direct_x*box_max_x,
		min_y_dir = direct_y*box_min_y, max_y_dir = direct_y*box_max_y,
		min_z_dir = direct_z*box_min_z, max_z_dir = direct_z*box_max_z;




	//double box_min_x = 1000.0, box_min_y = 1000.0, box_min_z = 1000.0, box_max_x = -1000.0, box_max_y = -1000.0, box_max_z = -1000.0;
	//int label_number = 0;
	//for (int point_index = 0; point_index < mm.P.size(); point_index++)
	//{
	//	if (mm.L[point_index] == 1)
	//	{
	//		orth::Point3d point = mm.P[point_index];
	//		if (point.x<box_min_x)
	//		{
	//			box_min_x = point.x;
	//		}
	//		if (point.x>box_max_x)
	//		{
	//			box_max_x = point.x;
	//		}
	//		if (point.y<box_min_y)
	//		{
	//			box_min_y = point.y;
	//		}
	//		if (point.y>box_max_y)
	//		{
	//			box_max_y = point.y;
	//		}
	//		if (point.z<box_min_z)
	//		{
	//			box_min_z = point.z;
	//		}
	//		if (point.z>box_max_z)
	//		{
	//			box_max_z = point.z;
	//		}
	//		label_number++;
	//	}
	//}

	////if (label_number > 0)
	////{
	////	mm2.P.resize(2);
	////	mm2.P[0].x = box_min_x;
	////	mm2.P[0].y = box_min_y;
	////	mm2.P[0].z = box_min_z;
	////	mm2.P[1].x = box_max_x;
	////	mm2.P[1].y = box_max_y;
	////	mm2.P[1].z = box_max_z;
	////}


	if (label_points.size()>0)
	{
		new_box.F.clear();
		new_box.F.resize(12);
		new_box.P.clear();
		new_box.P.resize(8);
		vector<orth::Point3d> box_vertex(8);
		new_box.P[0] = box_center + min_x_dir + min_y_dir + min_z_dir;
		new_box.P[1] = box_center + max_x_dir + min_y_dir + min_z_dir;
		new_box.P[2] = box_center + max_x_dir + min_y_dir + max_z_dir;
		new_box.P[3] = box_center + min_x_dir + min_y_dir + max_z_dir;
		new_box.P[4] = box_center + min_x_dir + max_y_dir + min_z_dir;
		new_box.P[5] = box_center + max_x_dir + max_y_dir + min_z_dir;
		new_box.P[6] = box_center + max_x_dir + max_y_dir + max_z_dir;
		new_box.P[7] = box_center + min_x_dir + max_y_dir + max_z_dir;

		//box_vertex[0].x = box_min_x; box_vertex[0].y = box_min_y; box_vertex[0].z = box_min_z; new_box.P.push_back(box_vertex[0]);
		//box_vertex[1].x = box_max_x; box_vertex[1].y = box_min_y; box_vertex[1].z = box_min_z; new_box.P.push_back(box_vertex[1]);
		//box_vertex[2].x = box_max_x; box_vertex[2].y = box_min_y; box_vertex[2].z = box_max_z; new_box.P.push_back(box_vertex[2]);
		//box_vertex[3].x = box_min_x; box_vertex[3].y = box_min_y; box_vertex[3].z = box_max_z; new_box.P.push_back(box_vertex[3]);
		//box_vertex[4].x = box_min_x; box_vertex[4].y = box_max_y; box_vertex[4].z = box_min_z; new_box.P.push_back(box_vertex[4]);
		//box_vertex[5].x = box_max_x; box_vertex[5].y = box_max_y; box_vertex[5].z = box_min_z; new_box.P.push_back(box_vertex[5]);
		//box_vertex[6].x = box_max_x; box_vertex[6].y = box_max_y; box_vertex[6].z = box_max_z; new_box.P.push_back(box_vertex[6]);
		//box_vertex[7].x = box_min_x; box_vertex[7].y = box_max_y; box_vertex[7].z = box_max_z; new_box.P.push_back(box_vertex[7]);

		//new_box.F[0].x = 0; new_box.F[0].y = 4; new_box.F[0].z = 1;
		//new_box.F[1].x = 1; new_box.F[1].y = 4; new_box.F[1].z = 5;
		//new_box.F[2].x = 1; new_box.F[2].y = 5; new_box.F[2].z = 6;
		//new_box.F[3].x = 1; new_box.F[3].y = 6; new_box.F[3].z = 2;
		//new_box.F[4].x = 2; new_box.F[4].y = 6; new_box.F[4].z = 7;
		//new_box.F[5].x = 2; new_box.F[5].y = 7; new_box.F[5].z = 3;
		//new_box.F[6].x = 0; new_box.F[6].y = 7; new_box.F[6].z = 4;
		//new_box.F[7].x = 0; new_box.F[7].y = 3; new_box.F[7].z = 7;
		//new_box.F[8].x = 5; new_box.F[8].y = 4; new_box.F[8].z = 7;
		//new_box.F[9].x = 5; new_box.F[9].y = 7; new_box.F[9].z = 6;
		//new_box.F[10].x = 0; new_box.F[10].y = 1; new_box.F[10].z = 2;
		//new_box.F[11].x = 0; new_box.F[11].y = 2; new_box.F[11].z = 3;

		new_box.F[0].x = 1; new_box.F[0].y = 6; new_box.F[0].z = 2;
		new_box.F[1].x = 1; new_box.F[1].y = 5; new_box.F[1].z = 6;
		new_box.F[2].x = 4; new_box.F[2].y = 7; new_box.F[2].z = 6;
		new_box.F[3].x = 4; new_box.F[3].y = 6; new_box.F[3].z = 5;
		new_box.F[4].x = 2; new_box.F[4].y = 6; new_box.F[4].z = 7;
		new_box.F[5].x = 2; new_box.F[5].y = 7; new_box.F[5].z = 3;
		new_box.F[6].x = 0; new_box.F[6].y = 3; new_box.F[6].z = 7;
		new_box.F[7].x = 0; new_box.F[7].y = 7; new_box.F[7].z = 4;
		new_box.F[8].x = 0; new_box.F[8].y = 1; new_box.F[8].z = 2;
		new_box.F[9].x = 0; new_box.F[9].y = 2; new_box.F[9].z = 3;
		new_box.F[10].x = 1; new_box.F[10].y = 0; new_box.F[10].z = 4;
		new_box.F[11].x = 1; new_box.F[11].y = 4; new_box.F[11].z = 5;

		new_box.L.resize(8);
		for (int l = 0; l < 8; l++)
		{
			new_box.L[l] = 1;
		}

		new_box.NormalUpdate();
	}
	else
	{
		new_box.Clear();
	}


	//vector<double> point_d, point_d2(3), point_d3(3);
	//for (int point_index = 0; point_index < label_points.size(); point_index++)
	//{
	//	point_d.push_back(label_points[point_index].x);
	//	point_d.push_back(label_points[point_index].y);
	//	point_d.push_back(label_points[point_index].z);

	//}
	//for (int point_index = 0; point_index < 50; point_index++)
	//{
	//	point_d.push_back(box_center.x + point_index*direct_x.x);
	//	point_d.push_back(box_center.y + point_index*direct_x.y);
	//	point_d.push_back(box_center.z + point_index*direct_x.z);
	//	point_d2.push_back(box_center.x + point_index*direct_y.x);
	//	point_d2.push_back(box_center.y + point_index*direct_y.y);
	//	point_d2.push_back(box_center.z + point_index*direct_y.z);
	//	point_d3.push_back(box_center.x + point_index*direct_z.x);
	//	point_d3.push_back(box_center.y + point_index*direct_z.y);
	//	point_d3.push_back(box_center.z + point_index*direct_z.z);
	//}
	//for (int i = 0; i < mm2.P.size(); i++)
	//{
	//	point_d3.push_back(mm2.P[i].x);
	//	point_d3.push_back(mm2.P[i].y);
	//	point_d3.push_back(mm2.P[i].z);
	//}

	//ColoredPoints3(point_d, point_d2, point_d3,3);


}

int CCutBoxObject::ChoseFace(const float point_x_in, const float point_y_in, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm)
{
	float point_x = point_x_in - (float)screen_width / 2.0;
	float point_y = (float)screen_height / 2.0 - point_y_in;

	point_x /= screen_width / 2;
	point_y /= screen_height / 2;

	orth::Point3d p1(point_x, point_y, -1);
	orth::Point3d p2(point_x, point_y, 1);

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	//model_matrix = model_matrix.t();
	model_matrix = cv::Mat::eye(4, 4, CV_32FC1);

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.t();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.t();

	cv::Mat final_matrix = projection_matrix*view_matrix*model_matrix;
	orth::PointCloudD screen_points(mm.P.size());
	vector<int> intersected_face_index;
	for (int face_index = 0; face_index < mm.F.size(); face_index++)
	{
		orth::Point3d point[3];
		point[0] = mm.P[mm.F[face_index].x];
		point[1] = mm.P[mm.F[face_index].y];
		point[2] = mm.P[mm.F[face_index].z];

		for (int point_index = 0; point_index < 3; point_index++)
		{
			orth::Point3d point_ = point[point_index];
			double x_ = final_matrix.at<float>(0, 0)*point_.x + final_matrix.at<float>(0, 1)*point_.y + final_matrix.at<float>(0, 2)*point_.z + final_matrix.at<float>(0, 3);
			double y_ = final_matrix.at<float>(1, 0)*point_.x + final_matrix.at<float>(1, 1)*point_.y + final_matrix.at<float>(1, 2)*point_.z + final_matrix.at<float>(1, 3);
			double z_ = final_matrix.at<float>(2, 0)*point_.x + final_matrix.at<float>(2, 1)*point_.y + final_matrix.at<float>(2, 2)*point_.z + final_matrix.at<float>(2, 3);
			double w_ = final_matrix.at<float>(3, 0)*point_.x + final_matrix.at<float>(3, 1)*point_.y + final_matrix.at<float>(3, 2)*point_.z + final_matrix.at<float>(3, 3);
			//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
			x_ /= w_;
			y_ /= w_;
			z_ /= w_;

			point[point_index].x = x_;
			point[point_index].y = y_;
			point[point_index].z = z_;
		}

		screen_points[mm.F[face_index].x] = point[0];
		screen_points[mm.F[face_index].y] = point[1];
		screen_points[mm.F[face_index].z] = point[2];

		if (orth::LineFaceIntersect(p1, p2, point[0], point[1], point[2]) == ACROSS)
		{
			intersected_face_index.push_back(face_index);
		}
	}

	int final_index = -1;
	double min_z = 1000;
	for (int point_index = 0; point_index < intersected_face_index.size(); point_index++)
	{
		int face_index = intersected_face_index[point_index];
		orth::Point3d point[3];
		point[0] = screen_points[mm.F[face_index].x];
		point[1] = screen_points[mm.F[face_index].y];
		point[2] = screen_points[mm.F[face_index].z];

		orth::Point3d center_point(0, 0, 0);
		for (int i = 0; i < 3; i++)
		{
			center_point += point[i];
		}
		center_point /= 3.0;

		if (center_point.z<min_z)
		{
			min_z = center_point.z;
			final_index = face_index;
		}

	}

	return final_index;


}


bool CCutBoxObject::CutModelInBox(orth::MeshModel &model_in)
{
	if (model_in.P.size() == 0 || model_in.F.size() == 0)
	{
		return false;
	}

	//double distance_px = orth::Point2PlaneDistance(box_center, new_box.P[1], new_box.P[6], new_box.P[5]);
	//double distance_nx = orth::Point2PlaneDistance(box_center, new_box.P[0], new_box.P[3], new_box.P[7]);
	model_in.L.clear();
	model_in.L.resize(model_in.P.size());
	int chose_face_number = 0;
	for (int face_index = 0; face_index < model_in.F.size(); face_index++)
	{
		orth::Point3d face_points[3];
		face_points[0] = model_in.P[model_in.F[face_index].x];
		face_points[1] = model_in.P[model_in.F[face_index].y];
		face_points[2] = model_in.P[model_in.F[face_index].z];
		//double f_distance_px[3] = { 0 };
		//double f_distance_nx[3] = { 0 };
		int right_number = 0;
		for (int i = 0; i < 3; i++)
		{
			double f_distance_px = orth::Point2PlaneDistance(face_points[i], new_box.P[1], new_box.P[6], new_box.P[2]);
			double f_distance_nx = orth::Point2PlaneDistance(face_points[i], new_box.P[0], new_box.P[7], new_box.P[3]);

			double f_distance_py = orth::Point2PlaneDistance(face_points[i], new_box.P[4], new_box.P[6], new_box.P[5]);
			double f_distance_ny = orth::Point2PlaneDistance(face_points[i], new_box.P[0], new_box.P[2], new_box.P[1]);

			double f_distance_pz = orth::Point2PlaneDistance(face_points[i], new_box.P[2], new_box.P[7], new_box.P[3]);
			double f_distance_nz = orth::Point2PlaneDistance(face_points[i], new_box.P[1], new_box.P[4], new_box.P[0]);

			if (f_distance_px > 0 || f_distance_nx < 0 || f_distance_py>0 || f_distance_ny < 0 || f_distance_pz>0 || f_distance_nz < 0)
			{
				break;
			}
			else
			{
				right_number++;
			}

		}
		if (right_number!=3)
		{
			model_in.L[model_in.F[face_index].x] = 1;
			model_in.L[model_in.F[face_index].y] = 1;
			model_in.L[model_in.F[face_index].z] = 1;
			chose_face_number++;
		}
	}
	cout << chose_face_number << endl;

	orth::PointCloudD points;
	orth::Faces faces;
	orth::PointNormal normals;
	orth::PointLabel labels;
	//orth::PointColor colors_(mm.P.size());
	vector<int> new_point_index(model_in.P.size(), -1);

	for (int point_index = 0; point_index < model_in.P.size(); point_index++)
	{
		//cout << "point number "<<point_index;
		if (!model_in.L[point_index])
		{
			//cout<< " good ";
			points.push_back(model_in.P[point_index]);
			//colors_.push_back(mm.C[point_index]);
			normals.push_back(model_in.N[point_index]);
			//labels.push_back(model_in.L[point_index]);
			labels.push_back(0.0f);
			new_point_index[point_index] = points.size() - 1;

		}
		//cout << endl;
	}

	for (int face_index = 0; face_index < model_in.F.size(); face_index++)
	{
		if (model_in.L[model_in.F[face_index].x] || model_in.L[model_in.F[face_index].y] || model_in.L[model_in.F[face_index].z])
		{
			continue;
		}
		else
		{
			orth::Face f;
			f.x = new_point_index[model_in.F[face_index].x];
			f.y = new_point_index[model_in.F[face_index].y];
			f.z = new_point_index[model_in.F[face_index].z];
			//if (f.x>mm.P.size()|| f.y>mm.P.size()|| f.z>mm.P.size())
			//{
			//	cout << mm.F[face_index].x << "; " << mm.F[face_index].y << "; " << mm.F[face_index].z << endl;
			//	cout << new_point_index[mm.F[face_index].x] << "; " << new_point_index[mm.F[face_index].y] << "; " << new_point_index[mm.F[face_index].z] << endl;
			//	cout << f.x << "; " << f.y << "; " << f.z << endl;
			//}
			faces.push_back(f);
		}
	}

	model_in.F.swap(faces);
	model_in.P.swap(points);
	model_in.N.swap(normals);
	model_in.L.swap(labels);
}

void CCutBoxObject::BoxTrans(const float dx, const float dy)
{
	if (face_index == -1)
	{
		return;
	}
	//cout << "face_index = " << face_index << "; ";
	if (face_index == BoxFace::x_p|| face_index == BoxFace::x_n)
	{
		QVector4D direct(direct_x.x, direct_x.y, direct_x.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] += direct_x*0.2;
			}
			box_center += direct_x*0.2;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] -= direct_x*0.2;
			}
			box_center -= direct_x*0.2;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::y_p|| face_index == BoxFace::y_n)
	{
		QVector4D direct(direct_y.x, direct_y.y, direct_y.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] += direct_y*0.2;
			}
			box_center += direct_y*0.2;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] -= direct_y*0.2;
			}
			box_center -= direct_y*0.2;
		}
		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = " << matched;
	}

	if (face_index == BoxFace::z_p|| face_index == BoxFace::z_n)
	{
		QVector4D direct(direct_z.x, direct_z.y, direct_z.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] += direct_z*0.2;
			}
			box_center += direct_z*0.2;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				new_box.P[point_index] -= direct_z*0.2;
			}
			box_center -= direct_z*0.2;
		}
		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = " << matched;
	}
	//cout << endl;
}

void CCutBoxObject::SetPos(QPointF p)
{
	m_lastPos = p;
}

void CCutBoxObject::BoxRot(QPointF p)
{
	if (face_index==-1)
	{
		return;
	}

	QVector3D lastPos3D = QVector3D(m_lastPos.x(), -m_lastPos.y(), 0.0f);
	float sqrZ = 1 - QVector3D::dotProduct(lastPos3D, lastPos3D);
	if (sqrZ > 0)
		lastPos3D.setZ(std::sqrt(sqrZ));
	else
		lastPos3D.normalize();

	QVector3D currentPos3D = QVector3D(p.x(), -p.y(), 0.0f);
	sqrZ = 1 - QVector3D::dotProduct(currentPos3D, currentPos3D);
	if (sqrZ > 0)
		currentPos3D.setZ(std::sqrt(sqrZ));
	else
		currentPos3D.normalize();

	QVector3D m_axis = QVector3D::crossProduct(lastPos3D, currentPos3D);
	float angle = 180 / PI * std::asin(std::sqrt(QVector3D::dotProduct(m_axis, m_axis)));

	m_axis.normalize();
	QQuaternion transformation;
	m_axis = transformation.rotatedVector(m_axis);

	//m_rotation = QQuaternion::fromAxisAndAngle(m_axis, angle) * m_rotation;
	m_rotation = QQuaternion::fromAxisAndAngle(m_axis, angle);
	//cout << "m_rotation x" << m_rotation.x() << "m_rotation y" << m_rotation.y() << "m_rotation z" << m_rotation.z() << endl;

	QMatrix4x4 m_view;
	m_view.rotate(m_rotation);
	//m_view = m_view.inverted();

	QVector4D dir_x(direct_x.x, direct_x.y, direct_x.z, 1);
	QVector4D dir_y(direct_y.x, direct_y.y, direct_y.z, 1);
	QVector4D dir_z(direct_z.x, direct_z.y, direct_z.z, 1);

	dir_x = m_view*dir_x;
	dir_y = m_view*dir_y;
	dir_z = m_view*dir_z;

	direct_x = orth::Point3d(dir_x.x(), dir_x.y(), dir_x.z());
	direct_y = orth::Point3d(dir_y.x(), dir_y.y(), dir_y.z());
	direct_z = orth::Point3d(dir_z.x(), dir_z.y(), dir_z.z());

	QVector4D qmin_x_dir(min_x_dir.x, min_x_dir.y, min_x_dir.z,1);
	QVector4D qmax_x_dir(max_x_dir.x, max_x_dir.y, max_x_dir.z,1);
	QVector4D qmin_y_dir(min_y_dir.x, min_y_dir.y, min_y_dir.z,1);
	QVector4D qmax_y_dir(max_y_dir.x, max_y_dir.y, max_y_dir.z,1);
	QVector4D qmin_z_dir(min_z_dir.x, min_z_dir.y, min_z_dir.z,1);
	QVector4D qmax_z_dir(max_z_dir.x, max_z_dir.y, max_z_dir.z,1);

	qmin_x_dir = m_view*qmin_x_dir;
	qmax_x_dir = m_view*qmax_x_dir;
	qmin_y_dir = m_view*qmin_y_dir;
	qmax_y_dir = m_view*qmax_y_dir;
	qmin_z_dir = m_view*qmin_z_dir;
	qmax_z_dir = m_view*qmax_z_dir;


	min_x_dir = orth::Point3d(qmin_x_dir.x(), qmin_x_dir.y(), qmin_x_dir.z());
	max_x_dir = orth::Point3d(qmax_x_dir.x(), qmax_x_dir.y(), qmax_x_dir.z());
	min_y_dir = orth::Point3d(qmin_y_dir.x(), qmin_y_dir.y(), qmin_y_dir.z());
	max_y_dir = orth::Point3d(qmax_y_dir.x(), qmax_y_dir.y(), qmax_y_dir.z());
	min_z_dir = orth::Point3d(qmin_z_dir.x(), qmin_z_dir.y(), qmin_z_dir.z());
	max_z_dir = orth::Point3d(qmax_z_dir.x(), qmax_z_dir.y(), qmax_z_dir.z());


	if (new_box.P.size()>0)
	{
		new_box.F.clear();
		new_box.F.resize(12);
		new_box.P.clear();
		new_box.P.resize(8);
		vector<orth::Point3d> box_vertex(8);
		new_box.P[0] = box_center + min_x_dir + min_y_dir + min_z_dir;
		new_box.P[1] = box_center + max_x_dir + min_y_dir + min_z_dir;
		new_box.P[2] = box_center + max_x_dir + min_y_dir + max_z_dir;
		new_box.P[3] = box_center + min_x_dir + min_y_dir + max_z_dir;
		new_box.P[4] = box_center + min_x_dir + max_y_dir + min_z_dir;
		new_box.P[5] = box_center + max_x_dir + max_y_dir + min_z_dir;
		new_box.P[6] = box_center + max_x_dir + max_y_dir + max_z_dir;
		new_box.P[7] = box_center + min_x_dir + max_y_dir + max_z_dir;

		new_box.F[0].x = 1; new_box.F[0].y = 6; new_box.F[0].z = 2;
		new_box.F[1].x = 1; new_box.F[1].y = 5; new_box.F[1].z = 6;
		new_box.F[2].x = 4; new_box.F[2].y = 7; new_box.F[2].z = 6;
		new_box.F[3].x = 4; new_box.F[3].y = 6; new_box.F[3].z = 5;
		new_box.F[4].x = 2; new_box.F[4].y = 6; new_box.F[4].z = 7;
		new_box.F[5].x = 2; new_box.F[5].y = 7; new_box.F[5].z = 3;
		new_box.F[6].x = 0; new_box.F[6].y = 3; new_box.F[6].z = 7;
		new_box.F[7].x = 0; new_box.F[7].y = 7; new_box.F[7].z = 4;
		new_box.F[8].x = 0; new_box.F[8].y = 1; new_box.F[8].z = 2;
		new_box.F[9].x = 0; new_box.F[9].y = 2; new_box.F[9].z = 3;
		new_box.F[10].x = 1; new_box.F[10].y = 0; new_box.F[10].z = 4;
		new_box.F[11].x = 1; new_box.F[11].y = 4; new_box.F[11].z = 5;

		new_box.L.resize(8);
		for (int l = 0; l < 8; l++)
		{
			new_box.L[l] = 1;
		}

		new_box.NormalUpdate();
	}
	else
	{
		new_box.Clear();
	}

	m_lastPos = p;

}

void CCutBoxObject::BoxStretch(const float dx, const float dy)
{
	if (face_index == -1)
	{
		return;
	}
	//cout << "face_index = " << face_index << "; ";
	if (face_index == BoxFace::x_p)
	{
		QVector4D direct(direct_x.x, direct_x.y, direct_x.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index==1|| point_index == 2 || point_index == 6 || point_index == 5 )
				{
					new_box.P[point_index] += direct_x*0.2;
				}
				
			}
			box_center += direct_x*0.1;
			max_x_dir += direct_x*0.1;
			min_x_dir -= direct_x*0.1;
		}
		else
		{
			double width_box = (new_box.P[1] - new_box.P[0]).dot(new_box.P[1] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 1 || point_index == 2 || point_index == 6 || point_index == 5)
				{
					new_box.P[point_index] -= direct_x*0.2;
				}
			}
			box_center -= direct_x*0.1;
			max_x_dir -= direct_x*0.1;
			min_x_dir += direct_x*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::x_n)
	{
		QVector4D direct(direct_x.x, direct_x.y, direct_x.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			double width_box = (new_box.P[1] - new_box.P[0]).dot(new_box.P[1] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 3 || point_index == 4 || point_index == 7)
				{
					new_box.P[point_index] += direct_x*0.2;
				}

			}
			box_center += direct_x*0.1;
			max_x_dir -= direct_x*0.1;
			min_x_dir += direct_x*0.1;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 3 || point_index == 4 || point_index == 7)
				{
					new_box.P[point_index] -= direct_x*0.2;
				}
			}
			box_center -= direct_x*0.1;
			max_x_dir += direct_x*0.1;
			min_x_dir -= direct_x*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::y_p)
	{
		QVector4D direct(direct_y.x, direct_y.y, direct_y.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 4 || point_index == 5 || point_index == 6 || point_index == 7)
				{
					new_box.P[point_index] += direct_y*0.2;
				}

			}
			box_center += direct_y*0.1;
			max_y_dir += direct_y*0.1;
			min_y_dir -= direct_y*0.1;
		}
		else
		{
			double width_box = (new_box.P[4] - new_box.P[0]).dot(new_box.P[4] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 4 || point_index == 5 || point_index == 6 || point_index == 7)
				{
					new_box.P[point_index] -= direct_y*0.2;
				}
			}
			box_center -= direct_y*0.1;
			max_y_dir -= direct_y*0.1;
			min_y_dir += direct_y*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::y_n)
	{
		QVector4D direct(direct_y.x, direct_y.y, direct_y.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			double width_box = (new_box.P[4] - new_box.P[0]).dot(new_box.P[4] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 1 || point_index == 2 || point_index == 3)
				{
					new_box.P[point_index] += direct_y*0.2;
				}

			}
			box_center += direct_y*0.1;
			max_y_dir -= direct_y*0.1;
			min_y_dir += direct_y*0.1;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 1 || point_index == 2 || point_index == 3)
				{
					new_box.P[point_index] -= direct_y*0.2;
				}
			}
			box_center -= direct_y*0.1;
			max_y_dir += direct_y*0.1;
			min_y_dir -= direct_y*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::z_p)
	{
		QVector4D direct(direct_z.x, direct_z.y, direct_z.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 2 || point_index == 3 || point_index == 6 || point_index == 7)
				{
					new_box.P[point_index] += direct_z*0.2;
				}

			}
			box_center += direct_z*0.1;
			max_z_dir += direct_z*0.1;
			min_z_dir -= direct_z*0.1;
		}
		else
		{
			double width_box = (new_box.P[3] - new_box.P[0]).dot(new_box.P[3] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 2 || point_index == 3 || point_index == 6 || point_index == 7)
				{
					new_box.P[point_index] -= direct_z*0.2;
				}
			}
			box_center -= direct_z*0.1;
			max_z_dir -= direct_z*0.1;
			min_z_dir += direct_z*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}

	if (face_index == BoxFace::z_n)
	{
		QVector4D direct(direct_z.x, direct_z.y, direct_z.z, 1);
		QVector4D direct_screen = end_matrix4x4*direct;
		float matched = direct_screen.x()*dx + direct_screen.y()*dy*-1;
		if (matched>0)
		{
			double width_box = (new_box.P[3] - new_box.P[0]).dot(new_box.P[3] - new_box.P[0]);
			if (width_box<1)
			{
				return;
			}
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 1 || point_index == 4 || point_index == 5)
				{
					new_box.P[point_index] += direct_z*0.2;
				}

			}
			box_center += direct_z*0.1;
			max_z_dir -= direct_z*0.1;
			min_z_dir += direct_z*0.1;
		}
		else
		{
			for (int point_index = 0; point_index < new_box.P.size(); point_index++)
			{
				if (point_index == 0 || point_index == 1 || point_index == 4 || point_index == 5)
				{
					new_box.P[point_index] -= direct_z*0.2;
				}
			}
			box_center -= direct_z*0.1;
			max_z_dir += direct_z*0.1;
			min_z_dir -= direct_z*0.1;
		}

		//cout << "dx = " << dx << ", " << "dy = " << dy << ", " << "sx = " << direct_screen.x() << ", " << "sy = " << direct_screen.y() << "; matched = "<<matched ;
	}
}

void CCutBoxObject::SetOriginalColor()
{
	face_index = -1;
	ColorUpdate();
}

void CCutBoxObject::ColorUpdate()
{
	face_color.clear();
	face_color.resize(12);
	if (face_index == BoxFace::x_p)
	{
		face_color[0] = press_color_x_color;
		face_color[1] = press_color_x_color;
	}
	else
	{
		face_color[0] = x_color;
		face_color[1] = x_color;
	}
	if (face_index == BoxFace::x_n)
	{
		face_color[6] = press_color_x_color;
		face_color[7] = press_color_x_color;
	}
	else
	{
		face_color[6] = x_color;
		face_color[7] = x_color;
	}
	/*-----------------------------------------------------------------*/

	if (face_index == BoxFace::y_p)
	{
		face_color[2] = press_color_y_color;
		face_color[3] = press_color_y_color;
	}
	else
	{
		face_color[2] = y_color;
		face_color[3] = y_color;
	}
	if (face_index == BoxFace::y_n)
	{
		face_color[8] = press_color_y_color;
		face_color[9] = press_color_y_color;
	}
	else
	{
		face_color[8] = y_color;
		face_color[9] = y_color;
	}
	/*-----------------------------------------------------------------*/

	if (face_index == BoxFace::z_p)
	{
		face_color[4] = press_color_z_color;
		face_color[5] = press_color_z_color;
	}
	else
	{
		face_color[4] = z_color;
		face_color[5] = z_color;
	}
	if (face_index == BoxFace::z_n)
	{
		face_color[10] = press_color_z_color;
		face_color[11] = press_color_z_color;
	}
	else
	{
		face_color[10] = z_color;
		face_color[11] = z_color;
	}

}

void CCutBoxObject::UpdateCutBoxObject()
{
	if (new_box.F.size() == 0)
	{
		return;
	}

	ColorUpdate();

	QVector<GLfloat> vertData;

	//x1 r
	for (int i = 0; i < new_box.F.size(); i++)
	{
		//float x, y, z;
		vertData.append(new_box.P[new_box.F[i].x].x);
		vertData.append(new_box.P[new_box.F[i].x].y);
		vertData.append(new_box.P[new_box.F[i].x].z);
		vertData.append(face_color[i].x());
		vertData.append(face_color[i].y());
		vertData.append(face_color[i].z());
		vertData.append(face_color[i].w());

		vertData.append(new_box.P[new_box.F[i].y].x);
		vertData.append(new_box.P[new_box.F[i].y].y);
		vertData.append(new_box.P[new_box.F[i].y].z);
		vertData.append(face_color[i].x());
		vertData.append(face_color[i].y());
		vertData.append(face_color[i].z());
		vertData.append(face_color[i].w());

		vertData.append(new_box.P[new_box.F[i].z].x);
		vertData.append(new_box.P[new_box.F[i].z].y);
		vertData.append(new_box.P[new_box.F[i].z].z);
		vertData.append(face_color[i].x());
		vertData.append(face_color[i].y());
		vertData.append(face_color[i].z());
		vertData.append(face_color[i].w());

	}

	makeObject(vertData, 36);

}

//int CCutBoxObject::ChoseFace(const float point_x_in, const float point_y_in, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm)
int CCutBoxObject::ChoseFace(const float point_x_in, const float point_y_in, const int screen_width, const int screen_height, QMatrix4x4 model_matrix, QMatrix4x4 view_matrix, QMatrix4x4 projection_matrix)
{
	
	float point_x = point_x_in - (float)screen_width / 2.0;
	float point_y = (float)screen_height / 2.0 - point_y_in;

	point_x /= screen_width / 2;
	point_y /= screen_height / 2;

	orth::Point3d p1(point_x, point_y, -1);
	orth::Point3d p2(point_x, point_y, 1);

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	//model_matrix = model_matrix.t();
	model_matrix.setToIdentity(); 

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.transposed();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.transposed();

	QMatrix4x4 final_matrix = projection_matrix*view_matrix*model_matrix;
	orth::PointCloudD screen_points(new_box.P.size());
	vector<int> intersected_face_index;
	for (int face_index = 0; face_index < new_box.F.size(); face_index++)
	{
		orth::Point3d point[3];
		point[0] = new_box.P[new_box.F[face_index].x];
		point[1] = new_box.P[new_box.F[face_index].y];
		point[2] = new_box.P[new_box.F[face_index].z];

		for (int point_index = 0; point_index < 3; point_index++)
		{
			orth::Point3d point_ = point[point_index];
			final_matrix.column(0).x();
			double x_ = final_matrix.column(0).x()*point_.x + final_matrix.column(1).x()*point_.y + final_matrix.column(2).x()*point_.z + final_matrix.column(3).x();
			double y_ = final_matrix.column(0).y()*point_.x + final_matrix.column(1).y()*point_.y + final_matrix.column(2).y()*point_.z + final_matrix.column(3).y();
			double z_ = final_matrix.column(0).z()*point_.x + final_matrix.column(1).z()*point_.y + final_matrix.column(2).z()*point_.z + final_matrix.column(3).z();
			double w_ = final_matrix.column(0).w()*point_.x + final_matrix.column(1).w()*point_.y + final_matrix.column(2).w()*point_.z + final_matrix.column(3).w();
			//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
			x_ /= w_;
			y_ /= w_;
			z_ /= w_;

			point[point_index].x = x_;
			point[point_index].y = y_;
			point[point_index].z = z_;
		}

		screen_points[new_box.F[face_index].x] = point[0];
		screen_points[new_box.F[face_index].y] = point[1];
		screen_points[new_box.F[face_index].z] = point[2];

		if (orth::LineFaceIntersect(p1, p2, point[0], point[1], point[2]) == ACROSS)
		{
			intersected_face_index.push_back(face_index);
		}
	}

	int final_index = -1;
	double min_z = 1000;
	for (int point_index = 0; point_index < intersected_face_index.size(); point_index++)
	{
		int face_index = intersected_face_index[point_index];
		orth::Point3d point[3];
		point[0] = screen_points[new_box.F[face_index].x];
		point[1] = screen_points[new_box.F[face_index].y];
		point[2] = screen_points[new_box.F[face_index].z];

		orth::Point3d center_point(0, 0, 0);
		for (int i = 0; i < 3; i++)
		{
			center_point += point[i];
		}
		center_point /= 3.0;

		if (center_point.z<min_z)
		{
			min_z = center_point.z;
			final_index = face_index;
		}

	}

	return final_index;


}


void CCutBoxObject::doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	end_matrix4x4 = v_Projection*v_View;

	
	
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_program->setUniformValue("projection", v_Projection);
	m_program->setUniformValue("view", v_View);
	m_program->setUniformValue("zoom", v_zoom_x,v_zoom_y,v_zoom_z);
	m_ModelMatrix.setToIdentity();
	// 	m_ModelMatrix.rotate(m_xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	// 	m_ModelMatrix.rotate(m_yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	// 	m_ModelMatrix.rotate(m_zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_ModelMatrix.rotate(m_ModelRotate);
	m_ModelMatrix.translate(QVector3D(v_trans_x, v_trans_y, v_trans_z));
	m_program->setUniformValue("model", m_ModelMatrix);
	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 7 * sizeof(GLfloat));
	m_program->enableAttributeArray(PROGRAM_COLOR_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_COLOR_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 4, 7 * sizeof(GLfloat));
	m_program->setUniformValue("ourColor", m_bkGroundColor);


	GLubyte xcolor[4] = { 255,0,0,255 },
		ycolor[4] = { 0,255,0,255 },
		zcolor[4] = { 0,0,255,255 };
	glLineWidth(5);
	glBegin(GL_LINES);
	//glColor3f(1, 0, 0);
	glColor4ubv(xcolor);
	glVertex3f((direct_x.x * 10 + box_center.x), (direct_x.y * 10 + box_center.y), (direct_x.z * 10 + box_center.z));
	glColor4ubv(xcolor);
	glVertex3f(box_center.x, box_center.y, box_center.z);
	glColor4ubv(ycolor);
	glVertex3f((direct_y.x * 10 + box_center.x), (direct_y.y * 10 + box_center.y), (direct_y.z * 10 + box_center.z));
	glColor4ubv(ycolor);
	glVertex3f(box_center.x, box_center.y, box_center.z);
	glColor4ubv(zcolor);
	glVertex3f((direct_z.x * 10 + box_center.x), (direct_z.y * 10 + box_center.y), (direct_z.z * 10 + box_center.z));
	glColor4ubv(zcolor);
	glVertex3f(box_center.x, box_center.y, box_center.z);
	glEnd();

	//glLineWidth(1);
	//glBegin(GL_LINES);

	//glEnd();

	//glLineWidth(1);
	//glBegin(GL_LINES);

	//glEnd();


	//glEnable(GL_CULL_FACE);

	//glDrawArrays(GL_QUADS, 0, m_totalFaceNum);
	glDepthMask(GL_TRUE);
	//glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glDrawArrays(GL_TRIANGLES, 0, m_totalFaceNum);
	glDisable(GL_BLEND);
	//glDisable(GL_CULL_FACE);
	//glDepthMask(GL_TRUE);
	//cout << glGetError() << endl;
}


