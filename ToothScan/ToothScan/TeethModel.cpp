#include "TeethModel.h"
#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_NORMAL_ATTRIBUTE 1
#define PROGRAM_MATERIAL_ATTRIBUTE 2
#define PROGRAM_STATE_ATTRIBUTE 3


CTeethModel::CTeethModel(shared_ptr<QOpenGLShaderProgram> v_Program,
	shared_ptr<QOpenGLBuffer> v_Vbo, orth::MeshModel &mm):BaseModel(v_Program,v_Vbo),
	m_xTrans(0),
	m_yTrans(0),
	m_model(mm)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->bindAttributeLocation("aNormal", PROGRAM_NORMAL_ATTRIBUTE);
	m_program->bindAttributeLocation("aMateriala", PROGRAM_MATERIAL_ATTRIBUTE);
	m_program->bindAttributeLocation("aState", PROGRAM_STATE_ATTRIBUTE);
	m_program->link();
}


CTeethModel::~CTeethModel()
{
}

void CTeethModel::doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	m_program->setUniformValue("projection", v_Projection);

	//cout << projection.data()[0] << ", " << projection.data()[4] << ", " << projection.data()[8] << ", " << projection.data()[12] <<  endl;
	//cout << projection.data()[1] << ", " << projection.data()[5] << ", " << projection.data()[9] << ", " << projection.data()[13] << endl;
	//cout << projection.data()[2] << ", " << projection.data()[6] << ", " << projection.data()[10] << ", " << projection.data()[14] << endl;
	//cout << projection.data()[3] << ", " << projection.data()[7] << ", " << projection.data()[11] << ", " << projection.data()[15] << endl;
	//cout << " ---------------------------------------------------- " << endl;
	//cout << endl;

	m_program->setUniformValue("view", v_View);
	m_program->setUniformValue("viewPos", v_View.column(3));
	//cout << view.data()[0] << ", " << view.data()[4] << ", " << view.data()[8] << ", " << view.data()[12] << endl;
	//cout << view.data()[1] << ", " << view.data()[5] << ", " << view.data()[9] << ", " << view.data()[13] << endl;
	//cout << view.data()[2] << ", " << view.data()[6] << ", " << view.data()[10] << ", " << view.data()[14] << endl;
	//cout << view.data()[3] << ", " << view.data()[7] << ", " << view.data()[11] << ", " << view.data()[15] << endl;
	//cout << endl;
	m_ModelMatrix.setToIdentity();
// 	m_ModelMatrix.rotate(m_xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_ModelMatrix.rotate(m_ModelRotate);
	m_program->setUniformValue("model", m_ModelMatrix);
	m_program->setUniformValue("inv_model", m_ModelMatrix.inverted());
	//m_program->setUniformValue("screenPos", QVector3D(0, 0, 0));
	m_program->setUniformValue("screenPos", QVector3D(m_xTrans, m_yTrans, 0));

	//memcpy(model1.data, model.data(), 16 * sizeof(float));
	//memcpy(view1.data, view.data(), 16 * sizeof(float));
	//memcpy(project1.data, projection.data(), 16 * sizeof(float));
	//point_end = model*point_;
	//cout << point_end.x() << ", " << point_end.y() << ", " << point_end.z() << ", " << point_end.w() << endl;
	//point_end = view*point_end;
	//cout << point_end.x() << ", " << point_end.y() << ", " << point_end.z() << ", " << point_end.w() << endl;
	//point_end = projection*point_end;
	//cout << point_end.x() / point_end.w() << ", " << point_end.y() / point_end.w() << ", " << point_end.z() / point_end.w() << ", " << point_end.w() / point_end.w() << endl;
	//cout << "----------------------------------------------" << endl;

	//cout << model.data()[0] << ", " << model.data()[4] << ", " << model.data()[8] << ", " << model.data()[12] << endl;
	//cout << model.data()[1] << ", " << model.data()[5] << ", " << model.data()[9] << ", " << model.data()[13] << endl;
	//cout << model.data()[2] << ", " << model.data()[6] << ", " << model.data()[10] << ", " << model.data()[14] << endl;
	//cout << model.data()[3] << ", " << model.data()[7] << ", " << model.data()[11] << ", " << model.data()[15] << endl;
	//cout << endl;
	//vector<float> a(4,0);
	//cout << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << endl;
	//for (int i = 0; i < 4; i++)
	//{
	//	for (int j = 0; j < 4; j++)
	//	{
	//		a[j] += model.data()[j + 4 * i] * fv[i];
	//	}
	//}
	//cout << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << endl;
	//cout << "-----------------------------" << endl;
	//vector<float> a1(4, 0);
	//for (int i = 0; i < 4; i++)
	//{
	//	a1[0] += view.column(i).x()*a[i];
	//	a1[1] += view.column(i).y()*a[i];
	//	a1[2] += view.column(i).z()*a[i];
	//	a1[3] += view.column(i).w()*a[i];
	//	
	//}
	//cout << a1[0] << ", " << a1[1] << ", " << a1[2] << ", " << a1[3] << endl;cout << "-----------------------------" << endl;
	//vector<float> a2(4, 0);
	//for (int i = 0; i < 4; i++)
	//{
	//	a2[0] += projection.column(i).x() *a1[i];
	//	a2[1] += projection.column(i).y() *a1[i];
	//	a2[2] += projection.column(i).z() *a1[i];
	//	a2[3] += projection.column(i).w() *a1[i];
	//	
	//}
	//cout << a2[0] << ", " << a2[1] << ", " << a2[2] << ", " << a2[3] << endl;cout << "-----------------------------" << endl;
	m_program->setUniformValue("lightPos", QVector3D(0, 0, 0));
	//cout << view.column(3).toVector3D()[0] <<", " << view.column(3).toVector3D()[1] << ", " << view.column(3).toVector3D()[2]<< endl;
	m_program->setUniformValue("lightColor", QVector3D(1.0f, 1.0f, 1.0f));
	m_program->setUniformValue("objectColor", QVector3D(1.2f, 0.7f, 0.71f));
	m_program->setUniformValue("selectColor", QVector3D(1.0f, 0.0f, 0.0f));
	//m_program->setUniformValue("gingivaColor", QVector3D(1.3f, 1.3f, 1.3f));
	//m_program->setUniformValue("teethColor", QVector3D(1.3f, 1.3f, 1.3f));

	m_program->setUniformValue("material1.ambient", 0.05f, 0.0f, 0.0f);
	m_program->setUniformValue("material1.diffuse", 0.5f, 0.4f, 0.4f);
	m_program->setUniformValue("material1.specular", 0.8f, 0.04f, 0.04f);
	m_program->setUniformValue("material1.shininess", 0.978125f);

	m_program->setUniformValue("material2.ambient", 0.95f, 0.95f, 0.95f);
	m_program->setUniformValue("material2.diffuse", 0.6f, 0.6f, 0.6f);
	m_program->setUniformValue("material2.specular", 1.2f, 1.2f, 1.2f);
	m_program->setUniformValue("material2.shininess", 32.0f);

	m_program->setUniformValue("material3.ambient", 0.0f, 0.5f, 0.0f);
	m_program->setUniformValue("material3.diffuse", 0.5f, 0.4f, 0.4f);
	m_program->setUniformValue("material3.specular", 0.8f, 0.04f, 0.04f);
	m_program->setUniformValue("material3.shininess", 0.978125f);

	// directional light
	m_program->setUniformValue("dirLight.direction", 0.0f, 0.0f, 1.0f);
	m_program->setUniformValue("dirLight.ambient", 0.45f, 0.45f, 0.45f);
	m_program->setUniformValue("dirLight.diffuse", 0.6f, 0.6f, 0.6f);
	m_program->setUniformValue("dirLight.specular", 0.7f, 0.7f, 0.7f);
	// point light 1
	m_program->setUniformValue("pointLights[0].position", 0.7f, 0.2f, 2.0f);
	m_program->setUniformValue("pointLights[0].ambient", 0.05f, 0.05f, 0.05f);
	m_program->setUniformValue("pointLights[0].diffuse", 0.8f, 0.8f, 0.8f);
	m_program->setUniformValue("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("pointLights[0].constant", 1.0f);
	m_program->setUniformValue("pointLights[0].linear", 0.09f);
	m_program->setUniformValue("pointLights[0].quadratic", 0.032f);
	// point light 2
	m_program->setUniformValue("pointLights[1].position", 2.3f, -3.3f, -4.0f);
	m_program->setUniformValue("pointLights[1].ambient", 0.05f, 0.05f, 0.05f);
	m_program->setUniformValue("pointLights[1].diffuse", 0.8f, 0.8f, 0.8f);
	m_program->setUniformValue("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("pointLights[1].constant", 1.0f);
	m_program->setUniformValue("pointLights[1].linear", 0.09f);
	m_program->setUniformValue("pointLights[1].quadratic", 0.032f);
	// point light 3
	m_program->setUniformValue("pointLights[2].position", -4.0f, 2.0f, -12.0f);
	m_program->setUniformValue("pointLights[2].ambient", 0.05f, 0.05f, 0.05f);
	m_program->setUniformValue("pointLights[2].diffuse", 0.8f, 0.8f, 0.8f);
	m_program->setUniformValue("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("pointLights[2].constant", 1.0f);
	m_program->setUniformValue("pointLights[2].linear", 0.09f);
	m_program->setUniformValue("pointLights[2].quadratic", 0.032f);
	// point light 4
	m_program->setUniformValue("pointLights[3].position", 0.0f, 0.0f, -3.0f);
	m_program->setUniformValue("pointLights[3].ambient", 0.05f, 0.05f, 0.05f);
	m_program->setUniformValue("pointLights[3].diffuse", 0.8f, 0.8f, 0.8f);
	m_program->setUniformValue("pointLights[3].specular", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("pointLights[3].constant", 1.0f);
	m_program->setUniformValue("pointLights[3].linear", 0.09f);
	m_program->setUniformValue("pointLights[3].quadratic", 0.032f);
	// spotLight
	m_program->setUniformValue("spotLight.position", 0.0f, 0.0f, 0.0f);
	m_program->setUniformValue("spotLight.direction", 0.0f, 0.0f, 1.0f);
	m_program->setUniformValue("spotLight.ambient", 0.0f, 0.0f, 0.0f);
	m_program->setUniformValue("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("spotLight.specular", 1.0f, 1.0f, 1.0f);
	m_program->setUniformValue("spotLight.constant", 1.0f);;
	m_program->setUniformValue("spotLight.linear", 0.09f);
	m_program->setUniformValue("spotLight.quadratic", 0.032f);
	m_program->setUniformValue("spotLight.cutOff", float(cos(0.2094)));
	m_program->setUniformValue("spotLight.outerCutOff", float(cos(0.2618)));


	//this->update();

	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_NORMAL_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_MATERIAL_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_STATE_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat));
	m_program->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 4 * sizeof(GLfloat), 3, 8 * sizeof(GLfloat));
	m_program->setAttributeBuffer(PROGRAM_MATERIAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 1, 8 * sizeof(GLfloat));
	m_program->setAttributeBuffer(PROGRAM_STATE_ATTRIBUTE, GL_FLOAT, 7 * sizeof(GLfloat), 1, 8 * sizeof(GLfloat));

	if (m_drawRectClickPosition != m_drawRectEndPosition) {
		ChangeSelectedColorEx(m_drawRectClickPosition, m_drawRectEndPosition,pParent);
	}
	else {
		m_program->setUniformValue("selectedAreaBegin", 0.0f, 0.0f);
		m_program->setUniformValue("selectedAreaEnd", 0.0f, 0.0f);
	}
	glDrawArrays(GL_TRIANGLES, 0, m_totalFaceNum * 3);
	//cout << glGetError() << endl;
}

void CTeethModel::setScreenPos(float xTrans, float yTrans)
{
	m_xTrans = xTrans;
	m_yTrans = yTrans;
}

void CTeethModel::ChangeSelectedColorEx(QPoint drawRectClickPosition, QPoint drawRectEndPosition, IParentInterface *pParent)
{
	QVector2D pointBegin, pointEnd;
	if (drawRectClickPosition.x() < drawRectEndPosition.x()) {
		pointBegin.setX(drawRectClickPosition.x());
		pointEnd.setX(drawRectEndPosition.x());
	}
	else {
		pointBegin.setX(drawRectEndPosition.x());
		pointEnd.setX(drawRectClickPosition.x());
	}
	if (drawRectClickPosition.y() < drawRectEndPosition.y()) {
		pointBegin.setY(drawRectClickPosition.y());
		pointEnd.setY(drawRectEndPosition.y());
	}
	else {
		pointBegin.setY(drawRectEndPosition.y());
		pointEnd.setY(drawRectClickPosition.y());
	}
	QVector3D pointBegin3d = pParent->screen2world(pointBegin.x(), pointBegin.y()),
		pointEnd3d = pParent->screen2world(pointEnd.x(), pointEnd.y());
	m_program->setUniformValue("selectedAreaBegin", pointBegin3d.x(), pointBegin3d.y());
	m_program->setUniformValue("selectedAreaEnd", pointEnd3d.x(), pointEnd3d.y());
}

void CTeethModel::ChosePoints(const float point1_x, const float point1_y, 
	const float point2_x, const float point2_y, const int screen_width, 
	const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, 
	IParentInterface *pParent)
{
	cv::Mat depthimage(2000, 2000, CV_32FC1);
	float min_x = min(point1_x, point2_x);
	float min_y = min(point1_y, point2_y);
	float max_x = max(point1_x, point2_x);
	float max_y = max(point1_y, point2_y);

	QVector3D minPoint = pParent->screen2world(min_x, min_y),
		maxPoint = pParent->screen2world(max_x, max_y);
	cout << min_x << " < x <" << max_x << endl;
	cout << min_y << " < y <" << max_y << endl;

	m_Selected.resize(m_model.P.size());
	m_Selected.clear();

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	model_matrix = cv::Mat(4, 4, CV_32F, m_ModelMatrix.data()).t();

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.t();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.t();

	cv::Mat final_matrix = projection_matrix*view_matrix*model_matrix;

	int selected_points = 0;
	for (int point_index = 0; point_index < m_model.P.size(); point_index++)
	{
		if (m_Selected[point_index])
		{
			continue;
		}

		orth::Point3d point_ = m_model.P[point_index];
		double x_ = final_matrix.at<float>(0, 0)*point_.x + final_matrix.at<float>(0, 1)*point_.y + final_matrix.at<float>(0, 2)*point_.z + final_matrix.at<float>(0, 3);
		double y_ = final_matrix.at<float>(1, 0)*point_.x + final_matrix.at<float>(1, 1)*point_.y + final_matrix.at<float>(1, 2)*point_.z + final_matrix.at<float>(1, 3);
		double z_ = final_matrix.at<float>(2, 0)*point_.x + final_matrix.at<float>(2, 1)*point_.y + final_matrix.at<float>(2, 2)*point_.z + final_matrix.at<float>(2, 3);
		double w_ = final_matrix.at<float>(3, 0)*point_.x + final_matrix.at<float>(3, 1)*point_.y + final_matrix.at<float>(3, 2)*point_.z + final_matrix.at<float>(3, 3);
		//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
		x_ /= w_;
		y_ /= w_;
		z_ /= w_;

		int u = 1000 + x_*1000.0;
		int v = 1000 + y_*1000.0;
		if (u > 0 && u < 1999 && v>0 && v < 1999)
		{
			depthimage.at<float>(v, u) = z_;
		}
		//cout << u << "; " << v << "; " << z_ << endl;
		if (x_ > minPoint.x() && x_<maxPoint.x() && y_>minPoint.y() && y_ < maxPoint.y())
			//if (x_ > 0 && z_ > 0 && y_ < 0)
		{
			m_Selected[point_index] = 1;
			continue;
		}
	}
}

void CTeethModel::setRectSelectRegion(QPoint beginPos, QPoint endPoint)
{
	m_drawRectClickPosition = beginPos;
	m_drawRectEndPosition = endPoint;
}

void CTeethModel::ChangeModelSelectedColor(QPoint beginPos, QPoint endPoint,IParentInterface * pParent)
{
	for (int i = 0; i < m_vertData.size() / 8; i++) {
		float x, y, z;
		x = m_vertData[8 * i + 0] / 1.0;
		y = m_vertData[8 * i + 1] / 1.0;
		z = m_vertData[8 * i + 2] / 1.0;
		QVector3D worldPos(x, y, z);
		worldPos = pParent->world2Screen(worldPos,m_ModelMatrix);
		QRect tmprect(QPoint(beginPos.x(), beginPos.y()), QPoint(endPoint.x(), endPoint.y()));
		if (tmprect.contains(worldPos.x(), worldPos.y())) {
			m_vertData[8 * i + 7] = (1.0f);

			//cout << "ChangeSelectedColor contains x" << worldPos.x() << "contains y" << worldPos.y()<<endl;
		}
		else {
			//verticesState[i] = 0.0f;
			m_vertData[8 * i + 7] = (0.0f);
		}
	}
	BaseModel::makeObject(m_vertData, m_model.F.size());
}

void CTeethModel::makeObject()
{
	m_vertData.clear();
	{
		for (int i = 0; i < m_model.F.size(); i++)
		{
			//float x, y, z;
			m_vertData.append(m_model.P[m_model.F[i].x].x);
			m_vertData.append(m_model.P[m_model.F[i].x].y);
			m_vertData.append(m_model.P[m_model.F[i].x].z);
			//m_vertData.append((float)m_model.L[m_model.F[i].x]);
			m_vertData.append(0.0f);
			m_vertData.append(m_model.N[m_model.F[i].x].x);
			m_vertData.append(m_model.N[m_model.F[i].x].y);
			m_vertData.append(m_model.N[m_model.F[i].x].z);
			m_vertData.append(0.0);
			m_vertData.append(m_model.P[m_model.F[i].y].x);
			m_vertData.append(m_model.P[m_model.F[i].y].y);
			m_vertData.append(m_model.P[m_model.F[i].y].z);
			//m_vertData.append((float)m_model.L[m_model.F[i].y]);
			m_vertData.append(0.0f);
			m_vertData.append(m_model.N[m_model.F[i].y].x);
			m_vertData.append(m_model.N[m_model.F[i].y].y);
			m_vertData.append(m_model.N[m_model.F[i].y].z);
			m_vertData.append(0.0);
			m_vertData.append(m_model.P[m_model.F[i].z].x);
			m_vertData.append(m_model.P[m_model.F[i].z].y);
			m_vertData.append(m_model.P[m_model.F[i].z].z);
			//m_vertData.append((float)m_model.L[m_model.F[i].z]);
			m_vertData.append(0.0f);
			m_vertData.append(m_model.N[m_model.F[i].z].x);
			m_vertData.append(m_model.N[m_model.F[i].z].y);
			m_vertData.append(m_model.N[m_model.F[i].z].z);
			m_vertData.append(0.0);

		}
	}
	BaseModel::makeObject(m_vertData, m_model.F.size());
}

void CTeethModel::delSelPoints()
{
	orth::PointCloudD points;
	orth::Faces faces;
	orth::PointNormal normals;
	orth::PointLabel labels;
	//orth::PointColor colors_(mm.P.size());
	vector<int> new_point_index(m_model.P.size(), -1);

	for (int point_index = 0; point_index < m_model.P.size(); point_index++)
	{
		//cout << "point number "<<point_index;
		if (!m_Selected[point_index])
		{
			//cout<< " good ";
			points.push_back(m_model.P[point_index]);
			//colors_.push_back(mm.C[point_index]);
			normals.push_back(m_model.N[point_index]);
			//labels.push_back(m_model.L[point_index]);
			labels.push_back(0.0f);
			new_point_index[point_index] = points.size() - 1;

		}
		//cout << endl;
	}

	for (int face_index = 0; face_index < m_model.F.size(); face_index++)
	{
		if (m_Selected[m_model.F[face_index].x] || m_Selected[m_model.F[face_index].y] || m_Selected[m_model.F[face_index].z])
		{
			continue;
		}
		else
		{
			orth::face f;
			f.x = new_point_index[m_model.F[face_index].x];
			f.y = new_point_index[m_model.F[face_index].y];
			f.z = new_point_index[m_model.F[face_index].z];
			//if (f.x>mm.P.size()|| f.y>mm.P.size()|| f.z>mm.P.size())
			//{
			//	cout << mm.F[face_index].x << "; " << mm.F[face_index].y << "; " << mm.F[face_index].z << endl;
			//	cout << new_point_index[mm.F[face_index].x] << "; " << new_point_index[mm.F[face_index].y] << "; " << new_point_index[mm.F[face_index].z] << endl;
			//	cout << f.x << "; " << f.y << "; " << f.z << endl;
			//}
			faces.push_back(f);
		}
	}

	m_model.F.swap(faces);
	m_model.P.swap(points);
	m_model.N.swap(normals);
	m_model.L.swap(labels);
	makeObject();
	//mm.C.swap(colors_);
}

void CTeethModel::cutModelUnderBg(QVector3D bgGroundModelPos)
{
	cv::Mat depthimage(2000, 2000, CV_32FC1);

	m_Selected.resize(m_model.P.size());
	m_Selected.clear();

	int selected_points = 0;
	for (int point_index = 0; point_index < m_model.P.size(); point_index++)
	{
		if (m_Selected[point_index])
		{
			continue;
		}

		orth::Point3d point_ = m_model.P[point_index];
		//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
		if (point_.y < bgGroundModelPos.y()) {
			m_Selected[point_index] = 1;
			continue;
		}
	}
	delSelPoints();
	makeObject();
}

void CTeethModel::getMeshModel(orth::MeshModel &meshModel)
{
	meshModel = this->m_model;
}