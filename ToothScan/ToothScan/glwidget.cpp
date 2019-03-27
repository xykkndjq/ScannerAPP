/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "glwidget.h"
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QMouseEvent>
#include "TeethModel.h"


void ChosePoints(const float point1_x, const float point1_y, const float point2_x, const float point2_y, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm)
{
	cv::Mat depthimage(2000, 2000, CV_32FC1);
	float min_x = min(point1_x, point2_x);
	float min_y = min(point1_y, point2_y);
	float max_x = max(point1_x, point2_x);
	float max_y = max(point1_y, point2_y);

	min_x /= screen_width;
	max_x /= screen_width;
	min_y /= screen_height;
	max_y /= screen_height;

	cout << min_x << " < x <" << max_x << endl;
	cout << min_y << " < y <" << max_y << endl;

	//if (mm.Selected.size() == 0)
	//{
	//	mm.Selected.resize(mm.P.size());
	//}
	orth::PointLabel Selected(mm.P.size());

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	model_matrix = model_matrix.t();

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.t();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.t();

	cv::Mat final_matrix = projection_matrix*view_matrix*model_matrix;

	int selected_points = 0;
	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		if (Selected[point_index])
		{
			continue;
		}

		orth::Point3d point_ = mm.P[point_index];
		double x_ = final_matrix.at<float>(0, 0)*point_.x + final_matrix.at<float>(0, 1)*point_.x + final_matrix.at<float>(0, 2)*point_.x + final_matrix.at<float>(0, 3);
		double y_ = final_matrix.at<float>(1, 0)*point_.x + final_matrix.at<float>(1, 1)*point_.x + final_matrix.at<float>(1, 2)*point_.x + final_matrix.at<float>(1, 3);
		double z_ = final_matrix.at<float>(2, 0)*point_.x + final_matrix.at<float>(2, 1)*point_.x + final_matrix.at<float>(2, 2)*point_.x + final_matrix.at<float>(2, 3);
		double w_ = final_matrix.at<float>(3, 0)*point_.x + final_matrix.at<float>(3, 1)*point_.x + final_matrix.at<float>(3, 2)*point_.x + final_matrix.at<float>(3, 3);
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
		//if (x_>min_x&&x_<max_x&&y_>min_y&&y_<max_y)
		if (x_ > 0 && z_ > 0 && y_ < 0)
		{
			Selected[point_index] = 1;
			continue;
		}

	}

	orth::PointCloudD points;
	orth::Faces faces;
	orth::PointNormal normals;
	orth::PointLabel labels;
	//orth::PointColor colors_(mm.P.size());
	vector<int> new_point_index(mm.P.size(), -1);

	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		//cout << "point number "<<point_index;
		if (!Selected[point_index])
		{
			//cout<< " good ";
			points.push_back(mm.P[point_index]);
			//colors_.push_back(mm.C[point_index]);
			normals.push_back(mm.N[point_index]);
			labels.push_back(mm.L[point_index]);
			new_point_index[point_index] = points.size() - 1;

		}
		//cout << endl;
	}

	for (int face_index = 0; face_index < mm.F.size(); face_index++)
	{
		if (Selected[mm.F[face_index].x] || Selected[mm.F[face_index].y] || Selected[mm.F[face_index].z])
		{
			continue;
		}
		else
		{
			orth::face f;
			f.x = new_point_index[mm.F[face_index].x];
			f.y = new_point_index[mm.F[face_index].y];
			f.z = new_point_index[mm.F[face_index].z];
			//if (f.x>mm.P.size()|| f.y>mm.P.size()|| f.z>mm.P.size())
			//{
			//	cout << mm.F[face_index].x << "; " << mm.F[face_index].y << "; " << mm.F[face_index].z << endl;
			//	cout << new_point_index[mm.F[face_index].x] << "; " << new_point_index[mm.F[face_index].y] << "; " << new_point_index[mm.F[face_index].z] << endl;
			//	cout << f.x << "; " << f.y << "; " << f.z << endl;
			//}
			faces.push_back(f);
		}
	}

	mm.F.swap(faces);
	mm.P.swap(points);
	mm.N.swap(normals);
	mm.L.swap(labels);
	//mm.C.swap(colors_);
}

QVector4D point_ = QVector4D(0, 0, 0, 0), point_end = QVector4D(0, 0, 0, 0);
GLWidget::GLWidget(QWidget *parent)
	: QOpenGLWidget(parent),
	clearColor(Qt::GlobalColor::white),
	xRot(0),
	yRot(0),
	zRot(0),
	xTrans(0),
	yTrans(0),
	zTrans(0),
	m_bSelectRegion(false),
	m_bkGroundProgram(0),
	m_bgGroundModelPos(0, 0, 0),
	m_bkGroundShow(false),
	m_bkGroundColor(0.65f, 0.79f, 1.0f, 0.7f),
	program(0)
{
	SCR_WIDTH = this->frameGeometry().width();
	SCR_HEIGHT = this->frameGeometry().height();

	orth::ModelRead mr("./0016.ply", mm);
	TeethSegmentRun("./0016.txt");

	model_Mat = cv::Mat::eye(4, 4, CV_32FC1);
	view_Mat = cv::Mat::eye(4, 4, CV_32FC1);
	//project1 = cv::Mat::eye(4, 4, CV_32FC1);
}

GLWidget::~GLWidget()
{
	makeCurrent();
	//vbo.destroy();
	//delete program;
// 	m_bkgroundvbo.destroy();
// 	delete m_bkGroundProgram;
	m_ModelsVt.clear();
	doneCurrent();
}

QSize GLWidget::minimumSizeHint() const
{
	return QSize(500, 500);
}

QSize GLWidget::sizeHint() const
{
	return QSize(1000, 1000);
}

void GLWidget::setWindowSize(QSize &size)
{
	SCR_WIDTH = size.width();
	SCR_HEIGHT = size.height();
	cout << "SCR_WIDTH = " << SCR_WIDTH << endl;
	cout << "SCR_HEIGHT = " << SCR_HEIGHT << endl;
}

void GLWidget::rotateBy(int xAngle, int yAngle, int zAngle)
{
	xRot += xAngle * 4;
	yRot += yAngle * 4;
	zRot += zAngle;
	//std::cout << xRot <<" -- " << yRot<< std::endl;
	update();
}

void GLWidget::translateBy(int xAngle, int yAngle, int zAngle)
{
	xTrans += /*xAngle; */(xAngle / 1920.0)*((1.37*150.0*300.0) / 150.0);
	yTrans += /*yAngle; */(yAngle / 1080.0)*((-1.37*150.0*300.0) / 150.0);
	zTrans += zAngle;
	vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
	for (; iter != m_ModelsVt.end(); iter++) {
		pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
		if (pModel) {
			pModel->setScreenPos(xTrans,yTrans);
		}
	}
	//std::cout << xTrans <<" -- " << yTrans << std::endl;
	update();
}

void GLWidget::setClearColor(const QColor &color)
{
	clearColor = color;
	update();
}

//const char* GLWidget::ReadShader(const char* Path)
//{
//	std::cout << Path << std::endl;
//	// 1. retrieve the vertex/fragment source code from filePath
//	std::string vertexCode;
//	std::ifstream vShaderFile;
//	// ensure ifstream objects can throw exceptions:
//	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
//	try
//	{
//		// open files
//		vShaderFile.open(Path);
//		std::stringstream vShaderStream, fShaderStream;
//		// read file's buffer contents into streams
//		vShaderStream << vShaderFile.rdbuf();
//		// close file handlers
//		vShaderFile.close();
//		// convert stream into string
//		vertexCode = vShaderStream.str();
//	}
//	catch (std::ifstream::failure e)
//	{
//		std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
//	}
//	const char* vShaderCode = vertexCode.c_str();
//	std::cout << vertexCode << std::endl;
//	return vShaderCode;
//	
//}

QByteArray GLWidget::ReadShader(const QString &Path)
{
	QByteArray data;
	QFile qrcFile(Path);
	if (qrcFile.open(QIODevice::ReadOnly))
	{
		data = qrcFile.readAll();
	}
	else {
		qWarning() << "Failed to load input Shader file, falling back to default";
	}
	return data;

}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	//createGeometry();
	


	//vertices_in.clear();
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);

	QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
	vshader->compileSourceCode(ReadShader("./gl2.vs"));

	QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
	fshader->compileSourceCode(ReadShader("./gl2.fs"));

	program = make_shared<QOpenGLShaderProgram>();
	program->addShader(vshader);
	program->addShader(fshader);
	makeObject();
	//program = new QOpenGLShaderProgram;
	//program->addShader(vshader);
	//program->addShader(fshader);
// 	program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
// 	program->bindAttributeLocation("aNormal", PROGRAM_NORMAL_ATTRIBUTE);
// 	program->bindAttributeLocation("aMateriala", PROGRAM_MATERIAL_ATTRIBUTE);
// 	program->bindAttributeLocation("aState", PROGRAM_STATE_ATTRIBUTE);
// 	program->link();
	//program->bind();

	

	cout << "initialize done !" << endl;


	
	//m_bkGroundProgram = new QOpenGLShaderProgram;
// 	m_bkGroundProgram = make_shared<QOpenGLShaderProgram>();
// 	QOpenGLShader *bgvshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
// 	bgvshader->compileSourceCode(ReadShader("./bgGround.vs"));
// 
// 	QOpenGLShader *bgfshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
// 	bgfshader->compileSourceCode(ReadShader("./bgGround.fs"));
// 	m_bkGroundProgram->addShader(bgvshader);
// 	m_bkGroundProgram->addShader(bgfshader);
	makeGroundObject();
// 	m_bkGroundProgram->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
// 	m_bkGroundProgram->link();

	makeAxisObject();
// 	m_AxisNodeProgram = new QOpenGLShaderProgram;
// 	QOpenGLShader *axisVsShader = new QOpenGLShader(QOpenGLShader::Vertex, this);
// 	axisVsShader->compileSourceCode(ReadShader("./AxisNode.vs"));
// 
// 	QOpenGLShader *axisFsShader = new QOpenGLShader(QOpenGLShader::Fragment, this);
// 	axisFsShader->compileSourceCode(ReadShader("./AxisNode.fs"));
// 	m_AxisNodeProgram->addShader(axisVsShader);
// 	m_AxisNodeProgram->addShader(axisFsShader);
//  	m_AxisNodeProgram->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
//  	m_AxisNodeProgram->bindAttributeLocation("aNormal", PROGRAM_NORMAL_ATTRIBUTE);
// 	m_AxisNodeProgram->link();

}

void GLWidget::paintGL()
{

	glClearColor(clearColor.redF(), clearColor.greenF(), clearColor.blueF(), clearColor.alphaF());
	//cout << clearColor.redF() << " ;" << clearColor.greenF() << " ;" << clearColor.blueF() << " ;" << endl;
	//glClearColor()
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	drawGradient();
	
	m_projection.setToIdentity();
	m_projection.perspective(FOV, (float)SCR_WIDTH / (float)SCR_HEIGHT, 1.0f, 300.0f);

	m_view.setToIdentity();
	m_view.translate(0.0f, 0.0f, -230.0f);
	m_view.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	m_view.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	m_view.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);

	vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
	for (; iter != m_ModelsVt.end(); iter++) {
		glUseProgram(0);
		(*iter)->OnPaint(m_projection,m_view,this);
	}

	iter = m_ToolsModelsVt.begin();
	for (; iter != m_ToolsModelsVt.end(); iter++) {
		glUseProgram(0);
		(*iter)->OnPaint(m_projection, m_view,this);
	}

	//drawAXIS();
// 	if (totalFaceNum > 0) {
// 		program->bind();
// 		vbo.bind();
// 		//QMatrix4x4 m;
// 		//m.ortho(-5.0f, +5.0f, +5.0f, -5.0f, -5.0f, 35.0f);
// 		//m.translate(0.0f, 0.0f, -10.0f);
// 		//m.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 		//m.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 		//m.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
// 
// 		//QMatrix4x4 projection, view, model;
// 		m_projection.setToIdentity();
// 		m_projection.perspective(FOV, (float)SCR_WIDTH / (float)SCR_HEIGHT, 1.0f, 300.0f);
// 		program->setUniformValue("projection", m_projection);
// 
// 		//cout << projection.data()[0] << ", " << projection.data()[4] << ", " << projection.data()[8] << ", " << projection.data()[12] <<  endl;
// 		//cout << projection.data()[1] << ", " << projection.data()[5] << ", " << projection.data()[9] << ", " << projection.data()[13] << endl;
// 		//cout << projection.data()[2] << ", " << projection.data()[6] << ", " << projection.data()[10] << ", " << projection.data()[14] << endl;
// 		//cout << projection.data()[3] << ", " << projection.data()[7] << ", " << projection.data()[11] << ", " << projection.data()[15] << endl;
// 		//cout << " ---------------------------------------------------- " << endl;
// 		//cout << endl;
// 
// 		m_view.setToIdentity();
// 		//m_view.translate(0.0f, 0.0f, -230.0f);
// 		m_view.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 		m_view.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 		m_view.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
// 		program->setUniformValue("view", m_view);
// 		program->setUniformValue("viewPos", m_view.column(3));
// 		//cout << view.data()[0] << ", " << view.data()[4] << ", " << view.data()[8] << ", " << view.data()[12] << endl;
// 		//cout << view.data()[1] << ", " << view.data()[5] << ", " << view.data()[9] << ", " << view.data()[13] << endl;
// 		//cout << view.data()[2] << ", " << view.data()[6] << ", " << view.data()[10] << ", " << view.data()[14] << endl;
// 		//cout << view.data()[3] << ", " << view.data()[7] << ", " << view.data()[11] << ", " << view.data()[15] << endl;
// 		//cout << endl;
// 
// 		m_model.setToIdentity();
// // 		m_model.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// // 		m_model.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// // 		m_model.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
// 		//model.translate(xRot*0.01, yRot*0.01, zRot*0.01);
// 		program->setUniformValue("model", m_model);
// 		program->setUniformValue("inv_model", m_model.inverted());
// 		program->setUniformValue("screenPos", QVector3D(xTrans, yTrans, 0));
// 
// 		//memcpy(model1.data, model.data(), 16 * sizeof(float));
// 		//memcpy(view1.data, view.data(), 16 * sizeof(float));
// 		//memcpy(project1.data, projection.data(), 16 * sizeof(float));
// 		//point_end = model*point_;
// 		//cout << point_end.x() << ", " << point_end.y() << ", " << point_end.z() << ", " << point_end.w() << endl;
// 		//point_end = view*point_end;
// 		//cout << point_end.x() << ", " << point_end.y() << ", " << point_end.z() << ", " << point_end.w() << endl;
// 		//point_end = projection*point_end;
// 		//cout << point_end.x() / point_end.w() << ", " << point_end.y() / point_end.w() << ", " << point_end.z() / point_end.w() << ", " << point_end.w() / point_end.w() << endl;
// 		//cout << "----------------------------------------------" << endl;
// 
// 		//cout << model.data()[0] << ", " << model.data()[4] << ", " << model.data()[8] << ", " << model.data()[12] << endl;
// 		//cout << model.data()[1] << ", " << model.data()[5] << ", " << model.data()[9] << ", " << model.data()[13] << endl;
// 		//cout << model.data()[2] << ", " << model.data()[6] << ", " << model.data()[10] << ", " << model.data()[14] << endl;
// 		//cout << model.data()[3] << ", " << model.data()[7] << ", " << model.data()[11] << ", " << model.data()[15] << endl;
// 		//cout << endl;
// 		//vector<float> a(4,0);
// 		//cout << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << endl;
// 		//for (int i = 0; i < 4; i++)
// 		//{
// 		//	for (int j = 0; j < 4; j++)
// 		//	{
// 		//		a[j] += model.data()[j + 4 * i] * fv[i];
// 		//	}
// 		//}
// 		//cout << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << endl;
// 		//cout << "-----------------------------" << endl;
// 		//vector<float> a1(4, 0);
// 		//for (int i = 0; i < 4; i++)
// 		//{
// 		//	a1[0] += view.column(i).x()*a[i];
// 		//	a1[1] += view.column(i).y()*a[i];
// 		//	a1[2] += view.column(i).z()*a[i];
// 		//	a1[3] += view.column(i).w()*a[i];
// 		//	
// 		//}
// 		//cout << a1[0] << ", " << a1[1] << ", " << a1[2] << ", " << a1[3] << endl;cout << "-----------------------------" << endl;
// 		//vector<float> a2(4, 0);
// 		//for (int i = 0; i < 4; i++)
// 		//{
// 		//	a2[0] += projection.column(i).x() *a1[i];
// 		//	a2[1] += projection.column(i).y() *a1[i];
// 		//	a2[2] += projection.column(i).z() *a1[i];
// 		//	a2[3] += projection.column(i).w() *a1[i];
// 		//	
// 		//}
// 		//cout << a2[0] << ", " << a2[1] << ", " << a2[2] << ", " << a2[3] << endl;cout << "-----------------------------" << endl;
// 		program->setUniformValue("lightPos", QVector3D(0, 0, 0));
// 		//cout << view.column(3).toVector3D()[0] <<", " << view.column(3).toVector3D()[1] << ", " << view.column(3).toVector3D()[2]<< endl;
// 		program->setUniformValue("lightColor", QVector3D(1.0f, 1.0f, 1.0f));
// 		program->setUniformValue("objectColor", QVector3D(1.2f, 0.7f, 0.71f));
// 		program->setUniformValue("selectColor", QVector3D(1.0f, 0.0f, 0.0f));
// 		//program->setUniformValue("gingivaColor", QVector3D(1.3f, 1.3f, 1.3f));
// 		//program->setUniformValue("teethColor", QVector3D(1.3f, 1.3f, 1.3f));
// 
// 		program->setUniformValue("material1.ambient", 0.05f, 0.0f, 0.0f);
// 		program->setUniformValue("material1.diffuse", 0.5f, 0.4f, 0.4f);
// 		program->setUniformValue("material1.specular", 0.8f, 0.04f, 0.04f);
// 		program->setUniformValue("material1.shininess", 0.978125f);
// 
// 		program->setUniformValue("material2.ambient", 0.95f, 0.95f, 0.95f);
// 		program->setUniformValue("material2.diffuse", 0.6f, 0.6f, 0.6f);
// 		program->setUniformValue("material2.specular", 1.2f, 1.2f, 1.2f);
// 		program->setUniformValue("material2.shininess", 32.0f);
// 
// 		program->setUniformValue("material3.ambient", 0.0f, 0.5f, 0.0f);
// 		program->setUniformValue("material3.diffuse", 0.5f, 0.4f, 0.4f);
// 		program->setUniformValue("material3.specular", 0.8f, 0.04f, 0.04f);
// 		program->setUniformValue("material3.shininess", 0.978125f);
// 
// 		// directional light
// 		program->setUniformValue("dirLight.direction", 0.0f, 0.0f, 1.0f);
// 		program->setUniformValue("dirLight.ambient", 0.45f, 0.45f, 0.45f);
// 		program->setUniformValue("dirLight.diffuse", 0.6f, 0.6f, 0.6f);
// 		program->setUniformValue("dirLight.specular", 0.7f, 0.7f, 0.7f);
// 		// point light 1
// 		program->setUniformValue("pointLights[0].position", 0.7f, 0.2f, 2.0f);
// 		program->setUniformValue("pointLights[0].ambient", 0.05f, 0.05f, 0.05f);
// 		program->setUniformValue("pointLights[0].diffuse", 0.8f, 0.8f, 0.8f);
// 		program->setUniformValue("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("pointLights[0].constant", 1.0f);
// 		program->setUniformValue("pointLights[0].linear", 0.09f);
// 		program->setUniformValue("pointLights[0].quadratic", 0.032f);
// 		// point light 2
// 		program->setUniformValue("pointLights[1].position", 2.3f, -3.3f, -4.0f);
// 		program->setUniformValue("pointLights[1].ambient", 0.05f, 0.05f, 0.05f);
// 		program->setUniformValue("pointLights[1].diffuse", 0.8f, 0.8f, 0.8f);
// 		program->setUniformValue("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("pointLights[1].constant", 1.0f);
// 		program->setUniformValue("pointLights[1].linear", 0.09f);
// 		program->setUniformValue("pointLights[1].quadratic", 0.032f);
// 		// point light 3
// 		program->setUniformValue("pointLights[2].position", -4.0f, 2.0f, -12.0f);
// 		program->setUniformValue("pointLights[2].ambient", 0.05f, 0.05f, 0.05f);
// 		program->setUniformValue("pointLights[2].diffuse", 0.8f, 0.8f, 0.8f);
// 		program->setUniformValue("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("pointLights[2].constant", 1.0f);
// 		program->setUniformValue("pointLights[2].linear", 0.09f);
// 		program->setUniformValue("pointLights[2].quadratic", 0.032f);
// 		// point light 4
// 		program->setUniformValue("pointLights[3].position", 0.0f, 0.0f, -3.0f);
// 		program->setUniformValue("pointLights[3].ambient", 0.05f, 0.05f, 0.05f);
// 		program->setUniformValue("pointLights[3].diffuse", 0.8f, 0.8f, 0.8f);
// 		program->setUniformValue("pointLights[3].specular", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("pointLights[3].constant", 1.0f);
// 		program->setUniformValue("pointLights[3].linear", 0.09f);
// 		program->setUniformValue("pointLights[3].quadratic", 0.032f);
// 		// spotLight
// 		program->setUniformValue("spotLight.position", 0.0f, 0.0f, 0.0f);
// 		program->setUniformValue("spotLight.direction", 0.0f, 0.0f, 1.0f);
// 		program->setUniformValue("spotLight.ambient", 0.0f, 0.0f, 0.0f);
// 		program->setUniformValue("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("spotLight.specular", 1.0f, 1.0f, 1.0f);
// 		program->setUniformValue("spotLight.constant", 1.0f);;
// 		program->setUniformValue("spotLight.linear", 0.09f);
// 		program->setUniformValue("spotLight.quadratic", 0.032f);
// 		program->setUniformValue("spotLight.cutOff", float(cos(0.2094)));
// 		program->setUniformValue("spotLight.outerCutOff", float(cos(0.2618)));
// 
// 
// 		//this->update();
// 
// 		program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
// 		program->enableAttributeArray(PROGRAM_NORMAL_ATTRIBUTE);
// 		program->enableAttributeArray(PROGRAM_MATERIAL_ATTRIBUTE);
// 		program->enableAttributeArray(PROGRAM_STATE_ATTRIBUTE);
// 		program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat));
// 		program->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 4 * sizeof(GLfloat), 3, 8 * sizeof(GLfloat));
// 		program->setAttributeBuffer(PROGRAM_MATERIAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 1, 8 * sizeof(GLfloat));
// 		program->setAttributeBuffer(PROGRAM_STATE_ATTRIBUTE, GL_FLOAT, 7 * sizeof(GLfloat), 1, 8 * sizeof(GLfloat));
// 
// 		if (getSelectReginValue()) {
// 			ChangeSelectedColorEx(m_drawRectClickPosition, m_drawRectEndPosition);
// 		}
// 		else {
// 			program->setUniformValue("selectedAreaBegin", 0.0f, 0.0f);
// 			program->setUniformValue("selectedAreaEnd", 0.0f, 0.0f);
// 		}
// 		glDrawArrays(GL_TRIANGLES, 0, totalFaceNum * 3);
// 	}

// 	if (getbkGroundShowValue()) {
// glUseProgram(0);
// m_bkGroundProgram->bind();
// m_bkgroundvbo->bind();
// 
//  		glDepthMask(GL_FALSE);
//  		glEnable(GL_BLEND);
//  		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  		
// 
//  		m_bkGroundProgram->setUniformValue("projection", m_projection);
//  		m_bkGroundProgram->setUniformValue("view", m_view);
//  		m_bgGroundModel.setToIdentity();
//  // 		m_bgGroundModel.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
//  // 		m_bgGroundModel.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
//  // 		m_bgGroundModel.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
//  		//m_bgGroundModel.translate(m_bgGroundModelPos);
//  		m_bkGroundProgram->setUniformValue("model", m_bgGroundModel);
//  		m_bkGroundProgram->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
//  		m_bkGroundProgram->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 3 * sizeof(GLfloat));
//  		m_bkGroundProgram->setUniformValue("ourColor", m_bkGroundColor);
//  		glDrawArrays(GL_QUADS, 0, 4);
//  		glDisable(GL_BLEND);
//  		glDepthMask(true);
// 	}

	if (getSelectReginValue()) {
		drawRect(m_drawRectClickPosition.x(), m_drawRectClickPosition.y(), m_drawRectEndPosition.x(), m_drawRectEndPosition.y());
	}
	//DrawAxisObject();
}

void GLWidget::resizeGL(int width, int height)
{
	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	if (getSelectReginValue()) {
		if (event->buttons()&Qt::LeftButton) {
			m_drawRectClickPosition = QPoint(event->x(), SCR_HEIGHT - event->y());
		}
	}
	else
		lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (getSelectReginValue()) {
		if (event->buttons()&Qt::LeftButton) {
			m_drawRectEndPosition = QPoint(event->x(), SCR_HEIGHT - event->y());
			vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
			for (; iter != m_ModelsVt.end(); iter++) {
				pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
				if (pModel) {
					pModel->setRectSelectRegion(m_drawRectClickPosition, m_drawRectEndPosition);
				}
			}
			update();
		}
	}
	else {
		int dx = event->x() - lastPos.x();
		int dy = event->y() - lastPos.y();
		//cout << dx << " ----- " << dy << endl;

		//if (event->buttons()&Qt::LeftButton)
		//{
		//	if (event->modifiers() == Qt::CTRL)
		//	{
		//		program->executeTranslateOperation(event->x(), event->y());
		//	}
		//	else
		//	{
		//		program->executeRotateOperation(event->x(), event->y());
		//	}
		//}

		if (event->buttons() & Qt::LeftButton) {
			rotateBy(dy, dx, 0);
		}
		else if (event->buttons() & Qt::RightButton) {
			translateBy(dx, dy, 0);
		}
		lastPos = event->pos();
	}
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
	if (event->delta() > 0)
	{
		FOV += 1.0f;
	}
	else
	{
		FOV -= 1.0f;
	}
	this->repaint();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *  event)
{
	cout << "release done !" << endl;
	//vertices_in.clear();
	//ChosePoints(0, 0, 500, 500, 1920, 1080, model1, view1, project1, mm);
	//makeObject();

	emit clicked();
	if (getSelectReginValue()) {
		vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
		for (; iter != m_ModelsVt.end(); iter++) {
			pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
			if (pModel) {
				pModel->ChangeModelSelectedColor(m_drawRectClickPosition, m_drawRectEndPosition,this);
				pModel->ChosePoints(m_drawRectClickPosition.x(), m_drawRectClickPosition.y(), m_drawRectEndPosition.x(), m_drawRectEndPosition.y(),
					SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), this);
				pModel->setRectSelectRegion(QPoint(0,0), QPoint(0, 0));
			}
		}
		//ChangeModelSelectedColor(m_drawRectClickPosition, m_drawRectEndPosition);
		//ChosePoints(m_drawRectClickPosition.x(), m_drawRectClickPosition.y(), m_drawRectEndPosition.x(), m_drawRectEndPosition.y(),\
			SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), this);
	}
}

void GLWidget::makeObject()
{
	/*point_.setX(mm.P[mm.F[0].x].x / 1.0);
	point_.setY(mm.P[mm.F[0].x].y / 1.0);
	point_.setZ(mm.P[mm.F[0].x].z / 1.0);
	point_.setW(1);*/

	/*for (int i = 0; i < mm.F.size(); i++)
	{
		vertices_in.push_back(mm.P[mm.F[i].x].x);
		vertices_in.push_back(mm.P[mm.F[i].x].y);
		vertices_in.push_back(mm.P[mm.F[i].x].z);
		label_in.push_back((float)mm.L[mm.F[i].x]);

		vertices_in.push_back(mm.P[mm.F[i].y].x);
		vertices_in.push_back(mm.P[mm.F[i].y].y);
		vertices_in.push_back(mm.P[mm.F[i].y].z);
		label_in.push_back((float)mm.L[mm.F[i].y]);

		vertices_in.push_back(mm.P[mm.F[i].z].x);
		vertices_in.push_back(mm.P[mm.F[i].z].y);
		vertices_in.push_back(mm.P[mm.F[i].z].z);
		label_in.push_back((float)mm.L[mm.F[i].z]);
	}

	if (mm.N.size()!=0)
	{

		for (int i = 0; i < mm.F.size(); i++)
		{
			normal_in.push_back(mm.N[mm.F[i].x].x);
			normal_in.push_back(mm.N[mm.F[i].x].y);
			normal_in.push_back(mm.N[mm.F[i].x].z);

			normal_in.push_back(mm.N[mm.F[i].y].x);
			normal_in.push_back(mm.N[mm.F[i].y].y);
			normal_in.push_back(mm.N[mm.F[i].y].z);

			normal_in.push_back(mm.N[mm.F[i].z].x);
			normal_in.push_back(mm.N[mm.F[i].z].y);
			normal_in.push_back(mm.N[mm.F[i].z].z);
		}
	}*/

	//vertData.clear();
	//if (vertices_in.size()>0)
// 	{
// 		for (int i = 0; i < mm.F.size(); i++)
// 		{
// 			//float x, y, z;
// 			vertData.append(mm.P[mm.F[i].x].x);
// 			vertData.append(mm.P[mm.F[i].x].y);
// 			vertData.append(mm.P[mm.F[i].x].z);
// 			vertData.append((float)mm.L[mm.F[i].x]);
// 			vertData.append(mm.N[mm.F[i].x].x);
// 			vertData.append(mm.N[mm.F[i].x].y);
// 			vertData.append(mm.N[mm.F[i].x].z);
// 			vertData.append(0.0);
// 			vertData.append(mm.P[mm.F[i].y].x);
// 			vertData.append(mm.P[mm.F[i].y].y);
// 			vertData.append(mm.P[mm.F[i].y].z);
// 			vertData.append((float)mm.L[mm.F[i].y]);
// 			vertData.append(mm.N[mm.F[i].y].x);
// 			vertData.append(mm.N[mm.F[i].y].y);
// 			vertData.append(mm.N[mm.F[i].y].z);
// 			vertData.append(0.0);
// 			vertData.append(mm.P[mm.F[i].z].x);
// 			vertData.append(mm.P[mm.F[i].z].y);
// 			vertData.append(mm.P[mm.F[i].z].z);
// 			vertData.append((float)mm.L[mm.F[i].z]);
// 			vertData.append(mm.N[mm.F[i].z].x);
// 			vertData.append(mm.N[mm.F[i].z].y);
// 			vertData.append(mm.N[mm.F[i].z].z);
// 			vertData.append(0.0);
// 
// 		}
// 		totalFaceNum += mm.F.size();
// 	}
	//if (m_vertices.size()>0)
	//{
	//	for (int i = 0; i < m_vertices.size(); i++)
	//	{
	//		vertData.append(m_vertices[i].x());
	//		vertData.append(m_vertices[i].y());
	//		vertData.append(m_vertices[i].z());
	//		vertData.append(m_normals[i].x());
	//		vertData.append(m_normals[i].y());
	//		vertData.append(m_normals[i].z());
	//	}	
	//}
	pQOpenGLBuffer vbo;
	vbo = make_shared<QOpenGLBuffer>();
// 	vbo->create();
// 	vbo->bind();
// 	vbo->allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));

	pCTeethModel l_TeethModel = make_shared<CTeethModel>(program, vbo,mm);
	l_TeethModel->makeObject();
	m_ModelsVt.push_back(l_TeethModel);
	this->update();
}

void GLWidget::TeethSegmentRun(const std::string label_file_path)
{
	//
	//		write your main logic:
	//		calculate PointLabel
	//

	char buffer[256];
	std::fstream outFile;
	outFile.open(label_file_path, std::ios::in);
	//std::cout << label_file_path << "--- all file is as follows:---" << std::endl;
	while (!outFile.eof())
	{
		outFile.getline(buffer, 256, '\n');//getline(char *,int,char) 表示该行字符达到256个或遇到换行就结束
										   //std::cout << buffer << "-" << std::endl;
		if (strcmp(buffer, "background") == 0)
		{
			mm.L.push_back(0);
		}
		else
		{
			mm.L.push_back(1);
		}
	}
	outFile.close();

}

void GLWidget::reSetValue()
{
	FOV = 20;
	xRot = 0;
	yRot = 0;
	zRot = 0;
}

void GLWidget::overView()
{
	reSetValue();
	xRot = -90 * 16;
	update();
}

void GLWidget::upwardView()
{
	reSetValue();
	xRot = 90 * 16;
	update();
}

void GLWidget::leftView()
{
	reSetValue();
	yRot = -90 * 16;
	update();
}

void GLWidget::rightView()
{
	reSetValue();
	yRot = 90 * 16;
	update();
}

void GLWidget::mainView()
{
	reSetValue();
	update();
}

void GLWidget::backView()
{

}

void GLWidget::enlargeView()
{
	FOV -= 1.0f;
	update();
}

void GLWidget::shrinkView()
{
	FOV += 1.0f;
	update();
}

void GLWidget::selectRegion(bool bSelected)
{
	m_drawRectClickPosition = QPoint(0, 0);
	m_drawRectEndPosition = QPoint(0, 0);
	setSelectRegionValue(bSelected);
	update();
}

void GLWidget::setSelectRegionValue(bool bSelected)
{
	m_bSelectRegion = bSelected;
}

bool GLWidget::getSelectReginValue()
{
	return m_bSelectRegion;
}

void GLWidget::delSelected()
{
	vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
	for (; iter != m_ModelsVt.end(); iter++) {
		pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
		if (pModel) {
			pModel->delSelPoints();
		}
	}
// 	delSelPoints();
// 	remakeObject();
	update();
}

void GLWidget::confirmSelRegion()
{

}

void GLWidget::drawRect(int x, int y, int x1, int y1)
{
	QOpenGLContext *context = QOpenGLContext::currentContext();
	QOpenGLFunctions *f = context->functions();
	f->glUseProgram(0);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	GLdouble width = SCR_WIDTH;
	GLdouble height = SCR_HEIGHT;
	cout << "drawRect width " << width << "drawRect height" << height << endl;
	glOrtho(0, width, 0, height, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_COLOR_LOGIC_OP);
	glLogicOp(GL_XOR);
	glColor3f(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	GLint start[] = { x,y };
	glVertex2iv(start);
	glVertex2f(x1, y);
	GLint end[] = { x1, y1 };
	glVertex2iv(end);
	glVertex2f(x, y1);
	glEnd();
	glDisable(GL_LOGIC_OP);

	// Closing 2D
	glPopAttrib();
	glPopMatrix(); // restore modelview
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void GLWidget::ChangeSelectedColorEx(QPoint drawRectClickPosition, QPoint drawRectEndPosition)
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
	QVector3D pointBegin3d = screen2world(pointBegin.x(), pointBegin.y()),
		pointEnd3d = screen2world(pointEnd.x(), pointEnd.y());
	program->setUniformValue("selectedAreaBegin", pointBegin3d.x(), pointBegin3d.y());
	program->setUniformValue("selectedAreaEnd", pointEnd3d.x(), pointEnd3d.y());
}

void GLWidget::ChangeModelSelectedColor(QPoint drawRectClickPosition, QPoint drawRectEndPosition)
{
	for (int i = 0; i < vertData.size() / 8; i++) {
		float x, y, z;
		x = vertData[8 * i + 0] / 1.0;
		y = vertData[8 * i + 1] / 1.0;
		z = vertData[8 * i + 2] / 1.0;
		QVector3D worldPos(x, y, z);
		worldPos = world2Screen(worldPos);
		QRect tmprect(QPoint(drawRectClickPosition.x(), drawRectClickPosition.y()), QPoint(drawRectEndPosition.x(), drawRectEndPosition.y()));
		if (tmprect.contains(worldPos.x(), worldPos.y())) {
			vertData[8 * i + 7] = (1.0f);

			//cout << "ChangeSelectedColor contains x" << worldPos.x() << "contains y" << worldPos.y()<<endl;
		}
		else {
			//verticesState[i] = 0.0f;
			vertData[8 * i + 7] = (0.0f);
		}
	}
// 	vbo->create();
// 	vbo->bind();
// 	vbo->allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
// 	this->update();
}

QVector3D GLWidget::screen2world(int x, int y)
{
	QVector3D worldPosition = QVector3D((float)((signed)(x - SCR_WIDTH / 2)) / (SCR_WIDTH / 2), (float)((signed)(y - SCR_HEIGHT / 2)) / (SCR_HEIGHT / 2), 0);
	return worldPosition;
}

QVector3D GLWidget::world2Screen(QVector3D worldPos)
{
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	GLint l_view[4] = { 0,0,SCR_WIDTH,SCR_HEIGHT };
	QVector3D worldPosition = worldPos.project(QMatrix4x4(m_model*m_view), QMatrix4x4(m_projection), QRect(l_view[0], l_view[1], l_view[2], l_view[3]));
	//worldPosition = QVector3D(x, SCR_HEIGHT - y, 1).unproject(QMatrix4x4(model), QMatrix4x4(projection), QRect(view[0], view[1], view[2], view[3]));
	return worldPosition;
}

void GLWidget::remakeObject()
{
	vertData.clear();
	{
		for (int i = 0; i < mm.F.size(); i++)
		{
			//float x, y, z;
			vertData.append(mm.P[mm.F[i].x].x);
			vertData.append(mm.P[mm.F[i].x].y);
			vertData.append(mm.P[mm.F[i].x].z);
			vertData.append((float)mm.L[mm.F[i].x]);
			vertData.append(mm.N[mm.F[i].x].x);
			vertData.append(mm.N[mm.F[i].x].y);
			vertData.append(mm.N[mm.F[i].x].z);
			vertData.append(0.0);

			vertData.append(mm.P[mm.F[i].y].x);
			vertData.append(mm.P[mm.F[i].y].y);
			vertData.append(mm.P[mm.F[i].y].z);
			vertData.append((float)mm.L[mm.F[i].y]);
			vertData.append(mm.N[mm.F[i].y].x);
			vertData.append(mm.N[mm.F[i].y].y);
			vertData.append(mm.N[mm.F[i].y].z);
			vertData.append(0.0);
			vertData.append(mm.P[mm.F[i].z].x);
			vertData.append(mm.P[mm.F[i].z].y);
			vertData.append(mm.P[mm.F[i].z].z);
			vertData.append((float)mm.L[mm.F[i].z]);
			vertData.append(mm.N[mm.F[i].z].x);
			vertData.append(mm.N[mm.F[i].z].y);
			vertData.append(mm.N[mm.F[i].z].z);
			vertData.append(0.0);

		}
		totalFaceNum = mm.F.size();
	}
// 	if (totalFaceNum > 0) {
// 		vbo->create();
// 		vbo->bind();
// 		vbo->allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
// 	}

	this->update();
}

void GLWidget::ChosePoints(const float point1_x, const float point1_y, const float point2_x, const float point2_y, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm)
{
	cv::Mat depthimage(2000, 2000, CV_32FC1);
	float min_x = min(point1_x, point2_x);
	float min_y = min(point1_y, point2_y);
	float max_x = max(point1_x, point2_x);
	float max_y = max(point1_y, point2_y);

	QVector3D minPoint = screen2world(min_x, min_y),
		maxPoint = screen2world(max_x, max_y);
	// 	min_x /= screen_width;
	// 	max_x /= screen_width;
	// 	min_y /= screen_height;
	// 	max_y /= screen_height;

	cout << min_x << " < x <" << max_x << endl;
	cout << min_y << " < y <" << max_y << endl;

	//if (mm.Selected.size() == 0)
	//{
	//	mm.Selected.resize(mm.P.size());
	//}
	//orth::PointLabel Selected(mm.P.size());
	m_Selected.resize(mm.P.size());
	m_Selected.clear();

	//cv::Mat model_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(model_.data, model_matrix, 16 * sizeof(float));
	model_matrix = model_matrix.t();

	//cv::Mat view_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(view_.data, view_matrix, 16 * sizeof(float));
	view_matrix = view_matrix.t();

	//cv::Mat projection_ = cv::Mat::eye(4, 4, CV_32FC1);
	//memcpy(projection_.data, projection_matrix, 16 * sizeof(float));
	projection_matrix = projection_matrix.t();

	cv::Mat final_matrix = projection_matrix*view_matrix*model_matrix;

	int selected_points = 0;
	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		if (m_Selected[point_index])
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

void GLWidget::delSelPoints()
{
	orth::PointCloudD points;
	orth::Faces faces;
	orth::PointNormal normals;
	orth::PointLabel labels;
	//orth::PointColor colors_(mm.P.size());
	vector<int> new_point_index(mm.P.size(), -1);

	for (int point_index = 0; point_index < mm.P.size(); point_index++)
	{
		//cout << "point number "<<point_index;
		if (!m_Selected[point_index])
		{
			//cout<< " good ";
			points.push_back(mm.P[point_index]);
			//colors_.push_back(mm.C[point_index]);
			normals.push_back(mm.N[point_index]);
			labels.push_back(mm.L[point_index]);
			new_point_index[point_index] = points.size() - 1;

		}
		//cout << endl;
	}

	for (int face_index = 0; face_index < mm.F.size(); face_index++)
	{
		if (m_Selected[mm.F[face_index].x] || m_Selected[mm.F[face_index].y] || m_Selected[mm.F[face_index].z])
		{
			continue;
		}
		else
		{
			orth::face f;
			f.x = new_point_index[mm.F[face_index].x];
			f.y = new_point_index[mm.F[face_index].y];
			f.z = new_point_index[mm.F[face_index].z];
			//if (f.x>mm.P.size()|| f.y>mm.P.size()|| f.z>mm.P.size())
			//{
			//	cout << mm.F[face_index].x << "; " << mm.F[face_index].y << "; " << mm.F[face_index].z << endl;
			//	cout << new_point_index[mm.F[face_index].x] << "; " << new_point_index[mm.F[face_index].y] << "; " << new_point_index[mm.F[face_index].z] << endl;
			//	cout << f.x << "; " << f.y << "; " << f.z << endl;
			//}
			faces.push_back(f);
		}
	}

	mm.F.swap(faces);
	mm.P.swap(points);
	mm.N.swap(normals);
	mm.L.swap(labels);
	//mm.C.swap(colors_);
}

void GLWidget::showBkGround(bool bShow) {
	setbkGroundShowValue(bShow);
	update();
}

void GLWidget::cutModelUnderBg()
{
	if (!getbkGroundShowValue())
		return;
// 	cv::Mat depthimage(2000, 2000, CV_32FC1);
// 
// 	m_Selected.resize(mm.P.size());
// 	m_Selected.clear();
// 
// 	int selected_points = 0;
// 	for (int point_index = 0; point_index < mm.P.size(); point_index++)
// 	{
// 		if (m_Selected[point_index])
// 		{
// 			continue;
// 		}
// 
// 		orth::Point3d point_ = mm.P[point_index];
// 		//cout << x_ << "; " << y_ << "; " << z_ << "; " << w_ <<"; ";
// 		if (point_.y < m_bgGroundModelPos.y()) {
// 			m_Selected[point_index] = 1;
// 			continue;
// 		}
// 	}
// 	delSelPoints();
// 	remakeObject();
	vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
	for (; iter != m_ModelsVt.end(); iter++) {
		pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
		if (pModel) {
			pModel->cutModelUnderBg(m_bgGroundModelPos);
		}
	}
	update();
}

void GLWidget::setBgColor(QVector4D color)
{
	m_bkGroundColor = color;
	update();
}

void GLWidget::setbkGroundShowValue(bool bShow)
{
	m_bkGroundShow = bShow;
	m_groundModel->Set_bVisible(bShow);
}

bool GLWidget::getbkGroundShowValue()
{
	return m_bkGroundShow;
}

void GLWidget::SetMatrix(cv::Mat &m, cv::Mat &v)
{
	//model_Mat = m.t();
	//view_Mat = v.t();

	for (int i = 0; i < 16; i++)
	{
		model_Mat.at<float>(i) = (float)m.at<double>(i);
		view_Mat.at<float>(i) = (float)v.at<double>(i);
	}

	model_Mat = model_Mat.t();
	view_Mat = view_Mat.t();

}

void GLWidget::GetMotorRot(float &xrot, float &yrot)
{
	//model_Mat = m.t();
	//view_Mat = v.t();

	xrot = xRot / 16.0f;
	yrot = yRot / 16.0f;

}

void GLWidget::makeGroundObject()
{
	QVector<GLfloat> vertData =
// 		 	{
// 		 		50.0f, -50.0f, 0.0f,  // left
// 		 		-50.0f, -50.0f, 0.0f,  // right
// 		 		-50.0f, 50.0f, 0.0f,   // top 
// 		 		50.0f, 50.0f, 0.0f   // top 
// 		 	};
	{ 50.0f,0.0f,50.0f,50.0f,0.0f,-50.0f,-50.0f,0.0f,-50.0f,-50.0f,0.0f,50.0f };
// 	pQOpenGLBuffer bkgroundvbo;
// 	bkgroundvbo = make_shared<QOpenGLBuffer>();
	m_groundModel = make_shared<CGroundObject>("./bgGround.vs", "./bgGround.fs",this);
	m_groundModel->makeObject(vertData, 4);
	m_groundModel->Set_bVisible(false);
	m_ToolsModelsVt.push_back(m_groundModel);
	this->update();
}

void GLWidget::makeAxisObject()
{
// 	vector<GLfloat> vertData =
// 	{
// 		0.0f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
// 		0.3f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
// 		0.0f, 0.0f, 0.0f, 0.0f,1.0f,0.0f,
// 		0.0f, 0.3f, 0.0f,0.0f,1.0f,0.0f,
// 		0.0f, 0.0f, 0.0f, 0.0f,0.0f,1.0f,
// 		0.0f, 0.0f, 0.3f,0.0f,0.0f,1.0f,
// 	};
// //	{ 50.0f, 0.0f, 50.0f, 50.0f, 0.0f, -50.0f, -50.0f, 0.0f, -50.0f, -50.0f, 0.0f, 50.0f };
// 	m_AxisNodevbo.create();
// 	m_AxisNodevbo.bind();
// 	m_AxisNodevbo.allocate(vertData.data(), vertData.size() * sizeof(GLfloat));
	m_axisMode = make_shared<CAxisModel>("./AxisNode.vs","./AxisNode.fs",this);
	m_axisMode->makeObject();
	m_ToolsModelsVt.push_back(m_axisMode);
}

void GLWidget::bgGroundmoveDown()
{
	m_bgGroundModelPos.setY(m_bgGroundModelPos.y() - 1);
	m_groundModel->translate(m_bgGroundModelPos);
	update();
}

void GLWidget::bgGroundmoveUp()
{
	m_bgGroundModelPos.setY(m_bgGroundModelPos.y() + 1);
	m_groundModel->translate(m_bgGroundModelPos);
	update();
}

void GLWidget::drawGradient()
{
	makeCurrent();
	glUseProgram(0);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	GLubyte  qBeginColor[4] = { 0,0,0 ,255 },
		qEndColor[4] = { 128,128,255,255 };
	glBegin(GL_TRIANGLE_STRIP);
	glColor4ubv(qBeginColor);  glVertex2f(-1, 1);
	glColor4ubv(qEndColor);  glVertex2f(-1, -1);
	glColor4ubv(qBeginColor);  glVertex2f(1, 1);
	glColor4ubv(qEndColor);  glVertex2f(1, -1);
	glEnd();
	glPopAttrib();
	glPopMatrix(); // restore modelview
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	//doneCurrent();
}

void GLWidget::drawAXIS()
{
	float linewidth = 2.0;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glLineWidth(linewidth);
	glPointSize(linewidth*1.5);
	GLdouble a[3] = { 50.0f, 0, 0 },
		_a[3] = { -50.0f, 0, 0 },
		b[3] = { 0, 50.0f, 0 },
		_b[3] = { 0, -50.0f, 0 },
		c[3] = { 0, 0, 50.0f },
		_c[3] = { 0, 0, -50.0f };
	QVector3D test;
	// Get gl state values
	double mm[16], mp[16];
	GLint vp[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, mm);
	glGetDoublev(GL_PROJECTION_MATRIX, mp);
	glGetIntegerv(GL_VIEWPORT, vp);
	float scalefactor = 1.0f;//size*0.02f;
	GLubyte  xcolor[4] = { 255,0,0 ,255 },
		ycolor[4] = { 0,255,0,255 },
		zcolor[4] = { 0,0,255,255 };
	glBegin(GL_LINES);
	glColor4ubv(xcolor);
	glVertex3dv(_a); glVertex3dv(a);
	glColor4ubv(ycolor);
	glVertex3dv(_b); glVertex3dv(b);
	glColor4ubv(zcolor);
	glVertex3dv(_c); glVertex3dv(c);
	glEnd();
	glGetError(); // Patch to buggy qt rendertext;
	glPopAttrib();
	assert(!glGetError());
}


void GLWidget::DrawAxisObject() {
	glUseProgram(0);
	m_AxisNodeProgram->bind();
	m_AxisNodevbo.bind();
	m_AxisNodeProgram->setUniformValue("projection", m_projection);
	//m_view.translate(0.5f, 0.5f);
	m_AxisNodeProgram->setUniformValue("view", m_view);
	m_AxisNodeModel.setToIdentity();
	// 		m_bgGroundModel.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	// 		m_bgGroundModel.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	// 		m_bgGroundModel.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_AxisNodeModel.translate(m_bgGroundModelPos);
// 	m_AxisNodeModel.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 	m_AxisNodeModel.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 	m_AxisNodeModel.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	m_AxisNodeProgram->setUniformValue("model", m_AxisNodeModel);
	m_AxisNodeProgram->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_AxisNodeProgram->enableAttributeArray(PROGRAM_NORMAL_ATTRIBUTE);
	m_AxisNodeProgram->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
	
	m_AxisNodeProgram->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 3*sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
	glDrawArrays(GL_LINES, 0, 6);
}