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
//#include <QOpenGLTexture>
#include <QMouseEvent>
#include "TeethModel.h"
#include <QPainter>
#include "trackball.h"

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
			orth::Face f;
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
	m_xMinRotLim(-21.7f),
	m_yMinRotLim(-180.0f),
	m_xMaxRotLim(158.3f),
	m_yMaxRotLim(180.0f),
	program(0)
{
	SCR_WIDTH = this->frameGeometry().width();
	SCR_HEIGHT = this->frameGeometry().height();

	//orth::ModelRead mr("./0016.ply", mm);
	//TeethSegmentRun("./0016.txt");

	model_Mat = cv::Mat::eye(4, 4, CV_32FC1);
	view_Mat = cv::Mat::eye(4, 4, CV_32FC1);
	m_trackBall = TrackBall(0.05f, QVector3D(0, 1, 0), TrackBall::Sphere);
	m_trackBallTest = m_trackBall;
	motor_rot_x = 0;
	motor_rot_y = 0;
	m_cutToothIndex = -1;
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
			pModel->setScreenPos(xTrans, yTrans);
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
	vshader->compileSourceCode(ReadShader(":/MainWidget/gl2.vs"));

	QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
	fshader->compileSourceCode(ReadShader(":/MainWidget/gl2.fs"));

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

	makeBackGround();

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

	makeCutBoxObject();
	
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
//	drawGradient();
	glUseProgram(0);
	m_backgroundModel->OnPaint(m_projection,m_view,this);
	return;
	//m_cutboxModel->OnPaint(m_projection, m_view, this);
	m_projection.setToIdentity();
	m_projection.perspective(FOV, (float)SCR_WIDTH / (float)SCR_HEIGHT, 1.0f, 300.0f);
	m_view.setToIdentity();
	m_view.translate(0.0f, 0.0f, -230.0f);
	m_view.rotate(m_trackBall.rotation());
	//m_model.setToIdentity();
	//m_model.rotate(m_trackBall.rotation());
	//m_view.rotate(30.0f, 1.0f, 0.0f, 0.0f);
	//m_view.rotate(30.0f, 0.0f, 1.0f, 0.0f);
	//m_view.rotate(30.0f, 0.0f, 0.0f, 1.0f);

	vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
	for (; iter != m_ModelsVt.end(); iter++) {
		glUseProgram(0);
		(*iter)->OnPaint(m_projection, m_view, this);
	}
	map<int, pCCutBoxObject>::iterator mapIter = m_cutBoxesMap.begin();
	for (; mapIter != m_cutBoxesMap.end(); mapIter++) {
		glUseProgram(0);
		mapIter->second->OnPaint(m_projection, m_view, this);
		//(*mapIter)->second()->OnPaint(m_projection, m_view, this);
	}
	//m_cutboxModel->OnPaint(m_projection, m_view, this);

	iter = m_ToolsModelsVt.begin();
	for (; iter != m_ToolsModelsVt.end(); iter++) {
		glUseProgram(0);
		(*iter)->OnPaint(m_projection, m_view, this);
	}

	

	//	glSwapBuffers();


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

QPointF GLWidget::pixelPosToViewPos(const QPointF& p)
{
	return QPointF(2.0 * float(p.x()) / width() - 1.0, \
		1.0 - 2.0 * float(p.y()) / height());

	// 	return QPointF(2.0 * float(p.x()) / 700 - 1.0, \
	// 		1.0 - 2.0 * float(p.y()) / 700);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{

	int screen_x = event->globalX();
	int screen_y = event->globalY();
	if (QApplication::keyboardModifiers() == Qt::ShiftModifier)
	{
		m_cutboxModel->SetPos(pixelPosToViewPos(event->globalPos()));
		if (event->button() == Qt::LeftButton)
		{
			qDebug() << "ShiftKey + MOuseLeftButton";
			m_cutboxModel->movement_type = BoxMovement::Trans;

			//int face_index = m_cutboxModel->ChoseFace((float)screen_x, (float)screen_y, SCR_WIDTH, SCR_HEIGHT, m_model, m_view, m_projection);
			int face_index = m_cutboxModel->ChoseFace((float)screen_x, (float)screen_y, SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), m_cutboxModel->new_box);

			if (face_index == 0 || face_index == 1)
			{
				m_cutboxModel->face_index = BoxFace::x_p;
			}
			if (face_index == 6 || face_index == 7)
			{
				m_cutboxModel->face_index = BoxFace::x_n;
			}

			if (face_index == 2 || face_index == 3)
			{
				m_cutboxModel->face_index = BoxFace::y_p;
			}
			if (face_index == 8 || face_index == 9)
			{
				m_cutboxModel->face_index = BoxFace::y_n;
			}

			if (face_index == 4 || face_index == 5)
			{
				m_cutboxModel->face_index = BoxFace::z_p;
			}
			if (face_index == 10 || face_index == 11)
			{
				m_cutboxModel->face_index = BoxFace::z_n;
			}

			m_cutboxModel->UpdateCutBoxObject();
			
			this->update();

			return;
		}
	}
	if (QApplication::keyboardModifiers() == Qt::ControlModifier)
	{
		m_cutboxModel->SetPos(pixelPosToViewPos(event->globalPos()));
		if (event->button() == Qt::LeftButton)
		{
			qDebug() << "ControlKey + MOuseLeftButton";
			m_cutboxModel->movement_type = BoxMovement::Rot;

			int face_index = m_cutboxModel->ChoseFace((float)screen_x, (float)screen_y, SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), m_cutboxModel->new_box);

			if (face_index == 0 || face_index == 1)
			{
				m_cutboxModel->face_index = BoxFace::x_p;
			}
			if (face_index == 6 || face_index == 7)
			{
				m_cutboxModel->face_index = BoxFace::x_n;
			}

			if (face_index == 2 || face_index == 3)
			{
				m_cutboxModel->face_index = BoxFace::y_p;
			}
			if (face_index == 8 || face_index == 9)
			{
				m_cutboxModel->face_index = BoxFace::y_n;
			}

			if (face_index == 4 || face_index == 5)
			{
				m_cutboxModel->face_index = BoxFace::z_p;
			}
			if (face_index == 10 || face_index == 11)
			{
				m_cutboxModel->face_index = BoxFace::z_n;
			}

			m_cutboxModel->UpdateCutBoxObject();

			this->update();

			return;
		}
	}
	if (QApplication::keyboardModifiers() == Qt::AltModifier)
	{
		m_cutboxModel->SetPos(pixelPosToViewPos(event->globalPos()));
		if (event->button() == Qt::LeftButton)
		{
			qDebug() << "AltKey + MOuseLeftButton";
			m_cutboxModel->movement_type = BoxMovement::Stretch;

			int face_index = m_cutboxModel->ChoseFace((float)screen_x, (float)screen_y, SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), m_cutboxModel->new_box);

			if (face_index == 0 || face_index == 1)
			{
				m_cutboxModel->face_index = BoxFace::x_p;
			}
			if (face_index == 6 || face_index == 7)
			{
				m_cutboxModel->face_index = BoxFace::x_n;
			}

			if (face_index == 2 || face_index == 3)
			{
				m_cutboxModel->face_index = BoxFace::y_p;
			}
			if (face_index == 8 || face_index == 9)
			{
				m_cutboxModel->face_index = BoxFace::y_n;
			}

			if (face_index == 4 || face_index == 5)
			{
				m_cutboxModel->face_index = BoxFace::z_p;
			}
			if (face_index == 10 || face_index == 11)
			{
				m_cutboxModel->face_index = BoxFace::z_n;
			}

			m_cutboxModel->UpdateCutBoxObject();

			this->update();

			return;
		}
	}
	if (event->button() == Qt::MiddleButton)
	{
		m_cutboxModel->SetPos(pixelPosToViewPos(event->globalPos()));
		if (QApplication::keyboardModifiers() == Qt::ControlModifier)
		{
			qDebug() << "CtrlKey + MiddleButton";

			vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
			for (; iter != m_ModelsVt.end(); iter++) {
				pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
				if (pModel) {
					m_cutboxModel->CutModelInBox(pModel->m_model);
					pModel->makeObject();
				}
			}

			update();
		}
		else
		{

			//orth::MeshModel mm2;
			m_cutboxModel->ChosePoints2((float)(screen_x - SCR_WIDTH / 2.0), (float)-1 * (screen_y - SCR_HEIGHT / 2.0), SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), mm);
			m_cutboxModel->Set_Visible(true);
			this->update();
		}
	}
	else {
		if (getSelectReginValue()) {
			if (event->buttons()&Qt::LeftButton) {
				m_drawRectClickPosition = QPoint(event->globalX(), SCR_HEIGHT - event->globalY());
			}
		}
		else {
			lastPos = event->pos();
			QQuaternion trans;
			float xRot = 0, yRot = 0;
			m_trackBallTest.push(pixelPosToViewPos(event->pos()), trans);
			GetRotateMotorRot(xRot, yRot, m_trackBallTest);
			//if (abs(xRot - m_xRotLim) >= 0 || abs(yRot - m_yRotLim) >= 0)
			if (xRot > m_xMaxRotLim || xRot<m_xMinRotLim)
				return;

			//m_trackBall.push(pixelPosToViewPos(event->pos()), trans);
			//m_trackBallTest = m_trackBall;
		}
	}
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (getSelectReginValue()) {
		if (event->buttons()&Qt::LeftButton) {
			m_drawRectEndPosition = QPoint(event->globalX(), SCR_HEIGHT - event->globalY());
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
		int dx = event->globalX() - lastPos.x();
		int dy = event->globalY() - lastPos.y();
		//cout << dx << " ----- " << dy << endl;

		//if (event->buttons()&Qt::LeftButton)
		//{
		//	if (event->modifiers() == Qt::CTRL)
		//	{
		//		program->executeTranslateOperation(event->globalX(), event->globalY());
		//	}
		//	else
		//	{
		//		program->executeRotateOperation(event->globalX(), event->globalY());
		//	}
		//}

		if (QApplication::keyboardModifiers() == Qt::AltModifier|| QApplication::keyboardModifiers() == Qt::ControlModifier|| QApplication::keyboardModifiers() == Qt::ShiftModifier)
		{
			if (m_cutboxModel->movement_type == BoxMovement::Trans)
			{
				m_cutboxModel->BoxTrans(dx, dy);
			}
			if (m_cutboxModel->movement_type == BoxMovement::Stretch)
			{
				m_cutboxModel->BoxStretch(dx, dy);
			}
			if (m_cutboxModel->movement_type == BoxMovement::Rot)
			{
				m_cutboxModel->BoxRot(pixelPosToViewPos(event->globalPos()));
			}
			m_cutboxModel->UpdateCutBoxObject();
			this->update();
			lastPos = event->globalPos();

		}
		else
		{
			if (event->buttons() & Qt::LeftButton) {
				rotateBy(dy, dx, 0);
			}
			else if (event->buttons() & Qt::RightButton) {
				//translateBy(dx, dy, 0);
			}
			if (event->buttons() & Qt::LeftButton) {
				float xRot = 0, yRot = 0;
				TrackBall trackBallTest;
				trackBallTest = m_trackBallTest;
				QQuaternion trans;
				trackBallTest.move(pixelPosToViewPos(event->pos()), trans);
				GetRotateMotorRot(xRot, yRot, trackBallTest);
				if (xRot> m_xMaxRotLim || xRot < m_xMinRotLim)
					return;

				//m_trackBall.move(pixelPosToViewPos(event->pos()), trans);
				//m_trackBallTest = m_trackBall;
				m_trackBallTest = trackBallTest;
				m_trackBall = m_trackBallTest;
				// 			vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
				// 			for (; iter != m_ModelsVt.end(); iter++) {
				// 				pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
				// 				if (pModel) {
				// 					pModel->rotate(m_trackBall.rotation());
				// 				}
				// 			}
				// 			m_groundModel->rotate(m_trackBall.rotation());
				// 			m_axisMode->rotate(m_trackBall.rotation());
			}
			else {
				QQuaternion trans;
				m_trackBallTest.release(pixelPosToViewPos(event->pos()), trans);
			}
			lastPos = event->pos();
		}


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
	//cout << "release done !" << endl;
	//vertices_in.clear();
	//ChosePoints(0, 0, 500, 500, 1920, 1080, model1, view1, project1, mm);
	//makeObject();

	emit clicked();
	if (getSelectReginValue()) {
		vector<pBaseModel>::iterator iter = m_ModelsVt.begin();
		for (; iter != m_ModelsVt.end(); iter++) {
			pCTeethModel pModel = static_pointer_cast<CTeethModel>(*iter);
			if (pModel) {
				pModel->ChangeModelSelectedColor(m_drawRectClickPosition, m_drawRectEndPosition, this);
				pModel->ChosePoints(m_drawRectClickPosition.x(), m_drawRectClickPosition.y(), m_drawRectEndPosition.x(), m_drawRectEndPosition.y(),
					SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), this);
				pModel->setRectSelectRegion(QPoint(0, 0), QPoint(0, 0));
			}
		}
		//ChangeModelSelectedColor(m_drawRectClickPosition, m_drawRectEndPosition);
		//ChosePoints(m_drawRectClickPosition.x(), m_drawRectClickPosition.y(), m_drawRectEndPosition.x(), m_drawRectEndPosition.y(),\
			SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), this);
	}
	if (event->button() == Qt::LeftButton) {
		float xRot = 0, yRot = 0;
		TrackBall trackBallTest;
		trackBallTest = m_trackBallTest;
		QQuaternion trans;
		trackBallTest.release(pixelPosToViewPos(event->pos()), trans);
		GetRotateMotorRot(xRot, yRot, trackBallTest);
		if (xRot > m_xMaxRotLim || xRot  < m_xMinRotLim )
			return;
		m_trackBallTest.release(pixelPosToViewPos(event->pos()), trans);
	}


	m_cutboxModel->SetOriginalColor();
	m_cutboxModel->UpdateCutBoxObject();
	this->update();
}
void GLWidget::CutModelInBox(orth::MeshModel &meshModel) {
	if (m_cutToothIndex == -1)
		return;
	map<int, pCCutBoxObject>::iterator mapIter = m_cutBoxesMap.begin(); 
	vector<orth::MeshModel> meshModelvt;
	for (; mapIter != m_cutBoxesMap.end();mapIter++) {
		orth::MeshModel tmp = meshModel;
		mapIter->second->CutModelInBox(tmp);
		meshModelvt.push_back(tmp);
	}
	if (meshModelvt.size()>0) {
		MergeModels(meshModelvt, meshModel);
	}
}
void GLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) {
		if (m_cutToothIndex == -1)
			return;
		pCCutBoxObject ptmp = makeCutBoxObject();
		if (!ptmp)
			return;
		int screen_x = event->globalX();
		int screen_y = event->globalY();
		ptmp->ChosePoints2((float)(screen_x - SCR_WIDTH / 2.0), (float)-1 * (screen_y - SCR_HEIGHT / 2.0), SCR_WIDTH, SCR_HEIGHT, cv::Mat(4, 4, CV_32F, m_model.data()), cv::Mat(4, 4, CV_32F, m_view.data()), cv::Mat(4, 4, CV_32F, m_projection.data()), mm);
		ptmp->Set_Visible(true);
		update();
	}
}

void GLWidget::glUseProgram(GLuint program)
{
	QOpenGLFunctions::glUseProgram(program);
}

pCTeethModel GLWidget::makeObject()
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

	pCTeethModel l_TeethModel = make_shared<CTeethModel>(program, vbo, mm);
	l_TeethModel->makeObject();
	m_ModelsVt.push_back(l_TeethModel);
	this->update();
	return l_TeethModel;
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
	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(-90, 0, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(-90, 0, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	update();
}

QQuaternion GLWidget::EulerAngle2Quaternion(const QVector3D &ea)
{
	float fCosHRoll = cos(ea.x()*M_PI/180.0 * .5f);
	float fSinHRoll = sin(ea.x()*M_PI / 180.0 * .5f);

	float fCosHPitch = cos(ea.y()*M_PI / 180.0 * .5f);
	float fSinHPitch = sin(ea.y()*M_PI / 180.0 * .5f);

	float fCosHYaw = cos(ea.z()*M_PI / 180.0 * .5f);
	float fSinHYaw = sin(ea.z()*M_PI / 180.0 * .5f);

	/// Cartesian coordinate System
	//float w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
	//float x = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
	//float y = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
	//float z = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;

	float w = fCosHRoll * fCosHPitch * fCosHYaw - fSinHRoll * fSinHPitch * fSinHYaw;
	float x = fCosHRoll * fSinHPitch * fSinHYaw + fSinHRoll * fCosHPitch * fCosHYaw;
	float y = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
	float z = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
	

	//float w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
	//float x = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
	//float y = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
	//float z = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;

	QQuaternion new_q(w,x,y,z);

	return new_q;
}

void GLWidget::upwardView()
{
	reSetValue();


	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(90,0,0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(90, 0, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));

	update();
	//QQuaternion::toRotationMatrix();
}

void GLWidget::leftView()
{
	reSetValue();
	
	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(0, 90, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(0, 90, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));

	update();
}

void GLWidget::rightView()
{
	reSetValue();
	
	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(0, -90, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(0, -90, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));

	//m_view.rotate(m_trackBall.rotation());
	//QMatrix4x4 qm;
	//qm.rotate(m_trackBall.m_rotation);
	//cv::Mat merot = cv::Mat(4, 4, CV_32F, qm.data());
	//orth::MeshModel mm;
	//for (int i = 0; i < 30; i++)
	//{
	//	orth::Point3d point(0, 0, (double)i);

	//	mm.P.push_back(point);
	//}
	//mm.Rotation((double *)merot.data);

	//vector<double> new_direct;
	//for (int i = 0; i < mm.P.size(); i++)
	//{
	//	new_direct.push_back(mm.P[i].x);
	//	new_direct.push_back(mm.P[i].y);
	//	new_direct.push_back(mm.P[i].z);
	//}

	//ColoredPoints(new_direct, 3);

	update();
}

void GLWidget::mainView()
{
	reSetValue();

	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(0, 0, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(0, 0, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));

	update();
}

void GLWidget::backView()
{
	reSetValue();
	m_trackBall.m_rotation = EulerAngle2Quaternion(QVector3D(0, 179.9, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));
	m_trackBallTest.m_rotation = EulerAngle2Quaternion(QVector3D(0, 179.9, 0));// QQuaternion::fromEulerAngles(QVector3D(90, 0, 0));



	update();
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
	glLineWidth(1);
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
		worldPos = world2Screen(worldPos, m_model);
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

QVector3D GLWidget::world2Screen(QVector3D worldPos, QMatrix4x4 modelMat)
{
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	GLint l_view[4] = { 0,0,SCR_WIDTH,SCR_HEIGHT };
	QVector3D worldPosition = worldPos.project(QMatrix4x4(modelMat*m_view), QMatrix4x4(m_projection), QRect(l_view[0], l_view[1], l_view[2], l_view[3]));
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
			orth::Face f;
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
	m_groundModel->Set_Visible(bShow);
	m_groundModel->rotate(60,0,0);
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
// 	xrot = xRot / 16.0f;
// 	yrot = yRot / 16.0f;
	return GetRotateMotorRot(xrot,yrot,m_trackBall);


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


	m_groundModel = make_shared<CGroundObject>(":/MainWidget/bgGround.vs", ":/MainWidget/bgGround.fs", this);
	m_groundModel->makeObject(vertData, 4);
	m_groundModel->Set_Visible(false);
	m_ToolsModelsVt.push_back(m_groundModel);
	this->update();
}

pCCutBoxObject GLWidget::makeCutBoxObject()
{
	if (m_cutToothIndex != -1 &&m_cutBoxesMap.find(m_cutToothIndex) != m_cutBoxesMap.end()) {
		return m_cutBoxesMap[m_cutToothIndex];
	}
	orth::MeshModel mm2;
	double box_min_x = -10, box_min_y = -10, box_min_z = -10, box_max_x = 10, box_max_y = 10, box_max_z = 10;
	pCCutBoxObject pcutboxModel;
	pcutboxModel = make_shared<CCutBoxObject>(":/MainWidget/Resources/images/cutbox.vs", ":/MainWidget/Resources/images/cutbox.fs", this);
	{
		mm2.F.resize(12);
		mm2.P.clear();
		vector<orth::Point3d> box_vertex(8);
		box_vertex[0].x = box_min_x; box_vertex[0].y = box_min_y; box_vertex[0].z = box_min_z; mm2.P.push_back(box_vertex[0]);
		box_vertex[1].x = box_max_x; box_vertex[1].y = box_min_y; box_vertex[1].z = box_min_z; mm2.P.push_back(box_vertex[1]);
		box_vertex[2].x = box_max_x; box_vertex[2].y = box_min_y; box_vertex[2].z = box_max_z; mm2.P.push_back(box_vertex[2]);
		box_vertex[3].x = box_min_x; box_vertex[3].y = box_min_y; box_vertex[3].z = box_max_z; mm2.P.push_back(box_vertex[3]);
		box_vertex[4].x = box_min_x; box_vertex[4].y = box_max_y; box_vertex[4].z = box_min_z; mm2.P.push_back(box_vertex[4]);
		box_vertex[5].x = box_max_x; box_vertex[5].y = box_max_y; box_vertex[5].z = box_min_z; mm2.P.push_back(box_vertex[5]);
		box_vertex[6].x = box_max_x; box_vertex[6].y = box_max_y; box_vertex[6].z = box_max_z; mm2.P.push_back(box_vertex[6]);
		box_vertex[7].x = box_min_x; box_vertex[7].y = box_max_y; box_vertex[7].z = box_max_z; mm2.P.push_back(box_vertex[7]);

		mm2.F[0].x = 1; mm2.F[0].y = 6; mm2.F[0].z = 2;
		mm2.F[1].x = 1; mm2.F[1].y = 5; mm2.F[1].z = 6;
		mm2.F[2].x = 4; mm2.F[2].y = 7; mm2.F[2].z = 6;
		mm2.F[3].x = 4; mm2.F[3].y = 6; mm2.F[3].z = 5;
		mm2.F[4].x = 2; mm2.F[4].y = 6; mm2.F[4].z = 7;
		mm2.F[5].x = 2; mm2.F[5].y = 7; mm2.F[5].z = 3;
		mm2.F[6].x = 0; mm2.F[6].y = 3; mm2.F[6].z = 7;
		mm2.F[7].x = 0; mm2.F[7].y = 7; mm2.F[7].z = 4;
		mm2.F[8].x = 0; mm2.F[8].y = 1; mm2.F[8].z = 2;
		mm2.F[9].x = 0; mm2.F[9].y = 2; mm2.F[9].z = 3;
		mm2.F[10].x = 1; mm2.F[10].y = 0; mm2.F[10].z = 4;
		mm2.F[11].x = 1; mm2.F[11].y = 4; mm2.F[11].z = 5;

	}

	pcutboxModel->new_box = mm2;
	
	QVector<GLfloat> vertData;

	//x1 r
	for (int i = 0; i < mm2.F.size(); i++)
	{
		//float x, y, z;
		vertData.append(mm2.P[mm2.F[i].x].x);
		vertData.append(mm2.P[mm2.F[i].x].y);
		vertData.append(mm2.P[mm2.F[i].x].z);
		vertData.append(pcutboxModel->face_color[i].x());
		vertData.append(pcutboxModel->face_color[i].y());
		vertData.append(pcutboxModel->face_color[i].z());
		vertData.append(pcutboxModel->face_color[i].w());

		vertData.append(mm2.P[mm2.F[i].y].x);
		vertData.append(mm2.P[mm2.F[i].y].y);
		vertData.append(mm2.P[mm2.F[i].y].z);
		vertData.append(pcutboxModel->face_color[i].x());
		vertData.append(pcutboxModel->face_color[i].y());
		vertData.append(pcutboxModel->face_color[i].z());
		vertData.append(pcutboxModel->face_color[i].w());

		vertData.append(mm2.P[mm2.F[i].z].x);
		vertData.append(mm2.P[mm2.F[i].z].y);
		vertData.append(mm2.P[mm2.F[i].z].z);
		vertData.append(pcutboxModel->face_color[i].x());
		vertData.append(pcutboxModel->face_color[i].y());
		vertData.append(pcutboxModel->face_color[i].z());
		vertData.append(pcutboxModel->face_color[i].w());

	}


	pcutboxModel->box_center = orth::Point3d(0, 0, 0);
	pcutboxModel->direct_x = orth::Point3d(1, 0, 0);
	pcutboxModel->direct_y = orth::Point3d(0, 1, 0);
	pcutboxModel->direct_z = orth::Point3d(0, 0, 1);
	pcutboxModel->max_x_dir = orth::Point3d(1, 0, 0);
	pcutboxModel->max_y_dir = orth::Point3d(0, 1, 0);
	pcutboxModel->max_z_dir = orth::Point3d(0, 0, 1);
	pcutboxModel->min_x_dir = orth::Point3d(-1, 0, 0);
	pcutboxModel->min_y_dir = orth::Point3d(0, -1, 0);
	pcutboxModel->min_z_dir = orth::Point3d(0, 0, -1);

	pcutboxModel->makeObject(vertData, 36);

	pcutboxModel->Set_Visible(false);
	//m_ToolsModelsVt.push_back(m_cutboxModel);
	m_cutBoxesMap.insert(make_pair(m_cutToothIndex,pcutboxModel));
	this->update();
	return pcutboxModel;
}



void GLWidget::makeBackGround()
{
	QVector<GLfloat> vertData =
	{ 
		1.0f,1.0f,1.0f,    1.0f,1.0f,
		1.0f,-1.0f,1.0f,   1.0f,0.0f,
		-1.0f,-1.0f,1.0f,  0.0f,0.0f,
		-1.0f,1.0f,1.0f ,  0.0f,1.0f
	};
	m_backgroundModel = make_shared<CBackGroundObject>(":/MainWidget/background.vs", ":/MainWidget/background.fs", this);
	QString name_ = "./Resources/images/background-grey3.png";
	m_backgroundModel->readPicture(name_);
	m_backgroundModel->makeObject(vertData, 4);
	m_backgroundModel->Set_Visible(true);
	//m_ToolsModelsVt.push_back(m_backgroundModel);
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
	m_axisMode = make_shared<CAxisModel>(":/MainWidget/AxisNode.vs", ":/MainWidget/AxisNode.fs", this);
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

	m_AxisNodeProgram->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
	glDrawArrays(GL_LINES, 0, 6);
}

/*----------------------------------- new rot x rot y --------------------------------*/
//全局
void GLWidget::GetRotateMotorRot(float &xrot, float &yrot, TrackBall & v_TrackBall) {
	QVector3D camera_pos2 = QVector3D(0.0f, 0.0f, 0.0f);
#define ToDeg(x) x*57.295780490442968321226628812406

	//任何旋转事件 改变 view矩阵后 计算camera位置
	QVector3D zaxis(0.0f, 0.0f, 1.0f);
	//QMatrix4x4 viewinv = view.inverted(0);
	QMatrix4x4 view;
	view.setToIdentity();
	view.translate(0.0f, 0.0f, -230.0f);
	view.rotate(v_TrackBall.rotation());


	QMatrix4x4 viewinv = view.inverted(0);//view.inverted(0);
	camera_pos2 = viewinv*zaxis;
	//cout << "###### ---- " << camera_pos2[0] << ", " << camera_pos2[1] << ", " << camera_pos2[2] << endl;

	//计算旋转角度
	double c_x = camera_pos2[0];
	double c_y = camera_pos2[1];
	double c_z = camera_pos2[2];
	double c_xy = sqrt(c_x*c_x + c_z*c_z);
	double ax = ToDeg(atan2(c_x, c_z));
	double ay = ToDeg(atan2(c_y, c_xy));
	//cout << " ax = " << ax << " ay = " << ay << endl;
	float yaw = -ax;
	float pitch = ay;
	xrot = pitch;
	yrot = yaw;
	//cout << " yaw = " << yaw << " pitch = " << pitch << endl;
}		

/*---------------------------------------------------------------------------------------*/