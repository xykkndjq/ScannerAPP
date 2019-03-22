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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <orthio.h>
#include <QDebug>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include "./include/3DScan.h"

using std::cout;
using std::endl;

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_NORMAL_ATTRIBUTE 1
#define PROGRAM_MATERIAL_ATTRIBUTE 2
#define PROGRAM_STATE_ATTRIBUTE 3

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;
    void rotateBy(int xAngle, int yAngle, int zAngle);
	void translateBy(int xAngle, int yAngle, int zAngle);
    void setClearColor(const QColor &color);
	void setWindowSize(QSize &size);
	void SetMatrix(cv::Mat &m, cv::Mat &v);
	//13 获取x y 角度
	void GetMotorRot(float &xrot, float &yrot);
	void makeObject();

	vector<float> vertices_in;
	vector<float> normal_in;
	vector<float> label_in;
	QVector<GLfloat> vertData;

	int totalFaceNum = 0;

	orth::MeshModel mm;

signals:
    void clicked();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
	//void keyPressEvent(QKeyEvent *event) override;
	//void keyReleaseEvent(QKeyEvent *event) override;

private:
    
	//const char* ReadShader(const char* vertexPath);
	QByteArray ReadShader(const QString &Path);
	void TeethSegmentRun(const std::string label_file_path);

	unsigned int SCR_WIDTH = 1920;
	unsigned int SCR_HEIGHT = 1080;
	float FOV = 20.0f;

    QColor clearColor;
    QPoint lastPos;
    float xRot;
	float yRot;
	float zRot;
	float xTrans;
	float yTrans;
	float zTrans;

	cv::Mat model_Mat;
	cv::Mat view_Mat;

	scan::RasterScan rs;
	

	//QOpenGLTexture *textures;
    QOpenGLShaderProgram *program;
    QOpenGLBuffer vbo;

	//bkgroundmodel
	QOpenGLShaderProgram *m_bkGroundProgram;
	QMatrix4x4 m_bgGroundModel;
	QVector3D m_bgGroundModelPos;
	QOpenGLBuffer m_bkgroundvbo;
	bool m_bkGroundShow;
	void makeGroundObject();
	QVector4D m_bkGroundColor;
	//bkgroundmodel


	QVector<QVector3D> m_vertices;
	QVector<QVector3D> m_normals;
	float fv[4];
	void createGeometry();
	void quad(qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3, qreal x4, qreal y4);
	void extrude(qreal x1, qreal y1, qreal x2, qreal y2);

public:
	//数据重置
	void reSetValue();
	//1 俯视
	void overView();
	//2 仰视
	void upwardView();
	//3 左视
	void leftView();
	//4 右视
	void rightView();
	//5 主视图
	void mainView();
	//6 后视图
	void backView();
	//	7 放大
	void enlargeView();
	//8 缩小
	void shrinkView();
	//9 框选 true 开始框选 false 取消框选
	void selectRegion(bool bSelected);
	void setSelectRegionValue(bool bSelected);
	bool getSelectReginValue();
	//10 删除框选
	void delSelected();
	//11 确定选择
	void confirmSelRegion();

	void drawRect(int x, int y, int x1, int y1);

	void ChangeSelectedColorEx(QPoint drawRectClickPosition, QPoint drawRectEndPosition);

	void ChangeModelSelectedColor(QPoint drawRectClickPosition, QPoint drawRectEndPosition);

	QVector3D screen2world(int x, int y);

	QVector3D world2Screen(QVector3D worldPos);

	void remakeObject();

	void ChosePoints(const float point1_x, const float point1_y, const float point2_x, const float point2_y, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm);
	void delSelPoints();
	void setbkGroundShowValue(bool bShow);
	bool getbkGroundShowValue();
	//上下移动
	void bgGroundmoveDown();
	void bgGroundmoveUp();
	void showBkGround(bool bShow);
	void cutModelUnderBg();
	void setBgColor(QVector4D color);
	//上下移动

private:
	bool m_bSelectRegion;
	bool m_bSelectAddRegion;
	QPoint m_drawRectClickPosition;
	QPoint m_drawRectEndPosition;
	QMatrix4x4 m_projection, m_view, m_model;
	orth::PointLabel m_Selected;
};

#endif
