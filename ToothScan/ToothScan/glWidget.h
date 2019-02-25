#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QToolButton>
#include <QDockWidget>
#include <QSpinBox>
#include <QSlider>
#include "io.h"
#include <QOpenGLBuffer>
#include <iostream>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)


class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	GLWidget(const QColor &background);
	~GLWidget();
	void initVariable();
	void constructIHM();
	void setConnections();
	
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	/*----------------------------------*/
	QSize minimumSizeHint() const override;
	QSize sizeHint() const override;
	void rotateBy(int xAngle, int yAngle, int zAngle);
	void setClearColor(const QColor &color);
protected:
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	//void mouseReleaseEvent(QMouseEvent *event) override;

private:
	QColor glbackground;

	QToolButton *leftWatchButton;
	QToolButton *rightWatchButton;
	QToolButton *topWatchButton;
	QToolButton *bottomWatchButton;
	QToolButton *frontWatchButton;
	QToolButton *backWatchButton;
	//相机显示窗口
	QDockWidget *cameraWindow;
	QSpinBox *spinCameraBox;
	QSlider *sliderCamera;

	/*--------------------------------------------------*/
	void makeObject();

	QColor clearColor;
	QPoint lastPos;
	int xRot;
	int yRot;
	int zRot;

	//QOpenGLTexture *textures;
	QOpenGLShaderProgram *program;
	QOpenGLBuffer vbo;

	vector<float> vertices_in;
	QVector<QVector3D> m_vertices;
	QVector<QVector3D> m_normals;
};

#endif