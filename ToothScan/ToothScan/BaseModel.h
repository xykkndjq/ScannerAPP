#pragma once
#include <iostream>
#include <memory>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLWidget>
#include <QDebug>
#include <QOpenGLFunctions>
#include "commondefine.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
using namespace std;

__interface IParentInterface
{
	virtual QVector3D screen2world(int x, int y);
	virtual QVector3D world2Screen(QVector3D worldPos, QMatrix4x4 modelMat);
	virtual void glUseProgram(GLuint program);
}; 


class BaseModel
{
public:
	BaseModel(shared_ptr<QOpenGLShaderProgram> v_Program,
		shared_ptr<QOpenGLBuffer> v_Vbo);
	BaseModel(const char * v_vsFile,
		const char * v_fsfile,QObject *parent = Q_NULLPTR);
	~BaseModel();
	void OnPaint(QMatrix4x4 v_Projection , QMatrix4x4 v_View, IParentInterface *pParent);
	virtual void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent) = 0;
	virtual void makeObject(QVector<GLfloat> v_vertData ,int nFaceNum);
	virtual void translate(QVector3D qvTranslate);
	virtual void rotate(float xRot,	float yRot,	float zRot);
	virtual void rotate(QQuaternion m);
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	QByteArray ReadShader(const QString &Path);
#pragma region			变量定义  get/set
	PARAMDEFINE(bool, b,Visible);
#pragma endregion

protected:
	shared_ptr<QOpenGLShaderProgram> m_program;
	QMatrix4x4 m_ModelMatrix;
	QQuaternion m_ModelRotate;
	shared_ptr<QOpenGLBuffer> m_vbo;
	int m_totalFaceNum;
	QVector3D m_qvTranslate;
	float m_xRot;
	float m_yRot;
	float m_zRot;
};


SharedPtr(BaseModel);
SharedPtr(QOpenGLShaderProgram) pQOpenGLShaderProgram;
SharedPtr(QOpenGLBuffer) pQOpenGLBuffer;

