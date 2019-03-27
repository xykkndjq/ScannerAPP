#pragma once
#include <iostream>
#include <memory>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLWidget>
#include <QDebug>
#include <QOpenGLFunctions>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
using namespace std;
#define PARAMDEFINE(type , prarm_name)	private:\
										type m_##prarm_name;\
										public:\
										void Set_##prarm_name(type v_##prarm_name){m_##prarm_name = v_##prarm_name;};\
										type Get_##prarm_name(){return m_##prarm_name;}

__interface IParentInterface
{
	virtual QVector3D screen2world(int x, int y);
	virtual QVector3D world2Screen(QVector3D worldPos);
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
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	QByteArray ReadShader(const QString &Path);
#pragma region			变量定义  get/set
	PARAMDEFINE(bool, bVisible);
#pragma endregion

protected:
	shared_ptr<QOpenGLShaderProgram> m_program;
	QMatrix4x4 m_ModelMatrix;
	shared_ptr<QOpenGLBuffer> m_vbo;
	int m_totalFaceNum;
	QVector3D m_qvTranslate;
	float m_xRot;
	float m_yRot;
	float m_zRot;
};

#define SharedPtr(ClassName) typedef shared_ptr<ClassName> p##ClassName;
SharedPtr(BaseModel);
SharedPtr(QOpenGLShaderProgram) pQOpenGLShaderProgram;
SharedPtr(QOpenGLBuffer) pQOpenGLBuffer;

