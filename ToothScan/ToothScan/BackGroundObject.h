#pragma once
#include "BaseModel.h"
#include <QOpenGLVertexArrayObject>  
#include <QOpenGLTexture>  
class CBackGroundObject :public BaseModel
{
public:
	CBackGroundObject(shared_ptr<QOpenGLShaderProgram> v_Program,
		shared_ptr<QOpenGLBuffer> v_Vbo);
	CBackGroundObject(const char * v_vsFile, const char * v_fsfile, QObject *parent);
	~CBackGroundObject();
	virtual void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent);
	void readPicture(QString &file_name);
	//QVector4D m_bkGroundColor;
	QOpenGLTexture *textures;

};
SharedPtr(CBackGroundObject);