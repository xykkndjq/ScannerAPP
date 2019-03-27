#pragma once
#include "BaseModel.h"
class CGroundObject:public BaseModel
{
public:
	CGroundObject(shared_ptr<QOpenGLShaderProgram> v_Program,
		shared_ptr<QOpenGLBuffer> v_Vbo);
	CGroundObject(const char * v_vsFile, const char * v_fsfile, QObject *parent);
	~CGroundObject();
	virtual void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent);
	QVector4D m_bkGroundColor;
	
};
SharedPtr(CGroundObject);

