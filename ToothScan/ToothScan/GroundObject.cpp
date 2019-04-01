#include "GroundObject.h"

#define PROGRAM_VERTEX_ATTRIBUTE 0
CGroundObject::CGroundObject(shared_ptr<QOpenGLShaderProgram> v_Program,
	shared_ptr<QOpenGLBuffer> v_Vbo) :BaseModel(v_Program, v_Vbo),
	m_bkGroundColor(0.65f, 0.79f, 1.0f, 0.7f)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->link();
}


CGroundObject::CGroundObject(const char * v_vsFile, const char * v_fsfile, QObject *parent)
	:BaseModel(v_vsFile, v_fsfile, parent)
	, m_bkGroundColor(0.65f, 0.79f, 1.0f, 0.7f)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->link();
}

CGroundObject::~CGroundObject()
{
}

void CGroundObject::doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_program->setUniformValue("projection", v_Projection);
	m_program->setUniformValue("view", v_View);
	m_ModelMatrix.setToIdentity();
// 	m_ModelMatrix.rotate(m_xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_ModelMatrix.rotate(m_ModelRotate);
	m_ModelMatrix.translate(m_qvTranslate);
	m_program->setUniformValue("model", m_ModelMatrix);
	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 3 * sizeof(GLfloat));
	m_program->setUniformValue("ourColor", m_bkGroundColor);
	glDrawArrays(GL_QUADS, 0, m_totalFaceNum);
	glDisable(GL_BLEND);
	glDepthMask(true);
	//cout << glGetError() << endl;
}


