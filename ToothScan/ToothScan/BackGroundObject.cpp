#include "BackGroundObject.h"

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1
CBackGroundObject::CBackGroundObject(shared_ptr<QOpenGLShaderProgram> v_Program,
	shared_ptr<QOpenGLBuffer> v_Vbo) :BaseModel(v_Program, v_Vbo)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->link();
	//memset(textures, 0, sizeof(textures));
}


CBackGroundObject::CBackGroundObject(const char * v_vsFile, const char * v_fsfile, QObject *parent)
	:BaseModel(v_vsFile, v_fsfile, parent)
{
	m_program->bindAttributeLocation("aPos", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->link();
	//memset(textures, 0, sizeof(textures));
}

CBackGroundObject::~CBackGroundObject()
{
	delete textures;
}
void CBackGroundObject::readPicture(QString &file_name)
{
	textures = new QOpenGLTexture(QImage(file_name).mirrored());
}
void CBackGroundObject::doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_TEXCOORD_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
	m_program->setAttributeBuffer(PROGRAM_TEXCOORD_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
	
	textures->bind();
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	glDisable(GL_BLEND);
	glDepthMask(true);
	//cout << glGetError() << endl;
}


