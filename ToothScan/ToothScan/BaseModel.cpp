#include "BaseModel.h"
#include <qfile.h>
#include <QOpenGLShaderProgram>


BaseModel::BaseModel(shared_ptr<QOpenGLShaderProgram> v_Program,
	shared_ptr<QOpenGLBuffer> v_Vbo)
	:m_program(v_Program),
	m_vbo(v_Vbo),
	m_totalFaceNum(0),
	m_bVisible(true),
	m_qvTranslate(0, 0, 0),
	m_xRot(0),
	m_yRot(0),
	m_zRot(0)
{
}


BaseModel::BaseModel(const char * v_vsFile, const char * v_fsfile, QObject *parent)
	:m_totalFaceNum(0),
	m_bVisible(true),
	m_qvTranslate(0, 0, 0),
	m_xRot(0),
	m_yRot(0),
	m_zRot(0)
{
	m_program = make_shared<QOpenGLShaderProgram>();
	QOpenGLShader *bgvshader = new QOpenGLShader(QOpenGLShader::Vertex, parent);
	bgvshader->compileSourceCode(ReadShader(v_vsFile));

	QOpenGLShader *bgfshader = new QOpenGLShader(QOpenGLShader::Fragment, parent);
	bgfshader->compileSourceCode(ReadShader(v_fsfile));
	m_program->addShader(bgvshader);
	m_program->addShader(bgfshader);
	m_vbo = make_shared<QOpenGLBuffer>();
}

BaseModel::~BaseModel()
{
}

void BaseModel::OnPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	//pParent->glUseProgram(0);
	if (!Get_Visible())
		return;
	m_program->bind();
	m_vbo->bind();
	doPaint(v_Projection, v_View, pParent);
	// 	m_vbo->release();
	// 	m_program->release();
}

void BaseModel::makeObject(QVector<GLfloat> v_vertData, int nFaceNum)
{
	m_vbo->destroy();

	if(m_vbo->create()){

		if(m_vbo->bind()){
			m_vbo->allocate(v_vertData.data(), v_vertData.size() * sizeof(GLfloat));
			m_totalFaceNum = nFaceNum;
		}
	}
}

void BaseModel::translate(QVector3D qvTranslate)
{
	m_qvTranslate = qvTranslate;
}

void BaseModel::rotate(float xRot, float yRot, float zRot)
{
	m_xRot = xRot;
	m_yRot = yRot;
	m_zRot = zRot;
}

void BaseModel::rotate(QQuaternion m)
{
	m_ModelRotate = m;
}

void BaseModel::mousePressEvent(QMouseEvent *event)
{

}

void BaseModel::mouseMoveEvent(QMouseEvent *event)
{

}

QByteArray BaseModel::ReadShader(const QString &Path)
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
