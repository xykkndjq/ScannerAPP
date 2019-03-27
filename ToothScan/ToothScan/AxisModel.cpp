#include "AxisModel.h"

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_NORMAL_ATTRIBUTE 1
#define PROGRAM_MATERIAL_ATTRIBUTE 2
#define PROGRAM_STATE_ATTRIBUTE 3

CAxisModel::CAxisModel(const char * v_vsFile, const char * v_fsfile, QObject *parent)
	:BaseModel(v_vsFile, v_fsfile, parent)
{
	m_program->bindAttributeLocation("inPosition", PROGRAM_VERTEX_ATTRIBUTE);
	m_program->bindAttributeLocation("inColor", PROGRAM_NORMAL_ATTRIBUTE);
	m_program->link();
}


CAxisModel::~CAxisModel()
{
}

void CAxisModel::doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent)
{
	m_program->setUniformValue("projection", v_Projection);
	//m_view.translate(0.5f, 0.5f);
	m_program->setUniformValue("view", v_View);
	m_ModelMatrix.setToIdentity();
	// 		m_bgGroundModel.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	// 		m_bgGroundModel.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	// 		m_bgGroundModel.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_AxisNodeModel.translate(m_bgGroundModelPos);
	// 	m_AxisNodeModel.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	// 	m_AxisNodeModel.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	// 	m_AxisNodeModel.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	m_program->setUniformValue("model", m_ModelMatrix);
	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_NORMAL_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));

	m_program->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
	glDrawArrays(GL_LINES, 0, m_totalFaceNum);
}

void CAxisModel::makeObject()
{
	QVector<GLfloat> vertData =
	{
		-50.0f, -30.0f, 0.0f,1.0f,0.0f,0.0f,
		-20.3f, -30.0f, 0.0f,1.0f,0.0f,0.0f,
		-50.0f, -30.0f, 0.0f, 0.0f,1.0f,0.0f,
		-50.0f, -10.3f, 0.0f,0.0f,1.0f,0.0f,
		-50.0f, -30.0f, 0.0f, 0.0f,0.0f,1.0f,
		-50.0f, -30.0f, 20.0f,0.0f,0.0f,1.0f,
	};
	//{ 50.0f, 0.0f, 50.0f, 50.0f, 0.0f, -50.0f, -50.0f, 0.0f, -50.0f, -50.0f, 0.0f, 50.0f };
	BaseModel::makeObject(vertData, 6);
}