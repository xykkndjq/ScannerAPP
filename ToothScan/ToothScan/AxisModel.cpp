#include "AxisModel.h"

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_NORMAL_ATTRIBUTE 1
#define PROGRAM_MATERIAL_ATTRIBUTE 2
#define PROGRAM_STATE_ATTRIBUTE 3

void drawString(const char* str) {
	static int isFirstCall = 1;
	static GLuint lists;

	if (isFirstCall) { // 如果是第一次调用，执行初始化
					   // 为每一个ASCII字符产生一个显示列表
		isFirstCall = 0;

		// 申请MAX_CHAR个连续的显示列表编号
		lists = glGenLists(255);

		// 把每个字符的绘制命令都装到对应的显示列表中
		wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, lists);
	}
	// 调用每个字符对应的显示列表，绘制每个字符
	for (; *str != '\0'; ++str)
		glCallList(lists + *str);
}

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
	QMatrix4x4 Projection;
	Projection.setToIdentity();
// 	Projection.ortho(0,0,1920,1080,-1,300);
// 	Projection.setToIdentity();
	Projection.perspective(20, (float)1920 / (float)1080, 1.0f, 300.0f);
	m_program->setUniformValue("projection", Projection);
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
	m_ModelMatrix.setToIdentity();
// 	m_ModelMatrix.rotate(m_xRot / 16.0f, 1.0f, 0.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_yRot / 16.0f, 0.0f, 1.0f, 0.0f);
// 	m_ModelMatrix.rotate(m_zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	//m_ModelMatrix.rotate(m_ModelRotate);
	m_program->setUniformValue("model", m_ModelMatrix);
	m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	m_program->enableAttributeArray(PROGRAM_NORMAL_ATTRIBUTE);
	m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));

	m_program->setAttributeBuffer(PROGRAM_NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glLineWidth(5);
	glPointSize(3);
	glDrawArrays(GL_LINES, 0, m_totalFaceNum);

	pParent->glUseProgram(0);
	glColor3f(1.0f, 0.0f, 0.0f);
	QVector4D stringPos = { 0.0f,0.0f,0.0f,1.0f};
	stringPos.setX(stringPos.x() + 11);
	//cout << "stringPos x" << stringPos.x() << "stringPos y" << stringPos.y() << "stringPos z" << stringPos.z() << endl;
	stringPos = Projection * v_View * stringPos;
	//cout << "stringPos x" << stringPos.x() << "stringPos y" << stringPos.y() << "stringPos z" << stringPos.z() <<"stringPos w"<< stringPos.w()<< endl;
	stringPos += QVector4D(stringPos.w() * -0.8, stringPos.w()*-0.8, 0,0);
	cout << "stringPos x" << stringPos.x() << "stringPos y" << stringPos.y()<< "stringPos z" <<stringPos.z()<< "stringPos w" << stringPos.w() << endl;
	glRasterPos3f(stringPos.x()/stringPos.w(), stringPos.y() / stringPos.w(), stringPos.z() / stringPos.w());
	drawString("X");

	glColor3f(0.0f, 1.0f, 0.0f);
	stringPos = { -0.0f,-0.0f,0.0f,1.0f};
	stringPos.setY(stringPos.y() + 11);
	stringPos = Projection * v_View * stringPos;
	stringPos += QVector4D(stringPos.w() * -0.8, stringPos.w()*-0.8, 0, 0);
	glRasterPos3f(stringPos.x() / stringPos.w(), stringPos.y() / stringPos.w(), stringPos.z() / stringPos.w());
	drawString("Y");

	glColor3f(0.0f, 0.0f, 1.0f);
	stringPos = { -0.0f,-0.0f,0.0f,1.0f};
	stringPos.setZ(stringPos.z() + 11);
	stringPos = Projection * v_View * stringPos;
	stringPos += QVector4D(stringPos.w() * -0.8, stringPos.w()*-0.8, 0, 0);
	glRasterPos3f(stringPos.x() / stringPos.w(), stringPos.y() / stringPos.w(), stringPos.z() / stringPos.w());
	drawString("Z");

// 	glLabel::Mode md;
// 	md.qFont.setBold(true);
// 	md.qFont.setPixelSize(12);
// 	float d = 10 + 1.5;
// 	if (p) {
// 		vcg::glLabel::render(p, vcg::Point3f(d, 0, 0), QString("X"), md);
// 		vcg::glLabel::render(p, vcg::Point3f(0, d, 0), QString("Y"), md);
// 		vcg::glLabel::render(p, vcg::Point3f(0, 0, d), QString("Z"), md);
// 	}

	/*QVector3D a(-50.0f, -30.0f, 0.0f);
	GLubyte xcolor[4] = { 255,0,0,255 },
		ycolor[4] = { 0,255,0,255 },
		zcolor[4] = { 0,0,255,255 };
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glLineWidth(2);
	glPointSize(3);
	glBegin(GL_LINES);
	glColor4ubv(xcolor);
	QVector3D start = v_Projection * v_View * a,
		end = v_Projection * v_View * QVector3D(a.x() + 10, a.y(), a.z());
	glVertex3f(start.x(), start.y(), start.y()); glVertex3f(end.x(), end.y(), end.z());
	glColor4ubv(ycolor);
	end = v_Projection * v_View * QVector3D(a.x(), a.y() + 10, a.z());
	glVertex3f(start.x(), start.y(), start.y()); glVertex3f(end.x(), end.y(), end.z());
	glColor4ubv(zcolor);
	end = v_Projection * v_View * QVector3D(a.x(), a.y(), a.z() + 10);
	glVertex3f(start.x(), start.y(), start.y()); glVertex3f(end.x(), end.y(), end.z());
	glEnd();*/
}

void CAxisModel::makeObject()
{
	QVector<GLfloat> vertData =
// 	{
// 		0.0f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
// 		0.3f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
// 		0.0f, 0.0f, 0.0f, 0.0f,1.0f,0.0f,
// 		0.0f, 0.3f, 0.0f,0.0f,1.0f,0.0f,
// 		0.0f, 0.0f, 0.0f, 0.0f,0.0f,1.0f,
// 		0.0f, 0.0f, 0.3f,0.0f,0.0f,1.0f,
// 	};
	{
		0.0f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
		10.0f, 0.0f, 0.0f,1.0f,0.0f,0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,1.0f,0.0f,
		0.0f, 10.0f, 0.0f,0.0f,1.0f,0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,0.0f,1.0f,
		0.0f, 0.0f, 10.0f,0.0f,0.0f,1.0f,
	};
// 	{
// 		-60.0f, -30.0f, 0.0f,1.0f,0.0f,0.0f,
// 		-50.3f, -30.0f, 0.0f,1.0f,0.0f,0.0f,
// 		-60.0f, -30.0f, 0.0f, 0.0f,1.0f,0.0f,
// 		-60.0f, -20.3f, 0.0f,0.0f,1.0f,0.0f,
// 		-60.0f, -30.0f, 0.0f, 0.0f,0.0f,1.0f,
// 		-60.0f, -30.0f, 10.0f,0.0f,0.0f,1.0f,
// 	};
	//{ 50.0f, 0.0f, 50.0f, 50.0f, 0.0f, -50.0f, -50.0f, 0.0f, -50.0f, -50.0f, 0.0f, 50.0f };
	BaseModel::makeObject(vertData, 6);
}