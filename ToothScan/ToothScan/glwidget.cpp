#include "glwidget.h"
#include <QPainter>
#include <QPaintEngine>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QCoreApplication>
#include <QMouseEvent>
#include <math.h>

#include "mainwidget.h"

//const int bubbleNum = 8;

GLWidget::GLWidget(const QColor &background) :
	clearColor(Qt::white),
	xRot(0),
	yRot(0),
	zRot(0),
	program(0)
{
	orth::ModelRead mr("./0016.off", vertices_in);
	glbackground = background;
	setMinimumSize(300, 250);
	this->initVariable();
	this->constructIHM();
	this->setConnections();
}

GLWidget::~GLWidget()
{
	makeCurrent();
	vbo.destroy();
	delete program;
	doneCurrent();
}

QSize GLWidget::minimumSizeHint() const
{
	return QSize(500, 500);
}

QSize GLWidget::sizeHint() const
{
	return QSize(1000, 1000);
}

void GLWidget::rotateBy(int xAngle, int yAngle, int zAngle)
{
	xRot += xAngle;
	yRot += yAngle;
	zRot += zAngle;
	update();
}

void GLWidget::setClearColor(const QColor &color)
{
	clearColor = color;
	update();
}

void GLWidget::initVariable()
{
	//cameraWindow
	cameraWindow = new QDockWidget(QStringLiteral("相机显示"),this);
	//buttontoolButtons
	leftWatchButton = new QToolButton(this);
	rightWatchButton = new QToolButton(this);
	topWatchButton = new QToolButton(this);
	bottomWatchButton = new QToolButton(this);
	frontWatchButton = new QToolButton(this);
	backWatchButton = new QToolButton(this);

	spinCameraBox = new QSpinBox(this);
	sliderCamera = new QSlider(Qt::Horizontal);
	spinCameraBox->setRange(0, 130);
	sliderCamera->setRange(0, 130);


}

void GLWidget::constructIHM()
{
	//相机设置
	cameraWindow->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable); // 设置停靠窗口特性，可移动,可关闭
	cameraWindow->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);//设置可停靠区域为主窗口左边和右边
	QTextEdit *textEdit = new QTextEdit("DockWindow First");
	
	QWidget *bottomCameraWidget = new QWidget(this);
	QHBoxLayout *bottomCameraHLayout = new QHBoxLayout(bottomCameraWidget);
	bottomCameraHLayout->addWidget(spinCameraBox);
	bottomCameraHLayout->addWidget(sliderCamera);
	bottomCameraHLayout->setContentsMargins(0, 0, 0, 0);

	QWidget *cameraWidget = new QWidget(this);
	QVBoxLayout *cameraVLayout = new QVBoxLayout(cameraWidget);
	cameraVLayout->addWidget(textEdit);
	cameraVLayout->addWidget(bottomCameraWidget);
	cameraVLayout->setContentsMargins(0, 0, 0, 0);

	cameraWindow->setWidget(cameraWidget);
	QWidget *totalCameraWidget = new QWidget(this);
	totalCameraWidget->setFixedSize(400, 400);
	QHBoxLayout *totalCameraHLayout = new QHBoxLayout(totalCameraWidget);
	totalCameraHLayout->addWidget(cameraWindow);
	totalCameraHLayout->addStretch();
	totalCameraHLayout->setContentsMargins(0, 0, 0, 0);

	//MiddleDialog中间对话提示框


	//BottomTool底部工具栏
	//leftWatchButton->setFixedSize(20,20);
	leftWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/LeftView.png"));
	leftWatchButton->setIconSize(QSize(leftWatchButton->width(), leftWatchButton->height()));
	leftWatchButton->setStyleSheet("border-style:flat");
	//connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	//rightWatchButton->setFixedSize(20, 20);
	rightWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/RightView.png"));
	rightWatchButton->setIconSize(QSize(rightWatchButton->width(), rightWatchButton->height()));
	rightWatchButton->setStyleSheet("border-style:flat");
	//connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	//topWatchButton->setFixedSize(20, 20);
	topWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/TopView.png"));
	topWatchButton->setIconSize(QSize(topWatchButton->width(), topWatchButton->height()));
	topWatchButton->setStyleSheet("border-style:flat");
	//connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	//bottomWatchButton->setFixedSize(20, 20);
	bottomWatchButton->setIcon(QIcon(":/MainWidget/Resources/images/BottomView.png"));
	bottomWatchButton->setIconSize(QSize(bottomWatchButton->width(), bottomWatchButton->height()));
	bottomWatchButton->setStyleSheet("border-style:flat");
	//connect(leftWatchButton, &QPushButton::clicked, this, &GLWidget::handleButtonPress);

	QWidget *bottomButtonWidget = new QWidget(this);
	bottomButtonWidget->setFixedSize(300,100);
	QHBoxLayout *bottomButtonHLayout = new QHBoxLayout(bottomButtonWidget);
	
	bottomButtonHLayout->addWidget(leftWatchButton);
	bottomButtonHLayout->addWidget(rightWatchButton);
	bottomButtonHLayout->addWidget(topWatchButton);
	bottomButtonHLayout->addWidget(bottomWatchButton);
	QWidget *bottomWidget = new QWidget(this);
	QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);
	bottomHLayout->addStretch();
	bottomHLayout->addWidget(bottomButtonWidget);
	bottomHLayout->addStretch();

	QVBoxLayout *totalVLayout = new QVBoxLayout(this);
	totalVLayout->addWidget(totalCameraWidget);
	totalVLayout->addStretch();
	totalVLayout->addWidget(bottomWidget);
}

void GLWidget::setConnections()
{
	QObject::connect(sliderCamera, &QSlider::valueChanged, spinCameraBox, &QSpinBox::setValue);
	void (QSpinBox:: *spinBoxSignal)(int) = &QSpinBox::valueChanged;
	QObject::connect(spinCameraBox, spinBoxSignal, sliderCamera, &QSlider::setValue);
	spinCameraBox->setValue(35);
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	
	makeObject();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

	QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
	const char *vsrc =
		"attribute highp vec4 vertex;\n"
		"attribute mediump vec3 normal;\n"
		"uniform mediump mat4 matrix;\n"
		"varying mediump vec4 color;\n"
		"void main(void)\n"
		"{\n"
		//"    vec3 toLight = normalize(vec3(0.0, 0.3, 1.0));\n"
		//"    float angle = max(dot(normal, toLight), 0.0);\n"
		//"    vec3 col = vec3(0.40, 1.0, 0.0);\n"
		//"    color = vec4(col * 0.2 + col * 0.8 * angle, 1.0);\n"
		//"    color = clamp(color, 0.0, 1.0);\n"
		"	 color = vec4(normal,1.0);\n"
		"    gl_Position = matrix * vertex;\n"
		"}\n";
	vshader->compileSourceCode(vsrc);

	QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
	const char *fsrc =
		"varying mediump vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"    gl_FragColor = color;\n"
		"}\n";

	fshader->compileSourceCode(fsrc);

	program = new QOpenGLShaderProgram;
	program->addShader(vshader);
	program->addShader(fshader);
	program->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
	program->bindAttributeLocation("normal", PROGRAM_TEXCOORD_ATTRIBUTE);
	program->link();
	program->bind();
}

void GLWidget::resizeGL(int width, int height)
{
	//glViewport(0, 0, w, h);
	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);
}

void GLWidget::paintGL()
{
	glClearColor(clearColor.redF(), clearColor.greenF(), clearColor.blueF(), clearColor.alphaF());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	QMatrix4x4 m;
	m.ortho(-0.5f, +0.5f, +0.5f, -0.5f, 4.0f, 15.0f);
	m.translate(0.0f, 0.0f, -10.0f);
	m.rotate(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	m.rotate(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	m.rotate(zRot / 16.0f, 0.0f, 0.0f, 1.0f);

	program->setUniformValue("matrix", m);
	program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	program->enableAttributeArray(PROGRAM_TEXCOORD_ATTRIBUTE);
	program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
	program->setAttributeBuffer(PROGRAM_TEXCOORD_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));

	glDrawArrays(GL_TRIANGLES, 0, vertices_in.size() / 3);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		rotateBy(8 * dy, 8 * dx, 0);
	}
	else if (event->buttons() & Qt::RightButton) {
		rotateBy(8 * dy, 0, 8 * dx);
	}
	lastPos = event->pos();
}

//void GLWidget::mouseReleaseEvent(QMouseEvent * /* event */)
//{
//	emit clicked();
//}

void GLWidget::makeObject()
{

	float min_y = 1, max_y = -1;
	for (int i = 0; i < vertices_in.size() / 3; i++)
	{
		float y = vertices_in[3 * i + 2] / 100.0;
		if (min_y>y)
		{
			min_y = y;
		}
		if (max_y<y)
		{
			max_y = y;
		}
	}

	QVector<GLfloat> vertData;
	if (vertices_in.size()>0)
	{

		for (int i = 0; i < vertices_in.size() / 3; i++)
		{
			float x, y, z;
			x = vertices_in[3 * i + 0] / 100.0;
			y = vertices_in[3 * i + 1] / 100.0;
			z = vertices_in[3 * i + 2] / 100.0;
			vertData.append(x);
			vertData.append(y);
			vertData.append(z);
			vertData.append((y - min_y) / (max_y - min_y));
			vertData.append(1 - (y - min_y) / (max_y - min_y));
			vertData.append(0.3);
		}
	}

	vbo.create();
	vbo.bind();
	vbo.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
}

