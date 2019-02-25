#include "mainwidget.h"

#include <QApplication>
#include <QMenuBar>
#include <QGroupBox>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QScrollArea>

MainWidget::MainWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->initVariable();
}

MainWidget::~MainWidget()
{

}

//!---------------------------------------------------------------------------------
//! \brief MainWidget::initVariable
//!
void MainWidget::initVariable() 
{
	glwidget = new GLWidget(qRgb(128, 128, 128));
	glwidget->showMaximized();
	leftTabPage = new TabMainGUI();
	leftTabPage->showMaximized();
}

