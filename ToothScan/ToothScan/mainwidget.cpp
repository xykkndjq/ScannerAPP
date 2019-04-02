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
	scanMainPage = new ScanMainGUI();	
	scanMainPage->showMaximized();
}

void MainWidget::setConnections()
{
}

