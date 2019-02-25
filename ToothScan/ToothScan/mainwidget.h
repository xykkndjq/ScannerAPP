#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QtWidgets/QWidget>
#include "ui_mainwidget.h"
#include "glWidget.h"
#include <QTimer>
#include <QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>

#include "TabMainGUI.h"

class MainWidget : public QWidget
{
	Q_OBJECT

public:
	MainWidget(QWidget *parent = 0);
	~MainWidget();

	void initVariable(); 

private slots:
	
private:
	Ui::MainWidgetClass ui;
	QVBoxLayout *totalVlayout;
	
	GLWidget *glwidget;
	TabMainGUI *leftTabPage;
};

#endif // MAINWIDGET_H
