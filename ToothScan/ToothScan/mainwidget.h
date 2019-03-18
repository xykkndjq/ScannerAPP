#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QtWidgets/QWidget>
#include "ui_mainwidget.h"
#include <QTimer>
#include <QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include "ScanMainGUI.h"

class MainWidget : public QWidget
{
	Q_OBJECT

public:
	MainWidget(QWidget *parent = 0);
	~MainWidget();

	void initVariable(); 
	void setConnections();
private:
	Ui::MainWidgetClass ui;
	QVBoxLayout *totalVlayout;
	
	ScanMainGUI *scanMainPage;


};

#endif // MAINWIDGET_H
