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
#include "SingleApplication.h"

class MainWidget : public QWidget, public IQWidegetsInterface
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
	virtual void recallWindow();

};

#endif // MAINWIDGET_H
