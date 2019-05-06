#include "mainwidget.h"
#include <QtWidgets/QApplication>
#include <vector>
#include "minidmp.h"
// #define _CRTDBG_MAP_ALLOC
// #include <stdlib.h>
// #include <crtdbg.h>
#define VLD_FORCE_ENABLE
#include<vld.h>
#include "TaskManager.h"
#include "SingleApplication.h"
using namespace Communication;
int main(int argc, char *argv[])
{
	
	std::ofstream log("toothscan.log");

	std::streambuf * oldbuf = std::cout.rdbuf(log.rdbuf());
	QApplication::addLibraryPath("./plugins");
	//RunCrashHandler();
	SetUnhandledExceptionFilter(ExceptionFilter);
	QApplication a(argc, argv);
	QFont f("微软雅黑", 14);
	f.setBold(true);
// 	widget->setStyleSheet("border:0px groove gray;border-radius:10px;padding:2px 4px;\
// 			border - color: rgb(128, 128, 128);\
// 		background - color: rgb(255, 255, 255); ");

	QPalette p;
	p.setColor(QPalette::ColorRole::ButtonText, QColor(128, 128, 128));
	a.setFont(f);
	a.setPalette(p);
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("gbk"));
	a.setQuitOnLastWindowClosed(true);
	SingleApplication abc(argc, argv);
	int nresult = 0;
	if (!abc.isRunning())
	{
		MainWidget w;
		//传入一个要激活程序的窗口，当多开时会激活已有进程的窗口，且多开失败
		abc.mainWindow = static_cast<IQWidegetsInterface *>(&w);
		//w.show();
		nresult = a.exec();
	}
	CTaskManager::getInstance()->DelAllTasks();
	return nresult;
}
