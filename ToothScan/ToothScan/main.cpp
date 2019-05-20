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
	QDateTime       m_DateTime;
	QString logPath = "log";
	QDir dir(logPath);

	if (false == dir.exists())
	{
		dir.mkpath(logPath);
	}
	std::ofstream log(logPath.toStdString() + QString("/toothscan.log").toStdString()+ m_DateTime.currentDateTime().toString("yyyy_MM_dd_hh_mm_ss_zzz").toStdString());

	std::streambuf * oldbuf = std::cout.rdbuf(log.rdbuf());
	QApplication::addLibraryPath("./plugins");
	//RunCrashHandler();
	SetUnhandledExceptionFilter(ExceptionFilter);
	SingleApplication a(argc, argv);
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
//	SingleApplication abc(argc, argv);
	int nresult = 0;
	if (!a.isRunning())
	{
		MainWidget w;
		//传入一个要激活程序的窗口，当多开时会激活已有进程的窗口，且多开失败
		a.mainWindow = static_cast<IQWidegetsInterface *>(&w);
		//w.show();.
		nresult = a.exec();
	}
	CTaskManager::getInstance()->DelAllTasks();
	return nresult;
}
