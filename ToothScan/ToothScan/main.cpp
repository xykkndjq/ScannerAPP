#include "mainwidget.h"
#include <QtWidgets/QApplication>
#include <vector>

using namespace Communication;
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("gbk"));
	a.setQuitOnLastWindowClosed(true);
	MainWidget w;
	return a.exec();
}
