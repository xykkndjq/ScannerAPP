#include "mainwidget.h"
#include <QtWidgets/QApplication>
#include <vector>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWidget w;
	return a.exec();
}
