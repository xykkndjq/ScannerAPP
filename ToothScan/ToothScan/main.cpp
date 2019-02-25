#include "toothscan.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ToothScan w;
	w.show();
	return a.exec();
}
