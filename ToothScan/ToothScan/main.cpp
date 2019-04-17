#include "mainwidget.h"
#include <QtWidgets/QApplication>
#include <vector>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QFont f("Î¢ÈíÑÅºÚ", 14);
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
	MainWidget w;
	return a.exec();
}
