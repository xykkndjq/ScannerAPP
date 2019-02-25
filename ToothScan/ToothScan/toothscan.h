#ifndef TOOTHSCAN_H
#define TOOTHSCAN_H

#include <QtWidgets/QWidget>
#include "ui_toothscan.h"

class ToothScan : public QWidget
{
	Q_OBJECT

public:
	ToothScan(QWidget *parent = 0);
	~ToothScan();

private:
	Ui::ToothScanClass ui;
};

#endif // TOOTHSCAN_H
