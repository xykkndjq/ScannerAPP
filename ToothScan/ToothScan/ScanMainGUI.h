#pragma once

#include <QWidget>
#include "ui_ScanMainGUI.h"

class ScanMainGUI : public QWidget
{
	Q_OBJECT

public:
	ScanMainGUI(QWidget *parent = Q_NULLPTR);
	~ScanMainGUI();

private:
	Ui::ScanMainGUI ui;
};
