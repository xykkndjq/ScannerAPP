#pragma once
#include<QPushButton>
#include <QEvent.h>
#include <QPainter>

QT_BEGIN_NAMESPACE
class CSplitModelBtn:public QPushButton
{
	Q_OBJECT
public:
	CSplitModelBtn(QColor crBgColor , QString strPic , const QString &text,QWidget *parent = Q_NULLPTR);
	~CSplitModelBtn();
protected:
	void paintEvent(QPaintEvent * qevent) Q_DECL_OVERRIDE;
private:
	QColor m_coBgColor;
	QString m_strPic;
};
QT_END_NAMESPACE

