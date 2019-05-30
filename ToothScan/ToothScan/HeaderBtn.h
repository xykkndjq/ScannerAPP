#pragma once
#include <QPushButton>
class CHeaderCheckBtn :public QPushButton
{
public:
	CHeaderCheckBtn(QString strCheckIconImg, QString strUnCheckIconImg , QString strText, QColor unCheckBgColor,QColor checkBgColor,QColor unCheckFontColor , QColor checkFontColor ,
		QWidget *parent = Q_NULLPTR);
	~CHeaderCheckBtn();
// 	void SetImage(QString strFile);
// 	void SetImage(QImage &image);
protected:
	void paintEvent(QPaintEvent *);
private:
	QString m_strCheckIcon;
	QString m_strUnCheckIcon;
	QColor m_unCheckBgColor;
	QColor m_checkBgColor;
	QColor m_unCheckFontColor;
	QColor m_checkFontColor;
};

