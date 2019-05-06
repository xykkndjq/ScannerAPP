#pragma once
#include <QPushButton>
class CImageBtn:public QPushButton
{
public:
	CImageBtn(QString strIconImg , QColor crBgColor,QString strText,QWidget *parent = Q_NULLPTR);
	~CImageBtn();
protected:
	void paintEvent(QPaintEvent *) ;
private:
	QColor m_clBgColor;
	QImage m_imBgImg;
};

class CTeethImgBtn:public QPushButton
{
public:
	CTeethImgBtn(QString strIconImg, QString strText, QWidget *parent = Q_NULLPTR);
	~CTeethImgBtn();
	void SetImage(QString strFile);
	void SetImage(QImage &image);
	QSize sizeHint();
protected:
	void paintEvent(QPaintEvent *);
	void mouseMoveEvent(QMouseEvent *e);
	void focusInEvent(QFocusEvent *) Q_DECL_OVERRIDE;
	void focusOutEvent(QFocusEvent *) Q_DECL_OVERRIDE;
private:
	QImage m_imBgImg;
};

