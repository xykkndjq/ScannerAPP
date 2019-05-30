#include "HeaderBtn.h"
#include <QPainter>
#include <QPaintEvent>
#include <qnamespace.h>



CHeaderCheckBtn::CHeaderCheckBtn(QString strIconImg, QString strUnCheckIconImg, QString strText, QColor unCheckBgColor, QColor checkBgColor, QColor unCheckFontColor, QColor checnFontColor,
	QWidget *parent)
	:QPushButton(strText, parent)
	,m_unCheckBgColor(unCheckBgColor)
	,m_checkBgColor(checkBgColor)
	,m_unCheckFontColor(unCheckFontColor)
	,m_checkFontColor(checnFontColor)
	, m_strCheckIcon(strIconImg)
	, m_strUnCheckIcon(strUnCheckIconImg)
{
	//m_imgIcon.load(strIconImg);
}


CHeaderCheckBtn::~CHeaderCheckBtn()
{
}

// void CHeaderCheckBtn::SetImage(QString strFile)
// {
// 
// }
// 
// void CHeaderCheckBtn::SetImage(QImage &image)
// {
// 
// }

void CHeaderCheckBtn::paintEvent(QPaintEvent * qEvent)
{
	QPainter pPainter(this);
	pPainter.begin(this);
	QRect rtBtn = qEvent->rect();
	QString strIcon;
	QColor crBgColor = m_unCheckBgColor, crFontColor;
	if (isChecked()) {
		strIcon = m_strCheckIcon;
		//crBgColor = m_checkBgColor;
		crFontColor = m_checkFontColor;
	}
	else {
		strIcon = m_strUnCheckIcon;
		//crBgColor = m_unCheckBgColor;
		crFontColor = m_unCheckFontColor;
	}
	QImage iconImg(strIcon);
	QPen pen(crFontColor);
	QBrush brush(crBgColor);
	pPainter.setPen(QPen(crBgColor));
	pPainter.setBrush(brush);
	pPainter.setBackgroundMode(Qt::BGMode::TransparentMode);
	pPainter.drawRect(rtBtn);
	QFontMetrics fm = pPainter.fontMetrics();
	int fontWidth = fm.width(text()),spinWhite = 8;
	int contentWidth = fontWidth + spinWhite + iconImg.width();
	pPainter.drawImage(QPoint((rtBtn.width()-contentWidth)/2,0),iconImg);
	pPainter.setPen(pen);
	pPainter.drawText(QPoint((rtBtn.width() - contentWidth) / 2 +iconImg.width()+ spinWhite, 20),text());
	if (isChecked()) {
		QPen pen(m_checkBgColor);
		QBrush brush(m_checkBgColor);
		pPainter.setPen(QPen(crBgColor));
		pPainter.setBrush(brush);
		pPainter.drawRoundedRect(QRect(0,rtBtn.height()-8,rtBtn.width(),8),5,5);
	}
	pPainter.end();
}
