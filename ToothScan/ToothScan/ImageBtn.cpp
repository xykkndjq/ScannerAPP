#include "ImageBtn.h"
	#include <QPainter>
	#include <QPaintEvent>
	#include <qnamespace.h>

CImageBtn::CImageBtn(QString strIconImg, QColor crBgColor, QString strText, QWidget *parent)
	:QPushButton(strText,parent)
	,m_clBgColor(crBgColor)
{
	m_imBgImg.load(strIconImg);
}


CImageBtn::~CImageBtn()
{
}

void CImageBtn::paintEvent(QPaintEvent * qEvent)
{
	QPainter pPainter(this);
	QRect rtBtn = qEvent->rect();
	pPainter.setRenderHint(QPainter::Antialiasing, true);
	pPainter.begin(this);
	pPainter.setBackgroundMode(Qt::BGMode::TransparentMode);
	pPainter.setCompositionMode(QPainter::CompositionMode_Source);
	//pPainter.fillRect(rtBtn, QColor(255, 255, 255, 255));
	pPainter.fillRect(rtBtn, QColor(255, 255, 255, 255));
	pPainter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
	QRect imgRect = QRect(m_imBgImg.width() / 2 + 10, rtBtn.height() / 2 - m_imBgImg.height() / 2, m_imBgImg.width(), m_imBgImg.height());
	pPainter.drawImage(imgRect, m_imBgImg);
	QTextOption option;
	QRect textRect = QRect(imgRect.right() + 10, 0, rtBtn.width() - imgRect.right() - 10, rtBtn.height());
	option.setAlignment(Qt::AlignLeft| Qt::AlignVCenter);
	pPainter.drawText(textRect, text(), option);
	QPen orginPen = pPainter.pen();
	pPainter.setPen(QPen(m_clBgColor, 2));//ÉèÖÃ»­±ÊÐÎÊ½ 
	pPainter.drawEllipse(rtBtn.width() - 50, rtBtn.height()/2 - 20, 40, 40);//»­Ô²
	QBrush orginBrush = pPainter.brush();
	if (isChecked()) {
		pPainter.setPen(QPen(m_clBgColor, 2));//ÉèÖÃ»­±ÊÐÎÊ½ 
		pPainter.setBrush(QBrush(QColor(255,255,255,0)));
		pPainter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
		pPainter.drawRect(rtBtn);
		pPainter.setBrush(QBrush(m_clBgColor));
		pPainter.drawEllipse(rtBtn.width() - 50, rtBtn.height() / 2 - 20, 40, 40);//»­Ô²
	}
	else {
	}
	pPainter.setPen(orginPen);
	pPainter.setBrush(orginBrush);
	pPainter.end();
}


CTeethImgBtn::CTeethImgBtn(QString strIconImg, QString strText, QWidget *parent)
	:QPushButton(strText, parent)
{
	m_imBgImg.load(strIconImg);
	setFixedSize(m_imBgImg.size());
}


CTeethImgBtn::~CTeethImgBtn()
{
}

void CTeethImgBtn::SetImage(QString strFile)
{
	m_imBgImg.load(strFile);
	update();
}

void CTeethImgBtn::SetImage(QImage &image)
{
	m_imBgImg = image;
	update();
}

QSize CTeethImgBtn::sizeHint()
{
	return m_imBgImg.size();
}

void CTeethImgBtn::paintEvent(QPaintEvent * qEvent)
{
	QPainter pPainter(this);
	QRect rtBtn = qEvent->rect();
	pPainter.begin(this);
	pPainter.setBackgroundMode(Qt::BGMode::TransparentMode);
	pPainter.drawImage(rtBtn, m_imBgImg);
	QTextOption option;
	option.setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
	pPainter.drawText(rtBtn, text(), option);
	pPainter.end();
}

void CTeethImgBtn::mouseMoveEvent(QMouseEvent *e)
{
	return;
}

void CTeethImgBtn::focusInEvent(QFocusEvent *)
{

}

void CTeethImgBtn::focusOutEvent(QFocusEvent *)
{

}

