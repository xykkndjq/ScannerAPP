#include "SplitModelBtn.h"



CSplitModelBtn::CSplitModelBtn(QColor crBgColor, QString strPic, const QString &text,QWidget *parent)
	:QPushButton(text,parent)
	, m_coBgColor(crBgColor)
	, m_strPic(strPic)
{
}


CSplitModelBtn::~CSplitModelBtn()
{
}

void CSplitModelBtn::paintEvent(QPaintEvent * qevent)
{
//	Q_UNUSED(event);
	QPainter painter(this);
	painter.begin(this);
	QRect dirtyRect = qevent->rect();
	//QImage resultImage;
	//resultImage = QImage(QSize(dirtyRect.width(),dirtyRect.height()), QImage::Format_ARGB32_Premultiplied);
	//QPainter imgPainter(&resultImage);
	painter.setCompositionMode(QPainter::CompositionMode_Source);
	if (isChecked()) {
		painter.setBackgroundMode(Qt::BGMode::TransparentMode);
		painter.setCompositionMode(QPainter::CompositionMode_SourceAtop);
		painter.fillRect(dirtyRect, m_coBgColor);
		painter.setCompositionMode(QPainter::RasterOp_SourceAndDestination);
		QImage sourceImage;
		sourceImage.load(m_strPic);
		painter.drawImage(sourceImage.width() / 2, dirtyRect.height() / 2 - sourceImage.height() / 2, sourceImage);
		QTextOption option;
		option.setAlignment(Qt::AlignCenter);
		painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
		painter.drawText(dirtyRect, text(), option);
		//imgPainter.end();
		//painter.drawImage(0, 0, resultImage);
	}
	else {
		painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
		QImage sourceImage;
		sourceImage.load(m_strPic);
		painter.drawImage(sourceImage.width()/2, dirtyRect.height()/2-sourceImage.height()/2, sourceImage);
		QTextOption option;
		option.setAlignment(Qt::AlignCenter);
		painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
		painter.drawText(dirtyRect, text(), option);
// 		painter.end();
// 		painter.drawImage(0, 0, resultImage);
	}
	
	painter.end();
}
