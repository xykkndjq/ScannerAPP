#include "hloghelper.h"
#include <QObject>
#include <QDir>
#include <QApplication>
#include <cstdarg>
#include <QByteArray>

HLogHelper * HLogHelper::m_pInstance = nullptr;

HLogHelper::HLogHelper()
{
	m_FileLogName = QString("");
	HInit(QString("log"));
}


// 初始化创建文件并打开文件
int HLogHelper::HInit(QString strFilePre)
{

	int len = strFilePre.length();

	QString  fileName("");

	// 设置文件名
	// 1、若strFilePre不为空
	if (0 < len)
	{
		// 获取当前日期
		QString date = m_DateTime.currentDateTime().toString("yyyy_MM_dd_hh_mm_ss_zzz");
		fileName = strFilePre + QString("_") + date;

	}
	else
	{
		//
		QString date = m_DateTime.currentDateTime().toString("yyyy_MM_dd_hh_mm_ss_zzz");
		fileName = date;
	}

	m_FileLogName = fileName + QString("_.log");




	// 2、打开文件

	// 若当前exe所在目录下不存在 HLog文件夹，则创建
	QString logPath = QApplication::applicationDirPath() + QString("/HLog/");
	QDir dir(logPath);

	if (false == dir.exists())
	{
		dir.mkpath(logPath);
	}

	// 构造文件
	m_FileLogName = logPath + m_FileLogName;
	m_File.setFileName(m_FileLogName);

	bool openFlag = m_File.open(QIODevice::Text | QIODevice::Truncate | QIODevice::WriteOnly | QIODevice::Append);
	if (false == openFlag)
	{
		return 1;
	}


	m_LogTextStream.setDevice(&m_File);


	return 0;
}

// 关闭文件
int HLogHelper::HUnInit()
{
	bool isExist = m_File.exists(m_FileLogName);

	// 若不存在
	if (false == isExist)
	{
		return 1;
	}

	// 文件存在，检查文件是否已经打开
	bool hasOepned = m_File.isOpen();

	// 文件打开了
	if (true == hasOepned)
	{
		m_File.flush();
		m_File.close();
	}

	return 0;
}


// 日志记录前带日期
int HLogHelper::HLogTime(QString str...)
{
	// 获取当前日期
	QString date = m_DateTime.currentDateTime().toString("yyyy_MM_dd hh_mm_ss_zzz:");

	QByteArray ba = (date + str).toLocal8Bit();
	char *pArr = ba.data();

	va_list al;
	va_start(al, pArr);
	QString strResult = QString::vasprintf(pArr, al);
	va_end(al);

	m_LogTextStream << strResult << endl;
	m_LogTextStream.flush();

	return 0;

}


// 日志前不带日期
int HLogHelper::HLog(QString str...)
{

	QByteArray ba = str.toLocal8Bit();
	char *pArr = ba.data();

	va_list al;
	va_start(al, pArr);
	QString strResult = QString::vasprintf(pArr, al);
	va_end(al);

	m_LogTextStream << strResult << endl;
	m_LogTextStream.flush();

	return 0;

}