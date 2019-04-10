#ifndef HLOGHELPER_H
#define HLOGHELPER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QMutex>
#include <QDateTime>





// 日志记录类
class HLogHelper : public QObject
{


	Q_OBJECT
public:


	HLogHelper();
	//---------------------------------------------------------------
	static HLogHelper * m_pInstance;
	static HLogHelper * getInstance() {
		if (m_pInstance == nullptr)
		{
			m_pInstance = new HLogHelper;
		}
		return m_pInstance;
	}



	// 设置文件名前缀，ABC_20180909_131415.log, 这里，参数就传递 ABC
	// 若不传递参数，默认以日期的方式命名：20180909_131415.log

	// 函数返回值：   0 - 创建文件成功，
	//              1 - 打开文件失败
	int HInit(QString strFilePre);


	// 对象销毁时，关闭文件
	//  返回值：    1 - 关闭失败，文件已经关闭

	//             0 - 关闭成功
	int HUnInit();

	//---------------------------------------------------------------


	// 时间 + 记录内容
	int HLogTime(QString str ...);

	// 记录内容
	int HLog(QString str...);




private:

	enum
	{
		// 日志文件大小
		he_log_file_size_1024kb = 1024,
	};


private:
	QString         m_FileLogName;
	QFile           m_File;
	QTextStream     m_LogTextStream;
	QMutex          m_FileLogMutex;
	QDateTime       m_DateTime;

};

#endif // HLOGHELPER_H