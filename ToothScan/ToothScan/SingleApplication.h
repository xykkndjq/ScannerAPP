
#ifndef SINGLEAPPLICATION_H
#define SINGLEAPPLICATION_H

#include <QObject>
#include <QApplication>
__interface IQWidegetsInterface
{
	virtual void recallWindow() = 0;
};
class QWidget;
class QLocalServer;

class SingleApplication : public QApplication
{
	Q_OBJECT
public:
	SingleApplication(int &argc, char **argv);
	bool isRunning();               // 是否已经有实例在运行
	IQWidegetsInterface *mainWindow;            // MainWindow指针

	private slots:
	// 有新连接时触发
	void newLocalConnection();

private:
	// 初始化本地连接
	void initLocalConnection();
	// 创建服务端
	void newLocalServer();
	bool bRunning;                  // 是否已经有实例在运行
	QLocalServer *localServer;      // 本地socket Server
	QString serverName;             // 服务名称
};

#endif // SINGLEAPPLICATION_H