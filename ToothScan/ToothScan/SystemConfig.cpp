#include "SystemConfig.h"
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <iostream>
#include <fstream>
#include <qfile.h>
using namespace std;

DEFINITION_SINGLEINSTANCE(CSystemConfig);
CSystemConfig::CSystemConfig()
{
}


CSystemConfig::~CSystemConfig()
{
}

void CSystemConfig::save()
{
	QJsonDocument document;
	QJsonArray jsonArray;
	QJsonObject simp_ayjson;
	map<string, string>::iterator mapIter = m_KeyValueMap.begin();
	int i = 0;
	for (; mapIter != m_KeyValueMap.end(); mapIter++) {
		simp_ayjson.insert(mapIter->first.c_str(), mapIter->second.c_str());
	}
	document.setObject(simp_ayjson);
	QByteArray simpbyte_array = document.toJson(QJsonDocument::Compact);
	QString simpjson_str(simpbyte_array);
	std::string filePathStr2 = "config.json";
	fstream f(filePathStr2, ios::out);//供写使用，文件不存在则创建，存在则清空原内容 
	f << simpjson_str.toStdString(); //写入数据
	f.close();
}

void CSystemConfig::load()
{
	QFile file2("config.json");
	if (!file2.open(QIODevice::ReadOnly))
	{
		return;
	}
	QByteArray ba = file2.readAll();
	QJsonParseError e;
	QJsonDocument jsonDoc = QJsonDocument::fromJson(ba, &e);
	if (e.error == QJsonParseError::NoError && !jsonDoc.isNull())
	{
	}
	if (jsonDoc.isObject()) {
		QJsonObject jsonObject = jsonDoc.object();
		QJsonObject::iterator jsonIter = jsonObject.begin();
		for (; jsonIter != jsonObject.end();jsonIter++) {
			QString strKey = jsonIter.key();
			QString strValue = jsonIter.value().toString();
			m_KeyValueMap.insert(make_pair(strKey.toStdString(), strValue.toStdString()));
		}
	}
}
