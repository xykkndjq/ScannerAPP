#pragma once
#include <map>
using namespace std;
#include "commondefine.h"
#define B_SAVESPLITEMODEL "bsaveSpliteModel"
#define STR_SCANDATAPATH "strScanDataPath"
#define B_GPU "bGPU"
class CSystemConfig
{
public:
	CSystemConfig();
	~CSystemConfig();
	string getValue(string strkey) {
		if (m_KeyValueMap.find(strkey) != m_KeyValueMap.end()) {
			return m_KeyValueMap[strkey];
		}
		else
			return "";
	}
	void setValue(string strKey , string strValue) {
		m_KeyValueMap[strKey] = strValue;
	}
	void save();
	void load();
private:
	map<string, string> m_KeyValueMap;

	DECLARE_SINGLEINSTANCE(CSystemConfig);
};

