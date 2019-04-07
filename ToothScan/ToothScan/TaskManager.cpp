#include "TaskManager.h"

extern const char *g_strScanName[7] = { { "全冠" },{ "牙冠" },{ "缺失牙" } ,
{ "嵌体" },
{ "上颌模型" },{ "下颌模型" },{ "全颌模型" } };

CTaskManager::CTaskManager()
{
	m_pCurrentTask = nullptr;
}


CTaskManager::~CTaskManager()
{
}

CScanTask::CScanTask()
{
	m_eTaskPro = eProgressNull;
	m_nAddModel = 0;
}

CScanTask::~CScanTask()
{
}

CTaskManager * CTaskManager::m_pInstance = nullptr;