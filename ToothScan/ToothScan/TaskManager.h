#pragma once
#include <iostream>
#include "commondefine.h"
#include <list>
#include <vector>
#include "basetype.h"
#include "TeethModel.h"
#include "datastream.h"
using namespace std;
enum eTaskType {
	eScan,
	eUpperStitching,
	eLowerStitching,
	eUpperTeethStit,
	eLowerTeethStit
};
enum eScanType {
	eScanNULL = -1,
	etotalCrown,
	etoothCrown,
	elossToothScan,
	einlayScan,
	eUpperJawScan,
	eLowerJawScan,
	eAllJawScan,

};
extern const char *g_strScanName[7];

enum eTaskProgress {
	eProgressNull = -1,
	eProgressScan,
	eProgressAdd,
	eProgressMesh,
	eProgressFinish
};

class CScanTask
{
public:
	CScanTask();
	~CScanTask();

public:
	vector<orth::MeshModel> m_mModel;//每次扫描的模型
	vector<vector<double>> m_points_cloud_globle;
	vector<double> m_points_cloud_end;
	vector<int> m_points_cloud_end_addSize;//每次补扫后m_points_cloud_end增加的长度
	int m_nAddModel;			//补扫了几次
	orth::MeshModel m_mAllModel;		//总模型
	vector<orth::MeshModel> m_mCutModel;	//剪切了几次模型 剪切后的模型
	pCTeethModel pTeethModel;
private:
	PARAMDEFINE(string, str, TaskName);
	PARAMDEFINE(eScanType, e, ScanType);
	PARAMDEFINE(int, i, TeethId);
	PARAMDEFINE(eTaskProgress, e, TaskPro);
	PARAMDEFINE(eTaskType, e, TaskType);
	PARAMDEFINE(string, str, ModelFileName);


public:
	virtual void StreamValue(datastream& kData, bool bSend);
	virtual char * getClassName() { return "CScanTask"; };
public:
	//friend ostream & operator<<(ostream & os, const CScanTask & c);
	//friend istream & operator >> (istream & is, CScanTask & c);
};
SharedPtr(CScanTask);

class CStitchingTask :public CScanTask
{
public:
	CStitchingTask() {
	}
	~CStitchingTask() {
	}
public:
	pCScanTask m_pSrcTask;
	pCScanTask m_pDstTask;
	virtual char * getClassName() { return "CStitchingTask"; };
};
SharedPtr(CStitchingTask);
class COralSubstituteScan :public CScanTask {
public:
	COralSubstituteScan() {}
	~COralSubstituteScan() {}
	void delToothByid(int iTeethId) {
		vector<int>::iterator iter = m_vtTeethId.begin();
		for (; iter != m_vtTeethId.end(); iter++) {
			if (iTeethId == (*iter)) {
				m_vtTeethId.erase(iter);
			}
		}
	}
	virtual void StreamValue(datastream& kData, bool bSend);
private:
	vector<int> m_vtTeethId;
};
SharedPtr(COralSubstituteScan);

class CGroupScan :public CScanTask {
public:
	CGroupScan() {}
	~CGroupScan() {}
public:
	vector<int> m_vtTeeth;

	virtual void StreamValue(datastream& kData, bool bSend);
};
SharedPtr(CGroupScan);

class CTaskManager
{
public:
	CTaskManager();
	~CTaskManager();
	void AddTask(pCScanTask baseTask, bool bFront = false) {
		if (bFront)
			m_vtBaseTask.push_front(baseTask);
		else
			m_vtBaseTask.push_back(baseTask);
	}
	pCScanTask getTask(string v_strTaskName) {
		list<pCScanTask>::iterator iter = m_vtBaseTask.begin();
		for (; iter != m_vtBaseTask.end(); iter++) {
			if ((*iter)->Get_TaskName() == v_strTaskName) {
				return (*iter);
			}
		}
		return nullptr;
	}
	pCScanTask getCurrentTask() {
		if (m_pCurrentTask == nullptr)
		{
			if (m_vtBaseTask.size() > 0)
				m_pCurrentTask = (*m_vtBaseTask.begin());
		}
		return m_pCurrentTask;
	}
	pCScanTask getNextTask() {
		list<pCScanTask>::iterator iter = m_vtBaseTask.begin();
		for (; iter != m_vtBaseTask.end(); iter++) {
			if ((*iter) == m_pCurrentTask) {
				if (++iter != m_vtBaseTask.end()) {
					m_pCurrentTask = (*iter);
				}
				else
					m_pCurrentTask = nullptr;
			}
		}
		return m_pCurrentTask;
	}
	pCScanTask getLastTask() {
		list<pCScanTask>::iterator iter = m_vtBaseTask.begin();
		for (; iter != m_vtBaseTask.end(); iter++) {
			if ((*iter) == m_pCurrentTask) {
				if (iter == m_vtBaseTask.begin()) {
					return nullptr;
				}
				else{
					m_pCurrentTask =*( --iter);
					return m_pCurrentTask;
				}
// 				if (++iter != m_vtBaseTask.end()) {
// 					m_pCurrentTask = (*iter);
// 				}
// 				else
// 					m_pCurrentTask = nullptr;
			}
		}
		return nullptr;
	}
	list<pCScanTask> &getTasks() {
		return m_vtBaseTask;
	}
	static CTaskManager * m_pInstance;
	static CTaskManager * getInstance() {
		if (m_pInstance == nullptr)
		{
			m_pInstance = new CTaskManager;
		}
		return m_pInstance;
	}
private:
	list<pCScanTask> m_vtBaseTask;
	pCScanTask m_pCurrentTask;
};

