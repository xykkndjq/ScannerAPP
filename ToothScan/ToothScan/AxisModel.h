#pragma once
#include "BaseModel.h"
class CAxisModel:public BaseModel
{
public:
	CAxisModel(const char * v_vsFile, const char * v_fsfile, QObject *parent);
	~CAxisModel();
	void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent);
	void makeObject();
};
SharedPtr(CAxisModel);

