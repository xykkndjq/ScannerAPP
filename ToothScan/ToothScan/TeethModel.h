#pragma once
#include "BaseModel.h"
#include "commondefine.h"
#include "./include/3DScan.h"

class CTeethModel:public BaseModel
{
public:
	CTeethModel(shared_ptr<QOpenGLShaderProgram> v_Program,
		shared_ptr<QOpenGLBuffer> v_Vbo, orth::MeshModel &mm);
	~CTeethModel();
	virtual void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent);
	void setScreenPos(float xTrans,	float yTrans);
	void ChangeSelectedColorEx(QPoint drawRectClickPosition, QPoint drawRectEndPosition, IParentInterface *pParent);
	void ChosePoints(const float point1_x, const float point1_y,
		const float point2_x, const float point2_y, const int screen_width,
		const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix,
		IParentInterface *pParent);
	void setRectSelectRegion(QPoint beginPos,QPoint endPoint);
	void ChangeModelSelectedColor(QPoint beginPos, QPoint endPoint,IParentInterface * pParent);
	void makeObject();
	void delSelPoints();
	void cutModelUnderBg(QVector3D bgGroundModelPos);
	void getMeshModel(orth::MeshModel &meshModel);
	orth::MeshModel m_model;
	void setMaterialType(float fMaterialType) {
		m_fMaterialType = fMaterialType;
	}
private:
	QPoint m_drawRectClickPosition;
	QPoint m_drawRectEndPosition;
	float m_xTrans;
	float m_yTrans;
	float m_fMaterialType;
	QVector<GLfloat> m_vertData;
	orth::PointLabel m_Selected;
	PARAMDEFINE(string, str, ModelFileName);
};
SharedPtr(CTeethModel);

