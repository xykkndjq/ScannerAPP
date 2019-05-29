#pragma once
#include "BaseModel.h"
#include <QOpenGLVertexArrayObject>  
#include <QOpenGLTexture>  
#include "include\basetype.h"
#include <opencv2/opencv.hpp>


#ifndef PI
#define PI 3.14159265358979
#endif // !#define PI 3.14159265358979


enum BoxFace {
	x_p = 0,
	y_p = 1,
	z_p = 2,
	x_n = 3,
	y_n = 4,
	z_n = 5
};

enum BoxMovement {
	Trans = 0,
	Rot = 1,
	Stretch = 2
};

enum BoxMoveDirection {
	Positive = 0,
	Negative = 1
};

class CCutBoxObject :public BaseModel
{

public:
	CCutBoxObject(shared_ptr<QOpenGLShaderProgram> v_Program,
		shared_ptr<QOpenGLBuffer> v_Vbo);
	CCutBoxObject(const char * v_vsFile, const char * v_fsfile, QObject *parent);
	~CCutBoxObject();
	virtual void doPaint(QMatrix4x4 v_Projection, QMatrix4x4 v_View, IParentInterface *pParent);
	//void readPicture(QString &file_name);
	//QVector4D m_bkGroundColor;
	//QOpenGLTexture *textures;
	QVector4D m_bkGroundColor;
	QVector4D x_color, y_color, z_color;
	QVector4D press_color_x_color, press_color_y_color, press_color_z_color;
	vector<QVector4D> face_color;

	void BoxTrans(const float x, const float y);

	void SetPos(QPointF p);

	void BoxRot(QPointF p);

	void BoxStretch(const float x, const float y);

	bool CutModelInBox(orth::MeshModel &model_in);

	int ChoseFace(const float point_x_in, const float point_y_in, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm);

	void UpdateCutBoxObject();

	void SetOriginalColor();

	void ColorUpdate();

	void ChosePoints2(const float point_x, const float point_y, const int screen_width, const int screen_height, cv::Mat &model_matrix, cv::Mat &view_matrix, cv::Mat &projection_matrix, orth::MeshModel &mm);

	int ChoseFace(const float point_x_in, const float point_y_in, const int screen_width, const int screen_height, QMatrix4x4 model_matrix, QMatrix4x4 view_matrix, QMatrix4x4 projection_matrix);

	float v_zoom_x, v_zoom_y, v_zoom_z;
	float v_trans_x, v_trans_y, v_trans_z;

	//orth::MeshModel axis_x, axis_y, axis_z;
	//orth::MeshModel rot_x, rot_y, rot_z;
	//orth::MeshModel box_x, box_y, box_z;

	int	movement_type;
	int face_index;
	int move_direction;

	QPointF m_lastPos;

	QQuaternion m_rotation;

	orth::Point3d box_center,direct_x, direct_y,direct_z;

	orth::Point3d min_x_dir, max_x_dir,
				  min_y_dir, max_y_dir,
				  min_z_dir, max_z_dir;

	orth::MeshModel new_box;

	QMatrix4x4 end_matrix4x4;
};
SharedPtr(CCutBoxObject);
