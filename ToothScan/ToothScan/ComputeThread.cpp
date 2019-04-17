#include "ComputeThread.h"
//#include "include/PoissonRecon.h"
//#include "VTK_render.h"
#include "HLogHelper.h"

#include <pcl/visualization/cloud_viewer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

typedef void(*Dllfun2)(orth::MeshModel &mm_target, orth::MeshModel &mm_source, cv::Mat &Rt_matrix, const float MaxCorrespondenceDistance, const float RANSACOutlier, const int MaxIteration);
Dllfun2 g_icp = NULL;
HINSTANCE g_hdll = NULL;
scan::Unwarp g_unwarp;
ComputeThread::ComputeThread(QObject *parent)
	: QObject(parent)
{
	argc = 9;

	argv[0] = "111";
	argv[1] = "--in";
	argv[2] = "12.ply";
	argv[3] = "--out";
	argv[4] = "123.ply";
	argv[5] = "--depth";
	argv[6] = "9";
	argv[7] = "--bType";
	argv[8] = "1";
	hdll = LoadLibrary(L"SSDRecon.dll");
	if (hdll == NULL)
	{
		FreeLibrary(hdll);
	}
	poissonRecon = (Dllfun)GetProcAddress(hdll, "reconstruction_ssd");
	if (poissonRecon == NULL)
	{
		FreeLibrary(hdll);
	}
	if (!g_hdll) {
		g_hdll = LoadLibrary(L"regist.dll");
		//hdll = LoadLibrary("PoissonRecon.dll");
		//hdll = LoadLibrary("SSDRecon.dll");
		if (g_hdll == NULL)
		{
			FreeLibrary(g_hdll);
			return;
		}
		if (!g_icp) {
			g_icp = (Dllfun2)GetProcAddress(g_hdll, "ICPRegistration");
			//maopao1 = (Dllfun)GetProcAddress(hdll, "reconstruction");	
			//maopao1 = (Dllfun)GetProcAddress(hdll, "reconstruction_ssd");

			if (g_icp == NULL)
			{
				FreeLibrary(g_hdll);
			}
		}
	}
	camera_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
}

ComputeThread::~ComputeThread()
{
	FreeLibrary(hdll);
}

void ComputeThread::InitParameters()
{
	cout << " RasterScan Start!!" << endl;
	rs->InitRasterScan("D:/dentalimage/dentalimage2/SystemCalibration.yml");

	//rt_r = cv::Mat::eye(4, 4, CV_64FC1);
	rt_r.at<double>(0, 0) = rs->Rot_r.at<double>(0, 0);
	rt_r.at<double>(0, 1) = rs->Rot_r.at<double>(0, 1);
	rt_r.at<double>(0, 2) = rs->Rot_r.at<double>(0, 2);
	rt_r.at<double>(1, 0) = rs->Rot_r.at<double>(1, 0);
	rt_r.at<double>(1, 1) = rs->Rot_r.at<double>(1, 1);
	rt_r.at<double>(1, 2) = rs->Rot_r.at<double>(1, 2);
	rt_r.at<double>(2, 0) = rs->Rot_r.at<double>(2, 0);
	rt_r.at<double>(2, 1) = rs->Rot_r.at<double>(2, 1);
	rt_r.at<double>(2, 2) = rs->Rot_r.at<double>(2, 2);
	rt_r.at<double>(0, 3) = rs->tvec_r.at<double>(0, 0);
	rt_r.at<double>(1, 3) = rs->tvec_r.at<double>(1, 0);
	rt_r.at<double>(2, 3) = rs->tvec_r.at<double>(2, 0);

	//plane_center = cv::Mat::zeros(4, 1, CV_64FC1);//-79.2578 -23.4466 105.556
	//plane_center.at<double>(0, 0) = 50; plane_center.at<double>(1, 0) = 40; plane_center.at<double>(3, 0) = 1;
	//camera_center = cv::Mat::zeros(4, 1, CV_64FC1);
	//camera_center.at<double>(0) = -79.2578; camera_center.at<double>(1) = -23.4466; camera_center.at<double>(2) = 105.556; camera_center.at<double>(3) = 1;
	//float start_xy = sqrt(pow(-camera_center.at<double>(0) - plane_center.at<double>(0), 2) + pow(-camera_center.at<double>(1) - plane_center.at<double>(1), 2));

	cout << rt_r << endl;

	//rt_r = rt_r.inv();

	Mat pre_move = Mat::eye(4, 4, CV_64FC1);
	pre_move.at<double>(0, 0) = 1; pre_move.at<double>(0, 1) = 0; pre_move.at<double>(0, 2) = 0; pre_move.at<double>(0, 3) = -50;
	pre_move.at<double>(1, 0) = 0; pre_move.at<double>(1, 1) = 1; pre_move.at<double>(1, 2) = 0; pre_move.at<double>(1, 3) = -40;
	pre_move.at<double>(2, 0) = 0; pre_move.at<double>(2, 1) = 0; pre_move.at<double>(2, 2) = 1; pre_move.at<double>(2, 3) = 0;
	pre_move.at<double>(3, 0) = 0; pre_move.at<double>(3, 1) = 0; pre_move.at<double>(3, 2) = 0; pre_move.at<double>(3, 3) = 1;

	Mat pre_pitch = Mat::eye(4, 4, CV_64FC1);
	//double pre_alpha = ToRad(0);  // x axis
	//pre_pitch.at<double>(0, 0) = 1; pre_pitch.at<double>(0, 1) = 0; pre_pitch.at<double>(0, 2) = 0; pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = 0; pre_pitch.at<double>(1, 1) = cosf(pre_alpha); pre_pitch.at<double>(1, 2) = sinf(pre_alpha); pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = 0; pre_pitch.at<double>(2, 1) = -sinf(pre_alpha); pre_pitch.at<double>(2, 2) = cosf(pre_alpha); pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//double pre_alpha = ToRad(90);  // z axis
	//pre_pitch.at<double>(0, 0) = cosf(pre_alpha); pre_pitch.at<double>(0, 1) = 0; pre_pitch.at<double>(0, 2) = sinf(pre_alpha); pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = 0; pre_pitch.at<double>(1, 1) = 1; pre_pitch.at<double>(1, 2) = 0; pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = -sinf(pre_alpha); pre_pitch.at<double>(2, 1) = 0; pre_pitch.at<double>(2, 2) = cosf(pre_alpha); pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//double pre_alpha = ToRad(90);  // z axis
	//pre_pitch.at<double>(0, 0) = cosf(pre_alpha); pre_pitch.at<double>(0, 1) = -sinf(pre_alpha); pre_pitch.at<double>(0, 2) = 0; pre_pitch.at<double>(0, 3) = 0;
	//pre_pitch.at<double>(1, 0) = sinf(pre_alpha); pre_pitch.at<double>(1, 1) = cosf(pre_alpha); pre_pitch.at<double>(1, 2) =0; pre_pitch.at<double>(1, 3) = 0;
	//pre_pitch.at<double>(2, 0) = 0; pre_pitch.at<double>(2, 1) = 0; pre_pitch.at<double>(2, 2) = 1; pre_pitch.at<double>(2, 3) = 0;
	//pre_pitch.at<double>(3, 0) = 0; pre_pitch.at<double>(3, 1) = 0; pre_pitch.at<double>(3, 2) = 0; pre_pitch.at<double>(3, 3) = 1;

	//----------------------------------------- init scan ---------------------------------------
	cv::FileStorage fs_g("D:/dentalimage/dentalimage2/external_parameter.yml", cv::FileStorage::READ);  //read the cameras file：external_parameter.yml
	for (int image_index = 0; image_index < 9; image_index++)
	{
		stringstream ss;
		ss << image_index;
		string index_;
		ss >> index_;
		cv::Mat rt;
		fs_g["rt_mat" + index_] >> rt;
		scanner_rt[image_index] = rt;
	}
	fs_g.release();
}

void ComputeThread::setFlage(bool flag)
{
	isStop = flag;
}

bool ComputeThread::isTriangle(const cv::Vec6f &triangle, const cv::Mat &point_cloud)
{
	const float distance = 0.5;

	int u1 = triangle[0], v1 = triangle[1], u2 = triangle[2], v2 = triangle[3], u3 = triangle[4], v3 = triangle[5];
	if (u1 >= point_cloud.cols || v1 >= point_cloud.rows || u2 >= point_cloud.cols || v2 >= point_cloud.rows || u3 >= point_cloud.cols || v3 >= point_cloud.rows)
	{
		return false;
	}
	if (u1 < 0 || v1 < 0 || u2 < 0 || v2 < 0 || u3 < 0 || v3 < 0)
	{
		return false;
	}
	if ((point_cloud.at<cv::Vec3d>(v1, u1)[0] != 0) && (point_cloud.at<cv::Vec3d>(v2, u2)[0] != 0) && (point_cloud.at<cv::Vec3d>(v3, u3)[0] != 0))
	{
		float x1 = point_cloud.at<cv::Vec3d>(v1, u1)[0], y1 = point_cloud.at<cv::Vec3d>(v1, u1)[1], z1 = point_cloud.at<cv::Vec3d>(v1, u1)[2];
		float x2 = point_cloud.at<cv::Vec3d>(v2, u2)[0], y2 = point_cloud.at<cv::Vec3d>(v2, u2)[1], z2 = point_cloud.at<cv::Vec3d>(v2, u2)[2];
		float x3 = point_cloud.at<cv::Vec3d>(v3, u3)[0], y3 = point_cloud.at<cv::Vec3d>(v3, u3)[1], z3 = point_cloud.at<cv::Vec3d>(v3, u3)[2];
		float dis1 = DistanceCalculate(x1, y1, z1, x2, y2, z2);
		float dis2 = DistanceCalculate(x3, y3, z3, x2, y2, z2);
		float dis3 = DistanceCalculate(x1, y1, z1, x3, y3, z3);
		return (dis1 < distance) && (dis2 < distance) && (dis3 < distance);
	}
	else
	{
		return false;
	}
	////float dis1 = DistanceCalculate(triangle[0], triangle[1], triangle[2], triangle[3]);
	//float dis1 = DistanceCalculate(point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[0], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[1], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[2], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[0], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[1], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[2]);
	////float dis2 = DistanceCalculate(triangle[2], triangle[3], triangle[4], triangle[5]);
	//float dis2 = DistanceCalculate(point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[0], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[1], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[2], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[0], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[1], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[2]);
	////float dis3 = DistanceCalculate(triangle[4], triangle[5], triangle[0], triangle[1]);
	//float dis3 = DistanceCalculate(point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[0], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[1], point_cloud.at<cv::Vec3d>(triangle[1], triangle[0])[2], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[0], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[1], point_cloud.at<cv::Vec3d>(triangle[3], triangle[2])[2]);

	//return (dis1 < distance) && (dis2 < distance) && (dis3 < distance);
}

float ComputeThread::DistanceCalculate(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float dis = (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1);
	return dis;
}

void ComputeThread::pointcloudrotation(vector<double> &pointcloud, cv::Mat &RT)
{
	vector<double> p;
	for (int point_index = 0; point_index < pointcloud.size() / 3; point_index++)
	{
		double x = pointcloud[point_index * 3 + 0], y = pointcloud[point_index * 3 + 1], z = pointcloud[point_index * 3 + 2];
		double v_c_x = RT.at<double>(0, 0)*x + RT.at<double>(0, 1)*y + RT.at<double>(0, 2)*z + RT.at<double>(0, 3);
		double v_c_y = RT.at<double>(1, 0)*x + RT.at<double>(1, 1)*y + RT.at<double>(1, 2)*z + RT.at<double>(1, 3);
		double v_c_z = RT.at<double>(2, 0)*x + RT.at<double>(2, 1)*y + RT.at<double>(2, 2)*z + RT.at<double>(2, 3);
		p.push_back(v_c_x);
		p.push_back(v_c_y);
		p.push_back(v_c_z);
	}
	p.swap(pointcloud);
}

void ComputeThread::pointcloudrotation(orth::PointCloudD &pointCloud, orth::PointNormal &pointNormal, cv::Mat &RT)
{

	orth::PointCloudD pointCloud2;
	orth::PointCloudD pointNormal2;
	for (int point_index = 0; point_index < pointCloud.size(); point_index++)
	{
		orth::Point3d p;
		orth::Point3d n;
		double x = pointCloud[point_index].x, y = pointCloud[point_index].y, z = pointCloud[point_index].z;
		double nx = pointNormal[point_index].x, ny = pointNormal[point_index].y, nz = pointNormal[point_index].z;
		p.x = RT.at<double>(0, 0)*x + RT.at<double>(0, 1)*y + RT.at<double>(0, 2)*z + RT.at<double>(0, 3);
		p.y = RT.at<double>(1, 0)*x + RT.at<double>(1, 1)*y + RT.at<double>(1, 2)*z + RT.at<double>(1, 3);
		p.z = RT.at<double>(2, 0)*x + RT.at<double>(2, 1)*y + RT.at<double>(2, 2)*z + RT.at<double>(2, 3);

		n.x = RT.at<double>(0, 0)*nx + RT.at<double>(0, 1)*ny + RT.at<double>(0, 2)*nz;
		n.y = RT.at<double>(1, 0)*nx + RT.at<double>(1, 1)*ny + RT.at<double>(1, 2)*nz;
		n.z = RT.at<double>(2, 0)*nx + RT.at<double>(2, 1)*ny + RT.at<double>(2, 2)*nz;

		pointCloud2.push_back(p);
		pointNormal2.push_back(n);
	}
	pointCloud2.swap(pointCloud);
	pointNormal2.swap(pointNormal);
}

double ComputeThread::MatchCalculate(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud2)
{

	pcl::gpu::Octree::PointCloud cloud_device;
	cloud_device.upload(pointcloud1->points);

	pcl::gpu::Octree octree_device;
	octree_device.setCloud(cloud_device);
	octree_device.build();

	pcl::gpu::Octree::Queries queries_device;
	queries_device.upload(pointcloud2->points);

	std::vector<float> radius;
	radius.push_back(5);
	radius.push_back(5);
	radius.push_back(5);

	pcl::gpu::Octree::Radiuses radiuses_device;
	radiuses_device.upload(radius);

	const int max_answers = 2;

	// Output buffer on the device
	//pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);
	pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);

	// Do the actual search
	//octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);
	octree_device.nearestKSearchBatch(queries_device, 1, result_device);

	std::vector<int> sizes, data;
	result_device.sizes.download(sizes);
	result_device.data.download(data);

	std::cout << "INFO: Data generated" << std::endl;
	std::cout << "INFO: found : " << data.size() << " data.size" << std::endl;
	std::cout << "INFO: found : " << sizes.size() << " sizes.size" << std::endl;

	int dis_sum = 0;
	for (size_t i = 0; i < sizes.size(); ++i)
	{
		//std::cout << "INFO: sizes : " << i << " size " << sizes[i] << std::endl;
		if (sizes[i] != 0)
		{
			double dis = pcl::distances::l2Sqr(pointcloud1->points[data[i]].getVector4fMap(), pointcloud2->points[i].getVector4fMap());

			if (dis < 1)
			{
				dis_sum++;
			}

		}
	}
	std::cout << "			INFO: sum distance " << dis_sum << std::endl;
	double aa = (double)dis_sum / (pointcloud2->size());
	std::cout << "			INFO: mean distance " << aa << std::endl;
	return aa;

}

bool ComputeThread::pointcloudICP(vector<double> &cloud1, vector<double> &cloud2, int sample_number1, int sample_number2, cv::Mat &Rt)
{
	typedef pcl::PointXYZ PointType;
	typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::ConstPtr CloudConstPtr;
	typedef Cloud::Ptr CloudPtr;

	double dist = 1;
	double rans = 10;
	int iter = 200;
	bool nonLinear = true;

	std::vector<int> pcd_indices;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp;
	if (nonLinear)
	{
		//std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
		icp.reset(new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>());
	}
	else
	{
		//std::cout << "Using IterativeClosestPoint" << std::endl;
		icp.reset(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
	}

	icp->setMaximumIterations(iter);
	icp->setMaxCorrespondenceDistance(dist);
	icp->setRANSACOutlierRejectionThreshold(rans);

	pcl::registration::IncrementalRegistration<pcl::PointXYZ> iicp;
	iicp.setRegistration(icp);

	CloudPtr data(new Cloud);
	for (int i = 0; i < cloud1.size() / 3; i += sample_number1)
	{
		pcl::PointXYZ point(cloud1[i * 3 + 0], cloud1[i * 3 + 1], cloud1[i * 3 + 2]);
		data->push_back(point);
	}

	if (!iicp.registerCloud(data))
	{
		//std::cout << "Registration failed. Resetting transform" << std::endl;
		iicp.reset();
		iicp.registerCloud(data);
	};
	//cout << "data 1 = " << data->points.size() << endl;
	//pcl::io::savePLYFileBinary("111.ply", *data);
	CloudPtr data2(new Cloud);
	for (int i = 0; i < cloud2.size() / 3; i += sample_number2)
	{
		pcl::PointXYZ point(cloud2[i * 3 + 0], cloud2[i * 3 + 1], cloud2[i * 3 + 2]);
		data2->push_back(point);
	}

	//cout << "data 2 = " << data2->points.size() << endl;
	//pcl::io::savePLYFileBinary("222.ply", *data2);
	clock_t start, finish;
	double  duration;
	start = clock();

	if (!iicp.registerCloud(data2))
	{
		//std::cout << "Registration failed. Resetting transform" << std::endl;
		iicp.reset();
		iicp.registerCloud(data2);
	};

	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	std::cout << "&&& regisitration calculate time is " << duration << " second " << std::endl;

	CloudPtr tmp(new Cloud);
	pcl::transformPointCloud(*data2, *tmp, iicp.getAbsoluteTransform());
	double meandis = MatchCalculate(data, tmp);

	if (meandis < 0.5)
	{
		return false;
	}

	Eigen::Matrix4f rt = iicp.getAbsoluteTransform();
	std::cout << iicp.getAbsoluteTransform() << std::endl;
	Rt = cv::Mat::eye(4, 4, CV_64FC1);
	Rt.at<double>(0, 0) = rt(0, 0); Rt.at<double>(0, 1) = rt(0, 1); Rt.at<double>(0, 2) = rt(0, 2); Rt.at<double>(0, 3) = rt(0, 3);
	Rt.at<double>(1, 0) = rt(1, 0); Rt.at<double>(1, 1) = rt(1, 1); Rt.at<double>(1, 2) = rt(1, 2); Rt.at<double>(1, 3) = rt(1, 3);
	Rt.at<double>(2, 0) = rt(2, 0); Rt.at<double>(2, 1) = rt(2, 1); Rt.at<double>(2, 2) = rt(2, 2); Rt.at<double>(2, 3) = rt(2, 3);
	Rt.at<double>(3, 0) = rt(3, 0); Rt.at<double>(3, 1) = rt(3, 1); Rt.at<double>(3, 2) = rt(3, 2); Rt.at<double>(3, 3) = rt(3, 3);

	return true;
}

void ComputeThread::NormalCalculate(vector<double> &pointcloud, vector<float> &pointnormal, cv::Mat &rt)
{

	string abc = pcl::gpu::getDeviceName(0);
	cout << abc << endl;
	pcl::gpu::setDevice(0);
	pcl::gpu::printShortCudaDeviceInfo(0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);

	//double* asbs = (double*)(cloud_1->points.data());

	for (int point_index = 0; point_index < pointcloud.size() / 3; point_index++)
	{
		pcl::PointXYZ xyz;
		xyz.x = pointcloud[point_index * 3 + 0];
		xyz.y = pointcloud[point_index * 3 + 1];
		xyz.z = pointcloud[point_index * 3 + 2];
		cloud_1->push_back(xyz);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr normal(new pcl::PointCloud<pcl::PointXYZ>);

	cout << cloud_1->points.size() << endl;

	pcl::PointXYZ minp, maxp;
	pcl::getMinMax3D(*cloud_1, minp, maxp);
	float sz = (maxp.x - minp.x + maxp.y - minp.y + maxp.z - minp.z) / 3;
	float radius = sz / 30;


	pcl::gpu::NormalEstimation::PointCloud cloud_device;
	cloud_device.upload(cloud_1->points);

	pcl::gpu::Octree octree_;

	octree_.setCloud(cloud_device);
	octree_.build();

	pcl::gpu::NeighborIndices nn_indices_;

	octree_.radiusSearch(cloud_device, radius, 50, nn_indices_);

	pcl::gpu::NormalEstimation ne_device;
	ne_device.setInputCloud(cloud_device);
	ne_device.setRadiusSearch(radius, 50);

	pcl::gpu::NormalEstimation::Normals normals_device;

	//-4.9391095901076270e+01, -3.6521765552486102e+01,2.0971747648573330e+02
	ne_device.setViewPoint(rt.at<double>(0), rt.at<double>(1), rt.at<double>(2));
	//cout << rt_inv.at<double>(0, 3) << " ;" << rt_inv.at<double>(1, 3) << " ;" << rt_inv.at<double>(2, 3) << " ;" << endl;
	//cout << rt_inv.at<double>(0) << " ;" << rt_inv.at<double>(1) << " ;" << rt_inv.at<double>(2) << " ;" << endl;
	//ne_device.setViewPoint(-4.9391095901076270e+01, -3.6521765552486102e+01, 2.0971747648573330e+02);
	ne_device.compute(normals_device);

	vector<pcl::PointXYZ> downloaded;
	normals_device.download(downloaded);

	for (int normal_index = 0; normal_index < downloaded.size(); normal_index++)
	{
		pointnormal.push_back(downloaded[normal_index].x);
		pointnormal.push_back(downloaded[normal_index].y);
		pointnormal.push_back(downloaded[normal_index].z);
	}

	//vector<float> normal_current(downloaded.size() * 4);
	//memcpy(normal_current.data(), downloaded.data(), sizeof(pcl::PointXYZ)*downloaded.size());
}

bool ComputeThread::chooseJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex, int index)
{
	vector<double> points_2;
	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);

	vector<cv::Mat> rt_matrixs;

	orth::MeshModel mModel;
	cv::Mat rt_curr = cv::Mat::eye(4,4,CV_64FC1);
	Motor2Rot(SMX_SCAN_ROTATE_DEGREE2[index], SMY_SCAN_ROTATE_DEGREE2[index], rt_curr);
	cout << "********************************" << endl;
	cout << rt_curr << endl;
	cout << "---------------------------------" << endl;
	//double rate = matched_pixel_image.size() / 3;

	//vector<double> pointcloud;
	//for (size_t u = 0; u < matched_pixel_image.cols; u++)
	//{

	//	for (size_t v = 0; v < matched_pixel_image.rows; v++)
	//	{
	//		cv::Vec3d point = matched_pixel_image.at<cv::Vec3d>(v, u);
	//		if (point[0]!=0)
	//		{
	//			pointcloud.push_back(point[0]);
	//			pointcloud.push_back(point[1]);
	//			pointcloud.push_back(point[2]);
	//		}
	//	}
	//}
	//ColoredPoints(pointcloud, 3);

	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, rt_r, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 4000, points_2);


	if (chooseJawIndex == 1)
	{
		int scan_index = upper_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);

		stringstream ss;
		string index_write;
		ss << index;
		ss >> index_write;
		orth::ModelIO mm_write(&mModel);
		//mm_write.writeModel("./data/" + index_write + ".stl", "stlb");
		
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		//if (upper_points_cloud_globle2.size())
		if (upper_mModel.size())
		{
			//pointcloudICP(upper_points_cloud_end2, points_2, 1, 1, rt_icp);
			//if (!pointcloudICP(upper_points_cloud_end2, points_2, 1, 1, rt_icp))
			g_icp(upper_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
// 			{
// 				return false;
// 			}
		}
		//cloudrot = rt_icp;
		unwarp->MeshRot((double*)rt_icp.data, &mModel);
		upper_mModel.push_back(mModel);


		//pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		upper_points_cloud_globle2.push_back(points_2);
		upper_points_cloud_end2.insert(upper_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The ICP time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 2)
	{
		int scan_index = lower_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
// 		if (lower_points_cloud_globle2.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(lower_points_cloud_end2, points_2, 1, 1, rt_icp))
// 			{
// 				//return false;
// 			}
// 		}
		if (lower_mModel.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
//			pointcloudICP(lower_mModel[0], mModel, 1, 1, rt_icp);
			g_icp(lower_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
// 			{
// 				//return false;
// 			}
		}

		lower_mModel.push_back(mModel);

		cloudrot = rt_icp;
		unwarp->MeshRot((double*)cloudrot.data, &lower_mModel[scan_index]);
		//pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
		lower_points_cloud_globle2.push_back(points_2);
		lower_points_cloud_end2.insert(lower_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The ICP time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 3)
	{
		int scan_index = all_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
// 		if (all_points_cloud_globle2.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(all_points_cloud_end2, points_2, 1, 1, rt_icp))
// 			{
// 				//return false;
// 			}
// 		}
		if (all_mModel.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
		//	if (!pointcloudICP(all_points_cloud_end2, points_2, 1, 1, rt_icp))
			g_icp(all_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
// 			{
// 				//return false;
// 			}
		}

		all_mModel.push_back(mModel);

		cloudrot = rt_icp;
		unwarp->MeshRot((double*)cloudrot.data, &all_mModel[scan_index]);
		//pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
		all_points_cloud_globle2.push_back(points_2);
		all_points_cloud_end2.insert(all_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The ICP time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	return true;
}


bool ComputeThread::chooseJawAndIcp(cv::Mat matched_pixel_image,
	vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int index, pCScanTask pScanTask)
{
	vector<double> points_2;
	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);

	vector<cv::Mat> rt_matrixs;

	orth::MeshModel mModel;
	cv::Mat rt_curr;
	Motor2Rot(SMX_SCAN_ROTATE_DEGREE2[index], SMY_SCAN_ROTATE_DEGREE2[index], rt_curr);
	cout << "********************************" << endl;
	cout << rt_curr << endl;
	cout << "*******************************" << endl;

	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, rt_r, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 4000, points_2);

	{
		int scan_index = pScanTask->m_mModel.size();
		clock_t time1, time2;
		time1 = clock();
		cv::Mat cloudrot = rt_curr;
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(points_2, cloudrot);
// 		if (pScanTask->m_points_cloud_globle.size())
// 		{
// 			if (!pointcloudICP(pScanTask->m_points_cloud_end, points_2, 1, 1, rt_icp))
// 			{
// 				//return false;
// 			}
// 		}

		//string index_;
		//stringstream ss;
		//ss << index;
		//ss >> index_;
		//orth::ModelIO l_io_write(&mModel);
		//l_io_write.writeModel("./data/" + index_ + ".ply", "ply");
		if (pScanTask->m_mModel.size())
		{
			//if (!pointcloudICP(pScanTask->m_points_cloud_end, points_2, 1, 1, rt_icp))
			//g_icp(pScanTask->m_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
// 			{
// 				//return false;
// 			}
		}

		pScanTask->m_mModel.push_back(mModel);

		cloudrot = rt_icp;
		unwarp->MeshRot((double*)cloudrot.data, &pScanTask->m_mModel[scan_index]);
		//pointcloudrotation(pScanTask->m_mModel[scan_index].P, pScanTask->m_mModel[scan_index].N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
		pScanTask->m_points_cloud_globle.push_back(points_2);
		pScanTask->m_points_cloud_end.insert(pScanTask->m_points_cloud_end.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The ICP time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	return true;
}
/*
vector<cv::Mat> images_l, images_r;
vector<cv::Mat> image_rgb;
unsigned char* im_l = 0;
unsigned char* im_r = 0;
im_l = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
im_r = (unsigned char *)malloc(15 * 1280 * 1024 * sizeof(unsigned char));
for (int image_index = 0; image_index < 19; image_index++)
{
stringstream ss1;
string index;
ss1 << image_index;
ss1 >> index;
Mat imgl = cv::imread("./ScanPic/0_" + index + "_L.png", 0);
Mat imgr = cv::imread("./ScanPic/0_" + index + "_R.png", 0);
//Mat imgl = cv::imread("./ScanPic-3-28/0_" + index + "_L.png", 0);
//Mat imgr = cv::imread("./ScanPic-3-28/0_" + index + "_R.png", 0);

if (image_index>0 && image_index<16)
{
memcpy(im_l + (image_index - 1) * 1280 * 1024, (unsigned char*)imgl.data, 1280 * 1024 * sizeof(unsigned char));
memcpy(im_r + (image_index - 1) * 1280 * 1024, (unsigned char*)imgr.data, 1280 * 1024 * sizeof(unsigned char));
images_l.push_back(imgl);
images_r.push_back(imgr);
}

if (image_index >= 16)
{
image_rgb.push_back(imgr);
}
}
*/
// void ComputeThread::controlComputeScan(int chooseJawIndex)
// {
// 
// 	int imageSize = IMG_ROW * IMG_COL;
// 	vector<double> dis_;
// 	unsigned char* im_l = 0;
// 	unsigned char* im_r = 0;
// 	im_l = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
// 	im_r = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
// 
// 	vector<cv::Mat> image_rgb;
// 	cv::Mat imageMat;
// 	imageMat = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
// 	image_rgb.resize(3, imageMat);
// 
// 	int bufferBias = 0;
// 	scan::Unwarp *unwarp = new scan::Unwarp();
// 
// 	for (int scan_index = 0; scan_index < DataSize; scan_index++)
// 	{
// 		cv::Mat matched_pixel_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
// 		cv::Mat normal_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
// 		//usedSpace.acquire();
// 
// // 		int image_bias = 0;
// // 		memcpy(camera_image.data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
// // 		image_bias++;
// // 		for (int j = 0; j < 15; j++)
// // 		{
// // 			memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
// // 			image_bias++;
// // 
// // 			memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
// // 			image_bias++;
// // 			QString strfilename;
// // 			strfilename.sprintf("./ScanPic/%d_%d_L.png", scan_index, j);
// // 			//Mat imgl = cv::imread("./ScanPic/0_" + j + "_L.png", 0);
// // 			Mat imgl = cv::imread(strfilename.toStdString().c_str(), 0);
// // 			strfilename.sprintf("./ScanPic/%d_%d_R.png", scan_index, j);
// // 			Mat imgr = cv::imread(strfilename.toStdString().c_str(), 0);
// // 		}
// // 		if (image_bias > 30)
// // 		{
// // 			for (int i = 0; i < 3; i++)
// // 			{
// // 				memcpy(image_rgb[i].data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
// // 				image_bias++;
// // 			}
// // 		}
// 		vector<cv::Mat> images_l, images_r;
// 		for (int image_index = 0; image_index < 19; image_index++)
// 		{
// 			stringstream ss1;
// 			string index;
// 			ss1 << image_index;
// 			ss1 >> index;
// 			QString strfilename;
// 			strfilename.sprintf("D:\\dentalimage\\dentalimage2\\ScanPic\\%d_%d_L.png", scan_index, image_index);
// 			//Mat imgl = cv::imread("./ScanPic/0_" + j + "_L.png", 0);
// 			Mat imgl = cv::imread(strfilename.toStdString().c_str(), 0);
// 			strfilename.sprintf("D:\\dentalimage\\dentalimage2\\ScanPic\\%d_%d_R.png", scan_index, image_index);
// 			Mat imgr = cv::imread(strfilename.toStdString().c_str(), 0);
// 
// 			if (image_index > 0 && image_index < 16)
// 			{
// 				memcpy(im_l + (image_index - 1) * 1280 * 1024, (unsigned char*)imgl.data, 1280 * 1024 * sizeof(unsigned char));
// 				memcpy(im_r + (image_index - 1) * 1280 * 1024, (unsigned char*)imgr.data, 1280 * 1024 * sizeof(unsigned char));
// 				images_l.push_back(imgl);
// 				images_r.push_back(imgr);
// 			}
// 
// 			if (image_index >= 16)
// 			{
// 				image_rgb.push_back(imgr);
// 			}
// 		}
// 		cout << "scan_index:" << scan_index << endl;
// 		unwarp->PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1000.0);
// 
// 		bool scanFlag = chooseJawAndIcp(matched_pixel_image, image_rgb, unwarp, chooseJawIndex, scan_index);
// 		if (scanFlag == true)
// 		{
// 			if (oldJawIndex == 0)
// 			{
// 				oldJawIndex = chooseJawIndex;
// 				emit showModeltoGlSingel(1);
// 			}
// 			else if (oldJawIndex != 0)
// 			{
// 				if (oldJawIndex == chooseJawIndex)
// 				{
// 					emit showModeltoGlSingel(0);
// 				}
// 				else if (oldJawIndex != chooseJawIndex)
// 				{
// 					emit showModeltoGlSingel(1);
// 				}
// 			}
// 		}
// 		emit cameraShowSignal();
// 		bufferBias++;
// 		cout << "The ComputeThread: " << scan_index << " has finished." << endl;
// 		//freeSpace.release();
// 		if (scan_index == (DataSize - 1))
// 		{
// 			emit computeFinish();
// 		}
// 	}
// }

 void ComputeThread::controlComputeScan(int chooseJawIndex)
 {

 	int imageSize = IMG_ROW * IMG_COL;
 	vector<double> dis_;
 	unsigned char* im_l = 0;
 	unsigned char* im_r = 0;
 	im_l = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
 	im_r = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
 
 	vector<cv::Mat> image_rgb;
 	cv::Mat imageMat;
 	imageMat = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
 	image_rgb.resize(3, imageMat);
 
 	int bufferBias = 0;
 	scan::Unwarp *unwarp = new scan::Unwarp();
 
 	for (int scan_index = 0; scan_index < DataSize; scan_index++)
 	{
		cv::Mat matched_pixel_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
		cv::Mat normal_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
 		usedSpace.acquire();
 
 		int image_bias = 0;
 		memcpy(camera_image.data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
 		image_bias++;
 		for (int j = 0; j < 15; j++)
 		{
 			memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
 			image_bias++;
 
 			memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
 			image_bias++;
 		}
 		if (image_bias > 30)
 		{
 			for (int i = 0; i < 3; i++)
 			{
 				memcpy(image_rgb[i].data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
 				image_bias++;
 			}
 		}
 		cout << "scan_index:" << scan_index << endl;
 		unwarp->PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1000.0);
 
 		bool scanFlag = chooseJawAndIcp(matched_pixel_image, image_rgb, unwarp, chooseJawIndex, scan_index);
 		if (scanFlag == true)
 		{
 			if (oldJawIndex == 0)
 			{
 				oldJawIndex = chooseJawIndex;
 				emit showModeltoGlSingel(1);
 			}
 			else if (oldJawIndex != 0)
 			{
 				if (oldJawIndex == chooseJawIndex)
 				{
 					emit showModeltoGlSingel(0);
 				}
 				else if (oldJawIndex != chooseJawIndex)
 				{
 					emit showModeltoGlSingel(1);
 				}
 			}
 		}
 		emit cameraShowSignal();
 		bufferBias++;
 		cout << "The ComputeThread: " << scan_index << " has finished." << endl;
 		freeSpace.release();
 		if (scan_index == (DataSize - 1))
 		{
 			//for (int i = 0; i < 9; i++)
 			//{
 			//	string modelNameStr = std::to_string(i) + ".ply";
 			//	orth::ModelIO finish_model_io(&upper_mModel[i]);
 			//	cout << "pathname: " << modelNameStr << endl;
 			//	finish_model_io.writeModel(modelNameStr, "stl");
 			//	//writefile(upper_mModel[i], name);
 			//}
 
 			emit computeFinish();
 		}
 	}
 }

void ComputeThread::Motor2Rot(const float pitch, const float yaw, cv::Mat &Rot)
{

	Mat rt_curr = Mat::eye(4, 4, CV_64FC1);
	double alpha = ToRad(pitch);  // x axis
	rt_curr.at<double>(0, 0) = 1; rt_curr.at<double>(0, 1) = 0; rt_curr.at<double>(0, 2) = 0; rt_curr.at<double>(0, 3) = 0;
	rt_curr.at<double>(1, 0) = 0; rt_curr.at<double>(1, 1) = cosf(alpha); rt_curr.at<double>(1, 2) = -sinf(alpha); rt_curr.at<double>(1, 3) = 0;
	rt_curr.at<double>(2, 0) = 0; rt_curr.at<double>(2, 1) = sinf(alpha); rt_curr.at<double>(2, 2) = cosf(alpha); rt_curr.at<double>(2, 3) = 0;
	rt_curr.at<double>(3, 0) = 0; rt_curr.at<double>(3, 1) = 0; rt_curr.at<double>(3, 2) = 0; rt_curr.at<double>(3, 3) = 1;

	Mat rt_curr2 = Mat::eye(4, 4, CV_64FC1);
	double alpha2 = ToRad(yaw);  // y axis
	rt_curr2.at<double>(0, 0) = cosf(alpha2); rt_curr2.at<double>(0, 1) = 0; rt_curr2.at<double>(0, 2) = sinf(alpha2); rt_curr2.at<double>(0, 3) = 0;
	rt_curr2.at<double>(1, 0) = 0; rt_curr2.at<double>(1, 1) = 1; rt_curr2.at<double>(1, 2) = 0; rt_curr2.at<double>(1, 3) = 0;
	rt_curr2.at<double>(2, 0) = -sinf(alpha2); rt_curr2.at<double>(2, 1) = 0; rt_curr2.at<double>(2, 2) = cosf(alpha2); rt_curr2.at<double>(2, 3) = 0;
	rt_curr2.at<double>(3, 0) = 0; rt_curr2.at<double>(3, 1) = 0; rt_curr2.at<double>(3, 2) = 0; rt_curr2.at<double>(3, 3) = 1;

	//Mat rt_curr2 = Mat::eye(4, 4, CV_64FC1);
	//double alpha2 = ToRad(yaw);  // z axis
	//rt_curr2.at<double>(0, 0) = cosf(alpha2); rt_curr2.at<double>(0, 1) = sinf(alpha2); rt_curr2.at<double>(0, 2) = 0; rt_curr2.at<double>(0, 3) = 0;
	//rt_curr2.at<double>(1, 0) = -sinf(alpha2); rt_curr2.at<double>(1, 1) = cosf(alpha2); rt_curr2.at<double>(1, 2) = 0; rt_curr2.at<double>(1, 3) = 0;
	//rt_curr2.at<double>(2, 0) = 0; rt_curr2.at<double>(2, 1) = 0; rt_curr2.at<double>(2, 2) = 1; rt_curr2.at<double>(2, 3) = 0;
	//rt_curr2.at<double>(3, 0) = 0; rt_curr2.at<double>(3, 1) = 0; rt_curr2.at<double>(3, 2) = 0; rt_curr2.at<double>(3, 3) = 1;

	Rot = rt_curr2.inv()*rt_curr.inv();
	cout << Rot << endl;
}

bool ComputeThread::chooseCompenJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, int chooseJawIndex)
{
	vector<double> points_2;

	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);
	cv::Mat rt_curr;
	Motor2Rot(c_scan_x, c_scan_y, rt_curr);
	orth::MeshModel mModel;
	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, rt_r, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 4000, points_2);

	if (chooseJawIndex == 1)
	{
		int scan_index = upper_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//stringstream ss;
		//ss << Indexes;
		//orth::ModelIO L_io(&mModel);
		//L_io.writeModel(".stl", "stlb");

		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		//pointcloudrotation(points_2, cloudrot);
// 		if (upper_points_cloud_globle2.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(upper_points_cloud_end2, points_2, 1, 1, rt_icp))
// 			{
// 				return false;
// 			}
// 		}
		if (upper_mModel.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			//if (!pointcloudICP(upper_points_cloud_end2, points_2, 1, 1, rt_icp))
			g_icp(upper_mModel[0], mModel, rt_icp, 1.0, 10.0, 30);
			{
				//return false;
			}
		}
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		upper_mModel.push_back(mModel);
		++addUpperCompensationNum;
		cloudrot = rt_icp;
		
		//pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		upper_points_cloud_globle2.push_back(points_2);
		upper_points_cloud_end2.insert(upper_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The compensation time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 2)
	{
		int scan_index = lower_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
// 		if (lower_points_cloud_globle2.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(lower_points_cloud_end2, points_2, 1, 1, rt_icp))
// 			{
// 				//return false;
// 			}
// 		}
		if (lower_mModel.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			//if (!pointcloudICP(lower_points_cloud_end2, points_2, 1, 1, rt_icp))
			g_icp(lower_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
			{
				//return false;
			}
		}

		lower_mModel.push_back(mModel);
		++addLowerCompensationNum;
		cloudrot = rt_icp;
		unwarp->MeshRot((double*)cloudrot.data, &lower_mModel[scan_index]);
		//pointcloudrotation(lower_mModel[scan_index].P, lower_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		lower_points_cloud_globle2.push_back(points_2);
		lower_points_cloud_end2.insert(lower_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The compensation time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 3)
	{
		int scan_index = all_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		unwarp->MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
// 		if (all_points_cloud_globle2.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(all_points_cloud_end2, points_2, 1, 1, rt_icp))
// 			{
// 				//return false;
// 			}
// 		}
		if (all_mModel.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			//if (!pointcloudICP(all_points_cloud_end2, points_2, 1, 1, rt_icp))
			g_icp(all_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
			{
				//return false;
			}
		}

		all_mModel.push_back(mModel);
		++addAllCompensationNum;
		cloudrot = rt_icp;
		unwarp->MeshRot((double*)cloudrot.data, &all_mModel[scan_index]);
		//pointcloudrotation(all_mModel[scan_index].P, all_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		all_points_cloud_globle2.push_back(points_2);
		all_points_cloud_end2.insert(all_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The compensation time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	return true;
}

bool ComputeThread::chooseCompenJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, scan::Unwarp *unwarp, pCScanTask pScanTask)
{
	vector<double> points_2;

	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);
	cv::Mat rt_curr;
	Motor2Rot(c_scan_x, c_scan_y, rt_curr);
	orth::MeshModel mModel;
	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, rt_r, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 4000, points_2);

	//	if (chooseJawIndex == 1)
	{
		int scan_index = pScanTask->m_mModel.size();
		clock_t time1, time2;
		time1 = clock();

		cv::Mat cloudrot = rt_curr;
		g_unwarp.MeshRot((double*)cloudrot.data, &mModel);
		//pointcloudrotation(mModel.P, mModel.N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
// 		if (pScanTask->m_points_cloud_globle.size())
// 		{
// 			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 			if (!pointcloudICP(pScanTask->m_points_cloud_end, points_2, 1, 1, rt_icp))
// 			{
// 				return false;
// 			}
// 		}
		if (pScanTask->m_mModel.size())
		{
			//if (!pointcloudICP(pScanTask->m_points_cloud_end, points_2, 1, 1, rt_icp))
			g_icp(pScanTask->m_mModel[0], mModel, rt_icp, 1.0, 10.0, 50);
			// 			{
			// 				//return false;
			// 			}
		}

		pScanTask->m_mModel.push_back(mModel);
		pScanTask->m_nAddModel++;
		cloudrot = rt_icp;
		g_unwarp.MeshRot((double*)cloudrot.data, &pScanTask->m_mModel[scan_index]);
		//pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		pScanTask->m_points_cloud_globle.push_back(points_2);
		pScanTask->m_points_cloud_end.insert(pScanTask->m_points_cloud_end.end(), points_2.begin(), points_2.end());
		pScanTask->m_points_cloud_end_addSize.push_back(points_2.size());
		time2 = clock();
		cout << "The compensation time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	return true;
}

void ComputeThread::compensationComputeScan(int chooseJawIndex)
{
	if (oldJawIndex == 0)
	{
		oldJawIndex = chooseJawIndex;
	}
	cv::Mat matched_pixel_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	cv::Mat normal_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	int imageSize = IMG_ROW * IMG_COL;
	vector<double> dis_;
	unsigned char* im_l = 0;
	unsigned char* im_r = 0;
	im_l = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
	im_r = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));

	vector<cv::Mat> image_rgb;
	cv::Mat imageMat;
	imageMat = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
	image_rgb.resize(3, imageMat);

	usedSpace.acquire();
	scan::Unwarp *unwarp = new scan::Unwarp();
	int image_bias = 0;
	memcpy(camera_image.data, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
	image_bias++;
	for (int j = 0; j < 15; j++)
	{
		memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;

		memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;
	}
	if (image_bias > 30)
	{
		for (int i = 0; i < 3; i++)
		{
			memcpy(image_rgb[i].data, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		}
	}

	unwarp->PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1000.0);

	/*vector<double> points_;
	vector<float> normal;
	vector<unsigned char> points_color;*/
	vector<double> points_2;
	/*vector<unsigned char> points_color2;
	vector<uint32_t> faces;*/

	bool scanFlag = chooseCompenJawAndIcp(matched_pixel_image, image_rgb, unwarp, chooseJawIndex);

	if (scanFlag == true)
	{
		if (oldJawIndex != 0)
		{
			if (oldJawIndex == chooseJawIndex)
			{
				emit showModeltoGlSingel(0);
			}
			else if (oldJawIndex != chooseJawIndex)
			{
				emit showModeltoGlSingel(1);
			}
		}
	}

	freeSpace.release();
	cout << "补扫一个角度图片计算完成" << endl;

	emit cameraShowSignal();
	emit computeFinish();
}

void ComputeThread::pointcloudrotationandtotalmesh(orth::PointCloudD &pointCloud, orth::PointNormal &pointNormal, cv::Mat &RT, orth::MeshModel &totalMeshModel)
{
	orth::PointCloudD pointCloud2;
	orth::PointCloudD pointNormal2;
	for (int point_index = 0; point_index < pointCloud.size(); point_index++)
	{
		orth::Point3d p;
		orth::Point3d n;
		double x = pointCloud[point_index].x, y = pointCloud[point_index].y, z = pointCloud[point_index].z;
		double nx = pointNormal[point_index].x, ny = pointNormal[point_index].y, nz = pointNormal[point_index].z;

		p.x = RT.at<double>(0, 0)*x + RT.at<double>(0, 1)*y + RT.at<double>(0, 2)*z + RT.at<double>(0, 3);
		p.y = RT.at<double>(1, 0)*x + RT.at<double>(1, 1)*y + RT.at<double>(1, 2)*z + RT.at<double>(1, 3);
		p.z = RT.at<double>(2, 0)*x + RT.at<double>(2, 1)*y + RT.at<double>(2, 2)*z + RT.at<double>(2, 3);

		n.x = RT.at<double>(0, 0)*nx + RT.at<double>(0, 1)*ny + RT.at<double>(0, 2)*nz + RT.at<double>(0, 3);
		n.y = RT.at<double>(1, 0)*nx + RT.at<double>(1, 1)*ny + RT.at<double>(1, 2)*nz + RT.at<double>(1, 3);
		n.z = RT.at<double>(2, 0)*nx + RT.at<double>(2, 1)*ny + RT.at<double>(2, 2)*nz + RT.at<double>(2, 3);

		pointCloud2.push_back(p);
		pointNormal2.push_back(n);
		totalMeshModel.P.push_back(p);
		totalMeshModel.N.push_back(n);
		//totalMeshModel.L.push_back(0);
	}
	pointCloud2.swap(pointCloud);
	pointNormal2.swap(pointNormal);
}

void ComputeThread::writefile(orth::MeshModel totalMeshModel, string name)
{
	pcl::PointXYZLNormal p;
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZLNormal>);
	for (int j = 0; j < totalMeshModel.P.size(); j++)
	{
		p.x = totalMeshModel.P[j].x;
		p.y = totalMeshModel.P[j].y;
		p.z = totalMeshModel.P[j].z;
		p.normal_x = totalMeshModel.N[j].x;
		p.normal_y = totalMeshModel.N[j].y;
		p.normal_z = totalMeshModel.N[j].z;
		outCloud->points.push_back(p);
	}
	pcl::io::savePLYFile(name, *outCloud); //将点云保存到PLY文件中
	//pcl::io::savePLYFileBinary(name, *outCloud); //将点云保存到PLY文件中
	cerr << "Saved " << outCloud->points.size() << " data points to totalMeshModel.ply." << endl;
}

void ComputeThread::writefile(vector<double> cloud, string name)
{
	pcl::PointXYZLNormal p;
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZLNormal>);
	for (int j = 0; j < cloud.size(); j++)
	{
		p.x = cloud[j];
		++j;
		p.y = cloud[j];
		++j;
		p.z = cloud[j];
		outCloud->points.push_back(p);
	}
	pcl::io::savePLYFile(name, *outCloud); //将点云保存到PLY文件中
										   //pcl::io::savePLYFileBinary(name, *outCloud); //将点云保存到PLY文件中
	cerr << "Saved " << outCloud->points.size() << " data points to cloud.ply." << endl;
}

void ComputeThread::GPAMeshing(int chooseJawIndex)
{

	vector<vector<double>> points_target;
	int TotalIterNum = 82;
	if (chooseJawIndex == 1)
	{
		if (upper_mModel.size() > 1)
		{
			vector<cv::Mat> rt_matrixs;
			clock_t time1, time2, time3, time4;
			time1 = clock();
			gpa.GpaRegistrationGPU(upper_mModel, rt_matrixs, TotalIterNum);
			time2 = clock();
			cout << "The GPU time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "GPA is finished..." << endl;



			orth::MeshModel totalMeshModel;
			for (int data_index = 0; data_index < upper_points_cloud_globle2.size(); data_index++)
			{
				pointcloudrotationandtotalmesh(upper_mModel[data_index].P, upper_mModel[data_index].N, rt_matrixs[data_index], totalMeshModel);
			}

			/*for (int i = 0; i < points_target.size(); i++)
			{
				string name1 = std::to_string(i) + "_rotation.ply";
				writefile(points_target[i], name1);

				string name2 = std::to_string(i) + "_totalpoint.ply";
				writefile(upper_mModel[i], name2);
			}*/
			/*string name = "totalMeshModel.ply";
			cout << name << endl;
			writefile(totalMeshModel, name);*/

			/*string modelNameStr = "totalMeshModel.stl";
			orth::ModelIO finish_model_io(&totalMeshModel);
			cout << "pathname: " << modelNameStr << endl;
			finish_model_io.writeModel(modelNameStr, "stlb");*/

			time3 = clock();
			//pr.Execute(totalMeshModel);
			poissonRecon(argc, argv, &totalMeshModel);
			time4 = clock();
			upper_mModel.push_back(totalMeshModel);
			cout << "The reconstruction is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "reconstruction is finished..." << endl;
		}
		else
		{
			clock_t time1, time2;
			time1 = clock();
			//recon::PoissonRec pr;
			//pr.Execute(upper_mModel[0]);
			orth::MeshModel totalMeshModel;
			totalMeshModel = upper_mModel[0];
			poissonRecon(argc, argv, &totalMeshModel);
			upper_mModel.push_back(totalMeshModel);
			time2 = clock();
			cout << "The Poisson time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		}
	}
	else if (chooseJawIndex == 2)
	{
		if (lower_mModel.size() > 1)
		{
			vector<cv::Mat> rt_matrixs;
			clock_t time1, time2, time3, time4;
			time1 = clock();
			gpa.GpaRegistrationGPU(lower_mModel, rt_matrixs, TotalIterNum);
			time2 = clock();
			cout << "The GPU time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "GPA is finished..." << endl;

			orth::MeshModel totalMeshModel;
			for (int data_index = 0; data_index < lower_points_cloud_globle2.size(); data_index++)
			{
				pointcloudrotationandtotalmesh(lower_mModel[data_index].P, lower_mModel[data_index].N, rt_matrixs[data_index], totalMeshModel);
			}
			//cout << "totalMeshModel: " << totalMeshModel.P.size() << endl;
			//writefile(totalMeshModel);
			//recon::PoissonRec pr;
			time3 = clock();
			//pr.Execute(totalMeshModel);
			poissonRecon(argc, argv, &totalMeshModel);
			lower_mModel.push_back(totalMeshModel);
			time4 = clock();
			cout << "The reconstruction is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "reconstruction is finished..." << endl;
		}
		else
		{
			clock_t time1, time2;
			time1 = clock();
			//recon::PoissonRec pr;
			orth::MeshModel totalMeshModel;
			totalMeshModel = lower_mModel[0];
			poissonRecon(argc, argv, &totalMeshModel);
			lower_mModel.push_back(totalMeshModel);
			//pr.Execute(lower_mModel[0]);
			time2 = clock();
			cout << "The Poisson time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		}
	}
	else if (chooseJawIndex == 3)
	{
		if (all_mModel.size() > 1)
		{
			vector<cv::Mat> rt_matrixs;
			clock_t time1, time2, time3, time4;
			time1 = clock();
			gpa.GpaRegistrationGPU(all_mModel, rt_matrixs, TotalIterNum);
			time2 = clock();
			cout << "The GPU time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "GPA is finished..." << endl;

			orth::MeshModel totalMeshModel;
			for (int data_index = 0; data_index < all_points_cloud_globle2.size(); data_index++)
			{
				pointcloudrotationandtotalmesh(all_mModel[data_index].P, all_mModel[data_index].N, rt_matrixs[data_index], totalMeshModel);
			}
			//cout << "totalMeshModel: " << totalMeshModel.P.size() << endl;
			//writefile(totalMeshModel);
			//recon::PoissonRec pr;

			time3 = clock();
			poissonRecon(argc, argv, &totalMeshModel);
			all_mModel.push_back(totalMeshModel);
			time4 = clock();
			cout << "The reconstruction is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "reconstruction is finished..." << endl;
		}
		else
		{
			clock_t time1, time2;
			time1 = clock();
			//recon::PoissonRec pr;
			orth::MeshModel totalMeshModel;
			totalMeshModel = all_mModel[0];
			poissonRecon(argc, argv, &totalMeshModel);
			all_mModel.push_back(totalMeshModel);
			time2 = clock();
			cout << "The Poisson time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		}
	}
	emit showModeltoGlSingel(2);
	emit computeFinish();
}

void ComputeThread::GPAMeshing()
{
	pCScanTask pScanTask;
	pScanTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pScanTask)
		return;
	//vector<vector<double>> points_target;
	int TotalIterNum = 82;
	{
		if (pScanTask->m_mModel.size() > 1)
		{
			vector<cv::Mat> rt_matrixs;
			clock_t time1, time2, time3, time4;
			time1 = clock();
			gpa.GpaRegistrationGPU(pScanTask->m_mModel, rt_matrixs, TotalIterNum);
			time2 = clock();
			cout << "The GPU time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "GPA is finished..." << endl;

			orth::MeshModel totalMeshModel;
			for (int data_index = 0; data_index < pScanTask->m_points_cloud_globle.size(); data_index++)
			{
				pointcloudrotationandtotalmesh(pScanTask->m_mModel[data_index].P, pScanTask->m_mModel[data_index].N, rt_matrixs[data_index], totalMeshModel);
			}
			time3 = clock();
			poissonRecon(argc, argv, &totalMeshModel);
			time4 = clock();
			pScanTask->m_mAllModel = totalMeshModel;
			cout << "The reconstruction is " << (double)(time4 - time3) / CLOCKS_PER_SEC << " s;" << endl;
			cout << "reconstruction is finished..." << endl;
		}
		else
		{
			clock_t time1, time2;
			time1 = clock();
			orth::MeshModel totalMeshModel;
			totalMeshModel = pScanTask->m_mModel[0];
			poissonRecon(argc, argv, &totalMeshModel);
			pScanTask->m_mAllModel = totalMeshModel;
			time2 = clock();
			cout << "The Poisson time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s;" << endl;
		}
	}
	//emit showModeltoGlSingel(2);
	//emit computeFinish();
	emit meshFinish();
}
//牙齿配准

void ComputeThread::taskTeethSititSignal()
{
	pCScanTask pScanTask;
	pScanTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pScanTask)
		return;
	pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pScanTask);
	if (!pTask)
		return;
	pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
	//1分割srcmodel pCTeethModel pTeethModel;
	orth::MeshModel l_tmpModel, l_dstModel;
	pSrcTask->pTeethModel->getMeshModel(l_tmpModel);
	pDstTask->pTeethModel->getMeshModel(l_dstModel);
	vector<orth::MeshModel> l_vtModel;
	l_tmpModel.ModelSplit(l_vtModel);
	//vector<orth::MeshModel>::iterator iter = l_vtModel.begin();
	//for (; iter != l_vtModel.end(); iter++) {
	for (int i = 0; i < l_vtModel.size();i++) {

		cv::Mat rt_out;
		g_icp(l_dstModel,l_vtModel[i] , rt_out, 1.0, 10.0, 50);
		g_unwarp.MeshRot((double*)rt_out.data, &l_vtModel[i]);
	}
	l_vtModel.push_back(l_dstModel);
	orth::MeshModel dstAllModel;
	orth::MergeModels(l_vtModel,dstAllModel);
	pTask->pTeethModel->m_model = dstAllModel;
	pTask->pTeethModel->makeObject();
	//emit showModeltoGlSingel(2);
	//emit computeFinish();
	emit meshFinish();
}

void ComputeThread::Stitching()
{
	pCScanTask pScanTask;
	pScanTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pScanTask)
		return;
	pCStitchingTask pTask = std::static_pointer_cast<CStitchingTask>(pScanTask);
	if (!pTask)
		return;
	pCScanTask pDstTask = pTask->m_pDstTask, pSrcTask = pTask->m_pSrcTask;
	if (!pDstTask || !pSrcTask)
		return;
	if (pDstTask->m_points_cloud_globle.size() && pSrcTask->m_points_cloud_globle.size())
	{
		cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);
		//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
// 		if (!pointcloudICP(pSrcTask->m_points_cloud_end, pDstTask->m_points_cloud_end, 1, 1, rt_icp))
// 		{
// 			return;
// 		}
		orth::MeshModel meshModel;
		pDstTask->pTeethModel->getMeshModel(meshModel);
		g_icp(pSrcTask->m_mAllModel, meshModel, rt_icp, 1.0, 10.0, 50);

		g_unwarp.MeshRot((double*)rt_icp.data, &meshModel);
	}

	//emit showModeltoGlSingel(2);
	//emit computeFinish();
	emit StitchFinish();
}

void ComputeThread::normalComputeScan()
{
	pCScanTask pScanTask;
	pScanTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pScanTask)
		return;
	cv::Mat matched_pixel_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	cv::Mat normal_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	int imageSize = IMG_ROW * IMG_COL;
	vector<double> dis_;
	unsigned char* im_l = 0;
	unsigned char* im_r = 0;
	im_l = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
	im_r = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));

	vector<cv::Mat> image_rgb;
	cv::Mat imageMat;
	imageMat = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
	image_rgb.resize(3, imageMat);

	int bufferBias = 0;
	//scan::Unwarp *unwarp = new scan::Unwarp();

	for (int scan_index = 0; scan_index < DataSize; scan_index++)
	{
		usedSpace.acquire();

		int image_bias = 0;
		memcpy(camera_image.data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;
		for (int j = 0; j < 15; j++)
		{
			memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;

			memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		}
		if (image_bias > 30)
		{
			for (int i = 0; i < 3; i++)
			{
				memcpy(image_rgb[i].data, totalNormalScanImageBuffer + bufferBias * 34 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
				image_bias++;
			}
		}
		cout << "scan_index:" << scan_index << endl;
		HLogHelper::getInstance()->HLogTime("scan_index %d", scan_index);
		g_unwarp.PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1.0);
		HLogHelper::getInstance()->HLogTime("PointCloudCalculateCuda2 finish");
		bool scanFlag = chooseJawAndIcp(matched_pixel_image, image_rgb, &g_unwarp, scan_index, pScanTask);
		HLogHelper::getInstance()->HLogTime("chooseJawAndIcp finish");
		emit showTaskModel();
		emit cameraShowSignal();
		bufferBias++;
		cout << "The ComputeThread: " << scan_index << " has finished." << endl;
		HLogHelper::getInstance()->HLogTime("The ComputeThread: %d has finished", scan_index);
		freeSpace.release();
		if (scan_index == (DataSize - 1))
		{
			//for (int i = 0; i < 9; i++)
			//{
			//	string modelNameStr = std::to_string(i) + ".ply";
			//	orth::ModelIO finish_model_io(&upper_mModel[i]);
			//	cout << "pathname: " << modelNameStr << endl;
			//	finish_model_io.writeModel(modelNameStr, "stl");
			//	//writefile(upper_mModel[i], name);
			//}

			emit computeFinish();
		}
	}
}

void ComputeThread::compensationCompute()
{
	pCScanTask pScanTask;
	pScanTask = CTaskManager::getInstance()->getCurrentTask();
	if (!pScanTask)
		return;
	cv::Mat matched_pixel_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	cv::Mat normal_image = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_64FC3);
	int imageSize = IMG_ROW * IMG_COL;
	vector<double> dis_;
	unsigned char* im_l = 0;
	unsigned char* im_r = 0;
	im_l = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));
	im_r = (unsigned char *)malloc(15 * imageSize * sizeof(unsigned char));

	vector<cv::Mat> image_rgb;
	cv::Mat imageMat;
	imageMat = cv::Mat::zeros(IMG_ROW, IMG_COL, CV_8UC1);
	image_rgb.resize(3, imageMat);

	usedSpace.acquire();
	//scan::Unwarp *unwarp = new scan::Unwarp();
	int image_bias = 0;
	memcpy(camera_image.data, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
	image_bias++;
	for (int j = 0; j < 15; j++)
	{
		memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;

		memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;
	}
	if (image_bias > 30)
	{
		for (int i = 0; i < 3; i++)
		{
			memcpy(image_rgb[i].data, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		}
	}

	g_unwarp.PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1.0);

	/*vector<double> points_;
	vector<float> normal;
	vector<unsigned char> points_color;*/
	vector<double> points_2;
	/*vector<unsigned char> points_color2;
	vector<uint32_t> faces;*/

	bool scanFlag = chooseCompenJawAndIcp(matched_pixel_image, image_rgb, &g_unwarp, pScanTask);

	// 	if (scanFlag == true)
	// 	{
	// 		if (oldJawIndex != 0)
	// 		{
	// 			if (oldJawIndex == chooseJawIndex)
	// 			{
	// 				emit showModeltoGlSingel(0);
	// 			}
	// 			else if (oldJawIndex != chooseJawIndex)
	// 			{
	// 				emit showModeltoGlSingel(1);
	// 			}
	// 		}
	// 	}
	emit showTaskModel();
	freeSpace.release();
	cout << "补扫一个角度图片计算完成" << endl;

	emit cameraShowSignal();
	emit computeFinish();
}

