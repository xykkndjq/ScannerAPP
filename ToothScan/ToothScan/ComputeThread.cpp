#include "ComputeThread.h"

ComputeThread::ComputeThread(QObject *parent)
	: QObject(parent)
{
	
}

ComputeThread::~ComputeThread()
{
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

	rt_r = rt_r.inv();

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
		double x = pointCloud[point_index].x, y = pointCloud[point_index].y, z = pointCloud[point_index ].z;
		double nx = pointCloud[point_index].x, ny = pointCloud[point_index].y, nz = pointCloud[point_index].z;
		p.x = RT.at<double>(0, 0)*x + RT.at<double>(0, 1)*y + RT.at<double>(0, 2)*z + RT.at<double>(0, 3);
		p.y = RT.at<double>(1, 0)*x + RT.at<double>(1, 1)*y + RT.at<double>(1, 2)*z + RT.at<double>(1, 3);
		p.z = RT.at<double>(2, 0)*x + RT.at<double>(2, 1)*y + RT.at<double>(2, 2)*z + RT.at<double>(2, 3);

		n.x = RT.at<double>(0, 0)*nx + RT.at<double>(0, 1)*ny + RT.at<double>(0, 2)*nz + RT.at<double>(0, 3);
		n.y = RT.at<double>(1, 0)*nx + RT.at<double>(1, 1)*ny + RT.at<double>(1, 2)*nz + RT.at<double>(1, 3);
		n.z = RT.at<double>(2, 0)*nx + RT.at<double>(2, 1)*ny + RT.at<double>(2, 2)*nz + RT.at<double>(2, 3);

		pointCloud2.push_back(p);
		pointNormal2.push_back(n);
	}
	pointCloud2.swap(pointCloud);
	pointNormal2.swap(pointNormal);
}

//void ComputeThread::pointcloudrotationandtotalmesh(orth::PointCloudD &pointCloud, orth::PointNormal &pointNormal, cv::Mat &RT, orth::MeshModel &totalMeshModel)
//{
//	orth::PointCloudD pointCloud2;
//	orth::PointCloudD pointNormal2;
//	for (int point_index = 0; point_index < pointCloud.size(); point_index++)
//	{
//		orth::Point3d p;
//		orth::Point3d n;
//		double x = pointCloud[point_index].x, y = pointCloud[point_index].y, z = pointCloud[point_index].z;
//		double nx = pointCloud[point_index].x, ny = pointCloud[point_index].y, nz = pointCloud[point_index].z;
//		p.x = RT.at<double>(0, 0)*x + RT.at<double>(0, 1)*y + RT.at<double>(0, 2)*z + RT.at<double>(0, 3);
//		p.y = RT.at<double>(1, 0)*x + RT.at<double>(1, 1)*y + RT.at<double>(1, 2)*z + RT.at<double>(1, 3);
//		p.z = RT.at<double>(2, 0)*x + RT.at<double>(2, 1)*y + RT.at<double>(2, 2)*z + RT.at<double>(2, 3);
//
//		n.x = RT.at<double>(0, 0)*nx + RT.at<double>(0, 1)*ny + RT.at<double>(0, 2)*nz + RT.at<double>(0, 3);
//		n.y = RT.at<double>(1, 0)*nx + RT.at<double>(1, 1)*ny + RT.at<double>(1, 2)*nz + RT.at<double>(1, 3);
//		n.z = RT.at<double>(2, 0)*nx + RT.at<double>(2, 1)*ny + RT.at<double>(2, 2)*nz + RT.at<double>(2, 3);
//
//		pointCloud2.push_back(p);
//		pointNormal2.push_back(n);
//	}
//	pointCloud2.swap(pointCloud);
//	pointNormal2.swap(pointNormal);
//}

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

			if (dis<1)
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
	//std::cout << "&&& regisitration calculate time is " << duration << " second " << std::endl;

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

void ComputeThread::delaunayAlgorithm(const cv::Mat &point_cloud, const cv::Mat &normal_cloud, vector<cv::Mat> &image_rgb, float color_red_parameter, float color_green_parameter, float color_blue_parameter, vector<double>& points,vector<float>& normal, vector<double>& points2,  vector<unsigned char>& points_color, vector<uint32_t> & faces)
{
	if (point_cloud.empty())
	{
		return;
	}

	clock_t start, time1, time2, time3, finish;
	double  duration1, duration2, duration3, duration4, duration5;

	const cv::Rect pageRc(0, 0, point_cloud.cols, point_cloud.rows);

	vector<cv::Vec6f> _temp_result;
	cv::Subdiv2D subdiv2d(pageRc);
	cv::Mat point_flag = cv::Mat::ones(point_cloud.size(), CV_64FC1);
	point_flag *= -1;
	start = clock();
	for (int u = 0; u < point_cloud.cols; u++)
	{
		for (int v = 0; v < point_cloud.rows; v++)
		{
			if (point_cloud.at<cv::Vec3d>(v, u)[0] != 0)
			{
				subdiv2d.insert(cv::Point2f(u, v));
			}
		}
	}

	time1 = clock();

	subdiv2d.getTriangleList(_temp_result);

	time2 = clock();

	int index_ = 0;
	for (const auto _tmp_vec : _temp_result)
	{
		if (isTriangle(_tmp_vec, point_cloud))
		{
			int face1 = -1, face2 = -1, face3 = -1;
			{
				int u = (int)_tmp_vec[0], v = (int)_tmp_vec[1];
				if ((int)(point_flag.at<double>(v, u)) == -1)
				{
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[0]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[1]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[2]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[0]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[1]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[2]);
					int red = image_rgb[0].at<unsigned char>(v, u)*color_red_parameter;
					int green = image_rgb[1].at<unsigned char>(v, u)*color_green_parameter;
					int blue = image_rgb[2].at<unsigned char>(v, u)*color_blue_parameter;
					points_color.push_back((unsigned char)(red < 255 ? red : 255));
					points_color.push_back((unsigned char)(green < 255 ? green : 255));
					points_color.push_back((unsigned char)(blue < 255 ? blue : 255));
					point_flag.at<double>(v, u) = index_;
					face1 = index_;
					index_++;
				}
				else
				{
					face1 = (int)(point_flag.at<double>(v, u));
				}
			}
			{
				int u = (int)_tmp_vec[2], v = (int)_tmp_vec[3];
				if ((int)(point_flag.at<double>(v, u)) == -1)
				{
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[0]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[1]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[2]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[0]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[1]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[2]);
					int red = image_rgb[0].at<unsigned char>(v, u)*color_red_parameter;
					int green = image_rgb[1].at<unsigned char>(v, u)*color_green_parameter;
					int blue = image_rgb[2].at<unsigned char>(v, u)*color_blue_parameter;
					points_color.push_back((unsigned char)(red < 255 ? red : 255));
					points_color.push_back((unsigned char)(green < 255 ? green : 255));
					points_color.push_back((unsigned char)(blue < 255 ? blue : 255));
					point_flag.at<double>(v, u) = index_;
					face2 = index_;
					index_++;
				}
				else
				{
					face2 = (int)(point_flag.at<double>(v, u));
				}
			}
			{
				int u = (int)_tmp_vec[4], v = (int)_tmp_vec[5];
				if ((int)(point_flag.at<double>(v, u)) == -1)
				{
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[0]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[1]);
					points.push_back(point_cloud.at<cv::Vec3d>(v, u)[2]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[0]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[1]);
					normal.push_back(normal_cloud.at<cv::Vec3d>(v, u)[2]);
					int red = image_rgb[0].at<unsigned char>(v, u)*color_red_parameter;
					int green = image_rgb[1].at<unsigned char>(v, u)*color_green_parameter;
					int blue = image_rgb[2].at<unsigned char>(v, u)*color_blue_parameter;
					points_color.push_back((unsigned char)(red < 255 ? red : 255));
					points_color.push_back((unsigned char)(green < 255 ? green : 255));
					points_color.push_back((unsigned char)(blue < 255 ? blue : 255));
					point_flag.at<double>(v, u) = index_;
					face3 = index_;
					index_++;
				}
				else
				{
					face3 = (int)(point_flag.at<double>(v, u));
				}
			}
			faces.push_back(face1);
			faces.push_back(face2);
			faces.push_back(face3);
		}

	}

	time3 = clock();

	for (int point_index = 0; point_index < points.size() / 3; point_index += 30)
	{
		points2.push_back(points[point_index * 3 + 0]);
		points2.push_back(points[point_index * 3 + 1]);
		points2.push_back(points[point_index * 3 + 2]);
	}

	finish = clock();
	duration1 = (double)(time1 - start) / CLOCKS_PER_SEC;
	duration2 = (double)(time2 - time1) / CLOCKS_PER_SEC;
	duration3 = (double)(time3 - time2) / CLOCKS_PER_SEC;
	duration4 = (double)(finish - time3) / CLOCKS_PER_SEC;
	duration5 = (double)(finish - start) / CLOCKS_PER_SEC;
	std::cout << "&&&calculate time mesh1 " << duration1 << "  mesh2 " << duration2 << "  mesh3 " << duration3 << "  mesh4 " << duration4 << "   " << duration5 << " second " << std::endl;

	cout << " sampled number " << points2.size() << " !!" << endl;

	//for (int u = 0; u < point_cloud.cols; u++)
	//{
	//	for (int v = 0; v < point_cloud.rows; v++)
	//	{
	//		if (point_cloud.at<cv::Vec3d>(v, u)[0] != 0)
	//		{
	//			points.push_back(point_cloud.at<cv::Vec3d>(v, u)[0]);
	//			points.push_back(point_cloud.at<cv::Vec3d>(v, u)[1]);
	//			points.push_back(point_cloud.at<cv::Vec3d>(v, u)[2]);
	//			int red = image_rgb[0].at<unsigned char>(v, u)*color_red_parameter;
	//			int green = image_rgb[1].at<unsigned char>(v, u)*color_green_parameter;
	//			int blue = image_rgb[2].at<unsigned char>(v, u)*color_blue_parameter;
	//			points_color.push_back((unsigned char)(red < 255 ? red : 255));
	//			points_color.push_back((unsigned char)(green < 255 ? green : 255));
	//			points_color.push_back((unsigned char)(blue < 255 ? blue : 255));
	//			subdiv2d.insert(cv::Point2f((float)u, (float)v));
	//			if (u % 4 == 0 && v % 4 == 0)
	//			{
	//				points2.push_back(point_cloud.at<cv::Vec3d>(v, u)[0]);
	//				points2.push_back(point_cloud.at<cv::Vec3d>(v, u)[1]);
	//				points2.push_back(point_cloud.at<cv::Vec3d>(v, u)[2]);
	//			}
	//		}
	//	}
	//}
	//return result;
}

void ComputeThread::chooseJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, int chooseJawIndex)
{
	vector<double> points_2;
	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);

	orth::MeshModel mModel;
	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 0.02, points_2);
	
	if (chooseJawIndex == 1)
	{
		int scan_index = upper_mModel.size();
		clock_t time1, time2;
		time1 = clock();
		upper_mModel.push_back(mModel);
		cv::Mat cloudrot = scanner_rt[scan_index];
		pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		if (upper_points_cloud_globle2.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			if (!pointcloudICP(upper_points_cloud_globle2[0], points_2, 1, 1, rt_icp))
			{
				//return;
			}
		}
		cloudrot = rt_icp;
		pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		upper_points_cloud_globle2.push_back(points_2);
		upper_points_cloud_end2.insert(upper_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 2)
	{
		int scan_index = lower_mModel.size();
		clock_t time1, time2;
		time1 = clock();
		lower_mModel.push_back(mModel);
		cv::Mat cloudrot = scanner_rt[scan_index];
		pointcloudrotation(lower_mModel[scan_index].P, lower_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		if (lower_points_cloud_globle2.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			if (!pointcloudICP(lower_points_cloud_globle2[0], points_2, 1, 1, rt_icp))
			{
				//return;
			}
		}
		cloudrot = rt_icp;
		pointcloudrotation(lower_mModel[scan_index].P, lower_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		lower_points_cloud_globle2.push_back(points_2);
		lower_points_cloud_end2.insert(lower_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
	else if (chooseJawIndex == 3)
	{
		int scan_index = all_mModel.size();
		clock_t time1, time2;
		time1 = clock();
		all_mModel.push_back(mModel);
		cv::Mat cloudrot = scanner_rt[scan_index];
		pointcloudrotation(all_mModel[scan_index].P, all_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		if (all_points_cloud_globle2.size())
		{
			//pointcloudICP(points_cloud_globle2[0], points_2, 1, 1, rt_icp);
			if (!pointcloudICP(all_points_cloud_globle2[0], points_2, 1, 1, rt_icp))
			{
				//return;
			}
		}
		cloudrot = rt_icp;
		pointcloudrotation(all_mModel[scan_index].P, all_mModel[scan_index].N, cloudrot);
		pointcloudrotation(points_2, cloudrot);
		all_points_cloud_globle2.push_back(points_2);
		all_points_cloud_end2.insert(all_points_cloud_end2.end(), points_2.begin(), points_2.end());
		time2 = clock();
		cout << "The time is " << (double)(time2 - time1) / CLOCKS_PER_SEC << " s." << endl;
	}
}

void ComputeThread::controlComputeScan(int chooseJawIndex)
{
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

	for (int scan_index = 0; scan_index < DataSize; scan_index++)
	{
		usedSpace.acquire();
		cout << "The ComputeThread: " << scan_index << " has finished." << endl;
		scan::Unwarp *unwarp = new scan::Unwarp();
		int image_bias = 0;
		for (int j = 0; j < 15; j++)
		{
			memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + bufferBias * 33 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		
			memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + bufferBias * 33 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		}
		if (image_bias > 29)
		{
			for (int i = 0; i < 3; i++)
			{
				memcpy(image_rgb[i].data, totalNormalScanImageBuffer + bufferBias * 33 * imageSize + image_bias * imageSize, imageSize * sizeof(unsigned char));
				image_bias++;
			}
		}
		cout << "scan_index:" << scan_index <<endl;
		unwarp->PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1000.0);
		                                                                                              
		chooseJawAndIcp(matched_pixel_image, image_rgb, chooseJawIndex);

		bufferBias++;

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
		
		freeSpace.release();
		
		if (scan_index == (DataSize-1))
		{
			emit computeFinish();
			//GPAMeshing(chooseJawIndex);
		}
	}
}

void ComputeThread::Motor2Rot(const float yaw, const float pitch, cv::Mat &Rot)
{

	Mat rt_curr = Mat::eye(4, 4, CV_64FC1);
	double alpha = ToRad(pitch);  // x axis
	rt_curr.at<double>(0, 0) = 1; rt_curr.at<double>(0, 1) = 0; rt_curr.at<double>(0, 2) = 0; rt_curr.at<double>(0, 3) = 0;
	rt_curr.at<double>(1, 0) = 0; rt_curr.at<double>(1, 1) = cosf(alpha); rt_curr.at<double>(1, 2) = sinf(alpha); rt_curr.at<double>(1, 3) = 0;
	rt_curr.at<double>(2, 0) = 0; rt_curr.at<double>(2, 1) = -sinf(alpha); rt_curr.at<double>(2, 2) = cosf(alpha); rt_curr.at<double>(2, 3) = 0;
	rt_curr.at<double>(3, 0) = 0; rt_curr.at<double>(3, 1) = 0; rt_curr.at<double>(3, 2) = 0; rt_curr.at<double>(3, 3) = 1;

	Mat rt_curr2 = Mat::eye(4, 4, CV_64FC1);
	double alpha2 = ToRad(yaw);  // z axis
	rt_curr2.at<double>(0, 0) = cosf(alpha2); rt_curr2.at<double>(0, 1) = sinf(alpha2); rt_curr2.at<double>(0, 2) = 0; rt_curr2.at<double>(0, 3) = 0;
	rt_curr2.at<double>(1, 0) = -sinf(alpha2); rt_curr2.at<double>(1, 1) = cosf(alpha2); rt_curr2.at<double>(1, 2) = 0; rt_curr2.at<double>(1, 3) = 0;
	rt_curr2.at<double>(2, 0) = 0; rt_curr2.at<double>(2, 1) = 0; rt_curr2.at<double>(2, 2) = 1; rt_curr2.at<double>(2, 3) = 0;
	rt_curr2.at<double>(3, 0) = 0; rt_curr2.at<double>(3, 1) = 0; rt_curr2.at<double>(3, 2) = 0; rt_curr2.at<double>(3, 3) = 1;

	Rot = rt_curr2*rt_curr;
	cout << Rot << endl;
}

void ComputeThread::chooseCompenJawAndIcp(cv::Mat matched_pixel_image, vector<cv::Mat> image_rgb, int chooseJawIndex)
{
	vector<double> points_2;

	cv::Mat rt_icp = cv::Mat::eye(4, 4, CV_64FC1);

	orth::MeshModel mModel;
	rs->delaunayAlgorithm(matched_pixel_image, image_rgb, color_red_parameter, color_green_parameter, color_blue_parameter, 0.5, mModel, 0.02, points_2);

	cv::Mat rt_curr;
	Motor2Rot(c_addscan_x, c_addscan_y, rt_curr);

	if (chooseJawIndex == 1)
	{
		int scan_index = upper_mModel.size();
		upper_mModel.push_back(mModel);
		
		pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, rt_curr);
		pointcloudrotation(points_2, rt_curr);

		if (!pointcloudICP(upper_points_cloud_end2, points_2, 1, 1, rt_icp))
		{
			return;
		}
		pointcloudrotation(upper_mModel[scan_index].P, upper_mModel[scan_index].N, rt_icp);
		pointcloudrotation(points_2, rt_icp);
		upper_points_cloud_globle2.push_back(points_2);
		upper_points_cloud_end2.insert(upper_points_cloud_end2.end(), points_2.begin(), points_2.end());
	}
	else if (chooseJawIndex == 2)
	{
		int scan_index = lower_mModel.size();
		lower_mModel.push_back(mModel);

		pointcloudrotation(lower_mModel[scan_index].P, upper_mModel[scan_index].N, rt_curr);
		pointcloudrotation(points_2, rt_curr);

		if (!pointcloudICP(lower_points_cloud_end2, points_2, 1, 1, rt_icp))
		{
			return;
		}
		pointcloudrotation(lower_mModel[scan_index].P, upper_mModel[scan_index].N, rt_icp);
		pointcloudrotation(points_2, rt_icp);
		lower_points_cloud_globle2.push_back(points_2);
		lower_points_cloud_end2.insert(lower_points_cloud_end2.end(), points_2.begin(), points_2.end());
	}
	else if (chooseJawIndex == 3)
	{
		int scan_index = all_mModel.size();
		all_mModel.push_back(mModel);

		pointcloudrotation(all_mModel[scan_index].P, upper_mModel[scan_index].N, rt_curr);
		pointcloudrotation(points_2, rt_curr);

		if (!pointcloudICP(all_points_cloud_end2, points_2, 1, 1, rt_icp))
		{
			return;
		}
		pointcloudrotation(all_mModel[scan_index].P, upper_mModel[scan_index].N, rt_icp);
		pointcloudrotation(points_2, rt_icp);
		all_points_cloud_globle2.push_back(points_2);
		all_points_cloud_end2.insert(all_points_cloud_end2.end(), points_2.begin(), points_2.end());
	}
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
	for (int j = 0; j < 15; j++)
	{
		memcpy(im_l + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;

		memcpy(im_r + j * imageSize, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
		image_bias++;
	}
	if (image_bias > 29)
	{
		for (int i = 0; i < 3; i++)
		{
			memcpy(image_rgb[i].data, totalNormalScanImageBuffer + image_bias * imageSize, imageSize * sizeof(unsigned char));
			image_bias++;
		}
	}

	unwarp->PointCloudCalculateCuda2(im_l, im_r, IMG_ROW, IMG_COL, (double*)rs->F.data, (double*)rs->Rot_l.data, (double*)rs->Rot_r.data, (double*)rs->tvec_l.data, (double*)rs->tvec_r.data, (double*)rs->intr1.data, (double*)rs->intr2.data, (double*)rs->distCoeffs[0].data, (double*)rs->distCoeffs[1].data, (double*)rs->c_p_system_r.data, (double*)matched_pixel_image.data, (double*)normal_image.data, 1.0);

	/*vector<double> points_;
	vector<float> normal;
	vector<unsigned char> points_color;*/
	vector<double> points_2;
	/*vector<unsigned char> points_color2;
	vector<uint32_t> faces;*/

	chooseCompenJawAndIcp(matched_pixel_image, image_rgb, chooseJawIndex);

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
	freeSpace.release();
	cout << "补扫一个角度图片计算完成" << endl;
	
	emit computeFinish();
}

//void ComputeThread::GPAMeshing(int chooseJawIndex)
//{
//	GPA gpa;
//	vector<vector<double>> points_target;
//	if (chooseJawIndex == 1)
//	{
//		vector<cv::Mat> rt_matrixs(upper_points_cloud_globle2.size());
//		gpa.GpaRegistrationGPU(upper_points_cloud_globle2, points_target, rt_matrixs, 80);
//		orth::MeshModel totalMeshModel;
//		for (int data_index = 0; data_index < upper_points_cloud_globle2.size(); data_index++)
//		{
//			pointcloudrotationandtotalmesh(upper_mModel[data_index].P, upper_mModel[data_index].N, rt_matrixs[data_index], totalMeshModel);
//		}
//		recon::PoissonRec pr;
//		pr()
//	}
//	else if (chooseJawIndex == 2)
//	{
//
//	}
//	else if (chooseJawIndex == 3)
//	{
//
//	}
//}
