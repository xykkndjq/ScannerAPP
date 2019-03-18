#include "3dScan.h"

RasterScan::RasterScan()
{

}

RasterScan::~RasterScan()
{
}

void RasterScan::InitRasterScan(const std::string calibration_parameter_name)
{

	//Mat cameraMatrix_[3], distCoeffs_[3],R, T, F, F2, Rot_l, tvec_l, Rot_r, tvec_r;
	cv::FileStorage fs_g(calibration_parameter_name, cv::FileStorage::READ);
	fs_g["R"] >> R;
	fs_g["T"] >> T;
	fs_g["F"] >> F;
	fs_g["F2"] >> F2;
	fs_g["Rot_l"] >> Rot_l;
	fs_g["tvec_l"] >> tvec_l;
	fs_g["Rot_r"] >> Rot_r;
	fs_g["tvec_r"] >> tvec_r;
	fs_g["Rot_p"] >> Rot_p;
	fs_g["tvec_p"] >> tvec_p;

	fs_g["cameraMatrix_l"] >> intr1;
	fs_g["distCoeffs_l"] >> distCoeffs[0];
	fs_g["cameraMatrix_r"] >> intr2;
	fs_g["distCoeffs_r"] >> distCoeffs[1];
	fs_g["projectorMatrix_r"] >> intrp;
	fs_g["projectordistCoeffs_r"] >> distCoeffs[2];
	fs_g["c_p_system_l"] >> c_p_system_l;
	fs_g["c_p_system_r"] >> c_p_system_r;
	fs_g.release();

	float deviation = 0.84089642;
	float a = 1 / (2 * 3.1415926*deviation*deviation);
	float b = -1 * 2.0*deviation*deviation;
	int Filter_size = 5;
	GaussTemplate.resize(Filter_size);
	for (int i = 0; i < Filter_size; i++)
	{
		GaussTemplate[i].resize(Filter_size);
		for (int j = 0; j < Filter_size; j++)
		{
			GaussTemplate[i][j].resize(Filter_size);
			for (int k = 0; k < Filter_size; k++)
			{
				GaussTemplate[i][j][k] = a*exp((double)((i - GaussTemplate.size() / 2)*(i - GaussTemplate.size() / 2) + (j - GaussTemplate.size() / 2)*(j - GaussTemplate.size() / 2) + (k - GaussTemplate.size() / 2)*(k - GaussTemplate.size() / 2)) / b);
				//cout << GaussTemplate[i][j][k] << " ";
			}
			//cout << endl;
		}
		//cout << endl;
	}
	//cout << "start!!************************************" << endl;


}

void RasterScan::ReadScanData(const string dir, vector<Mat> &image_data, bool flag)
{
	for (int image_index = 1; image_index <16; image_index++)
	{
		stringstream ss;
		ss << image_index;
		string index_;
		ss >> index_;
		Mat image;
		if (!flag)
		{
			image = cv::imread(dir + "L_" + index_ + ".png", 0);
			cv::flip(image, image, -1);
			image = Distortion(image, 0);
		}
		else
		{
			image = cv::imread(dir + "R_" + index_ + ".png", 0);
			cv::flip(image, image, -1);
			image = Distortion(image, 1);
		}


		image_data.push_back(image);
	}
}

float RasterScan::Distance(float x1, float y1, float x2, float y2)
{
	float dx = x1 - x2;
	float dy = y1 - y2;
	float dis = sqrt(dx*dx + dy*dy);
	return dis;
}

double RasterScan::Distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx = x1 - x2;
	double dy = y1 - y2;
	double dz = z1 - z2;
	double dis = sqrt(dx*dx + dy*dy + dz*dz);
	return dis;
}

Eigen::Vector3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2)
{
	//cout << "@@@" << x1 << "," << y1 << ";" << x2 << "," << y2 << ";" << endl;
	Eigen::Vector3f vc;
	vc(0) = (y2 - y1) / (x2 - x1);
	vc(1) = -1;
	vc(2) = (x2*y1 - y2*x1) / (x2 - x1);
	//cout << "!!!" << vc << endl;
	return vc;
}

Eigen::Vector3f RasterScan::CalculateLine(float x1, float y1, float x2, float y2, float x3, float y3)
{
	//cout << "@@@" << x1 << "," << y1 << ";" << x2 << "," << y2 << ";" << x3 << "," << y3 << ";" << endl;

	Eigen::Vector3f vc;
	float X1 = (x1 + x2) / 2;
	float Y1 = (y1 + y2) / 2;
	float X2 = (x3 + x2) / 2;
	float Y2 = (y3 + y2) / 2;

	vc(0) = (Y2 - Y1) / (X2 - X1);
	vc(1) = -1;
	vc(2) = (X2*Y1 - Y2*X1) / (X2 - X1);

	//cout << "!!!" << vc << endl;
	return vc;
}

void RasterScan::CircleCenterCalculate2(Mat &image, std::vector<cv::Point2d> &imagePoints)
{
	Mat cameraMatrix, dist;

	Mat b_img;
	//image = Distortion(image, cameraMatrix, distCoeffs);
	image.copyTo(b_img);
	b_img = 255 - b_img;
	//b_img = Distortion(b_img, cameraMatrix, distCoeffs);
	//image = b_img;
	//cvtColor(b_img, b_img, CV_BGR2GRAY);
	//b_img.convertTo(b_img, CV_32FC1, 1.0 / 255.0);

	// Set up the detector with default parameters.
	//SimpleBlobDetector detector;
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 60;
	params.maxThreshold = 230;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 3000;
	params.maxArea = 10000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.3;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.9;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.1;

	// Detect blobs.
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	//detector->detect(img, keypoints);
	detector->detect(b_img, keypoints);
	//params.detect(image, keypoints);

	// Draw detected blobs as red circles.
	//DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	for (int i = 0; i < keypoints.size(); i++)
	{
		imagePoints.push_back(keypoints[i].pt);
		circle(b_img, cv::Point(round(keypoints[i].pt.x), round(keypoints[i].pt.y)), 0.5, cv::Scalar(0, 0, 255), 1);
		cout << "point" << i << " : " << keypoints[i].pt << endl;
	}
	drawKeypoints(b_img, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

}

void RasterScan::CalculatePlane_XY_theta(const vector<Eigen::VectorXd> &Points, Eigen::Vector4d &Plane)
{
	Plane = Eigen::Vector4d::Zero();
	Eigen::MatrixXd A_plane = Eigen::MatrixXd(Points.size(), 3);
	Eigen::VectorXd B_plane = Eigen::VectorXd(Points.size());
	Eigen::VectorXd X_plane = Eigen::VectorXd(3);

	for (int point_index = 0; point_index < Points.size(); point_index++)
	{
		A_plane(point_index, 0) = Points[point_index](0);
		A_plane(point_index, 1) = Points[point_index](1);
		A_plane(point_index, 2) = Points[point_index](2);
		B_plane(point_index) = -1;
	}

	X_plane = A_plane.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_plane);
	//cout << "Aplane : " << A_plane << endl;
	//cout << "Xplane : " << X_plane << endl;
	double M = (X_plane(0)*X_plane(0) + X_plane(1)*X_plane(1) + X_plane(2)*X_plane(2));
	Plane(3) = sqrt(1 / M);
	Plane(0) = X_plane(0)*Plane(3);
	Plane(1) = X_plane(1)*Plane(3);
	Plane(2) = X_plane(2)*Plane(3);
}

void RasterScan::FindCorners(const Mat &Image, vector<cv::Point2f>& imagePoints, const cv::Size &boardSize)
{

	bool found;
	found = findChessboardCorners(Image, boardSize, imagePoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
	if (found) {
		Mat viewGray;
		cvtColor(Image, viewGray, cv::COLOR_BGR2GRAY);
		cornerSubPix(viewGray, imagePoints, cv::Size(11, 11),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
	}
}

void RasterScan::mergeImg(Mat & dst, Mat &src1, Mat &src2)
{
	int rows = 485;
	int cols = 1290;
	CV_Assert(src1.type() == src2.type());
	dst.create(rows, cols, src1.type());
	Mat img1, img2;
	resize(src1, img1, cv::Size(640, 480), 0, 0, CV_INTER_LINEAR);
	resize(src2, img2, cv::Size(640, 480), 0, 0, CV_INTER_LINEAR);
	img1.copyTo(dst(cv::Rect(0, 0, 640, 480)));
	img2.copyTo(dst(cv::Rect(650, 0, 640, 480)));
}

void RasterScan::EpiLinesFindPoint(Mat &img1, Mat &img2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2)
{

	//首先根据对应点计算出两视图的基础矩阵，基础矩阵包含了两个相机的外参数关系
	//vector<cv::Point2f> points11;
	//vector<cv::Point2f> points22;
	Mat img1_show = img1;
	Mat img2_show = img2;
	//Mat Epilines1(img_1.rows, img_1.cols, CV_32FC3);
	//Mat Epilines2(img_1.rows, img_1.cols, CV_32FC3);

	vector<cv::Vec3d> Epilines2;
	vector<cv::Point2d> points11; vector<cv::Point2d> points22;

	for (int point_index = 0; point_index < points2.size(); point_index++)
	{
		Mat point(3, 1, cv::DataType<double>::type);
		point.at<double>(0, 0) = points2[point_index].x;
		point.at<double>(1, 0) = points2[point_index].y;
		point.at<double>(2, 0) = 1;
		Mat e(3, 1, cv::DataType<double>::type);
		e = (point.t()*F).t();
		cv::Vec3d epilines(e.at<double>(0, 0), e.at<double>(1,
			0), e.at<double>(2, 0));
		Epilines2.push_back(epilines);
		float min_dis = 4000;
		int index = 1000;
		for (int point_match_index = 0; point_match_index < points1.size(); point_match_index++)
		{
			float dis = abs(epilines[0] * points1[point_match_index].x + epilines[1] * points1[point_match_index].y + epilines[2]) / sqrt(epilines[0] * epilines[0] + epilines[1] * epilines[1]);
			//cout << point_index << " : " << point_match_index << " = " << dis << endl;
			if (dis<min_dis)
			{
				min_dis = dis;
				index = point_match_index;
			}
		}
		if (min_dis<3)
		{
			points11.push_back(points1[index]);
			//points1.erase(points1.begin() + index);
			points22.push_back(points2[point_index]);
		}


	}


	//for (uint i = 0; i < points2.size(); i++)
	//{
	//	////划线
	//	//找左图对应点
	//	//img_1.copyTo(img1);
	//	cv::LineIterator it(img1, cv::Point(0, -Epilines2[i][2] / Epilines2[i][1]), cv::Point(img1.cols, -(Epilines2[i][2] + Epilines2[i][0] * img1.cols) / Epilines2[i][1]), 8);
	//	vector<cv::Vec3b> buf(it.count);
	//	cv::Point2f point_max;
	//	cv::RNG& rng = cv::theRNG();
	//	cv::Scalar color_epilines = cv::Scalar(255, 10, 10);
	//	//cv::Scalar color_line = cv::Scalar(10, 255, 10);
	//	for (int j = 0; j < it.count; j++, ++it) {
	//		circle(img1_show, it.pos(), 0.5, color_epilines);
	//		int x2 = it.pos().x;
	//		int y2 = it.pos().y;
	//		
	//	}
	//}

	//for (uint i = 0; i < points11.size(); i++)
	//{
	//	stringstream ss;
	//	ss << i;
	//	string index_;
	//	ss >> index_;
	//	cv::RNG& rng = cv::theRNG();
	//	cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
	//	circle(img1_show, points11[i], 0.5, color2);
	//	putText(img1_show, index_, points11[i], cv::FONT_HERSHEY_SIMPLEX, 1, color2, 4);
	//	circle(img2_show, points22[i], 0.5, color2);
	//	putText(img2_show, index_, points22[i], cv::FONT_HERSHEY_SIMPLEX, 1, color2, 4);
	//}

	////imwrite("point_code.png", img1_show);
	//imshow("111", img1_show);
	//imshow("222", img2_show);
	//cv::waitKey(0);

	points1.swap(points11);
	points2.swap(points22);

}

void RasterScan::MatchPoint(Mat &img_k_1, Mat &img_k_2, Mat &img_t_1, Mat &img_t_2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &dis_)
{
	std::ofstream ofile;               //定义输出文件
	ofile.open("./myfile.txt");     //作为输出文件打开

									//首先根据对应点计算出两视图的基础矩阵，基础矩阵包含了两个相机的外参数关系
	vector<cv::Point2d> points11;
	vector<cv::Point2d> points22;
	vector<double> point1_t;
	vector<double> point1_k;

	Mat img1_show(img_k_1.size(), CV_64FC3);
	Mat img2_show(img_k_2.size(), CV_64FC3);
	//Mat img1_show = img_k_1;
	//Mat img2_show = img_k_2;
	//Mat Epilines1(img_1.rows, img_1.cols, CV_32FC3);
	//Mat Epilines2(img_1.rows, img_1.cols, CV_32FC3);
	int width = img_t_1.cols;
	int height = img_t_1.rows;

	for (int u = 0; u < width; u++)
	{
		for (int v = 0; v < height; v++)
		{
			double k2 = img_k_2.at<double>(v, u);
			double t2 = img_t_2.at<double>(v, u);

			int u_start_2 = fmax(u - 1, 0), u_end_2 = fmin(u + 1, width - 1);
			int v_start_2 = fmax(v - 1, 0), v_end_2 = fmin(v + 1, height - 1);
			int good_k = 0;
			for (int U = u_start_2; U <= u_end_2; U++)
			{
				for (int V = v_start_2; V <= v_end_2; V++)
				{
					double k_ = img_k_2.at<double>(V, U);
					double t_ = img_t_2.at<double>(V, U);
					if (k_ == k2)
					{
						good_k++;
					}
				}
			}
			if (good_k<3)
			{
				//cout << good_k << endl;
				continue;
			}

			if (k2 != -1 && t2 != -1 && k2>0)
			{
				Mat point(3, 1, cv::DataType<double>::type);
				point.at<double>(0, 0) = (double)u;
				point.at<double>(1, 0) = (double)v;
				point.at<double>(2, 0) = 1;
				Mat e(3, 1, cv::DataType<double>::type);
				e = (point.t()*F).t();
				cv::Vec3d epilines(e.at<double>(0, 0), e.at<double>(1,
					0), e.at<double>(2, 0));
				//if (u==1000&&v==1000)
				//{
				//	cout << point << endl << endl;
				//	cout << F << endl << endl;
				//	cout << epilines << endl << endl;
				//}
				cv::LineIterator it(img_t_1, cv::Point(0, -epilines[2] / epilines[1]), cv::Point(img_t_1.cols, -(epilines[2] + epilines[0] * img_t_1.cols) / epilines[1]), 8);
				vector<cv::Vec3b> buf(it.count);
				cv::Point2i point_match;
				double t_match = 0;
				double k_match = 0;
				double min_dis = 10.0;
				for (int j = 0; j < it.count; j++, ++it) {

					double k1 = img_k_1.at<double>(it.pos().y, it.pos().x);
					double t1 = img_t_1.at<double>(it.pos().y, it.pos().x);
					if (k1 != k2 || k1 == -1 || t1 == -1 || k1 == 0)
					{
						continue;
					}
					//cout << "k1 = " << k1 << " t1 = " << t1 << " k2 = " << k2 << " t2 = " << t2 << endl;
					//

					double distance_ = abs(t2 - t1);
					if (distance_<min_dis&&min_dis>0.03)
					{
						min_dis = distance_;
						t_match = t1;
						k_match = k1;
						point_match.x = it.pos().x;
						point_match.y = it.pos().y;
					}
				}

				if (t_match == 0)
				{
					continue;
				}
				//circle(img2_show, cv::Point2i(u,v), 1, cv::Scalar(5));
				//cout << "distance = " << min_dis <<" t1 = "<<t_match<<" t2 = "<<t2<< endl;

				vector<Eigen::VectorXd> t_points;
				int u_start = fmax(point_match.x - 1, 0), u_end = fmin(point_match.x + 1, width - 1);
				int v_start = fmax(point_match.y - 1, 0), v_end = fmin(point_match.y + 1, height - 1);
				for (int U = u_start; U <= u_end; U++)
				{
					for (int V = v_start; V <= v_end; V++)
					{
						double k1 = img_k_1.at<double>(V, U);
						double t1 = img_t_1.at<double>(V, U);
						double dis = abs(t2 - t1);
						//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
						//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));
						if (k1 != k2 || dis>0.06)
						{
							continue;
						}
						Eigen::VectorXd t_point(3);
						t_point(0) = U;
						t_point(1) = V;
						t_point(2) = t1;
						t_points.push_back(t_point);
						//cout << t_point << endl << endl;
						if (t_points.size() >= 3)
						{
							break;
						}
					}
				}
				//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
				if (t_points.size()<4)
				{
					continue;
				}

				//double tm0 = img_t_1.at<double>(point_match.y, point_match.x);
				//double tm1 = img_t_1.at<double>(point_match.y, point_match.x + 1);
				//if (abs(tm1 - t2) > 0.06) { continue; }
				//double tm2 = img_t_1.at<double>(point_match.y + 1, point_match.x);
				//if (abs(tm2 - t2)>0.06) { continue; }

				//double dt = t2 - tm0,dt1 = (tm1 - tm0),dt2 = (tm2 - tm0);
				//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " dt = " << dt << " dt1 = " << dt1 << " dt2 = " << dt2 << endl;

				//double u_1 = point_match.x + dt / dt1;
				//double v_1 = point_match.y + dt / dt2;

				////vector<Eigen::VectorXd> t_points;
				////{	
				////	
				////	if ((point_match.x + 1)>=image_width|| (point_match.y + 1) >= image_height)
				////	{
				////		continue;
				////	}
				////	double t1 = img_t_1.at<double>(point_match.y, point_match.x);
				////	double t2 = img_t_1.at<double>(point_match.y, point_match.x+1);
				////	double t3 = img_t_1.at<double>(point_match.y+1, point_match.x);
				////	Eigen::VectorXd t_point1(3);
				////	t_point1(0) = double(point_match.x); t_point1(1) = double(point_match.y); t_point1(2) = t1;					
				////	t_points.push_back(t_point1);
				////	Eigen::VectorXd t_point2(3);
				////	t_point2(0) = double(point_match.x+1); t_point2(1) = double(point_match.y); t_point2(2) = t2;
				////	t_points.push_back(t_point2);
				////	Eigen::VectorXd t_point3(3);
				////	t_point3(0) = double(point_match.x); t_point3(1) = double(point_match.y+1); t_point3(2) = t3;
				////	t_points.push_back(t_point3);
				////}


				Eigen::Vector4d Plane;
				CalculatePlane(t_points, Plane);

				//vector<double> km;
				//km.push_back(img_k_1.at<double>(point_match.y, point_match.x));
				//km.push_back(img_k_1.at<double>(v_start, u_start));
				//km.push_back(img_k_1.at<double>(v_start, u_end));
				//km.push_back(img_k_1.at<double>(v_end, u_start));
				//km.push_back(img_k_1.at<double>(v_end, u_end));
				//vector<double> tm;
				//tm.push_back(img_t_1.at<double>(point_match.y, point_match.x));
				//tm.push_back(img_t_1.at<double>(v_start, u_start));
				//tm.push_back(img_t_1.at<double>(v_start, u_end));
				//tm.push_back(img_t_1.at<double>(v_end, u_start));
				//tm.push_back(img_t_1.at<double>(v_end, u_end));

				//int cflag = 0;
				//for (int point_size = 0; point_size < 5; point_size++)
				//{
				//	double dis = abs(t2 - tm[point_size]);
				//	//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
				//	//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

				//	if (km[point_size] != k2 || dis>0.06/*dis>0.1*/)
				//	{
				//		cflag++;
				//	}

				//}
				//if (cflag>0)
				//{
				//	continue;
				//}
				//double m41x = 2, m41y = 2, m41z = tm[4] - tm[1];
				//double m23x = 2, m23y = -2, m23z = tm[2] - tm[3];
				//double a = (m41y * m23z)-(m41z * m23y);
				//double b = (m41x * m23z)-(m41z * m23x);
				//double c = (m41x * m23y)-(m41y * m23x);
				//double abc = a*a + b*b + c*c;
				//double a2 = a / sqrt(abc);
				//double b2 = b / sqrt(abc);
				//double c2 = c / sqrt(abc);
				//double d2 = -1 * a2*point_match.x - b2*point_match.y - c2*tm[0];
				////cout << a2 << ", " << b2 << ", " << c2 << ", " << d << ", " << endl;
				////cout << Plane(0) << ", " << Plane(1) << ", " << Plane(2) << ", " << Plane(3) << ", " << endl;

				double A = Plane(0), B = Plane(1), C = Plane(2)*t2 + Plane(3);
				//double A = a2, B = b2, C = c2*t2 + d2;
				double u_1 = (B*epilines[2] - epilines[1] * C) / (A*epilines[1] - epilines[0] * B);
				double v_1 = (C*epilines[0] - epilines[2] * A) / (A*epilines[1] - epilines[0] * B);



				////for (int i = 0; i < t_points.size(); i++)
				////{
				////	ofile << t_points[i](2) <<" ";
				////}
				//double ddu = u_1 - point_match.x;
				//double ddv = v_1 - point_match.y;
				////ofile << endl;
				////ofile << point_match.x << ", " << point_match.y << "; " << u_1 << ", " << v_1 << "; " << ddu << ", " << ddv << endl;
				//if (sqrt(ddu*ddu+ddv*ddv)>2)
				//{
				//	cout << img_t_1.at<double>(v_1, u_1) << "  ;  " << t2 << endl;
				//}


				//double u_1 = point_match.x;
				//double v_1 = point_match.y;

				//cout << u_1 << " : " << v_1 << endl;
				if (u_1<0 || u_1 >= width || v_1<0 || v_1 >= height)
				{
					continue;
				}
				cv::Point2d point_1; point_1.x = u_1; point_1.y = v_1;

				//cv::Point2d point_1; point_1.x = point_match.x; point_1.y = point_match.y;
				cv::Point2d point_2; point_2.x = u; point_2.y = v;

				//cout << "k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;

				//if (u==645&&v==317)
				//{
				//	circle(img1_show, point_1, 5, cv::Scalar(0 , 0, 1));
				//	circle(img2_show, point_2, 5, cv::Scalar(0 , 0, 1));
				//}

				circle(img1_show, point_1, 0.5, cv::Scalar(k_match, (1 - k_match), 0.5));
				circle(img2_show, point_2, 0.5, cv::Scalar(k_match, (1 - k_match), 0.5));

				points11.push_back(point_1);
				points22.push_back(point_2);
				point1_t.push_back(t2);
				point1_k.push_back(k2);


			}
		}

	}

	//for (int point_index = 0; point_index < points11.size(); point_index++)
	//{
	//	//double k2 = point1_k[point_index];
	//	//double t2 = point1_t[point_index];
	//	double k2 = img_k_1.at<double>(points11[point_index]);
	//	double t2 = img_t_1.at<double>(points11[point_index]);

	//	Mat point(3, 1, cv::DataType<double>::type);
	//	point.at<double>(0, 0) = points11[point_index].x;
	//	point.at<double>(1, 0) = points11[point_index].y;
	//	point.at<double>(2, 0) = 1;
	//	Mat e(3, 1, cv::DataType<double>::type);
	//	e = (point.t()*F2).t();
	//	cv::Vec3d epilines(e.at<double>(0, 0), e.at<double>(1,
	//		0), e.at<double>(2, 0));

	//	cv::LineIterator it(img_t_2, cv::Point(0, -epilines[2] / epilines[1]), cv::Point(img_t_2.cols, -(epilines[2] + epilines[0] * img_t_2.cols) / epilines[1]), 8);
	//	vector<cv::Vec3b> buf(it.count);
	//	cv::Point2i point_match;
	//	double t_match = 0;
	//	double k_match = 0;
	//	double min_dis = 10.0;
	//	for (int j = 0; j < it.count; j++, ++it) 
	//	{

	//		double k1 = img_k_2.at<double>(it.pos().y, it.pos().x);
	//		double t1 = img_t_2.at<double>(it.pos().y, it.pos().x);
	//		if (k1 != k2 || k1 == -1 || t1 == -1 || k1 == 0)
	//		{
	//			continue;
	//		}
	//		//cout << "k1 = " << k1 << " t1 = " << t1 << " k2 = " << k2 << " t2 = " << t2 << endl;
	//		//

	//		double distance_ = abs(t2 - t1);
	//		if (distance_<min_dis&&min_dis>0.03)
	//		{
	//			min_dis = distance_;
	//			t_match = t1;
	//			k_match = k1;
	//			point_match.x = it.pos().x;
	//			point_match.y = it.pos().y;
	//		}
	//	}
	//	double dis = sqrt(((double)(point_match.x) - points22[point_index].x)*((double)(point_match.x) - points22[point_index].x)+ ((double)(point_match.y) - points22[point_index].y)*((double)(point_match.y) - points22[point_index].y));
	//	
	//	//if (dis>2)
	//	//{
	//		circle(img1_show, points11[point_index], 0.5, cv::Scalar(0, 0, dis/10));
	//		circle(img2_show, points22[point_index], 0.5, cv::Scalar(0, 0, dis/10));
	//	//}
	//	dis_.push_back(dis);

	//}

	points1.swap(points11);
	points2.swap(points22);

	ofile.close();
}

void RasterScan::CornerDistanceTest(Mat &img_k_1, Mat &img_k_2, Mat &img_t_1, Mat &img_t_2, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<cv::Point2f> &point_corner)
{

	//首先根据对应点计算出两视图的基础矩阵，基础矩阵包含了两个相机的外参数关系
	vector<cv::Point2d> points11;
	vector<cv::Point2d> points22;
	vector<double> point1_t;
	vector<double> point1_k;

	//Mat img1_show(img_k_1.size(), CV_64FC3);
	//Mat img2_show(img_k_2.size(), CV_64FC3);
	Mat img1_show = img_k_1;
	Mat img2_show = img_k_2;
	//Mat Epilines1(img_1.rows, img_1.cols, CV_32FC3);
	//Mat Epilines2(img_1.rows, img_1.cols, CV_32FC3);
	int width = img_t_1.cols;
	int height = img_t_1.rows;

	vector<double> corner_phase;
	vector<double> corner_periodic;

	for (int corner_index = 0; corner_index < point_corner.size(); corner_index++)
	{

		cv::Point2f point_match = point_corner[corner_index];
		double k2 = img_k_2.at<double>(point_match.y, point_match.x);
		double t2 = img_t_2.at<double>(point_match.y, point_match.x);

		if (k2 == -1 || t2 == -1)
		{
			corner_phase.push_back(-1);
			corner_periodic.push_back(-1);
			continue;
		}

		vector<Eigen::VectorXd> t_points;
		int u_start = fmax(point_match.x - 2, 0), u_end = fmin(point_match.x + 2, width - 1);
		int v_start = fmax(point_match.y - 2, 0), v_end = fmin(point_match.y + 2, height - 1);
		for (int U = u_start; U <= u_end; U++)
		{
			for (int V = v_start; V <= v_end; V++)
			{
				double k1 = img_k_2.at<double>(V, U);
				double t1 = img_t_2.at<double>(V, U);
				double dis = abs(t2 - t1);
				//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
				//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

				if (k1 != k2 || dis>0.06)
				{
					continue;
				}
				Eigen::VectorXd t_point(3);
				t_point(0) = U;
				t_point(1) = V;
				t_point(2) = t1;
				t_points.push_back(t_point);
				//cout << t_point << endl << endl;
			}
		}

		//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
		if (t_points.size()<3)
		{
			corner_phase.push_back(-1);
			corner_periodic.push_back(-1);
			continue;
		}

		Eigen::Vector4d Plane;
		CalculatePlane(t_points, Plane);

		double phase_ = -1 * (Plane(0)*point_match.x + Plane(1)*point_match.y + Plane(3)) / Plane(2);
		corner_phase.push_back(phase_);
		corner_periodic.push_back(k2);
	}

	for (int corner_index = 0; corner_index < point_corner.size(); corner_index++)
	{
		float u = point_corner[corner_index].x;
		float v = point_corner[corner_index].y;

		double k2 = corner_periodic[corner_index];
		double t2 = corner_phase[corner_index];

		if (k2 != -1 && t2 != -1 && k2>0)
		{
			Mat point(3, 1, cv::DataType<double>::type);
			point.at<double>(0, 0) = (double)u;
			point.at<double>(1, 0) = (double)v;
			point.at<double>(2, 0) = 1;
			Mat e(3, 1, cv::DataType<double>::type);
			e = (point.t()*F).t();
			cv::Vec3d epilines(e.at<double>(0, 0), e.at<double>(1,
				0), e.at<double>(2, 0));

			cv::LineIterator it(img_t_1, cv::Point(0, -epilines[2] / epilines[1]), cv::Point(img_t_1.cols, -(epilines[2] + epilines[0] * img_t_1.cols) / epilines[1]), 8);
			vector<cv::Vec3b> buf(it.count);
			cv::Point2i point_match;
			double t_match = 0;
			double k_match = 0;
			double min_dis = 10.0;
			for (int j = 0; j < it.count; j++, ++it) {

				double k1 = img_k_1.at<double>(it.pos().y, it.pos().x);
				double t1 = img_t_1.at<double>(it.pos().y, it.pos().x);
				if (k1 != k2 || k1 == -1 || t1 == -1 || k1 == 0)
				{
					continue;
				}
				//cout << "k1 = " << k1 << " t1 = " << t1 << " k2 = " << k2 << " t2 = " << t2 << endl;
				//

				double distance_ = abs(t2 - t1);
				if (distance_<min_dis&&min_dis>0.03)
				{
					min_dis = distance_;
					t_match = t1;
					k_match = k1;
					point_match.x = it.pos().x;
					point_match.y = it.pos().y;
				}
			}

			if (t_match == 0)
			{
				continue;
			}
			//circle(img2_show, cv::Point2i(u,v), 1, cv::Scalar(5));
			//cout << "distance = " << min_dis <<" t1 = "<<t_match<<" t2 = "<<t2<< endl;

			vector<Eigen::VectorXd> t_points;
			int u_start = fmax(point_match.x - 2, 0), u_end = fmin(point_match.x + 2, width - 1);
			int v_start = fmax(point_match.y - 2, 0), v_end = fmin(point_match.y + 2, height - 1);
			for (int U = u_start; U <= u_end; U++)
			{
				for (int V = v_start; V <= v_end; V++)
				{
					double k1 = img_k_1.at<double>(V, U);
					double t1 = img_t_1.at<double>(V, U);
					double dis = abs(t2 - t1);
					//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
					//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

					if (k1 != k2 || dis>0.06)
					{
						continue;
					}
					Eigen::VectorXd t_point(3);
					t_point(0) = U;
					t_point(1) = V;
					t_point(2) = t1;
					t_points.push_back(t_point);
					//cout << t_point << endl << endl;
				}
			}
			//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
			if (t_points.size()<3)
			{
				continue;
			}

			Eigen::Vector4d Plane;
			CalculatePlane(t_points, Plane);

			double A = Plane(0), B = Plane(1), C = Plane(2)*t2 + Plane(3);
			double u_1 = (B*epilines[2] - epilines[1] * C) / (A*epilines[1] - epilines[0] * B);
			double v_1 = (C*epilines[0] - epilines[2] * A) / (A*epilines[1] - epilines[0] * B);

			//cout << u_1 << " : " << v_1 << endl;
			if (u_1<0 || u_1 >= width || v_1<0 || v_1 >= height)
			{
				continue;
			}
			cv::Point2d point_1; point_1.x = u_1; point_1.y = v_1;

			//cv::Point2d point_1; point_1.x = point_match.x; point_1.y = point_match.y;
			cv::Point2d point_2; point_2.x = u; point_2.y = v;

			//cout << "k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;

			if (u == 645 && v == 317)
			{
				circle(img1_show, point_1, 5, cv::Scalar(0, 0, 1));
				circle(img2_show, point_2, 5, cv::Scalar(0, 0, 1));
			}

			circle(img1_show, point_1, 0.5, cv::Scalar(k_match, (1 - k_match), 0.5));
			circle(img2_show, point_2, 0.5, cv::Scalar(k_match, (1 - k_match), 0.5));

			points11.push_back(point_1);
			points22.push_back(point_2);
			point1_t.push_back(t2);
			point1_k.push_back(k2);
		}

	}



	points1.swap(points11);
	points2.swap(points22);

}

void RasterScan::DrawEpiLines(vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<float> &points) {



	std::vector<Eigen::Matrix4f> intrs;
	std::vector<Eigen::Matrix3f> Rots;
	std::vector<Eigen::Vector3f> Trans;

	vector<Eigen::Matrix4f> A;

	Eigen::Matrix3f R2 = Eigen::Matrix3f::Identity();
	Eigen::Matrix4f i2 = Eigen::Matrix4f::Identity();
	Eigen::Vector3f T2(tvec_l.at<double>(0, 0), tvec_l.at<double>(1, 0), tvec_l.at<double>(2, 0));
	Trans.push_back(T2);
	float fx = (float)intr1.at<double>(0, 0);
	float fy = (float)intr1.at<double>(1, 1);
	float cx = (float)intr1.at<double>(0, 2);
	float cy = (float)intr1.at<double>(1, 2);
	R2 << Rot_l.at<double>(0, 0), Rot_l.at<double>(0, 1), Rot_l.at<double>(0, 2),
		Rot_l.at<double>(1, 0), Rot_l.at<double>(1, 1), Rot_l.at<double>(1, 2),
		Rot_l.at<double>(2, 0), Rot_l.at<double>(2, 1), Rot_l.at<double>(2, 2);
	i2 << fx, 0, cx, 0,
		0, fy, cy, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Rots.push_back(R2);
	intrs.push_back(i2);
	Eigen::Affine3f RT = (Eigen::Translation3f(Trans[0]) * Eigen::AngleAxisf(Rots[0]));
	A.push_back(intrs[0] * RT.matrix());

	T2 << tvec_r.at<double>(0, 0), tvec_r.at<double>(1, 0),
		tvec_r.at<double>(2, 0);

	R2 << Rot_r.at<double>(0, 0), Rot_r.at<double>(0, 1), Rot_r.at<double>(0, 2),
		Rot_r.at<double>(1, 0), Rot_r.at<double>(1, 1), Rot_r.at<double>(1, 2),
		Rot_r.at<double>(2, 0), Rot_r.at<double>(2, 1), Rot_r.at<double>(2, 2);

	i2 << intr2.at<double>(0, 0), 0., intr2.at<double>(0, 2), 0,
		0., intr2.at<double>(1, 1), intr2.at<double>(1, 2), 0,
		0., 0., 1., 0,
		0, 0, 0, 1;
	Trans.push_back(T2);
	Rots.push_back(R2);
	intrs.push_back(i2);
	RT = (Eigen::Translation3f(Trans[1]) * Eigen::AngleAxisf(Rots[1]));
	A.push_back(intrs[1] * RT.matrix());
	int n1, n2;
	Eigen::VectorXf f(8);

	for (int i = 0; i < points1.size(); i++)
	{
		if (points1[i].x != -1)
		{
			f(0) = points1[i].x; f(1) = points1[i].y;
			f(4) = points2[i].x; f(5) = points2[i].y;

			f(2) = 1; f(3) = 0;
			f(6) = 1; f(7) = 0;
			//cout << f << endl;
			//std::cout << "(u1,v1) (u2,v2) = (" << f(0) << ", " << f(1) << ")  (" << f(4) << ", " << f(5) << ")" << endl; std::cout << endl;

			Eigen::MatrixXf F(5, 5);

			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					F(i, j) = A[0](i, j) + A[1](i, j);
				}
			}
			F(0, 3) = -1 * f(0); F(0, 4) = -1 * f(4);
			F(1, 3) = -1 * f(1); F(1, 4) = -1 * f(5);
			F(2, 3) = -1; F(2, 4) = -1;

			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					F(i + 3, j) = A[0](0, j) + A[1](i + 1, j);
				}
			}
			F(3, 3) = -1 * f(0); F(3, 4) = -1 * f(5);
			F(4, 3) = -1 * f(0); F(4, 4) = -1;

			Eigen::VectorXf y(5);
			y(0) = -1 * (A[0](0, 3) + A[1](0, 3));
			y(1) = -1 * (A[0](1, 3) + A[1](1, 3));
			y(2) = -1 * (A[0](2, 3) + A[1](2, 3));
			y(3) = -1 * (A[0](0, 3) + A[1](1, 3));
			y(4) = -1 * (A[0](0, 3) + A[1](2, 3));

			//cout << "start solving ...";
			Eigen::VectorXf x(5);
			x = F.lu().solve(y);
			cout << x << endl;
			Eigen::Vector3f xyz;

			//xyz(0) = x(0);
			//xyz(1) = x(1);
			//xyz(2) = x(2);
			if (abs(x(0))>2000 || abs(x(1))>2000 || abs(x(2))>2000)
			{
				continue;
			}
			points.push_back(x(0));
			points.push_back(x(1));
			points.push_back(x(2));

		}

	}

	//waitKey(0);
}

void RasterScan::Calculate3DPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, vector<double> &new_points) {

	vector<Eigen::VectorXd> points_end;
	vector<Eigen::VectorXd> points_end2;
	//vector<float> new_points;

	for (int point_index = 0; point_index < points1.size(); point_index++)
	{
		//cout <<point_index<<" : "<< points2[point_index].x<<" ,"<< points2[point_index].y<< endl;
		Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
		Eigen::VectorXd B_ = Eigen::VectorXd(4);
		Eigen::VectorXd X_ = Eigen::VectorXd(3);

		double u1 = points1[point_index].x; double v1 = points1[point_index].y;
		double u2 = points2[point_index].x; double v2 = points2[point_index].y;
		double r1 = Rot_l.at<double>(0, 0), r2 = Rot_l.at<double>(0, 1), r3 = Rot_l.at<double>(0, 2),
			r4 = Rot_l.at<double>(1, 0), r5 = Rot_l.at<double>(1, 1), r6 = Rot_l.at<double>(1, 2),
			r7 = Rot_l.at<double>(2, 0), r8 = Rot_l.at<double>(2, 1), r9 = Rot_l.at<double>(2, 2);
		double t1 = tvec_l.at<double>(0), t2 = tvec_l.at<double>(1), t3 = tvec_l.at<double>(2);
		double fx1 = intr1.at<double>(0, 0), fy1 = intr1.at<double>(1, 1), cx1 = intr1.at<double>(0, 2), cy1 = intr1.at<double>(1, 2);
		double r_1 = Rot_r.at<double>(0, 0), r_2 = Rot_r.at<double>(0, 1), r_3 = Rot_r.at<double>(0, 2),
			r_4 = Rot_r.at<double>(1, 0), r_5 = Rot_r.at<double>(1, 1), r_6 = Rot_r.at<double>(1, 2),
			r_7 = Rot_r.at<double>(2, 0), r_8 = Rot_r.at<double>(2, 1), r_9 = Rot_r.at<double>(2, 2);
		double t_1 = tvec_r.at<double>(0), t_2 = tvec_r.at<double>(1), t_3 = tvec_r.at<double>(2);
		double fx2 = intr2.at<double>(0, 0), fy2 = intr2.at<double>(1, 1), cx2 = intr2.at<double>(0, 2), cy2 = intr2.at<double>(1, 2);

		A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
		A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

		A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
		A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;

		//cout << "The least-squares solution is:\n"
		//	<< A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_)<<endl;
		X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);
		//cout << X_ << endl;
		//if (X_(2)>1500 || X_(2)<600)
		//{
		//	continue;
		//}
		double x = X_(0), y = X_(1), z = X_(2);

		points.push_back(x);
		points.push_back(y);
		points.push_back(z);

		points_end.push_back(X_);
	}

	//Eigen::Vector4d Plane;
	//CalculatePlane(points_end, Plane);

	//double D = Plane(3);
	//double A = Plane(0);
	//double B = Plane(1);
	//double C = Plane(2);
	//double plane_sq = sqrt(A*A + B*B + C*C);
	//vector<Eigen::VectorXd> points_1;
	//for (int point_indexx = 0; point_indexx < points_end.size(); point_indexx++)
	//{
	//	double dis_point_plane = abs(points_end[point_indexx](0)*A + points_end[point_indexx](1)*B + points_end[point_indexx](2)*C + D) / plane_sq;

	//	if (dis_point_plane<80)
	//	{
	//		points_1.push_back(points_end[point_indexx]);
	//	}
	//}

	//CalculatePlane(points_end, Plane);
	//D = Plane(3);
	//A = Plane(0);
	//B = Plane(1);
	//C = Plane(2);
	//plane_sq = sqrt(A*A + B*B + C*C);
	//vector<Eigen::VectorXd> points_2;
	//for (int point_indexx = 0; point_indexx < points_end.size(); point_indexx++)
	//{
	//	double dis_point_plane = abs(points_end[point_indexx](0)*A + points_end[point_indexx](1)*B + points_end[point_indexx](2)*C + D) / plane_sq;

	//	if (dis_point_plane<20)
	//	{
	//		points_2.push_back(points_end[point_indexx]);
	//	}
	//}

	//CalculatePlane(points_2, Plane);
	//D = Plane(3);
	//A = Plane(0);
	//B = Plane(1);
	//C = Plane(2);
	//for (int point_index = 0; point_index < points_2.size(); point_index++)
	//{
	//	double xi = points_2[point_index](0), yi = points_2[point_index](1), zi = points_2[point_index](2);
	//	double t = (xi*A + yi*B + zi*C + D) / (A*A + B*B + C*C);
	//	double x = xi - A*t;
	//	double y = yi - B*t;
	//	double z = zi - C*t;
	//	new_points.push_back(x / 1000);
	//	new_points.push_back(y / 1000);
	//	new_points.push_back(z / 1000);
	//	Eigen::Vector3d np(x, y, z);
	//	points_end2.push_back(np);
	//	//cout << np << endl;
	//}
	double sum = 0;
	int	num = 0;
	//for (int point_index = 4; point_index < points_end.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 4;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end[point_index - 4] - points_end[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//	if (distance<61&&distance>59)
	//	{
	//		sum += distance;
	//		num++;
	//	}
	//}
	//sum = sum / (double)num;
	//cout << "mean distance = " << sum << "mm"<<endl;
	//cout << "real distance = 60mm" << endl;
	//cout << "relative error = " << abs(60 - sum) << "mm" << endl;

	for (int point_index = 0; point_index < 5; point_index++)
	{
		stringstream ss1, ss2;
		ss1 << point_index + 1;
		ss2 << point_index;
		string s1, s2;
		ss1 >> s1;
		ss2 >> s2;
		Eigen::VectorXd d_ = points_end[(point_index + 20)] - points_end[point_index];
		double distance = sqrt(d_.transpose()*d_);
		cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
		sum += distance;
		num++;
	}
	sum = sum / (double)num;
	cout << "mean distance = " << sum << "mm" << endl;
	cout << "real distance = 60mm" << endl;
	cout << "relative error = " << abs(60 - sum) << "mm" << endl;

	//for (int point_index = 1; point_index < points_end2.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end2[point_index - 1] - points_end2[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}


	/*------------------------------------------------ Registration ----------------------------------------*/
	//Eigen::Vector4d Plane;
	//CalculatePlane(points_end, Plane);

	//double D = Plane(3);
	//double A = Plane(0);
	//double B = Plane(1);
	//double C = Plane(2);
	////cout << " A = " << A << " B = " << B << " C = " << C << " D = " << D<< endl;

	//for (int point_index = 0; point_index < points_end.size(); point_index++)
	//{
	//	double xi = points_end[point_index](0), yi = points_end[point_index](1), zi = points_end[point_index](2);
	//	double t = (xi*A + yi*B + zi*C + D)/ (A*A + B*B + C*C);
	//	double x = xi - A*t;
	//	double y = yi - B*t;
	//	double z = zi - C*t;
	//	new_points.push_back(x / 1000);
	//	new_points.push_back(y / 1000);
	//	new_points.push_back(z / 1000);
	//	Eigen::Vector3d np(x, y, z);
	//	points_end2.push_back(np);
	//	//cout << np << endl;
	//}

	//for (int point_index = 1; point_index < points_end.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end[point_index - 1] - points_end[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}

	//for (int point_index = 1; point_index < points_end2.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end2[point_index - 1] - points_end2[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}

}

void RasterScan::Calculate3DPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, Mat &right_depth_map) {

	for (int point_index = 0; point_index < points1.size(); point_index++)
	{
		//cout << point_index << " : " << points2[point_index].x << " ," << points2[point_index].y << endl;
		Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
		Eigen::VectorXd B_ = Eigen::VectorXd(4);
		Eigen::VectorXd X_ = Eigen::VectorXd(3);

		double u1 = points1[point_index].x; double v1 = points1[point_index].y;
		double u2 = points2[point_index].x; double v2 = points2[point_index].y;
		double r1 = Rot_l.at<double>(0, 0), r2 = Rot_l.at<double>(0, 1), r3 = Rot_l.at<double>(0, 2),
			r4 = Rot_l.at<double>(1, 0), r5 = Rot_l.at<double>(1, 1), r6 = Rot_l.at<double>(1, 2),
			r7 = Rot_l.at<double>(2, 0), r8 = Rot_l.at<double>(2, 1), r9 = Rot_l.at<double>(2, 2);
		double t1 = tvec_l.at<double>(0), t2 = tvec_l.at<double>(1), t3 = tvec_l.at<double>(2);
		double fx1 = intr1.at<double>(0, 0), fy1 = intr1.at<double>(1, 1), cx1 = intr1.at<double>(0, 2), cy1 = intr1.at<double>(1, 2);
		double r_1 = Rot_r.at<double>(0, 0), r_2 = Rot_r.at<double>(0, 1), r_3 = Rot_r.at<double>(0, 2),
			r_4 = Rot_r.at<double>(1, 0), r_5 = Rot_r.at<double>(1, 1), r_6 = Rot_r.at<double>(1, 2),
			r_7 = Rot_r.at<double>(2, 0), r_8 = Rot_r.at<double>(2, 1), r_9 = Rot_r.at<double>(2, 2);
		double t_1 = tvec_r.at<double>(0), t_2 = tvec_r.at<double>(1), t_3 = tvec_r.at<double>(2);
		double fx2 = intr2.at<double>(0, 0), fy2 = intr2.at<double>(1, 1), cx2 = intr2.at<double>(0, 2), cy2 = intr2.at<double>(1, 2);

		A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
		A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

		A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
		A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;

		//cout << "The least-squares solution is:\n"
		//	<< A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_)<<endl;
		X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);

		double x = X_(0), y = X_(1), z = X_(2);

		points.push_back(x);
		points.push_back(y);
		points.push_back(z);

		right_depth_map.at<double>(v2, u2) = (x*r_7 + y*r_8 + z*r_9 + t_3);
	}



}

void RasterScan::XY_TsdfVolume(const double* size, const int* resolution, const double &truncation, const int width_, const int height_)
{
	resolution_[0] = resolution[0];
	resolution_[1] = resolution[1];
	resolution_[2] = resolution[2];
	tranc_dist_ = truncation;
	image_width = width_;
	image_height = height_;

	volume_Data.resize(resolution_[0]);
	for (int x = 0; x < resolution_[0]; x++)
	{
		volume_Data[x].resize(resolution_[1]);
		for (int y = 0; y < resolution_[1]; y++)
		{
			volume_Data[x][y].resize(resolution_[2]);
			for (int z = 0; z < resolution_[2]; z++)
			{
				volume_Data[x][y][z].distance_ = 0;
				volume_Data[x][y][z].weight_ = 0;
			}
		}
	}


	cell_size[0] = size[0] / resolution_[0];
	cell_size[1] = size[1] / resolution_[1];
	cell_size[2] = size[2] / resolution_[2];

	start_x = 0;
	start_y = 0;
	start_z = 0;

}

void RasterScan::XY_TsdfVolume(double* tsdf_info, float* tsdf_data)
{

	resolution_[0] = tsdf_info[0];
	resolution_[1] = tsdf_info[1];
	resolution_[2] = tsdf_info[2];
	tranc_dist_ = tsdf_info[6];
	//image_width = width_;
	//image_height = height_;

	volume_Data2.resize(resolution_[0] * resolution_[1] * resolution_[2]);
	memcpy(volume_Data2.data(), tsdf_data, resolution_[0] * resolution_[1] * resolution_[2] * sizeof(float));

	cell_size[0] = tsdf_info[3] / resolution_[0];
	cell_size[1] = tsdf_info[4] / resolution_[1];
	cell_size[2] = tsdf_info[5] / resolution_[2];
	cout << cell_size[0] << endl;
	cout << cell_size[1] << endl;
	cout << cell_size[2] << endl;
}

void RasterScan::XY_PointFilterVolume(const double* size, const int* resolution)
{
	resolution_[0] = resolution[0];
	resolution_[1] = resolution[1];
	resolution_[2] = resolution[2];

	volume_point.resize(resolution_[0]);
	for (int x = 0; x < resolution_[0]; x++)
	{
		volume_point[x].resize(resolution_[1]);
		for (int y = 0; y < resolution_[1]; y++)
		{
			volume_point[x][y].resize(resolution_[2]);
		}
	}

	//cell_size[0] = size[0] / resolution_[0];
	//cell_size[1] = size[1] / resolution_[1];
	//cell_size[2] = size[2] / resolution_[2];

	//start_x = cell_size[2] * -0.2*resolution_[2];
	//start_y = 0;
	//start_z = cell_size[2] * -0.5*resolution_[2];
	end_x = -1000, end_y = -1000, end_z = -1000, start_x = 1000, start_y = 1000, start_z = 1000;

}

void RasterScan::PointCloudAddIn(vector<double> &point_cloud_, vector<float> &point_normal_, cv::Mat &Rt_)
{

	for (int point_index = 0; point_index < point_cloud_.size() / 3; point_index++)
	{
		double x = Rt_.at<double>(0, 0)*point_cloud_[point_index * 3 + 0] + Rt_.at<double>(0, 1)*point_cloud_[point_index * 3 + 1] + Rt_.at<double>(0, 2)*point_cloud_[point_index * 3 + 2] + Rt_.at<double>(0, 3);
		double y = Rt_.at<double>(1, 0)*point_cloud_[point_index * 3 + 0] + Rt_.at<double>(1, 1)*point_cloud_[point_index * 3 + 1] + Rt_.at<double>(1, 2)*point_cloud_[point_index * 3 + 2] + Rt_.at<double>(1, 3);
		double z = Rt_.at<double>(2, 0)*point_cloud_[point_index * 3 + 0] + Rt_.at<double>(2, 1)*point_cloud_[point_index * 3 + 1] + Rt_.at<double>(2, 2)*point_cloud_[point_index * 3 + 2] + Rt_.at<double>(2, 3);
		point_cloud_whole.push_back(x);
		point_cloud_whole.push_back(y);
		point_cloud_whole.push_back(z);
		double x2 = Rt_.at<double>(0, 0)*point_normal_[point_index * 3 + 0] + Rt_.at<double>(0, 1)*point_normal_[point_index * 3 + 1] + Rt_.at<double>(0, 2)*point_normal_[point_index * 3 + 2];
		double y2 = Rt_.at<double>(1, 0)*point_normal_[point_index * 3 + 0] + Rt_.at<double>(1, 1)*point_normal_[point_index * 3 + 1] + Rt_.at<double>(1, 2)*point_normal_[point_index * 3 + 2];
		double z2 = Rt_.at<double>(2, 0)*point_normal_[point_index * 3 + 0] + Rt_.at<double>(2, 1)*point_normal_[point_index * 3 + 1] + Rt_.at<double>(2, 2)*point_normal_[point_index * 3 + 2];

		point_normal_whole.push_back(x2);
		point_normal_whole.push_back(y2);
		point_normal_whole.push_back(z2);

		if (x<start_x)
		{
			start_x = x;
		}
		if (x>end_x)
		{
			end_x = x;
		}
		if (y<start_y)
		{
			start_y = y;
		}
		if (y>end_y)
		{
			end_y = y;
		}
		if (z<start_z)
		{
			start_z = z;
		}
		if (z>end_z)
		{
			end_z = z;
		}
	}


}

void RasterScan::ReductionWholePoints(vector<double> &point_cloud_, vector<float> &point_normal_)
{
	cell_size[0] = (end_x - start_x) / resolution_[0];
	cell_size[1] = (end_y - start_y) / resolution_[1];
	cell_size[2] = (end_z - start_z) / resolution_[2];

	cout << (end_x - start_x) << endl;
	cout << (end_y - start_y) << endl;
	cout << (end_z - start_z) << endl;

	vector<int> count(30);

	for (int point_index = 0; point_index < point_cloud_whole.size() / 3; point_index++)
	{
		double x = point_cloud_whole[point_index * 3 + 0];
		double y = point_cloud_whole[point_index * 3 + 1];
		double z = point_cloud_whole[point_index * 3 + 2];

		Eigen::Vector3f epoint(x, y, z);
		Eigen::Vector3i voxel_index = getVoxel(epoint);
		volume_point[voxel_index(0)][voxel_index(1)][voxel_index(2)].point_index.push_back(point_index);

	}

	for (int x = 0; x < volume_point.size(); x++)
	{
		//cout <<"volume x = "<< x << endl;
		for (int y = 0; y < volume_point[0].size(); y++)
		{

			for (int z = 0; z < volume_point[0][0].size(); ++z)
			{
				if (volume_point[x][y][z].point_index.size()>0)
				{
					double a = 0, b = 0, c = 0, an = 0, bn = 0, cn = 0;

					for (int i = 0; i < volume_point[x][y][z].point_index.size(); i++)
					{
						a += point_cloud_whole[volume_point[x][y][z].point_index[i] * 3 + 0];
						b += point_cloud_whole[volume_point[x][y][z].point_index[i] * 3 + 1];
						c += point_cloud_whole[volume_point[x][y][z].point_index[i] * 3 + 2];
						an += point_normal_whole[volume_point[x][y][z].point_index[i] * 3 + 0];
						bn += point_normal_whole[volume_point[x][y][z].point_index[i] * 3 + 1];
						cn += point_normal_whole[volume_point[x][y][z].point_index[i] * 3 + 2];
						//cout << an << " ; " << bn << " ; " << cn << endl;
					}
					a /= volume_point[x][y][z].point_index.size();
					b /= volume_point[x][y][z].point_index.size();
					c /= volume_point[x][y][z].point_index.size();
					an /= volume_point[x][y][z].point_index.size();
					bn /= volume_point[x][y][z].point_index.size();
					cn /= volume_point[x][y][z].point_index.size();

					//cout << an << " ; " << bn << " ; " << cn << endl;
					//cout << "-----------------------------" << endl;
					//cout << a << " ; " << b << " ; " << c << endl;
					count[volume_point[x][y][z].point_index.size()]++;

					point_cloud_.push_back(a);
					point_cloud_.push_back(b);
					point_cloud_.push_back(c);

					double norm_ = sqrt(an*an + bn*bn + cn*cn);
					point_normal_.push_back(an / norm_);
					point_normal_.push_back(bn / norm_);
					point_normal_.push_back(cn / norm_);
				}
			}
		}
	}

	//volume_point.clear();
	//volume_point.resize(resolution_[0]);
	//for (int x = 0; x < resolution_[0]; x++)
	//{
	//	volume_point[x].resize(resolution_[1]);
	//	for (int y = 0; y < resolution_[1]; y++)
	//	{
	//		volume_point[x][y].resize(resolution_[2]);
	//	}
	//}
	point_cloud_whole.clear();
	point_normal_whole.clear();

	for (int i = 0; i < 30; i++)
	{
		cout << count[i] << endl;
	}
}

void RasterScan::bilateralKernel(const Mat &depth, Mat &depth_filterd, float sigma_space2_inv_half, float sigma_color2_inv_half)
{
	for (int x = 0; x < depth.cols; x++)
	{
		for (int y = 0; y < depth.rows; y++)
		{
			if (x >= depth.cols || y >= depth.rows)
				return;

			const int R = 6;       //static_cast<int>(sigma_space * 1.5);
			const int D = R * 2 + 1;

			int value = depth.at<double>(y, x);

			int tx = (x - D / 2 + D)<(depth.cols - 1) ? (x - D / 2 + D) : (depth.cols - 1);
			int ty = (y - D / 2 + D)<(depth.rows - 1) ? (y - D / 2 + D) : (depth.rows - 1);

			float sum1 = 0;
			float sum2 = 0;
			//(y-D/2)>0? (y - D / 2):0
			for (int cy = ((y - D / 2)>0 ? (y - D / 2) : 0); cy < ty; ++cy)
			{
				for (int cx = ((x - D / 2)>0 ? (x - D / 2) : 0); cx < tx; ++cx)
				{
					int tmp = depth.at<double>(cy, cx);

					float space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
					float color2 = (value - tmp) * (value - tmp);

					float weight = exp(-(space2 * sigma_space2_inv_half + color2 * sigma_color2_inv_half));

					sum1 += tmp * weight;
					sum2 += weight;
				}
			}

			float floatmax = 65526;
			float res = sum1 / sum2;
			float res2 = (res < floatmax ? res : floatmax);
			depth_filterd.at<double>(y, x) = (0>res2 ? 0 : res2);
		}
	}

	cout << "bilateral filter done" << endl;
}

void RasterScan::scaleDepth(const Mat &depth, Mat &scaled_depth)
{
	const float sigma_color = 30;     //in mm
	const float sigma_space = 4.5;     // in pixels
	Mat Dep_filterd = Mat::zeros(depth.size(), CV_64FC1);

	bilateralKernel(depth, Dep_filterd, 0.5f / (sigma_space * sigma_space), 0.5f / (sigma_color * sigma_color));

	double fx = intr2.at<double>(0, 0), fy = intr2.at<double>(1, 1), cx = intr2.at<double>(0, 2), cy = intr2.at<double>(1, 2);

	for (int x = 0; x < depth.cols; x++)
	{
		for (int y = 0; y < depth.rows; y++)
		{
			//if (x >= depth.cols() || y >= depth.rows())
			//	return;

			float Dp = depth.at<double>(y, x);

			float xl = (x - cx) / fx;
			float yl = (y - cy) / fy;
			float lambda = sqrtf(xl * xl + yl * yl + 1);

			scaled_depth.at<double>(y, x) = Dp * lambda; //meters
		}
	}
	cout << "scaleDepth Done!" << endl;
	return;
}

void RasterScan::CloseFilter(cv::Mat &depth)
{
	cv::Mat filted_image = cv::Mat::zeros(depth.size(), depth.type());
	for (int u = 1; u < depth.cols - 1; u++)
	{

		for (int v = 1; v < depth.rows - 1; v++)
		{
			double value = depth.at<double>(v, u);
			if (value == 0)
			{
				int	number_ = 0;
				double sum_ = 0;
				for (int du = u - 1; du < u + 2; du++)
				{

					for (int dv = v - 1; dv < v + 2; dv++)
					{

						double value_ = depth.at<double>(dv, du);
						if (value_ != 0)
						{
							sum_ += value_;
							number_++;
						}
					}
				}

				if (number_>4)
				{
					sum_ /= number_;
					filted_image.at<double>(v, u) = sum_;
				}
			}
			else
			{
				filted_image.at<double>(v, u) = value;
			}
		}
	}
	depth = filted_image;
}

void RasterScan::TSDF(const Mat &depth, cv::Mat &RT2 /*const Eigen::Matrix4d &RT2*/)
{

	Mat depthScaled = Mat::zeros(depth.size(), CV_64FC1);

	scaleDepth(depth, depthScaled);

	double fx = intr2.at<double>(0, 0), fy = intr2.at<double>(1, 1), cx = intr2.at<double>(0, 2), cy = intr2.at<double>(1, 2);

	//Eigen::Matrix4d RT_r = Eigen::Matrix4d::Identity();
	//RT_r(0, 0) = Rot_r.at<double>(0, 0); RT_r(0, 1) = Rot_r.at<double>(0, 1); RT_r(0, 2) = Rot_r.at<double>(0, 2); RT_r(0, 3) = tvec_r.at<double>(0, 0);
	//RT_r(1, 0) = Rot_r.at<double>(1, 0); RT_r(1, 1) = Rot_r.at<double>(1, 1); RT_r(1, 2) = Rot_r.at<double>(1, 2); RT_r(1, 3) = tvec_r.at<double>(1, 0);
	//RT_r(2, 0) = Rot_r.at<double>(2, 0); RT_r(2, 1) = Rot_r.at<double>(2, 1); RT_r(2, 2) = Rot_r.at<double>(2, 2); RT_r(2, 3) = tvec_r.at<double>(2, 0);

	//Eigen::Matrix4d RT = RT_r*RT2;
	cv::Mat RT_r = cv::Mat::eye(4, 4, CV_64FC1);
	RT_r.at<double>(0, 0) = Rot_r.at<double>(0, 0); RT_r.at<double>(0, 1) = Rot_r.at<double>(0, 1); RT_r.at<double>(0, 2) = Rot_r.at<double>(0, 2); RT_r.at<double>(0, 3) = tvec_r.at<double>(0, 0);
	RT_r.at<double>(1, 0) = Rot_r.at<double>(1, 0); RT_r.at<double>(1, 1) = Rot_r.at<double>(1, 1); RT_r.at<double>(1, 2) = Rot_r.at<double>(1, 2); RT_r.at<double>(1, 3) = tvec_r.at<double>(1, 0);
	RT_r.at<double>(2, 0) = Rot_r.at<double>(2, 0); RT_r.at<double>(2, 1) = Rot_r.at<double>(2, 1); RT_r.at<double>(2, 2) = Rot_r.at<double>(2, 2); RT_r.at<double>(2, 3) = tvec_r.at<double>(2, 0);
	cv::Mat RT = RT_r*RT2.inv();

	//double start_x = cell_size[0]*-0.5*resolution_[0], start_y = cell_size[1] *-0.5*resolution_[1], start_z = cell_size[2] * -0.5*resolution_[2];

	for (int x = 0; x < volume_Data.size(); x++)
	{
		//cout <<"volume x = "<< x << endl;
		for (int y = 0; y < volume_Data[0].size(); y++)
		{

			for (int z = 0; z < volume_Data[0][0].size(); ++z)
			{
				double v_g_x = (x + 0.5f) * cell_size[0] + start_x;
				double v_g_y = (y + 0.5f) * cell_size[1] + start_y;
				double v_g_z = (z + 0.5f) * cell_size[2] + start_z;

				//Eigen::Vector4d point_g(v_g_x, v_g_y, v_g_z, 1);
				//Eigen::Vector4d point_c = RT*point_g;
				//double v_c_x = point_c(0);
				//double v_c_y = point_c(1);
				//double v_c_z = point_c(2);

				double v_c_x = RT.at<double>(0, 0)*v_g_x + RT.at<double>(0, 1)*v_g_y + RT.at<double>(0, 2)*v_g_z + RT.at<double>(0, 3);
				double v_c_y = RT.at<double>(1, 0)*v_g_x + RT.at<double>(1, 1)*v_g_y + RT.at<double>(1, 2)*v_g_z + RT.at<double>(1, 3);
				double v_c_z = RT.at<double>(2, 0)*v_g_x + RT.at<double>(2, 1)*v_g_y + RT.at<double>(2, 2)*v_g_z + RT.at<double>(2, 3);

				double coo_u = (v_c_x*fx + v_c_z*cx) / v_c_z;
				double coo_v = (v_c_y*fy + v_c_z*cy) / v_c_z;

				double dis = sqrt(v_c_x*v_c_x + v_c_y*v_c_y + v_c_z*v_c_z);
				//cout << "------------" << endl;
				//cout << point_c << endl;
				//cout << dis << endl;
				// 将坐标投影到图像坐标
				int coo[2] =
				{
					(int)(coo_u),
					(int)(coo_v)
				};


				if (coo[0] > 0 && coo[1] > 0 && coo[0] < (depthScaled.cols - 1) && coo[1] < (depthScaled.rows - 1))        //6
				{
					int u_start_2 = (coo[0] - 1)/* > 0 ? (coo[0] - 1) : 0*/, u_end_2 = (coo[0] + 1)/* < (image_width - 1) ? (coo[0] + 1) : (image_width - 1)*/;
					int v_start_2 = (coo[1] - 1)/* > 0 ? (coo[1] - 1) : 0*/, v_end_2 = (coo[1] + 1)/* < (image_height - 1) ? (coo[1] + 1) : (image_height - 1)*/;

					int good_k = 0;
					vector<Eigen::VectorXd> uvpoints;
					for (int U = u_start_2; U <= u_end_2; U++)
					{
						for (int V = v_start_2; V <= v_end_2; V++)
						{
							double Dp_scaled2 = depthScaled.at<double>(V, U);
							if (Dp_scaled2 == 0)
							{
								continue;
							}
							Eigen::VectorXd uvpoint(3);
							uvpoint[0] = U;
							uvpoint[1] = V;
							uvpoint[2] = Dp_scaled2;
							uvpoints.push_back(uvpoint);
							good_k++;
						}
					}
					if (good_k<3)
					{
						continue;
					}
					Eigen::Vector4d plane;
					CalculatePlane(uvpoints, plane);
					double Dp_scaled = -1 * (plane[0] * coo_u + plane[1] * coo_v + plane[3]) / plane[2];

					//double sum_d = 0;
					//int sum_i = 0;
					//for (int U = u_start_2; U <= u_end_2; U++)
					//{
					//	for (int V = v_start_2; V <= v_end_2; V++)
					//	{
					//		double tm = depthScaled.at<double>(V, U);
					//		if (tm!=0)
					//		{
					//			sum_d += depthScaled.at<double>(V, U);
					//			sum_i++;
					//		}

					//	}
					//}
					//double Dp_scaled = 0;
					//if(sum_i)
					//{
					//	Dp_scaled = sum_d / sum_i;
					//}

					//double tm0 = depthScaled.at<double>(coo[1], coo[0]);
					//double tm1 = depthScaled.at<double>(v_start_2, u_start_2);
					//if (tm1 == 0) continue;
					//double tm2 = depthScaled.at<double>(v_start_2, u_end_2);
					//if (tm2 == 0) continue;
					//double tm3 = depthScaled.at<double>(v_end_2, u_start_2);
					//if (tm3 == 0) continue;
					//double tm4 = depthScaled.at<double>(v_end_2, u_end_2);
					//if (tm4 == 0) continue;

					//double m41x = 2, m41y = 2, m41z = tm4 - tm1;
					//double m23x = 2, m23y = -2, m23z = tm2 - tm3;
					//double a = (m41y * m23z) - (m41z * m23y);
					//double b = (m41x * m23z) - (m41z * m23x);
					//double c = (m41x * m23y) - (m41y * m23x);
					//double abc = a*a + b*b + c*c;
					//double a2 = a / sqrt(abc);
					//double b2 = b / sqrt(abc);
					//double c2 = c / sqrt(abc);
					//double d2 = -1 * a2*coo[0] - b2*coo[1] - c2*tm0;
					//double Dp_scaled = -1 * (a2 * coo_u + b2 * coo_v + d2) / c2;

					//深度尺度
					//double Dp_scaled = depthScaled.at<double>(coo[1], coo[0]); //meters
					if (Dp_scaled == 0)
					{
						continue;
					}
					//深度尺度与截断数据误差
					double sdf = Dp_scaled - dis;
					//if (abs(sdf)<5)
					//{
					//	cout << "scaled:" << Dp_scaled << " dis:" << dis << endl;
					//}

					if (Dp_scaled != 0 && sdf >= -tranc_dist_ && sdf <= tranc_dist_) //meters
					{
						double tsdf = fmin(1.0f, sdf / tranc_dist_);
						double tsdf_prev = volume_Data[x][y][z].distance_;
						int weight_prev = volume_Data[x][y][z].weight_;
						const int Wrk = 1;


						volume_Data[x][y][z].distance_ = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
						volume_Data[x][y][z].weight_ = weight_prev + Wrk > 128 ? 128 : weight_prev + Wrk;

					}
				}
			}       // for(int z = 0; z < VOLUME_Z; ++z)
		}
	}
	cout << "tsdf done!" << endl;

	//vector<double> points4;
	//for (int x = 0; x < volume_Data.size(); x++)
	//{

	//	for (int y = 0; y < volume_Data[0].size(); y++)
	//	{

	//		for (int z = 0; z < volume_Data[0][0].size(); ++z)
	//		{
	//			double v_g_x = (x + 0.5f) * cell_size;
	//			double v_g_y = (y + 0.5f) * cell_size;
	//			double v_g_z = (0 + 0.5f) * cell_size * -1;

	//			if (true)
	//			{

	//			}
	//		}
	//	}
	//}
}

float RasterScan::TSDFGaussCore(int x, int y, int z)
{
	int size;
	size = GaussTemplate.size();

	double sum = 0, sum2 = 0;
	int x_start = x - size / 2;
	int y_start = y - size / 2;
	int z_start = z - size / 2;
	for (int i = 0; i < size; i++)
	{

		for (int j = 0; j < size; j++)
		{

			for (int k = 0; k < size; k++)
			{
				float dis = 0, W = 0;
				dis = volume_Data[x_start + i][y_start + j][z_start + k].distance_; W = GaussTemplate[i][j][k];

				if (dis != 0)
				{
					sum += dis*W;
					sum2 += W;
				}
			}
		}
	}
	float data;
	if (sum2 != 0)
		data = sum / sum2;
	else
		data = 0;

	return data;
}

void RasterScan::TSDFGaussFilter()
{
	int radius = GaussTemplate.size() / 2;

	int XStart = resolution_[0];
	int XEnd = 0;
	int YStart = resolution_[1];
	int YEnd = 0;
	int ZStart = resolution_[2];
	int ZEnd = 0;

	for (int x = 0; x < resolution_[0]; x++)
	{
		for (int y = 0; y < resolution_[1]; y++)
		{
			for (int z = 0; z < resolution_[2]; z++)
			{
				if (volume_Data[x][y][z].distance_ != 0)
				{
					//cout << "," << endl;
					if (XStart>x)
					{
						XStart = x;
					}
					if (XEnd<x)
					{
						XEnd = x;
					}

					if (YStart>y)
					{
						YStart = y;
					}
					if (YEnd<y)
					{
						YEnd = y;
					}

					if (ZStart>z)
					{
						ZStart = z;
					}
					if (ZEnd<z)
					{
						ZEnd = z;
					}
				}

			}
		}
	}

	XStart -= radius; XStart = XStart<(radius) ? (radius) : XStart;
	XEnd += radius; XEnd = XEnd>(resolution_[0] - radius) ? (resolution_[0] - radius) : XEnd;
	YStart -= radius; YStart = YStart<(radius) ? (radius) : YStart;
	YEnd += radius; YEnd = YEnd>(resolution_[1] - radius) ? (resolution_[1] - radius) : YEnd;
	ZStart -= radius; ZStart = ZStart<(radius) ? (radius) : ZStart;
	ZEnd += radius; ZEnd = ZEnd>(resolution_[2] - radius) ? (resolution_[2] - radius) : ZEnd;

	Eigen::Vector3i filter_start(XStart, YStart, ZStart);
	Eigen::Vector3i filter_end(XEnd, YEnd, ZEnd);

	Volume GaussTSDF;
	GaussTSDF.resize(resolution_[0]);
	for (int x = filter_start(0); x < filter_end(0); x++)
	{
		GaussTSDF[x].resize(resolution_[1]);
		for (int y = filter_start(1); y < filter_end(1); y++)
		{
			GaussTSDF[x][y].resize(resolution_[2]);
			for (int z = filter_start(2); z < filter_end(2); z++)
			{
				GaussTSDF[x][y][z].distance_ = TSDFGaussCore(x, y, z);
			}
		}
	}

	for (int x = filter_start(0); x < filter_end(0); x++)
	{
		for (int y = filter_start(1); y < filter_end(1); y++)
		{
			for (int z = filter_start(2); z < filter_end(2); z++)
			{
				volume_Data[x][y][z].distance_ = GaussTSDF[x][y][z].distance_;
			}
		}
	}

}

void RasterScan::MarchingCube()
{

	verts.clear();
	norms.clear();
	vertexIndicies.clear();

	meshedge.resize(resolution_[0]);//---------------------- mesh edge
	int Redundant_num = 0;

	for (int x = 0; x < (resolution_[0]); x++)
	{
		meshedge[x].resize(resolution_[1]);
		for (int y = 0; y < (resolution_[1]); y++)
		{
			meshedge[x][y].resize(resolution_[2]);
			for (int z = 0; z < resolution_[2]; z++)
			{
				meshedge[x][y][z].edge1_node_ = -1;
				meshedge[x][y][z].edge2_node_ = -1;
				meshedge[x][y][z].edge3_node_ = -1;
			}
		}
	}

	for (int x = 0; x < (resolution_[0] - 1); x++)
	{

		for (int y = 0; y < (resolution_[1] - 1); y++)
		{

			for (int z = 0; z < (resolution_[2] - 1); z++)
			{
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}
				float f[8], tt[12];
				int cubeindex = computeCubeIndex(x, y, z, f);

				if (cubeindex == -1 || cubeindex == 0)
				{
					continue;
				}
				// calculate cell vertex positions
				std::vector<Eigen::Vector3f> V(8);
				V[0] = getNodeCoo(x, y, z);
				V[1] = getNodeCoo(x + 1, y, z);
				V[2] = getNodeCoo(x + 1, y + 1, z);
				V[3] = getNodeCoo(x, y + 1, z);
				V[4] = getNodeCoo(x, y, z + 1);
				V[5] = getNodeCoo(x + 1, y, z + 1);
				V[6] = getNodeCoo(x + 1, y + 1, z + 1);
				V[7] = getNodeCoo(x, y + 1, z + 1);

				// find the vertices where the surface intersects the cube
				// use shared memory to avoid using local
				std::vector<Eigen::Vector3f> vertlist(12);

				vertlist[0] = vertex_interp(V[0], V[1], f[0], f[1], tt[0]);
				vertlist[1] = vertex_interp(V[1], V[2], f[1], f[2], tt[1]);
				vertlist[2] = vertex_interp(V[2], V[3], f[2], f[3], tt[2]);
				vertlist[3] = vertex_interp(V[3], V[0], f[3], f[0], tt[3]);
				vertlist[4] = vertex_interp(V[4], V[5], f[4], f[5], tt[4]);
				vertlist[5] = vertex_interp(V[5], V[6], f[5], f[6], tt[5]);
				vertlist[6] = vertex_interp(V[6], V[7], f[6], f[7], tt[6]);
				vertlist[7] = vertex_interp(V[7], V[4], f[7], f[4], tt[7]);
				vertlist[8] = vertex_interp(V[0], V[4], f[0], f[4], tt[8]);
				vertlist[9] = vertex_interp(V[1], V[5], f[1], f[5], tt[9]);
				vertlist[10] = vertex_interp(V[2], V[6], f[2], f[6], tt[10]);
				vertlist[11] = vertex_interp(V[3], V[7], f[3], f[7], tt[11]);

				// output triangle vertices
				int numVerts = numVertsTable_[cubeindex];

				for (int i = 0; i < numVerts; i++)
				{
					int TotleVertsIndex = verts.size() / 3;
					int LocalEdgeIndex = triTable_[cubeindex][i];

					switch (LocalEdgeIndex)
					{
					case 0:
					{
						if (meshedge[x][y][z].edge1_node_ == -1)
							meshedge[x][y][z].edge1_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y][z].edge1_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 1:
					{
						if (meshedge[x + 1][y][z].edge2_node_ == -1)
							meshedge[x + 1][y][z].edge2_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x + 1][y][z].edge2_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 2:
					{
						if (meshedge[x][y + 1][z].edge1_node_ == -1)
							meshedge[x][y + 1][z].edge1_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y + 1][z].edge1_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 3:
					{
						if (meshedge[x][y][z].edge2_node_ == -1)
							meshedge[x][y][z].edge2_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y][z].edge2_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 4:
					{
						if (meshedge[x][y][z + 1].edge1_node_ == -1)
							meshedge[x][y][z + 1].edge1_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y][z + 1].edge1_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 5:
					{
						if (meshedge[x + 1][y][z + 1].edge2_node_ == -1)
							meshedge[x + 1][y][z + 1].edge2_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x + 1][y][z + 1].edge2_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 6:
					{
						if (meshedge[x][(y + 1)][z + 1].edge1_node_ == -1)
							meshedge[x][(y + 1)][z + 1].edge1_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][(y + 1)][z + 1].edge1_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 7:
					{
						if (meshedge[x][(y)][z + 1].edge2_node_ == -1)
							meshedge[x][(y)][z + 1].edge2_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][(y)][z + 1].edge2_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 8:
					{
						if (meshedge[x][y][z].edge3_node_ == -1)
							meshedge[x][y][z].edge3_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y][z].edge3_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 9:
					{
						if (meshedge[(x + 1)][y][z].edge3_node_ == -1)
							meshedge[(x + 1)][y][z].edge3_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[(x + 1)][y][z].edge3_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 10:
					{
						if (meshedge[(x + 1)][y + 1][z].edge3_node_ == -1)
							meshedge[(x + 1)][y + 1][z].edge3_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[(x + 1)][y + 1][z].edge3_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					case 11:
					{
						if (meshedge[x][y + 1][z].edge3_node_ == -1)
							meshedge[x][y + 1][z].edge3_node_ = TotleVertsIndex;
						else
						{
							vertexIndicies.push_back(meshedge[x][y + 1][z].edge3_node_);
							Redundant_num++;
							continue;
						}
						break;
					}
					default:
						break;
					}

					Eigen::Vector3f point;
					for (int xyz = 0; xyz < 3; xyz++)
					{
						verts.push_back(vertlist[LocalEdgeIndex](xyz));

						//if (tt[LocalEdgeIndex]>2)
						//{
						//	cout << point << endl;
						//}
						point(xyz) = vertlist[LocalEdgeIndex](xyz);

					}

					Eigen::Vector3f normal = MeshNormalCalculate(point);
					for (int xyz = 0; xyz < 3; xyz++)
					{
						norms.push_back(normal(xyz));
					}

					vertexIndicies.push_back(TotleVertsIndex);

				}

				//if (verts.size()>750000)
				//{
				//	return;
				//}
			}
		}
	}
	meshedge.clear();
	cout << "marching cube vertex number = " << verts.size() / 3 << "; Redundant number = " << Redundant_num << endl;
}

void RasterScan::MarchingCube2()
{

	verts.clear();
	norms.clear();
	vertexIndicies.clear();

	for (int x = 1; x < (resolution_[0] - 2); x++)
	{

		for (int y = 1; y < (resolution_[1] - 2); y++)
		{

			for (int z = 1; z < (resolution_[2] - 2); z++)
			{

				float f[8], tt[12];
				int cubeindex = computeCubeIndex(x, y, z, f);

				if (cubeindex == -1 || cubeindex == 0)
				{
					continue;
				}
				// calculate cell vertex positions
				std::vector<Eigen::Vector3f> V(8);
				V[0] = getNodeCoo(x, y, z);
				V[1] = getNodeCoo(x + 1, y, z);
				V[2] = getNodeCoo(x + 1, y + 1, z);
				V[3] = getNodeCoo(x, y + 1, z);
				V[4] = getNodeCoo(x, y, z + 1);
				V[5] = getNodeCoo(x + 1, y, z + 1);
				V[6] = getNodeCoo(x + 1, y + 1, z + 1);
				V[7] = getNodeCoo(x, y + 1, z + 1);

				// find the vertices where the surface intersects the cube
				// use shared memory to avoid using local
				std::vector<Eigen::Vector3f> vertlist(12);

				vertlist[0] = vertex_interp(V[0], V[1], f[0], f[1], tt[0]);
				vertlist[1] = vertex_interp(V[1], V[2], f[1], f[2], tt[1]);
				vertlist[2] = vertex_interp(V[2], V[3], f[2], f[3], tt[2]);
				vertlist[3] = vertex_interp(V[3], V[0], f[3], f[0], tt[3]);
				vertlist[4] = vertex_interp(V[4], V[5], f[4], f[5], tt[4]);
				vertlist[5] = vertex_interp(V[5], V[6], f[5], f[6], tt[5]);
				vertlist[6] = vertex_interp(V[6], V[7], f[6], f[7], tt[6]);
				vertlist[7] = vertex_interp(V[7], V[4], f[7], f[4], tt[7]);
				vertlist[8] = vertex_interp(V[0], V[4], f[0], f[4], tt[8]);
				vertlist[9] = vertex_interp(V[1], V[5], f[1], f[5], tt[9]);
				vertlist[10] = vertex_interp(V[2], V[6], f[2], f[6], tt[10]);
				vertlist[11] = vertex_interp(V[3], V[7], f[3], f[7], tt[11]);

				// output triangle vertices
				int numVerts = numVertsTable_[cubeindex];

				for (int i = 0; i < numVerts; i++)
				{
					int TotleVertsIndex = verts.size() / 3;
					int LocalEdgeIndex = triTable_[cubeindex][i];

					Eigen::Vector3f point;
					for (int xyz = 0; xyz < 3; xyz++)
					{
						verts.push_back(vertlist[LocalEdgeIndex](xyz));
						point(xyz) = vertlist[LocalEdgeIndex](xyz);

					}

					Eigen::Vector3f normal = MeshNormalCalculate(point);
					for (int xyz = 0; xyz < 3; xyz++)
					{
						norms.push_back(normal(xyz));
					}

					vertexIndicies.push_back(TotleVertsIndex);

				}

				//if (verts.size()>750000)
				//{
				//	return;
				//}
			}
		}
	}
	meshedge.clear();
}

int RasterScan::computeCubeIndex(int x, int y, int z, float f[8]) const
{
	int weight;
	readTsdf(x, y, z, f[0]); if (f[0] == 0) return -1;
	readTsdf(x + 1, y, z, f[1]); if (f[1] == 0) return -1;
	readTsdf(x + 1, y + 1, z, f[2]); if (f[2] == 0) return -1;
	readTsdf(x, y + 1, z, f[3]); if (f[3] == 0) return -1;
	readTsdf(x, y, z + 1, f[4]); if (f[4] == 0) return -1;
	readTsdf(x + 1, y, z + 1, f[5]); if (f[5] == 0) return -1;
	readTsdf(x + 1, y + 1, z + 1, f[6]); if (f[6] == 0) return -1;
	readTsdf(x, y + 1, z + 1, f[7]); if (f[7] == 0) return -1;

	// calculate flag indicating if each vertex is inside or outside isosurface
	int cubeindex;
	cubeindex = int(f[0] < isoValue);
	cubeindex += int(f[1] < isoValue) * 2;
	cubeindex += int(f[2] < isoValue) * 4;
	cubeindex += int(f[3] < isoValue) * 8;
	cubeindex += int(f[4] < isoValue) * 16;
	cubeindex += int(f[5] < isoValue) * 32;
	cubeindex += int(f[6] < isoValue) * 64;
	cubeindex += int(f[7] < isoValue) * 128;

	return cubeindex;
}

void RasterScan::readTsdf(int x, int y, int z, float& tsdf, int& weight) const
{
	tsdf = volume_Data[x][y][z].distance_;
	weight = volume_Data[x][y][z].weight_;
}

void RasterScan::readTsdf(int x, int y, int z, float& tsdf) const
{
	tsdf = volume_Data2[x*resolution_[1] * resolution_[2] + y*resolution_[2] + z];
}

Eigen::Vector3f RasterScan::getNodeCoo(int x, int y, int z)
{
	Eigen::Vector3f coo(0, 0, 0); //shift to volume cell center;

	coo(0) = cell_size[0] * (x + 0.5f);
	coo(1) = cell_size[1] * (y + 0.5f);
	coo(2) = cell_size[2] * (z + 0.5f);

	return coo;
}

Eigen::Vector3f RasterScan::vertex_interp(Eigen::Vector3f &p0, Eigen::Vector3f &p1, float f0, float f1, float &t2)
{
	//cout << "123 = " << f1 - f0 << endl;
	float t = (isoValue - f0) / (f1 - f0 + 1e-15f);
	//t = 0.5;
	float x = p0(0) + t * (p1(0) - p0(0));
	float y = p0(1) + t * (p1(1) - p0(1));
	float z = p0(2) + t * (p1(2) - p0(2));

	t2 = t;
	Eigen::Vector3f ver(x, y, z);
	return ver;
}

Eigen::Vector3f RasterScan::MeshNormalCalculate(const Eigen::Vector3f &point)
{

	//Eigen::Vector3f point;//---------------------- 空间中任意一点 
	Eigen::Vector3f t = point;
	Eigen::Vector3f n;

	t(0) += cell_size[0];
	float Fx1 = interpolateTrilineary(t);

	t = point;
	t(0) -= cell_size[0];
	float Fx2 = interpolateTrilineary(t);

	n(0) = (Fx1 - Fx2);

	t = point;
	t(1) += cell_size[1];
	float Fy1 = interpolateTrilineary(t);

	t = point;
	t(1) -= cell_size[1];
	float Fy2 = interpolateTrilineary(t);

	n(1) = (Fy1 - Fy2);

	t = point;
	t(2) += cell_size[2];
	float Fz1 = interpolateTrilineary(t);

	t = point;
	t(2) -= cell_size[2];
	float Fz2 = interpolateTrilineary(t);

	n(2) = (Fz1 - Fz2);
	n.normalize();
	return n;

}

float RasterScan::interpolateTrilineary(const Eigen::Vector3f &point)
{
	Eigen::Vector3i g = getVoxel(point);

	if (g(0) <= 0 || g(0) >= resolution_[0] - 1)
		return 0;

	if (g(1) <= 0 || g(1) >= resolution_[1] - 1)
		return 0;

	if (g(2) <= 0 || g(2) >= resolution_[2] - 1)
		return 0;

	float vx = (g(0) + 0.5f) * cell_size[0];
	float vy = (g(1) + 0.5f) * cell_size[1];
	float vz = (g(2) + 0.5f) * cell_size[2];

	g(0) = (point(0) < vx) ? (g(0) - 1) : g(0);
	g(1) = (point(1) < vy) ? (g(1) - 1) : g(1);
	g(2) = (point(2) < vz) ? (g(2) - 1) : g(2);

	float a = (point(0) - (g(0) + 0.5f) * cell_size[0]) / cell_size[0];
	float b = (point(1) - (g(1) + 0.5f) * cell_size[1]) / cell_size[1];
	float c = (point(2) - (g(2) + 0.5f) * cell_size[2]) / cell_size[2];
	float res1 = 0, res2 = 0, res3 = 0, res4 = 0, res5 = 0, res6 = 0, res7 = 0, res8 = 0, res = 0;
	readTsdf(g(0) + 0, g(1) + 0, g(2) + 0, res1);
	readTsdf(g(0) + 0, g(1) + 0, g(2) + 1, res2);
	readTsdf(g(0) + 0, g(1) + 1, g(2) + 0, res3);
	readTsdf(g(0) + 0, g(1) + 1, g(2) + 1, res4);
	readTsdf(g(0) + 1, g(1) + 0, g(2) + 0, res5);
	readTsdf(g(0) + 1, g(1) + 0, g(2) + 1, res6);
	readTsdf(g(0) + 1, g(1) + 1, g(2) + 0, res7);
	readTsdf(g(0) + 1, g(1) + 1, g(2) + 1, res8);
	res = res1*(1 - a) * (1 - b) * (1 - c) + res2*(1 - a) * (1 - b) * c + res3*(1 - a) * b * (1 - c) + res4*(1 - a) * b * c + res5*a * (1 - b) * (1 - c) + res6*a * (1 - b) * c + res7*a * b * (1 - c) + res8*a * b * c;
	return res;
}

Eigen::Vector3i RasterScan::getVoxel(const Eigen::Vector3f point)const
{
	int vx = (int)((point(0) - start_x) / cell_size[0]);        // round to negative infinity
	int vy = (int)((point(1) - start_y) / cell_size[1]);
	int vz = (int)((point(2) - start_z) / cell_size[2]);
	vx = fmin(resolution_[0] - 1, fmax(0, vx));
	vy = fmin(resolution_[1] - 1, fmax(0, vy));
	vz = fmin(resolution_[2] - 1, fmax(0, vz));
	Eigen::Vector3i Voxel(vx, vy, vz);
	return Voxel;
}

void RasterScan::Calculate3DMaskPoints(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, vector<Eigen::VectorXd> &points_end2) {

	points_end2.clear();
	vector<Eigen::VectorXd> points_end;
	//vector<float> new_points;

	for (int point_index = 0; point_index < points1.size(); point_index++)
	{
		Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
		Eigen::VectorXd B_ = Eigen::VectorXd(4);
		Eigen::VectorXd X_ = Eigen::VectorXd(3);

		double u1 = points1[point_index].x; double v1 = points1[point_index].y;
		double u2 = points2[point_index].x; double v2 = points2[point_index].y;
		double r1 = Rot_l.at<double>(0, 0), r2 = Rot_l.at<double>(0, 1), r3 = Rot_l.at<double>(0, 2),
			r4 = Rot_l.at<double>(1, 0), r5 = Rot_l.at<double>(1, 1), r6 = Rot_l.at<double>(1, 2),
			r7 = Rot_l.at<double>(2, 0), r8 = Rot_l.at<double>(2, 1), r9 = Rot_l.at<double>(2, 2);
		double t1 = tvec_l.at<double>(0), t2 = tvec_l.at<double>(1), t3 = tvec_l.at<double>(2);
		double fx1 = intr1.at<double>(0, 0), fy1 = intr1.at<double>(1, 1), cx1 = intr1.at<double>(0, 2), cy1 = intr1.at<double>(1, 2);
		double r_1 = Rot_r.at<double>(0, 0), r_2 = Rot_r.at<double>(0, 1), r_3 = Rot_r.at<double>(0, 2),
			r_4 = Rot_r.at<double>(1, 0), r_5 = Rot_r.at<double>(1, 1), r_6 = Rot_r.at<double>(1, 2),
			r_7 = Rot_r.at<double>(2, 0), r_8 = Rot_r.at<double>(2, 1), r_9 = Rot_r.at<double>(2, 2);
		double t_1 = tvec_r.at<double>(0), t_2 = tvec_r.at<double>(1), t_3 = tvec_r.at<double>(2);
		double fx2 = intr2.at<double>(0, 0), fy2 = intr2.at<double>(1, 1), cx2 = intr2.at<double>(0, 2), cy2 = intr2.at<double>(1, 2);

		A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
		A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

		A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
		A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;


		X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);

		double x = X_(0), y = X_(1), z = X_(2);
		//double dis = sqrt(x*x + y*y + z*z);
		//if (z>pix_error)
		//{
		//	continue;
		//}
		points.push_back(x);
		points.push_back(y);
		points.push_back(z);

		points_end2.push_back(X_);
	}

	//Eigen::Vector4d Plane;
	//CalculatePlane(points_end, Plane);

	//double D = Plane(3);
	//double A = Plane(0);
	//double B = Plane(1);
	//double C = Plane(2);

	//for (int point_index = 0; point_index < points_end.size(); point_index++)
	//{
	//	double xi = points_end[point_index](0), yi = points_end[point_index](1), zi = points_end[point_index](2);
	//	double t = (xi*A + yi*B + zi*C + D) / (A*A + B*B + C*C);
	//	double x = xi - A*t;
	//	double y = yi - B*t;
	//	double z = zi - C*t;
	//	Eigen::Vector3d np(x, y, z);
	//	double distance = Distance(xi,yi,zi,x,y,z);
	//	if (distance>5)
	//	{
	//		continue;
	//	}
	//	points_end2.push_back(np);
	//	//cout << np << endl;
	//}

}

void RasterScan::Calculate3DPointsEvaluate(vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &points, vector<double> &new_points) {

	vector<Eigen::VectorXd> points_end;
	vector<Eigen::VectorXd> points_end2;
	//vector<float> new_points;

	for (int point_index = 0; point_index < points1.size(); point_index++)
	{
		Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
		Eigen::VectorXd B_ = Eigen::VectorXd(4);
		Eigen::VectorXd X_ = Eigen::VectorXd(3);

		double u1 = points1[point_index].x; double v1 = points1[point_index].y;
		double u2 = points2[point_index].x; double v2 = points2[point_index].y;
		double r1 = Rot_l.at<double>(0, 0), r2 = Rot_l.at<double>(0, 1), r3 = Rot_l.at<double>(0, 2),
			r4 = Rot_l.at<double>(1, 0), r5 = Rot_l.at<double>(1, 1), r6 = Rot_l.at<double>(1, 2),
			r7 = Rot_l.at<double>(2, 0), r8 = Rot_l.at<double>(2, 1), r9 = Rot_l.at<double>(2, 2);
		double t1 = tvec_l.at<double>(0), t2 = tvec_l.at<double>(1), t3 = tvec_l.at<double>(2);
		double fx1 = intr1.at<double>(0, 0), fy1 = intr1.at<double>(1, 1), cx1 = intr1.at<double>(0, 2), cy1 = intr1.at<double>(1, 2);
		double r_1 = Rot_r.at<double>(0, 0), r_2 = Rot_r.at<double>(0, 1), r_3 = Rot_r.at<double>(0, 2),
			r_4 = Rot_r.at<double>(1, 0), r_5 = Rot_r.at<double>(1, 1), r_6 = Rot_r.at<double>(1, 2),
			r_7 = Rot_r.at<double>(2, 0), r_8 = Rot_r.at<double>(2, 1), r_9 = Rot_r.at<double>(2, 2);
		double t_1 = tvec_r.at<double>(0), t_2 = tvec_r.at<double>(1), t_3 = tvec_r.at<double>(2);
		double fx2 = intr2.at<double>(0, 0), fy2 = intr2.at<double>(1, 1), cx2 = intr2.at<double>(0, 2), cy2 = intr2.at<double>(1, 2);

		A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
		A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

		A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
		A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;

		//cout << "The least-squares solution is:\n"
		//	<< A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_)<<endl;
		X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);
		//cout << X_ << endl;
		//if (X_(2)>1500 || X_(2)<600)
		//{
		//	continue;
		//}
		double x = X_(0), y = X_(1), z = X_(2);

		points.push_back(x);
		points.push_back(y);
		points.push_back(z);

		points_end.push_back(X_);
	}

	//Eigen::Vector4d Plane;
	//CalculatePlane(points_end, Plane);

	//double D = Plane(3);
	//double A = Plane(0);
	//double B = Plane(1);
	//double C = Plane(2);
	//double plane_sq = sqrt(A*A + B*B + C*C);
	//vector<Eigen::VectorXd> points_1;
	//for (int point_indexx = 0; point_indexx < points_end.size(); point_indexx++)
	//{
	//	double dis_point_plane = abs(points_end[point_indexx](0)*A + points_end[point_indexx](1)*B + points_end[point_indexx](2)*C + D) / plane_sq;

	//	if (dis_point_plane<80)
	//	{
	//		points_1.push_back(points_end[point_indexx]);
	//	}
	//}

	//CalculatePlane(points_end, Plane);
	//D = Plane(3);
	//A = Plane(0);
	//B = Plane(1);
	//C = Plane(2);
	//plane_sq = sqrt(A*A + B*B + C*C);
	//vector<Eigen::VectorXd> points_2;
	//for (int point_indexx = 0; point_indexx < points_end.size(); point_indexx++)
	//{
	//	double dis_point_plane = abs(points_end[point_indexx](0)*A + points_end[point_indexx](1)*B + points_end[point_indexx](2)*C + D) / plane_sq;

	//	if (dis_point_plane<20)
	//	{
	//		points_2.push_back(points_end[point_indexx]);
	//	}
	//}

	//CalculatePlane(points_2, Plane);
	//D = Plane(3);
	//A = Plane(0);
	//B = Plane(1);
	//C = Plane(2);
	//for (int point_index = 0; point_index < points_2.size(); point_index++)
	//{
	//	double xi = points_2[point_index](0), yi = points_2[point_index](1), zi = points_2[point_index](2);
	//	double t = (xi*A + yi*B + zi*C + D) / (A*A + B*B + C*C);
	//	double x = xi - A*t;
	//	double y = yi - B*t;
	//	double z = zi - C*t;
	//	new_points.push_back(x / 1000);
	//	new_points.push_back(y / 1000);
	//	new_points.push_back(z / 1000);
	//	Eigen::Vector3d np(x, y, z);
	//	points_end2.push_back(np);
	//	//cout << np << endl;
	//}
	double mean_distance = 0;
	int number = 0;
	for (int point_index = 1; point_index < points_end.size(); point_index++)
	{
		stringstream ss1, ss2;
		ss1 << point_index - 1;
		ss2 << point_index;
		string s1, s2;
		ss1 >> s1;
		ss2 >> s2;
		Eigen::VectorXd d_ = points_end[point_index - 1] - points_end[point_index];
		double distance = sqrt(d_.transpose()*d_);

		if (distance<70)
		{
			mean_distance += distance;
			number++;
			cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
		}

	}
	mean_distance = mean_distance / number;
	cout << "mean distance " << mean_distance << endl;

	//for (int point_index = 1; point_index < points_end2.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end2[point_index - 1] - points_end2[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}


	/*------------------------------------------------ Registration ----------------------------------------*/
	//Eigen::Vector4d Plane;
	//CalculatePlane(points_end, Plane);

	//double D = Plane(3);
	//double A = Plane(0);
	//double B = Plane(1);
	//double C = Plane(2);
	////cout << " A = " << A << " B = " << B << " C = " << C << " D = " << D<< endl;

	//for (int point_index = 0; point_index < points_end.size(); point_index++)
	//{
	//	double xi = points_end[point_index](0), yi = points_end[point_index](1), zi = points_end[point_index](2);
	//	double t = (xi*A + yi*B + zi*C + D)/ (A*A + B*B + C*C);
	//	double x = xi - A*t;
	//	double y = yi - B*t;
	//	double z = zi - C*t;
	//	new_points.push_back(x / 1000);
	//	new_points.push_back(y / 1000);
	//	new_points.push_back(z / 1000);
	//	Eigen::Vector3d np(x, y, z);
	//	points_end2.push_back(np);
	//	//cout << np << endl;
	//}

	//for (int point_index = 1; point_index < points_end.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end[point_index - 1] - points_end[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}

	//for (int point_index = 1; point_index < points_end2.size(); point_index++)
	//{
	//	stringstream ss1, ss2;
	//	ss1 << point_index - 1;
	//	ss2 << point_index;
	//	string s1, s2;
	//	ss1 >> s1;
	//	ss2 >> s2;
	//	Eigen::VectorXd d_ = points_end2[point_index - 1] - points_end2[point_index];
	//	double distance = sqrt(d_.transpose()*d_);
	//	//cout << "distance " << s1 << "-" << s2 << " : " << distance << endl;
	//}

}

void RasterScan::CalculatePlane(const vector<Eigen::VectorXd> &Points, Eigen::Vector4d &Plane)
{
	Plane = Eigen::Vector4d::Zero();
	Eigen::MatrixXd A_plane = Eigen::MatrixXd(Points.size(), 3);
	Eigen::VectorXd B_plane = Eigen::VectorXd(Points.size());
	Eigen::VectorXd X_plane = Eigen::VectorXd(3);

	for (int point_index = 0; point_index < Points.size(); point_index++)
	{
		A_plane(point_index, 0) = Points[point_index](0);
		A_plane(point_index, 1) = Points[point_index](1);
		A_plane(point_index, 2) = Points[point_index](2);
		B_plane(point_index) = -1;
	}

	X_plane = A_plane.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_plane);
	//cout << "Aplane : " << A_plane << endl;
	//cout << "Xplane : " << X_plane << endl;
	double M = (X_plane(0)*X_plane(0) + X_plane(1)*X_plane(1) + X_plane(2)*X_plane(2));
	Plane(3) = sqrt(1 / M);
	Plane(0) = X_plane(0)*Plane(3);
	Plane(1) = X_plane(1)*Plane(3);
	Plane(2) = X_plane(2)*Plane(3);
}

std::vector<cv::Point2f> RasterScan::Generate2DPoints()
{
	std::vector<cv::Point2f> points;

	float x, y;

	x = 282; y = 274;
	points.push_back(cv::Point2f(x, y));

	x = 397; y = 227;
	points.push_back(cv::Point2f(x, y));

	x = 577; y = 271;
	points.push_back(cv::Point2f(x, y));

	x = 462; y = 318;
	points.push_back(cv::Point2f(x, y));

	x = 270; y = 479;
	points.push_back(cv::Point2f(x, y));

	x = 450; y = 523;
	points.push_back(cv::Point2f(x, y));

	x = 566; y = 475;
	points.push_back(cv::Point2f(x, y));

	for (unsigned int i = 0; i < points.size(); ++i)
	{
		std::cout << points[i] << std::endl;
	}

	return points;
}

std::vector<cv::Point3f> RasterScan::Generate3DPoints()
{
	std::vector<cv::Point3f> points;


	float x, y, z;
	x = 0; y = 0; z = 0;
	for (int i = 0; i < 7; i++, y += 15)
	{
		x = 0;
		for (int j = 0; j < 5; j++, x += 15)
		{
			points.push_back(cv::Point3f(x, y, z));
		}
	}
	return points;
}

Mat RasterScan::Distortion(Mat& image, bool flag)
{
	Mat intrinsic_matrix, distortion_coeffs;
	if (flag)
	{
		intrinsic_matrix = intr2;
		distortion_coeffs = distCoeffs[1];
	}
	else
	{
		intrinsic_matrix = intr1;
		distortion_coeffs = distCoeffs[0];
	}


	cv::Size image_size = image.size();
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
	Mat t = image.clone();
	cv::remap(image, t, mapx, mapy, cv::INTER_LINEAR);
	return t;
}

//void FindLaserPoints(const Mat &LaserImage, vector<cv::Point2f> &points)
//{
//	Mat Gray;
//	cvtColor(LaserImage, Gray, CV_BGR2GRAY);
//	int width = LaserImage.cols;
//	int height = LaserImage.rows;
//	int low = 30;// parser.get<int>("low");
//	int high = 100;// parser.get<int>("high");
//	double alpha = 1.0;// parser.get<double>("alpha");
//	int mode = 1;// parser.get<int>("mode");
//	Mat img_show;
//	LaserImage.copyTo(img_show);
//	cv::Vec3b color1(255, 10, 10);
//
//	vector<Contour> contours;
//	vector<cv::Vec4i> hierarchy;
//	EdgesSubPix(Gray, alpha, low, high, contours, hierarchy, mode);
//	//cout << contours.size() << endl;
//	vector<vector<cv::Point2f>> row_point;
//	row_point.resize(height);
//	for (int i = 0; i < contours.size(); i++)
//	{
//		for (int j = 0; j < contours[i].points.size(); j++)
//		{
//			int rownum = floor(contours[i].points[j].y);
//			rownum = rownum > 0 ? rownum : 0;
//			row_point[rownum].push_back(contours[i].points[j]);
//			img_show.at<cv::Vec3b>(round(contours[i].points[j].y), round(contours[i].points[j].x)) = color1;
//		}
//	}
//
//	cv::RNG& rng = cv::theRNG();
//	for (int v = 0; v < height; v++)
//	{
//		double sum_u = 0;
//		double sum_v = 0;
//		double sum_weight = 0;
//		if (row_point[v].size()<2)
//		{
//			continue;
//		}
//		for (int contour_num = 0; contour_num < row_point[v].size(); contour_num++)
//		{
//			double weight = (double)(Gray.at<unsigned char>(row_point[v][contour_num]));
//			sum_u += row_point[v][contour_num].x * weight;
//			sum_v += row_point[v][contour_num].y * weight;
//			sum_weight += weight;
//		}
//
//		if (sum_weight == 0)
//		{
//			continue;
//		}
//
//		float sub_u = sum_u / sum_weight;
//		float sub_v = sum_v / sum_weight;
//
//		if (Gray.at<unsigned char>(v, (int)sub_u) <= color_threshold)
//		{
//			continue;
//		}
//
//		cv::Point2f point;
//		point.x = sub_u;
//		point.y = sub_v;
//		points.push_back(point);
//		cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
//		cv::circle(img_show, point, 0.5, color2);
//		//cout << point << endl;
//		//cout << endl;
//
//		//double sum_I = 0;
//		//double sum_XI = 0;
//		//for (int u = 0; u < width; u++)
//		//{
//		//	//Vec3b point = LaserImage.at<Vec3b>(v, u);
//		//	//float g = point[0] * 0.3 + point[1] * 0.3 + point[2] * 0.4;
//		//	float g = Gray.at<unsigned char>(v, u);
//		//	if (g>color_threshold)
//		//	{
//		//		sum_I += g;
//		//		sum_XI += g*u;
//		//	}
//		//}
//		//if (sum_I==0)
//		//{
//		//	continue;
//		//}
//		//float sub_u = sum_XI / sum_I;
//		//if (Gray.at<unsigned char>(v,(int)sub_u)<= color_threshold)
//		//{
//		//	continue;
//		//}
//		//Point2f point;
//		//point.x = sub_u;
//		//point.y = v;
//		//points.push_back(point);
//		//Scalar color2 = Scalar(rng(256), rng(256), rng(256));
//		//circle(img_show, point, 0.5, color2);
//	}
//}

void RasterScan::FindLaserPoints(const Mat &LaserImage, vector<cv::Point2f> &points)
{
	Mat Gray;

	if (LaserImage.type() == CV_8UC1)
	{
		LaserImage.copyTo(Gray);
	}
	else if (LaserImage.type() == CV_8UC3)
	{
		cvtColor(LaserImage, Gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		cout << "unknow img type\n" << endl;
		exit(0);
	}

	int width = LaserImage.cols;
	int height = LaserImage.rows;

	Mat image_show;
	LaserImage.copyTo(image_show);
	cv::RNG& rng = cv::theRNG();

	for (int v = 0; v < height; v++)
	{
		double sum_weight = 0;
		double sum_u = 0;
		for (int u = 0; u < width; u++)
		{
			//Vec3b point = LaserImage.at<Vec3b>(v, u);
			//float g = point[0] * 0.3 + point[1] * 0.3 + point[2] * 0.4;
			float g = Gray.at<unsigned char>(v, u);
			if (g>color_threshold)
			{
				float weight = g*g;
				sum_weight += weight;
				sum_u += weight*u;
			}

		}
		if (sum_weight == 0)
		{
			continue;
		}
		float sub_u = sum_u / sum_weight;
		cv::Point2f point;
		point.x = sub_u;
		point.y = v;
		points.push_back(point);

		cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
		cv::circle(image_show, point, 0.5, color2);
	}

}

void RasterScan::CircleCenterCalculate(Mat &image, std::vector<cv::Point2d> &imagePoints, bool flag)
{
	Mat cameraMatrix, dist;

	Mat b_img;
	image = Distortion(image, flag);
	image.copyTo(b_img);
	//b_img = Distortion(b_img, cameraMatrix, distCoeffs);
	//image = b_img;
	//cvtColor(b_img, b_img, CV_BGR2GRAY);
	//b_img.convertTo(b_img, CV_32FC1, 1.0 / 255.0);

	// Set up the detector with default parameters.
	//SimpleBlobDetector detector;
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 0;
	params.maxThreshold = 150;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 50;
	params.maxArea = 300;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	// Detect blobs.
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	//detector->detect(img, keypoints);
	detector->detect(b_img, keypoints);
	//params.detect(image, keypoints);

	// Draw detected blobs as red circles.
	//DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	for (int i = 0; i < keypoints.size(); i++)
	{
		imagePoints.push_back(keypoints[i].pt);
		circle(b_img, cv::Point(round(keypoints[i].pt.x), round(keypoints[i].pt.y)), 0.5, cv::Scalar(0, 0, 255), 1);
		cout << "point" << i << " : " << keypoints[i].pt << endl;
	}
	drawKeypoints(b_img, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

}

//Eigen::Matrix4d RasterScan::EdgeSearch(vector<Eigen::VectorXd> &points_mask)
//{
//	Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
//
//	//first time create edgelist
//	if (mask_points_list.size()==0)
//	{
//		
//		for (int point_index = 0; point_index < (points_mask.size()-1); point_index++)
//		{
//			mask_points_list.push_back(points_mask[point_index]);
//
//			for (int point_index2 = point_index + 1; point_index2 < points_mask.size(); point_index2++)
//			{
//				Eigen::VectorXd d_ = points_mask[point_index] - points_mask[point_index2];
//				double distance = sqrt(d_.transpose()*d_);
//				MaskEdge me;
//				me.length = distance;
//				me.p1 = point_index;
//				me.p2 = point_index2;
//				mask_edge_list.push_back(me);
//			}
//		}
//
//		mask_points_list.push_back(points_mask[points_mask.size() - 1]);
//
//		return Rt;
//	}
//
//	//create current edgelist
//	vector<MaskEdge> current_mask_edge_list;
//	for (int point_index = 0; point_index < (points_mask.size() - 1); point_index++)
//	{
//		for (int point_index2 = point_index + 1; point_index2 < points_mask.size(); point_index2++)
//		{
//			Eigen::VectorXd d_ = points_mask[point_index] - points_mask[point_index2];
//			double distance = sqrt(d_.transpose()*d_);
//			MaskEdge me;
//			me.length = distance;
//			me.p1 = point_index;
//			me.p2 = point_index2;
//			current_mask_edge_list.push_back(me);
//		}
//	}
//
//	//match current points
//	vector<MastPoint> mp(points_mask.size());
//	for (int curr_edge_index = 0; curr_edge_index < current_mask_edge_list.size(); curr_edge_index++)
//	{
//		bool matched_ = false;
//		for (int edge_index = 0; edge_index < mask_edge_list.size(); edge_index++)
//		{
//			double dis = current_mask_edge_list[curr_edge_index].length - mask_edge_list[edge_index].length;
//			if (dis < 1)
//			{
//				mp[current_mask_edge_list[curr_edge_index].p1].candidateP1_ = mask_edge_list[edge_index].p1;
//				mp[current_mask_edge_list[curr_edge_index].p1].candidateP2_ = mask_edge_list[edge_index].p2;
//				mp[current_mask_edge_list[curr_edge_index].p2].candidateP1_ = mask_edge_list[edge_index].p1;
//				mp[current_mask_edge_list[curr_edge_index].p2].candidateP2_ = mask_edge_list[edge_index].p2;
//			}
//		}
//	}
//
//	vector<Eigen::VectorXd> points_1;
//	vector<cv::Point2d> points_2;
//
//	for (int point_index = 0; point_index < (points_mask.size() - 1); point_index++)
//	{
//
//		for (int point_index2 = point_index + 1; point_index2 < points_mask.size(); point_index2++)
//		{
//
//			Eigen::VectorXd d_ = points_mask[point_index] - points_mask[point_index2];
//			double distance = sqrt(d_.transpose()*d_);
//
//			for (int edge_index = 0; edge_index < mask_edge_list.size(); edge_index++)
//			{
//				if (distance == mask_edge_list[edge_index].length)
//				{
//
//				}
//
//			}
//
//
//			MaskEdge me;
//			me.length = distance;
//			me.p1 = point_index;
//			me.p2 = point_index2;
//			mask_edge_list.push_back(me);
//		}
//
//
//
//	}
//}

void match_show(vector<PointEdges> &mask_points_edge)
{
	for (int i = 0; i < mask_points_edge.size(); i++)
	{
		cout << "&&& " << endl << mask_points_edge[i].point_coor_ << endl;
		for (int j = 0; j < mask_points_edge[i].Edges.size(); j++)
		{
			cout << " " << mask_points_edge[i].Edges[j] << " ,";
		}
		cout << endl;
	}

}

bool RasterScan::EdgeSearch(vector<Eigen::VectorXd> &points_mask, Eigen::Matrix4d &Rt)
{
	Rt = Eigen::Matrix4d::Identity();

	//first time create edgelist
	if (mask_points_edge.size() == 0)
	{
		mask_points_edge.resize(points_mask.size());
		for (int point_index = 0; point_index < (points_mask.size() - 1); point_index++)
		{
			mask_points_edge[point_index].point_coor_ = points_mask[point_index];
			//cout << "mask points coor = " << mask_points_edge[point_index].point_coor_ << endl;
			for (int point_index2 = point_index + 1; point_index2 < points_mask.size(); point_index2++)
			{
				Eigen::VectorXd d_ = points_mask[point_index] - points_mask[point_index2];
				double distance = sqrt(d_.transpose()*d_);
				mask_points_edge[point_index].Edges.push_back(distance);
				mask_points_edge[point_index2].Edges.push_back(distance);
				//cout << "mask_points_edge = " << mask_points_edge[point_index].Edges[]<< endl;
			}
		}
		mask_points_edge[points_mask.size() - 1].point_coor_ = points_mask[points_mask.size() - 1];
		return true;
	}
	cout << "globle" << endl;
	match_show(mask_points_edge); //cout << endl;
	cout << endl;
	//create current edgelist
	vector<PointEdges> current_mask_points_edge;
	current_mask_points_edge.resize(points_mask.size());
	for (int point_index = 0; point_index < (points_mask.size() - 1); point_index++)
	{
		current_mask_points_edge[point_index].point_coor_ = points_mask[point_index];
		for (int point_index2 = point_index + 1; point_index2 < points_mask.size(); point_index2++)
		{
			Eigen::VectorXd d_ = points_mask[point_index] - points_mask[point_index2];
			double distance = sqrt(d_.transpose()*d_);
			current_mask_points_edge[point_index].Edges.push_back(distance);
			current_mask_points_edge[point_index2].Edges.push_back(distance);
		}
	}
	current_mask_points_edge[points_mask.size() - 1].point_coor_ = points_mask[points_mask.size() - 1];
	//mask_points_edge[points_mask.size() - 1].point_coor_ = points_mask[points_mask.size() - 1];
	cout << "current" << endl;
	match_show(current_mask_points_edge);

	//matching points
	vector<int> match_list(points_mask.size());

	//vector<double> match_list_dis(points_mask.size());
	int matched_number = 0;
	for (int current_points_index = 0; current_points_index < current_mask_points_edge.size(); current_points_index++)
	{
		//cout << "c" << current_points_index << " :" << endl;
		vector<double> match_list_dis2(mask_points_edge.size());
		vector<int>	matched_edge_index_;
		vector<int> matched_edge_number(mask_points_edge.size());
		for (int points_index = 0; points_index < mask_points_edge.size(); points_index++)
		{
			//cout << "g" << points_index << " " << endl;
			for (int current_edge_index = 0; current_edge_index < current_mask_points_edge[current_points_index].Edges.size(); current_edge_index++)
			{

				for (int edge_index = 0; edge_index < mask_points_edge[points_index].Edges.size(); edge_index++)
				{
					double dis = abs(mask_points_edge[points_index].Edges[edge_index] - current_mask_points_edge[current_points_index].Edges[current_edge_index]);
					if (dis<0.05)
					{
						//cout << " Y " << dis << " ";
						//match_list_dis[current_points_index] += dis;
						match_list_dis2[points_index] += dis;
						mask_points_edge[points_index].Edges[edge_index] = 0.5*(mask_points_edge[points_index].Edges[edge_index] + current_mask_points_edge[current_points_index].Edges[current_edge_index]);
						matched_edge_number[points_index]++;
						break;
					}
					else
					{
						//cout << " N " << dis << " ";
					}
				}
				//cout << endl;
			}
			if (matched_edge_number[points_index]>1)
			{
				matched_edge_index_.push_back(points_index);
			}
		}
		if (matched_edge_index_.size()>0)
		{
			double min_value = 2;
			int last_index = -1;
			int max_matched_num = 0;
			for (int me_index = 0; me_index < matched_edge_index_.size(); me_index++)
			{
				if (matched_edge_number[matched_edge_index_[me_index]]>max_matched_num)
				{
					max_matched_num = matched_edge_number[matched_edge_index_[me_index]];
					last_index = matched_edge_index_[me_index];
				}
			}
			match_list[current_points_index] = last_index;
			matched_number++;

			//for (int me_index = 0; me_index < matched_edge_index_.size(); me_index++)
			//{
			//	
			//	if (match_list_dis2[matched_edge_index_[me_index]]<min_value)
			//	{
			//		last_index = matched_edge_index_[me_index];
			//		min_value = match_list_dis2[matched_edge_index_[me_index]];
			//	}
			//}
			//match_list[current_points_index] = last_index;
			//matched_number++;
		}
		else
		{
			match_list[current_points_index] = -1;
		}
	}

	int same_num = 0;
	for (int m_index = 0; m_index < match_list.size(); m_index++)
	{
		for (int m_index2 = m_index + 1; m_index2 < match_list.size(); m_index2++)
		{
			int a = match_list[m_index];
			int	b = match_list[m_index2];
			if (a == b&&a != -1)
			{
				same_num++;
			}
		}
	}

	//for (int matched_index_ = 0; matched_index_ < match_list.size()-1; matched_index_++)
	//{
	//	
	//	if (match_list[matched_index_]==-1)
	//	{
	//		continue;
	//	}
	//	for (int list_index = matched_index_+1; list_index < match_list.size(); list_index++)
	//	{
	//		if (match_list[matched_index_] == match_list[list_index])
	//		{
	//			
	//			if (match_list_dis[matched_index_]>match_list_dis[list_index])
	//			{
	//				match_list[list_index] = -1;
	//			}
	//			else
	//			{
	//				match_list[matched_index_] = -1;
	//			}
	//			matched_number--;
	//		}
	//	}
	//}
	//cout <<"matched number"<< matched_number << endl;

	//if match number < 3 RT have no answer
	if (matched_number<3 || same_num>0)
	{
		return false;
	}

	////calculate transform matrix
	//Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(3 * matched_number, 12);
	//Eigen::VectorXd B_ = Eigen::VectorXd::Zero(3 * matched_number);
	Eigen::VectorXd X_ = Eigen::VectorXd::Zero(12);

	//for (int curr_point_index = 0; curr_point_index < matched_number; curr_point_index++)
	//{
	//	if (match_list[curr_point_index]==-1)
	//	{
	//		continue;
	//	}
	//	A_(3 * curr_point_index + 0, 0) = points_mask[curr_point_index](0); 
	//	A_(3 * curr_point_index + 0, 1) = points_mask[curr_point_index](1);
	//	A_(3 * curr_point_index + 0, 2) = points_mask[curr_point_index](2);
	//	A_(3 * curr_point_index + 0, 3) = 1;
	//	A_(3 * curr_point_index + 1, 4) = points_mask[curr_point_index](0);
	//	A_(3 * curr_point_index + 1, 5) = points_mask[curr_point_index](1);
	//	A_(3 * curr_point_index + 1, 6) = points_mask[curr_point_index](2);
	//	A_(3 * curr_point_index + 1, 7) = 1;
	//	A_(3 * curr_point_index + 2, 8) = points_mask[curr_point_index](0);
	//	A_(3 * curr_point_index + 2, 9) = points_mask[curr_point_index](1);
	//	A_(3 * curr_point_index + 2, 10) = points_mask[curr_point_index](2);
	//	A_(3 * curr_point_index + 2, 11) = 1;
	//	B_(3 * curr_point_index + 0) = mask_points_edge[match_list[curr_point_index]].point_coor_(0);
	//	B_(3 * curr_point_index + 1) = mask_points_edge[match_list[curr_point_index]].point_coor_(1);
	//	B_(3 * curr_point_index + 2) = mask_points_edge[match_list[curr_point_index]].point_coor_(2);
	//}

	////cout << "The least-squares solution is:\n"
	////	<< A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_)<<endl;
	////cout << A_ << endl<<endl;
	////cout << B_ << endl;

	//X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);
	X_(0) = 1; X_(5) = 1; X_(10) = 1;
	GaussNewTon(X_, points_mask, match_list, matched_number);

	Rt(0, 0) = X_[0]; Rt(0, 1) = X_[1]; Rt(0, 2) = X_[2]; Rt(0, 3) = X_[3];
	Rt(1, 0) = X_[4]; Rt(1, 1) = X_[5]; Rt(1, 2) = X_[6]; Rt(1, 3) = X_[7];
	Rt(2, 0) = X_[8]; Rt(2, 1) = X_[9]; Rt(2, 2) = X_[10]; Rt(2, 3) = X_[11];

	//add new edges and points
	for (int current_points_index = 0; current_points_index < match_list.size(); current_points_index++)
	{
		//cout << "current index :" << match_list[current_points_index] << endl;
		if (match_list[current_points_index] == -1)
		{
			PointEdges pe;
			Eigen::Vector4d point_curr(points_mask[current_points_index](0), points_mask[current_points_index](1), points_mask[current_points_index](2), 1);
			point_curr = Rt*point_curr;
			Eigen::Vector3d point_add(point_curr(0), point_curr(1), point_curr(2));
			pe.point_coor_ = point_add;
			for (int points_index = 0; points_index < mask_points_edge.size(); points_index++)
			{
				Eigen::Vector3d d_ = pe.point_coor_ - mask_points_edge[points_index].point_coor_;
				double distance = sqrt(d_.transpose()*d_);
				mask_points_edge[points_index].Edges.push_back(distance);
				pe.Edges.push_back(distance);
			}
			mask_points_edge.push_back(pe);
		}
	}

	return true;
}

double RasterScan::Function(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, int matched_number)
{
	double error = 0;
	for (int point_index = 0; point_index < points_mask.size(); point_index++)
	{
		if (match_list[point_index] == -1)
		{
			continue;
		}
		double a1 = points_mask[point_index](0);
		double a2 = points_mask[point_index](1);
		double a3 = points_mask[point_index](2);
		double b1 = mask_points_edge[match_list[point_index]].point_coor_(0);
		double b2 = mask_points_edge[match_list[point_index]].point_coor_(1);
		double b3 = mask_points_edge[match_list[point_index]].point_coor_(2);
		double err1 = pow(a1*X_(0) + a2*X_(1) + a3*X_(2) + X_(3) - b1, 2);
		double err2 = pow(a1*X_(4) + a2*X_(5) + a3*X_(6) + X_(7) - b2, 2);
		double err3 = pow(a1*X_(8) + a2*X_(9) + a3*X_(10) + X_(11) - b3, 2);
		error += err1 + err2 + err3;
	}

	error += pow(X_(0) * X_(0) + X_(4) * X_(4) + X_(8) * X_(8) - 1, 2);
	error += pow(X_(1) * X_(1) + X_(5) * X_(5) + X_(9) * X_(9) - 1, 2);
	error += pow(X_(2) * X_(2) + X_(6) * X_(6) + X_(10) * X_(10) - 1, 2);

	error += pow(X_(0) * X_(1) + X_(4) * X_(5) + X_(8) * X_(9), 2);
	error += pow(X_(0) * X_(2) + X_(4) * X_(6) + X_(8) * X_(10), 2);
	error += pow(X_(1) * X_(2) + X_(5) * X_(6) + X_(9) * X_(10), 2);

	cout << "error = " << error << endl;
	return error;
}

void RasterScan::computF(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, Eigen::VectorXd &f, int matched_number)
{
	int p_index = 0;
	for (int point_index = 0; point_index < points_mask.size(); point_index++)
	{
		if (match_list[point_index] == -1)
		{
			continue;
		}
		double a1 = points_mask[point_index](0);
		double a2 = points_mask[point_index](1);
		double a3 = points_mask[point_index](2);
		double b1 = mask_points_edge[match_list[point_index]].point_coor_(0);
		double b2 = mask_points_edge[match_list[point_index]].point_coor_(1);
		double b3 = mask_points_edge[match_list[point_index]].point_coor_(2);
		f(p_index * 3 + 0) = a1*X_(0) + a2*X_(1) + a3*X_(2) + X_(3) - b1;
		f(p_index * 3 + 1) = a1*X_(4) + a2*X_(5) + a3*X_(6) + X_(7) - b2;
		f(p_index * 3 + 2) = a1*X_(8) + a2*X_(9) + a3*X_(10) + X_(11) - b3;
		p_index++;
	}

	f(matched_number * 3 + 0) = X_(0) * X_(0) + X_(4) * X_(4) + X_(8) * X_(8) - 1;
	f(matched_number * 3 + 1) = X_(1) * X_(1) + X_(5) * X_(5) + X_(9) * X_(9) - 1;
	f(matched_number * 3 + 2) = X_(2) * X_(2) + X_(6) * X_(6) + X_(10) * X_(10) - 1;

	f(matched_number * 3 + 3) = X_(0) * X_(1) + X_(4) * X_(5) + X_(8) * X_(9);
	f(matched_number * 3 + 4) = X_(0) * X_(2) + X_(4) * X_(6) + X_(8) * X_(10);
	f(matched_number * 3 + 5) = X_(1) * X_(2) + X_(5) * X_(6) + X_(9) * X_(10);

}

void RasterScan::computJ(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, Eigen::MatrixXd &j, int matched_number)
{
	int p_index = 0;
	for (int point_index = 0; point_index < points_mask.size(); point_index++)
	{
		if (match_list[point_index] == -1)
		{
			continue;
		}
		double a1 = points_mask[point_index](0);
		double a2 = points_mask[point_index](1);
		double a3 = points_mask[point_index](2);
		double b1 = mask_points_edge[match_list[point_index]].point_coor_(0);
		double b2 = mask_points_edge[match_list[point_index]].point_coor_(1);
		double b3 = mask_points_edge[match_list[point_index]].point_coor_(2);
		j(p_index * 3 + 0, 0) = a1; j(p_index * 3 + 0, 1) = a2; j(p_index * 3 + 0, 2) = a3; j(p_index * 3 + 0, 3) = 1;
		j(p_index * 3 + 1, 4) = a1; j(p_index * 3 + 1, 5) = a2; j(p_index * 3 + 1, 6) = a3; j(p_index * 3 + 1, 7) = 1;
		j(p_index * 3 + 2, 8) = a1; j(p_index * 3 + 2, 9) = a2; j(p_index * 3 + 2, 10) = a3; j(p_index * 3 + 2, 11) = 1;
		p_index++;
	}

	j(matched_number * 3 + 0, 0) = 2 * X_(0); j(matched_number * 3 + 0, 4) = 2 * X_(4); j(matched_number * 3 + 0, 8) = 2 * X_(8);
	j(matched_number * 3 + 1, 1) = 2 * X_(1); j(matched_number * 3 + 1, 5) = 2 * X_(5); j(matched_number * 3 + 1, 9) = 2 * X_(9);
	j(matched_number * 3 + 2, 2) = 2 * X_(2); j(matched_number * 3 + 2, 6) = 2 * X_(6); j(matched_number * 3 + 2, 10) = 2 * X_(10);

	j(matched_number * 3 + 3, 0) = X_(1); j(matched_number * 3 + 3, 1) = X_(0);
	j(matched_number * 3 + 3, 4) = X_(5); j(matched_number * 3 + 3, 5) = X_(4);
	j(matched_number * 3 + 3, 8) = X_(9); j(matched_number * 3 + 3, 9) = X_(8);
	j(matched_number * 3 + 4, 0) = X_(2); j(matched_number * 3 + 4, 2) = X_(0);
	j(matched_number * 3 + 4, 4) = X_(6); j(matched_number * 3 + 4, 6) = X_(4);
	j(matched_number * 3 + 4, 8) = X_(10); j(matched_number * 3 + 4, 10) = X_(8);
	j(matched_number * 3 + 5, 1) = X_(2); j(matched_number * 3 + 5, 2) = X_(1);
	j(matched_number * 3 + 5, 5) = X_(6); j(matched_number * 3 + 5, 6) = X_(5);
	j(matched_number * 3 + 5, 9) = X_(10); j(matched_number * 3 + 5, 10) = X_(9);

}

void RasterScan::GaussNewTon(Eigen::VectorXd &X_, vector<Eigen::VectorXd> &points_mask, vector<int> &match_list, int matched_number)
{
	const double epsilon = 0.0001;
	double Fk = 0, Fk1;
	int iter_times = 60;
	int Econ_num = 0;

	Eigen::VectorXd x2 = Eigen::VectorXd::Zero(12);
	for (int i = 0; i < 12; i++)
	{
		x2(i) = X_(i);
	}

	Fk1 = Function(x2, points_mask, match_list, matched_number);
	for (int iter_count = 0; (iter_count < iter_times) && (abs(Fk1 - Fk) >= epsilon*(1 + Fk1)); ++iter_count)
	{
		//std::cout << "iteration " << iter_count << ": E = " << Fk1 << std::endl;
		Fk = Fk1;

		Eigen::VectorXd f2 = Eigen::VectorXd::Zero(matched_number * 3 + 6);
		computF(x2, points_mask, match_list, f2, matched_number);
		//std::cout << "computef end " << std::endl;
		//cout << f2 << endl << endl;

		Eigen::MatrixXd j2 = Eigen::MatrixXd::Zero(matched_number * 3 + 6, 12);
		computJ(x2, points_mask, match_list, j2, matched_number);
		//std::cout << "computeJ end " << std::endl;
		//cout << j2 << endl;

		Eigen::MatrixXd jt = j2.transpose();
		Eigen::MatrixXd jtj = jt*j2;

		Eigen::VectorXd f3 = jt*f2*-1;

		Eigen::MatrixXd i2 = Eigen::MatrixXd::Identity(12, 12);
		i2 = i2*0.001;

		Eigen::MatrixXd jtjd = jtj + i2;

		//std::cout << "start solve !!" << std::endl;
		Eigen::VectorXd h2(12);
		//Eigen::LDLT<Eigen::MatrixXd> chol(jtjd);
		//h2 = chol.solve(f3);

		//h2 = jtjd.ldlt().solve(f3);
		//h2 = jtjd.qr().solve(f3);
		h2 = jtjd.colPivHouseholderQr().solve(f3);
		x2 += h2;

		Fk1 = Function(x2, points_mask, match_list, matched_number);
		//std::cout << "########" << abs(Fk1 - Fk) << ";" << epsilon*(1 + Fk1) << ";" << std::endl;
	}

	for (int i = 0; i < 12; i++)
	{
		X_[i] = x2(i);
	}

}


void CircleCenterCalculate(Mat &image, std::vector<cv::Point2f> &imagePoints, int flag)
{
	//flag==0 position points ; flag==1 calibration points
	Mat b_img;
	//image = Distortion(image, cameraMatrix, distCoeffs);
	image.copyTo(b_img);
	b_img = 255 - b_img;
	//b_img = Distortion(b_img, cameraMatrix, distCoeffs);
	//image = b_img;
	//cvtColor(b_img, b_img, CV_BGR2GRAY);
	//b_img.convertTo(b_img, CV_32FC1, 1.0 / 255.0);

	// Set up the detector with default parameters.
	//SimpleBlobDetector detector;
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 60;
	params.maxThreshold = 230;

	// Filter by Area.
	params.filterByArea = true;
	if (flag == 0)
	{
		params.minArea = 4000;// 5000;
	}
	else
	{
		params.minArea = 800;// 1800;
	}

	params.maxArea = 9000;// 13000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.7;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.9;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.2;

	// Detect blobs.
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	//detector->detect(img, keypoints);
	detector->detect(b_img, keypoints);
	//params.detect(image, keypoints);

	// Draw detected blobs as red circles.
	//DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;

	drawKeypoints(b_img, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	for (int i = 0; i < keypoints.size(); i++)
	{
		stringstream ss;
		string index_;
		ss << i;
		ss >> index_;
		imagePoints.push_back(keypoints[i].pt);
		circle(b_img, cv::Point(round(keypoints[i].pt.x), round(keypoints[i].pt.y)), 0.5, cv::Scalar(0, 0, 255), 1);
		cv::putText(im_with_keypoints, index_, keypoints[i].pt, 0, 0.5, cv::Scalar(128, 130, 255), 1);
		//circle(b_img, cv::Point(round(keypoints[i].pt.x), round(keypoints[i].pt.y)), 0.5, cv::Scalar(0, 0, 255), 1);
		//cout << "point" << i << " : " << keypoints[i].pt << endl;
	}

}

float ImagePointDistance(cv::Point2f p1, cv::Point2f p2)
{
	cv::Point2f p1_minus_p2 = p1 - p2;
	return sqrt(pow(p1_minus_p2.x, 2) + pow(p1_minus_p2.y, 2));
}

void PositionPointCal(Mat &image, std::vector<cv::Point2f> &position_points)
{
	struct P2P
	{
		int index_from;
		int index_to;
		float distance_;
		float k_;
		float b_;
	};
	vector<int> final_index = { -1,-1,-1,-1,-1 };

	Mat image_show_1;
	image.copyTo(image_show_1);
	for (int i = 0; i < position_points.size(); i++)
	{
		stringstream ss;
		string index_;
		ss << i;
		ss >> index_;

		circle(image_show_1, position_points[i], 0.5, cv::Scalar(0, 0, 255), 1);
		cv::putText(image_show_1, index_, position_points[i], 0, 0.5, cv::Scalar(128, 130, 255), 1);
	}

	//connect
	vector<P2P> distances;
	for (int from_index_ = 0; from_index_ < position_points.size() - 1; from_index_++)
	{

		for (int to_index_ = from_index_ + 1; to_index_ < position_points.size(); to_index_++)
		{
			P2P id;
			id.distance_ = ImagePointDistance(position_points[from_index_], position_points[to_index_]);
			id.index_from = from_index_;
			id.index_to = to_index_;

			//cout << "from:" << from_index_ << " to:" << to_index_ << " distance:" << id.distance_ << endl;

			id.k_ = (position_points[from_index_].y - position_points[to_index_].y) / (position_points[from_index_].x - position_points[to_index_].x);
			id.b_ = position_points[from_index_].y - id.k_*position_points[from_index_].x;

			distances.push_back(id);
		}
	}

	//min max distance
	float min_dis = 1000;
	float max_dis = 0;
	int min_index_ = -1;
	int max_index_ = -1;
	for (int index_ = 0; index_ < distances.size(); index_++)
	{

		if (distances[index_].distance_<min_dis)
		{
			min_dis = distances[index_].distance_;
			min_index_ = index_;
		}

		if (distances[index_].distance_>max_dis)
		{
			max_dis = distances[index_].distance_;
			max_index_ = index_;
		}
	}
	//cout << "max:" << max_index_ << " max dis:" << max_dis << " min:" << min_index_ << " min dis:" << min_dis << endl;
	final_index[0] = distances[min_index_].index_from;
	final_index[1] = distances[min_index_].index_to;
	final_index[2] = distances[max_index_].index_from;
	final_index[3] = distances[max_index_].index_to;

	vector<int> index_J = { -1,-1,-1,-1,-1 };
	for (int i = 0; i < 4; i++)
	{
		index_J[final_index[i]] = 0;
	}
	for (int i = 0; i < 5; i++)
	{
		if (index_J[i] == -1)
			final_index[4] = i;
	}
	//cout << final_index[0] << " ;" << final_index[1] << " ;" << final_index[2] << " ;" << final_index[3] << endl;

	//judgement p0 p1
	{
		float k1 = (position_points[final_index[4]].y - position_points[final_index[0]].y) / (position_points[final_index[4]].x - position_points[final_index[0]].x);
		float b1 = position_points[final_index[4]].y - k1*position_points[final_index[4]].x;
		float k2 = (position_points[final_index[4]].y - position_points[final_index[1]].y) / (position_points[final_index[4]].x - position_points[final_index[1]].x);
		float b2 = position_points[final_index[4]].y - k2*position_points[final_index[4]].x;

		cv::Point2f cross_point1;
		cross_point1.x = (distances[max_index_].b_ - b1) / (k1 - distances[max_index_].k_);
		cross_point1.y = k1*cross_point1.x + b1;

		cv::Point2f cross_point2;
		cross_point2.x = (distances[max_index_].b_ - b2) / (k2 - distances[max_index_].k_);
		cross_point2.y = k2*cross_point2.x + b2;

		circle(image_show_1, cross_point1, 0.5, cv::Scalar(0, 0, 255), 1);
		cv::putText(image_show_1, "0", cross_point1, 0, 0.5, cv::Scalar(128, 130, 255), 1);
		circle(image_show_1, cross_point2, 0.5, cv::Scalar(0, 0, 255), 1);
		cv::putText(image_show_1, "1", cross_point2, 0, 0.5, cv::Scalar(128, 130, 255), 1);

		float disc1_2 = ImagePointDistance(position_points[final_index[2]], cross_point1);
		float disc1_3 = ImagePointDistance(position_points[final_index[3]], cross_point1);
		float disc2_2 = ImagePointDistance(position_points[final_index[2]], cross_point2);
		float disc2_3 = ImagePointDistance(position_points[final_index[3]], cross_point2);
		//cout << disc1_2 << " ;" << disc1_3 << " ;" << disc2_2 << " ;" << disc2_3 << endl;

		float d_c1 = abs(disc1_2 - disc1_3);
		float d_c2 = abs(disc2_2 - disc2_3);

		if (d_c1<d_c2)
		{
			int trans = final_index[0];
			final_index[0] = final_index[1];
			final_index[1] = trans;
		}
	}

	//judgement p2 p3
	float dis02 = ImagePointDistance(position_points[final_index[0]], position_points[final_index[2]]);
	float dis03 = ImagePointDistance(position_points[final_index[0]], position_points[final_index[3]]);
	if (dis02>dis03)
	{
		int trans = final_index[2];
		final_index[2] = final_index[3];
		final_index[3] = trans;
	}

	////judgement p0 p1 p2 p3
	//float dis02 = ImagePointDistance(position_points[final_index[0]], position_points[final_index[2]]);
	//float dis03 = ImagePointDistance(position_points[final_index[0]], position_points[final_index[3]]);
	//float dis12 = ImagePointDistance(position_points[final_index[1]], position_points[final_index[2]]);
	//float dis13 = ImagePointDistance(position_points[final_index[1]], position_points[final_index[3]]);

	//if (abs(dis02-dis03)<abs(dis12-dis13))
	//{
	//	int trans = final_index[0];
	//	final_index[0] = final_index[1];
	//	final_index[1] = trans;

	//	if (dis12>dis13)
	//	{
	//		trans = final_index[2];
	//		final_index[2] = final_index[3];
	//		final_index[3] = trans;
	//	}

	//}

	//else if (dis02>dis03)
	//{
	//	int	trans = final_index[2];
	//	final_index[2] = final_index[3];
	//	final_index[3] = trans;
	//}


	//////theta
	////float min_theta = 100;
	////int min_index1 = -1;
	////int min_index2 = -1;
	////for (int from_index_ = 0; from_index_ < distances.size()-1; from_index_++)
	////{
	////	for (int to_index_ = from_index_ + 1; to_index_ < distances.size(); to_index_++)
	////	{
	////		float theta = abs((distances[from_index_].k_ - distances[to_index_].k_) / (1 - distances[from_index_].k_ * distances[to_index_].k_));
	////		cout << theta << endl;
	////		if (theta<min_theta)
	////		{
	////			min_index1 = from_index_;
	////			min_index2 = to_index_;
	////			min_theta = theta;
	////		}
	////	}
	////}
	////cout << "0:" << distances[min_index1].index_from << " 1:" << distances[min_index1].index_to << " 2:" << distances[min_index2].index_from << " 3:" << distances[min_index2].index_to << endl;


	vector<cv::Point2f> p_swap;
	for (int index_ = 0; index_ < position_points.size(); index_++)
	{
		p_swap.push_back(position_points[final_index[index_]]);
	}
	p_swap.swap(position_points);

	Mat image_show;
	image.copyTo(image_show);
	for (int i = 0; i < position_points.size(); i++)
	{
		stringstream ss;
		string index_;
		ss << i;
		ss >> index_;

		circle(image_show, position_points[i], 0.5, cv::Scalar(0, 0, 255), 1);
		cv::putText(image_show, index_, position_points[i], 0, 0.5, cv::Scalar(128, 130, 255), 1);
	}
}

void AffineTransform(Mat &warp_mat, std::vector<cv::Point2f> &position_points)
{

	//cv::Point2f srcTri[3];
	//cv::Point2f dstTri[3];
	///// 设置源图像和目标图像上的三组点以计算仿射变换
	//srcTri[0] = cv::Point2f(40, 20);
	//srcTri[1] = cv::Point2f(20, 40);
	//srcTri[2] = cv::Point2f(80, 40);
	//dstTri[0] = position_points[0];
	//dstTri[1] = position_points[2];
	//dstTri[2] = position_points[3];
	///// 求得仿射变换
	//warp_mat = getAffineTransform(srcTri, dstTri);

	//{
	//	Mat A(7, 3, CV_64FC1);
	//	Mat vec(3, 1, CV_64FC1);//最后的答案 
	//	for (int i = 0; i<7; i++)
	//		for (int j = 0; j<3; ++j)
	//			A.at<double>(i, j) = i*j - i;//初始化A的值 
	//	cv::SVD::solveZ(A, vec);
	//}

	//new tested projecttransform solve (effect excellent)
	{
		Mat A = Mat::zeros(position_points.size() * 2, 9, CV_32FC1);

		A.at<float>(0, 0) = 4* cali_size; A.at<float>(0, 1) = 2* cali_size; A.at<float>(0, 2) = 1;
		A.at<float>(0, 6) = -1 * position_points[0].x * 4 * cali_size;
		A.at<float>(0, 7) = -1 * position_points[0].x * 2 * cali_size;
		A.at<float>(0, 8) = -1 * position_points[0].x;
		A.at<float>(1, 3) = 4 * cali_size; A.at<float>(1, 4) = 2 * cali_size; A.at<float>(1, 5) = 1;
		A.at<float>(1, 6) = -1 * position_points[0].y * 4 * cali_size;
		A.at<float>(1, 7) = -1 * position_points[0].y * 2 * cali_size;
		A.at<float>(1, 8) = -1 * position_points[0].y;

		A.at<float>(2, 0) = 5 * cali_size; A.at<float>(2, 1) = 2 * cali_size; A.at<float>(2, 2) = 1;
		A.at<float>(2, 6) = -1 * position_points[1].x * 5 * cali_size;
		A.at<float>(2, 7) = -1 * position_points[1].x * 2 * cali_size;
		A.at<float>(2, 8) = -1 * position_points[1].x;
		A.at<float>(3, 3) = 5 * cali_size; A.at<float>(3, 4) = 2 * cali_size; A.at<float>(3, 5) = 1;
		A.at<float>(3, 6) = -1 * position_points[1].y * 5 * cali_size;
		A.at<float>(3, 7) = -1 * position_points[1].y * 2 * cali_size;
		A.at<float>(3, 8) = -1 * position_points[1].y;

		A.at<float>(4, 0) = 2 * cali_size; A.at<float>(4, 1) = 4 * cali_size; A.at<float>(4, 2) = 1;
		A.at<float>(4, 6) = -1 * position_points[2].x * 2 * cali_size;
		A.at<float>(4, 7) = -1 * position_points[2].x * 4 * cali_size;
		A.at<float>(4, 8) = -1 * position_points[2].x;
		A.at<float>(5, 3) = 2 * cali_size; A.at<float>(5, 4) = 4 * cali_size; A.at<float>(5, 5) = 1;
		A.at<float>(5, 6) = -1 * position_points[2].y * 2 * cali_size;
		A.at<float>(5, 7) = -1 * position_points[2].y * 4 * cali_size;
		A.at<float>(5, 8) = -1 * position_points[2].y;

		A.at<float>(6, 0) = 8 * cali_size; A.at<float>(6, 1) = 4 * cali_size; A.at<float>(6, 2) = 1;
		A.at<float>(6, 6) = -1 * position_points[3].x * 8 * cali_size;
		A.at<float>(6, 7) = -1 * position_points[3].x * 4 * cali_size;
		A.at<float>(6, 8) = -1 * position_points[3].x;
		A.at<float>(7, 3) = 8 * cali_size; A.at<float>(7, 4) = 4 * cali_size; A.at<float>(7, 5) = 1;
		A.at<float>(7, 6) = -1 * position_points[3].y * 8 * cali_size;
		A.at<float>(7, 7) = -1 * position_points[3].y * 4 * cali_size;
		A.at<float>(7, 8) = -1 * position_points[3].y;

		A.at<float>(8, 0) = 5 * cali_size; A.at<float>(8, 1) = 6 * cali_size; A.at<float>(8, 2) = 1;
		A.at<float>(8, 6) = -1 * position_points[4].x * 5 * cali_size;
		A.at<float>(8, 7) = -1 * position_points[4].x * 6 * cali_size;
		A.at<float>(8, 8) = -1 * position_points[4].x;
		A.at<float>(9, 3) = 5 * cali_size; A.at<float>(9, 4) = 6 * cali_size; A.at<float>(9, 5) = 1;
		A.at<float>(9, 6) = -1 * position_points[4].y * 5 * cali_size;
		A.at<float>(9, 7) = -1 * position_points[4].y * 6 * cali_size;
		A.at<float>(9, 8) = -1 * position_points[4].y;

		//Mat B = Mat::zeros(position_points.size() * 2, 1, CV_32FC1);
		warp_mat = Mat::zeros(9, 1, CV_32FC1);
		cv::SVD::solveZ(A, warp_mat);
		//cv::solve(A, B, x, cv::DECOMP_SVD);
	}

	////old tested affinetransform solve (effect normal)
	//{
	//	Mat A = Mat::ones(5, 3, CV_32FC1);
	//	A.at<float>(0, 0) = 40; A.at<float>(0, 1) = 20;
	//	A.at<float>(1, 0) = 50; A.at<float>(1, 1) = 20;
	//	A.at<float>(2, 0) = 20; A.at<float>(2, 1) = 40;
	//	A.at<float>(3, 0) = 80; A.at<float>(3, 1) = 40;
	//	A.at<float>(4, 0) = 50; A.at<float>(4, 1) = 60;
	//	Mat x1 = Mat::zeros(3, 1, CV_32FC1);
	//	Mat B1 = Mat::zeros(5, 1, CV_32FC1);
	//	B1.at<float>(0, 0) = position_points[0].x;
	//	B1.at<float>(1, 0) = position_points[1].x;
	//	B1.at<float>(2, 0) = position_points[2].x;
	//	B1.at<float>(3, 0) = position_points[3].x;
	//	B1.at<float>(4, 0) = position_points[4].x;
	//	Mat x2 = Mat::zeros(3, 1, CV_32FC1);
	//	Mat B2 = Mat::zeros(5, 1, CV_32FC1);
	//	B2.at<float>(0, 0) = position_points[0].y;
	//	B2.at<float>(1, 0) = position_points[1].y;
	//	B2.at<float>(2, 0) = position_points[2].y;
	//	B2.at<float>(3, 0) = position_points[3].y;
	//	B2.at<float>(4, 0) = position_points[4].y;
	//	cv::solve(A, B1, x1, cv::DECOMP_SVD);
	//	cv::solve(A, B2, x2, cv::DECOMP_SVD);
	//	warp_mat.at<float>(0, 0) = x1.at<float>(0, 0); warp_mat.at<float>(0, 1) = x1.at<float>(1, 0); warp_mat.at<float>(0, 2) = x1.at<float>(2, 0);
	//	warp_mat.at<float>(1, 0) = x2.at<float>(0, 0); warp_mat.at<float>(1, 1) = x2.at<float>(1, 0); warp_mat.at<float>(1, 2) = x2.at<float>(2, 0);
	//}

}

void InSightPointIndex(Mat &image, int width_number, int height_number, std::vector<cv::Point2f> &calibration_points, Mat &warp_mat, std::vector<int> &in_sight_points_index_)
{
	std::vector<cv::Point2f> all_points;
	std::vector<cv::Point2f> all_points_image;
	std::vector<cv::Point2f> final_points;
	int points_number = width_number*height_number;
	in_sight_points_index_.clear();

	for (int points_index = 0; points_index < points_number; points_index++)
	{
		float y = cali_size * (points_index / width_number);
		float x = cali_size * (points_index % width_number);
		all_points.push_back(cv::Point2f(x, y));
	}

	for (int points_index = 0; points_index < points_number; points_index++)
	{
		//float x = warp_mat.at<float>(0, 0)*all_points[points_index].x + warp_mat.at<float>(0, 1)*all_points[points_index].y + warp_mat.at<float>(0, 2);
		//float y = warp_mat.at<float>(1, 0)*all_points[points_index].x + warp_mat.at<float>(1, 1)*all_points[points_index].y + warp_mat.at<float>(1, 2);
		float x = (warp_mat.at<float>(0)*all_points[points_index].x + warp_mat.at<float>(1)*all_points[points_index].y + warp_mat.at<float>(2)) /
			(warp_mat.at<float>(6)*all_points[points_index].x + warp_mat.at<float>(7)*all_points[points_index].y + warp_mat.at<float>(8));
		float y = (warp_mat.at<float>(3)*all_points[points_index].x + warp_mat.at<float>(4)*all_points[points_index].y + warp_mat.at<float>(5)) /
			(warp_mat.at<float>(6)*all_points[points_index].x + warp_mat.at<float>(7)*all_points[points_index].y + warp_mat.at<float>(8));
		all_points_image.push_back(cv::Point2f(x, y));

		stringstream ss;
		string index_;
		ss << points_index;
		ss >> index_;
		circle(image, cv::Point2f(x, y), 0.5, cv::Scalar(0, 0, 0), 1);
		cv::putText(image, index_, cv::Point2f(x, y), 0, 0.5, cv::Scalar(128, 130, 128), 1);
	}

	for (int points_index = 0; points_index < points_number; points_index++)
	{
		float min_dis = 10000;
		int min_dis_index = -1;

		for (int ccp_index = 0; ccp_index < calibration_points.size(); ccp_index++)
		{
			float dis = ImagePointDistance(all_points_image[points_index], calibration_points[ccp_index]);

			if (dis<min_dis)
			{
				min_dis = dis;
				min_dis_index = ccp_index;
			}
		}

		if (min_dis<1.0)
		{
			in_sight_points_index_.push_back(points_index);
			final_points.push_back(calibration_points[min_dis_index]);

			stringstream ss;
			string index_;
			ss << points_index;
			ss >> index_;
			circle(image, calibration_points[min_dis_index], 2, cv::Scalar(0, 0, 255), 2);
			cv::putText(image, index_, calibration_points[min_dis_index], 0, 1, cv::Scalar(255, 130, 128), 2);
		}
		else
		{
			cout << min_dis << " ;" << min_dis_index << endl;
		}
	}

	calibration_points.swap(final_points);
	//cv::warpAffine(all_points, all_points, warp_mat, cv::Size(all_points.size()));
	//cv::estimateAffine2D()
}

std::vector<cv::Point3f> Generate3DPoints(int width_number, int height_number, std::vector<int> &in_sight_points_index_)
{
	std::vector<cv::Point3f> points;

	for (int points_index = 0; points_index < in_sight_points_index_.size(); points_index++)
	{
		float y = cali_size * (in_sight_points_index_[points_index] / width_number);
		float x = cali_size * (in_sight_points_index_[points_index] % width_number);
		float z = 0;
		points.push_back(cv::Point3f(x, y, z));

	}
	return points;
}

int CodingCircleCenterRecognition(cv::Mat &image_in, cv::Mat &image, std::vector<std::vector<cv::Point3f>> &objectPoints, std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<int>& CCPoints_index)
{
	image = cv::Mat::zeros(image_in.size(), CV_8UC3);
	//image_in.copyTo(image);
	cv::cvtColor(image_in, image, CV_GRAY2BGR);

	std::vector<cv::Point2f> PCPoints, CCPoints;
	//std::vector<int> CCPoints_index;

	//计算定位点中心位置
	CircleCenterCalculate(image_in, PCPoints, 0);


	if (PCPoints.size() == 5)
	{

		//定位点中心位置排序
		PositionPointCal(image_in, PCPoints);

		//计算仿射变换矩阵
		Mat warp_mat;
		AffineTransform(warp_mat, PCPoints);

		//计算可视平面点与空间点索引计算
		CircleCenterCalculate(image_in, CCPoints, 1);
		InSightPointIndex(image, cali_width, cali_height, CCPoints, warp_mat, CCPoints_index);

		if (CCPoints_index.size()<10)
		{
			return 0;
		}

		//按照索引生成对应标定板三维点
		std::vector<cv::Point3f> calibration_3d_points = Generate3DPoints(cali_width, cali_height, CCPoints_index);
		//for (int point_index = 0; point_index < calibration_3d_points.size(); point_index++)
		//{
		//	cout << "point index:" << CCPoints_index[point_index] << " - " << calibration_3d_points[point_index] << " - " << CCPoints[point_index] << endl;
		//}
		objectPoints.push_back(calibration_3d_points);
		imagePoints.push_back(CCPoints);
		//cv::imshow("CalibrationImages left", img_input_l);
		//cv::waitKey();

		return 1;
	}
	else
	{
		return 0;
	}
}

double computeReprojectionErrors(
	const vector<vector<cv::Point3f> >& objectPoints,
	const vector<vector<cv::Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), cv::NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

void RasterScan::SystemCalibration(const int projector_width, const int projector_height, const int image_number, std::vector<std::vector<cv::Mat>> &image_groups, std::string out_put_name)
{

	std::vector<std::vector<cv::Point3f>> objectPoints_l;
	std::vector<std::vector<cv::Point3f>> objectPoints_r;
	std::vector<std::vector<cv::Point3f>> objectPoints_projector;
	std::vector<std::vector<cv::Point2f>> imagePoints_l;
	std::vector<std::vector<cv::Point2f>> imagePoints_r;
	std::vector<std::vector<cv::Point2f>> imagePoints_projector;
	std::vector<std::vector<int>> CCPoints_index_l;
	std::vector<std::vector<int>> CCPoints_index_r;

	std::vector<std::vector<double>> projector_phase_whole_l;
	std::vector<std::vector<double>> projector_period_whole_l;
	std::vector<std::vector<double>> projector_phase_whole_r;
	std::vector<std::vector<double>> projector_period_whole_r;

	cv::Mat print_out = cv::Mat::ones(cv::Size(960, 540), CV_8SC3);
	print_out *= 80;
	int print_out_y = 30;

	if (image_groups.size() != image_number)
	{
		cout << "Image Groups Number Wrong!" << endl;
		return;
	}

	vector<vector<Mat>> images_h_l(image_number), images_v_l(image_number);
	vector<vector<Mat>> images_h_r(image_number), images_v_r(image_number);
	for (int image_groups_index = 0; image_groups_index < image_number; image_groups_index++)
	{
		if (image_groups[image_groups_index].size() != 62)
		{
			cout << "Image Number Wrong!" << endl;
			return;
		}

		for (int image_index = 0; image_index < 15; image_index++)
		{
			images_h_l[image_groups_index].push_back(image_groups[image_groups_index][image_index + 1]);
			images_v_l[image_groups_index].push_back(image_groups[image_groups_index][image_index + 16]);
			images_h_r[image_groups_index].push_back(image_groups[image_groups_index][image_index + 32]);
			images_v_r[image_groups_index].push_back(image_groups[image_groups_index][image_index + 47]);
		}
	}

	int image_width = image_groups[0][0].cols, image_height = image_groups[0][0].rows;
	cv::namedWindow("Calibration left camera Images", cv::WINDOW_NORMAL);
	cv::namedWindow("Calibration right camera Images", cv::WINDOW_NORMAL);
	cv::namedWindow("Calibration fringe horizontal Images", cv::WINDOW_NORMAL);
	cv::namedWindow("Calibration projector circle center Images", cv::WINDOW_NORMAL);

	cv::resizeWindow("Calibration left camera Images", 960, 540);
	cv::moveWindow("Calibration left camera Images", 0, 0);

	cv::resizeWindow("Calibration right camera Images", 960, 540);
	cv::moveWindow("Calibration right camera Images", 960, 0);

	cv::resizeWindow("Calibration fringe horizontal Images", 960, 540);
	cv::moveWindow("Calibration fringe horizontal Images", 0, 540);

	cv::resizeWindow("Calibration projector circle center Images", 960, 540);
	cv::moveWindow("Calibration projector circle center Images", 960, 540);


	vector<int> calibrate_image_index_l;
	vector<int> calibrate_image_index_r;

	// all image groups select key points
	for (int image_index = 0; image_index < image_number; image_index++)
	{
		stringstream ss;
		string index_;
		ss << image_index;
		ss >> index_;

		//Mat img_input_l = cv::imread("./new_scan/a" + index_ + "/L_0.png", 0);
		//cv::flip(img_input_l, img_input_l, -1);
		//Mat img_input_r = cv::imread("./new_scan/a" + index_ + "/R_0.png", 0);
		//cv::flip(img_input_r, img_input_r, -1);

		cv::Mat image_show_l, image_show_r;
		Mat periodic_h, periodic_v;
		Mat phase_h, phase_v;
		vector<int> CCPoints_index1, CCPoints_index2;
		cv::putText(print_out, "image " + index_, cv::Point(30, print_out_y), 0, 1, cv::Scalar(10, 255, 80), 2);

		//if image groups have good calibration parameters
		if (CodingCircleCenterRecognition(image_groups[image_index][0], image_show_l, objectPoints_l, imagePoints_l, CCPoints_index1))
		{

			cv::imshow("Calibration left camera Images", image_show_l);
			calibrate_image_index_l.push_back(image_index);

			std::vector<cv::Point3f> objectPoints_p;
			std::vector<cv::Point2f> projector_image_points;

			//ReadScanData("./new_scan/a" + index_ + "/", images_h, 0);
			//ReadScanData("./new_scan/a" + index_ + "/", images_v, 1);
			UnwrapComp(images_h_l[image_index], phase_h, periodic_h);
			UnwrapComp(images_v_l[image_index], phase_v, periodic_v);
			//cv::flip(phase_h, phase_h, -1);
			//cv::flip(periodic_h, periodic_h, -1);
			//cv::flip(phase_v, phase_v, -1);
			//cv::flip(periodic_v, periodic_v, -1);


			cv::Mat ppp = periodic_h * 64 + phase_h;

			//ColoredImage3DDistribution(ppp, 2);

			vector<double> phase_h_list;
			vector<double> periodic_h_list;
			vector<double> phase_v_list;
			vector<double> periodic_v_list;

			cv::Mat projector_image(cv::Size(projector_width, projector_height), CV_8UC3);

			for (int uv_index = 0; uv_index < imagePoints_l[imagePoints_l.size() - 1].size(); uv_index++)
			{
				cv::Point2f point_match = imagePoints_l[imagePoints_l.size() - 1][uv_index];
				double k2 = periodic_h.at<double>(point_match.y, point_match.x);
				double t2 = phase_h.at<double>(point_match.y, point_match.x);

				//cv::RNG& rng = cv::theRNG();
				//cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
				//cv::circle(img_input_l2, point_match, 5, color2);
				//cv::circle(phase_left, point_match, 5, cv::Scalar(0));
				//cv::circle(periodic_left, point_match, 5, cv::Scalar(0));

				if (k2 == -1 || t2 == -1)
				{
					phase_h_list.push_back(-1);
					periodic_h_list.push_back(-1);
					continue;
				}

				vector<Eigen::VectorXd> t_points;
				int u_start = fmax(point_match.x - 2, 0), u_end = fmin(point_match.x + 2, phase_h.cols - 1);
				int v_start = fmax(point_match.y - 2, 0), v_end = fmin(point_match.y + 2, phase_h.rows - 1);
				for (int U = u_start; U <= u_end; U++)
				{
					for (int V = v_start; V <= v_end; V++)
					{
						double k1 = periodic_h.at<double>(V, U);
						double t1 = phase_h.at<double>(V, U);
						double dis = abs(t2 - t1);
						//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
						//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

						if (k1 != k2 || dis>0.06)
						{
							continue;
						}
						Eigen::VectorXd t_point(3);
						t_point(0) = U;
						t_point(1) = V;
						t_point(2) = t1;
						t_points.push_back(t_point);
						//cout << t_point << endl << endl;
					}
				}

				//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
				if (t_points.size()<3)
				{
					phase_h_list.push_back(-1);
					periodic_h_list.push_back(-1);
					continue;
				}

				Eigen::Vector4d Plane;
				CalculatePlane(t_points, Plane);

				double phase_ = -1 * (Plane(0)*point_match.x + Plane(1)*point_match.y + Plane(3)) / Plane(2);
				phase_h_list.push_back(phase_);



				//if (frame_index == 1/* && (points_index % 3 == 2||  points_index % 2 == 1)*/)
				//{
				//	cout << "point " << points_index << " : " << phase_ << " ;" << k2 * 16 << " ;" << point_match.x << " ;" << point_match.y << endl;
				//}

				periodic_h_list.push_back(k2);
			}

			/// should be replaced with undistortion projector points
			projector_period_whole_l.push_back(periodic_h_list);
			projector_phase_whole_l.push_back(phase_h_list);

			for (int uv_index = 0; uv_index < imagePoints_l[imagePoints_l.size() - 1].size(); uv_index++)
			{
				cv::Point2f point_match = imagePoints_l[imagePoints_l.size() - 1][uv_index];
				double k2 = periodic_v.at<double>(point_match.y, point_match.x);
				double t2 = phase_v.at<double>(point_match.y, point_match.x);

				//cv::RNG& rng = cv::theRNG();
				//cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
				//cv::circle(img_input_l2, point_match, 5, color2);
				//cv::circle(phase_left, point_match, 5, cv::Scalar(0));
				//cv::circle(periodic_left, point_match, 5, cv::Scalar(0));

				if (k2 == -1 || t2 == -1)
				{
					phase_v_list.push_back(-1);
					periodic_v_list.push_back(-1);
					continue;
				}

				vector<Eigen::VectorXd> t_points;
				int u_start = fmax(point_match.x - 2, 0), u_end = fmin(point_match.x + 2, phase_v.cols - 1);
				int v_start = fmax(point_match.y - 2, 0), v_end = fmin(point_match.y + 2, phase_v.rows - 1);
				for (int U = u_start; U <= u_end; U++)
				{
					for (int V = v_start; V <= v_end; V++)
					{
						double k1 = periodic_v.at<double>(V, U);
						double t1 = phase_v.at<double>(V, U);
						double dis = abs(t2 - t1);
						//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
						//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

						if (k1 != k2 || dis>0.06)
						{
							continue;
						}
						Eigen::VectorXd t_point(3);
						t_point(0) = U;
						t_point(1) = V;
						t_point(2) = t1;
						t_points.push_back(t_point);
						//cout << t_point << endl << endl;
					}
				}

				//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
				if (t_points.size()<3)
				{
					phase_v_list.push_back(-1);
					periodic_v_list.push_back(-1);
					continue;
				}

				Eigen::Vector4d Plane;
				CalculatePlane(t_points, Plane);

				double phase_ = -1 * (Plane(0)*point_match.x + Plane(1)*point_match.y + Plane(3)) / Plane(2);
				phase_v_list.push_back(phase_);



				//if (frame_index == 1/* && (points_index % 3 == 2||  points_index % 2 == 1)*/)
				//{
				//	cout << "point " << points_index << " : " << phase_ << " ;" << k2 * 16 << " ;" << point_match.x << " ;" << point_match.y << endl;
				//}

				periodic_v_list.push_back(k2);
			}

			for (int point_index_ = 0; point_index_ < objectPoints_l[objectPoints_l.size() - 1].size(); point_index_++)
			{
				if (periodic_h_list[point_index_] != 0 && periodic_v_list[point_index_] != 0)
				{
					float u = projector_width * (periodic_h_list[point_index_] * 64.0 + phase_h_list[point_index_]) / 64.0;
					float v = projector_height * (periodic_v_list[point_index_] * 64.0 - 28 + phase_v_list[point_index_]) / 36;
					//cout << periodic_h_list[point_index_] << ", " << phase_h_list[point_index_] << ", " << periodic_v_list[point_index_] << ", " << phase_v_list[point_index_] << ", " << u << ", " << v << ", " << endl;
					objectPoints_p.push_back(objectPoints_l[objectPoints_l.size() - 1][point_index_]);
					projector_image_points.push_back(cv::Point2f(u, v));
					cv::circle(projector_image, cv::Point2f(u, v), 5, cv::Scalar(255, 156, 0), 2);
				}
			}

			cv::putText(print_out, " left OK!", cv::Point(180, print_out_y), 0, 1, cv::Scalar(10, 255, 80), 2);
			cv::imshow("Calibration projector circle center Images", projector_image);

			objectPoints_projector.push_back(objectPoints_p);
			imagePoints_projector.push_back(projector_image_points);
		}
		else
		{
			cv::putText(print_out, " left failure!", cv::Point(180, print_out_y), 0, 1, cv::Scalar(10, 70, 255), 2);
		}
		CCPoints_index_l.push_back(CCPoints_index1);

		periodic_h = cv::Mat::zeros(periodic_h.size(), periodic_h.type()); periodic_v = cv::Mat::zeros(periodic_v.size(), periodic_v.type());
		phase_h = cv::Mat::zeros(phase_h.size(), phase_h.type()); phase_v = cv::Mat::zeros(phase_v.size(), phase_v.type());
		if (CodingCircleCenterRecognition(image_groups[image_index][31], image_show_r, objectPoints_r, imagePoints_r, CCPoints_index2))
		{
			cv::imshow("Calibration right camera Images", image_show_r);
			calibrate_image_index_r.push_back(image_index);

			//ReadScanData("./new_scan/a" + index_ + "/", images_h, 0);
			//ReadScanData("./new_scan/a" + index_ + "/", images_v, 1);
			UnwrapComp(images_h_r[image_index], phase_h, periodic_h);
			UnwrapComp(images_v_r[image_index], phase_v, periodic_v);
			//cv::flip(phase_h, phase_h, -1);
			//cv::flip(periodic_h, periodic_h, -1);
			//cv::flip(phase_v, phase_v, -1);
			//cv::flip(periodic_v, periodic_v, -1);


			cv::Mat ppp = periodic_h * 64 + phase_h;

			//ColoredImage3DDistribution(ppp, 2);

			vector<double> phase_h_list;
			vector<double> periodic_h_list;
			vector<double> phase_v_list;
			vector<double> periodic_v_list;

			cv::Mat projector_image(cv::Size(projector_width, projector_height), CV_8UC3);

			for (int uv_index = 0; uv_index < imagePoints_r[imagePoints_r.size() - 1].size(); uv_index++)
			{
				cv::Point2f point_match = imagePoints_r[imagePoints_r.size() - 1][uv_index];
				double k2 = periodic_h.at<double>(point_match.y, point_match.x);
				double t2 = phase_h.at<double>(point_match.y, point_match.x);

				//cv::RNG& rng = cv::theRNG();
				//cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
				//cv::circle(img_input_l2, point_match, 5, color2);
				//cv::circle(phase_left, point_match, 5, cv::Scalar(0));
				//cv::circle(periodic_left, point_match, 5, cv::Scalar(0));

				if (k2 == -1 || t2 == -1)
				{
					phase_h_list.push_back(-1);
					periodic_h_list.push_back(-1);
					continue;
				}

				vector<Eigen::VectorXd> t_points;
				int u_start = fmax(point_match.x - 2, 0), u_end = fmin(point_match.x + 2, phase_h.cols - 1);
				int v_start = fmax(point_match.y - 2, 0), v_end = fmin(point_match.y + 2, phase_h.rows - 1);
				for (int U = u_start; U <= u_end; U++)
				{
					for (int V = v_start; V <= v_end; V++)
					{
						double k1 = periodic_h.at<double>(V, U);
						double t1 = phase_h.at<double>(V, U);
						double dis = abs(t2 - t1);
						//circle(img1_show, cv::Point2i(U, V), 0.5, cv::Scalar(255, 255, 255));
						//circle(img2_show, cv::Point2i(u, v), 0.5, cv::Scalar(255,  255, 255));

						if (k1 != k2 || dis>0.06)
						{
							continue;
						}
						Eigen::VectorXd t_point(3);
						t_point(0) = U;
						t_point(1) = V;
						t_point(2) = t1;
						t_points.push_back(t_point);
						//cout << t_point << endl << endl;
					}
				}

				//cout << "u1 = " << point_match.x << " v1 = " << point_match.y << " k1 = " << k_match << " t1 = " << t_match << " k2 = " << k2 << " t2 = " << t2 << endl;
				if (t_points.size()<3)
				{
					phase_h_list.push_back(-1);
					periodic_h_list.push_back(-1);
					continue;
				}

				Eigen::Vector4d Plane;
				CalculatePlane(t_points, Plane);

				double phase_ = -1 * (Plane(0)*point_match.x + Plane(1)*point_match.y + Plane(3)) / Plane(2);
				phase_h_list.push_back(phase_);



				//if (frame_index == 1/* && (points_index % 3 == 2||  points_index % 2 == 1)*/)
				//{
				//	cout << "point " << points_index << " : " << phase_ << " ;" << k2 * 16 << " ;" << point_match.x << " ;" << point_match.y << endl;
				//}

				periodic_h_list.push_back(k2);
			}

			/// should be replaced with undistortion projector points
			projector_period_whole_r.push_back(periodic_h_list);
			projector_phase_whole_r.push_back(phase_h_list);

			cv::putText(print_out, " right OK!", cv::Point(380, print_out_y), 0, 1, cv::Scalar(10, 255, 80), 2);

		}
		else
		{
			cv::putText(print_out, " right failure!", cv::Point(380, print_out_y), 0, 1, cv::Scalar(10, 70, 255), 2);
		}
		CCPoints_index_r.push_back(CCPoints_index2);

		cv::imshow("Calibration fringe horizontal Images", print_out);
		print_out_y += 25;
		cv::waitKey(1000);

		if (imagePoints_l.size() == 0 || imagePoints_r.size() == 0)
		{
			cout << " image 0 wrong !!" << endl;
			return;
		}
	}

	cout << projector_height << endl;
	cout << projector_width << endl;

	Mat cameraMatrix_[3]/*,cameraMatrix_2,cameraMatrix_3,cameraMatrix_4*/, distCoeffs_[3]/*, distCoeffs_2, distCoeffs_3, distCoeffs_4*/;
	vector<Mat> rvecs_l, rvecs_r, tvecs_l, tvecs_r, rvecs_projector, tvecs_projector;
	cv::calibrateCamera(objectPoints_l, imagePoints_l, cv::Size(image_width, image_height), cameraMatrix_[0], distCoeffs_[0], rvecs_l, tvecs_l);
	cv::calibrateCamera(objectPoints_r, imagePoints_r, cv::Size(image_width, image_height), cameraMatrix_[1], distCoeffs_[1], rvecs_r, tvecs_r);
	cv::calibrateCamera(objectPoints_projector, imagePoints_projector, cv::Size(projector_width, projector_height), cameraMatrix_[2], distCoeffs_[2], rvecs_projector, tvecs_projector);

	//vector<cv::Point2f> pp(1, cv::Point2f(50,50)), pp2;
	//cout << cameraMatrix_[0] << endl;
	//cout << distCoeffs_[0] << endl;
	//cv::undistortPoints(pp, pp2, cameraMatrix_[0], distCoeffs_[0], cv::noArray(), cameraMatrix_[0]);
	//cout << pp2 << endl;
	//cout << "-----------------------------------" << endl;
	//cv::calibrateCamera(objectPoints_l, imagePoints_l, cv::Size(image_width, image_height), cameraMatrix_2, distCoeffs_2, rvecs_l, tvecs_l, CV_CALIB_RATIONAL_MODEL);
	//cout << cameraMatrix_2 << endl;
	//cout << distCoeffs_2 << endl;
	//cv::undistortPoints(pp, pp2, cameraMatrix_2, distCoeffs_2, cv::noArray(), cameraMatrix_2);
	//cout << pp2 << endl;
	//cout << "-----------------------------------" << endl;
	//cv::calibrateCamera(objectPoints_l, imagePoints_l, cv::Size(image_width, image_height), cameraMatrix_3, distCoeffs_3, rvecs_l, tvecs_l, CV_CALIB_THIN_PRISM_MODEL);
	//cout << cameraMatrix_3 << endl;
	//cout << distCoeffs_3 << endl;
	//cv::undistortPoints(pp, pp2, cameraMatrix_3, distCoeffs_3, cv::noArray(), cameraMatrix_3);
	//cout << pp2 << endl;
	//cout << "-----------------------------------" << endl;
	//cv::calibrateCamera(objectPoints_l, imagePoints_l, cv::Size(image_width, image_height), cameraMatrix_4, distCoeffs_3, rvecs_l, tvecs_l, CV_CALIB_TILTED_MODEL| CV_CALIB_THIN_PRISM_MODEL|CV_CALIB_THIN_PRISM_MODEL);
	//cout << cameraMatrix_4 << endl;
	//cout << distCoeffs_4 << endl;
	//cv::undistortPoints(pp, pp2, cameraMatrix_4, distCoeffs_4, cv::noArray(), cameraMatrix_4);
	//cout << pp2 << endl;
	//cout << "-----------------------------------" << endl;

	std::vector<float> per_view_error;
	float final_error_l = computeReprojectionErrors(objectPoints_l, imagePoints_l, rvecs_l, tvecs_l, cameraMatrix_[0], distCoeffs_[0], per_view_error);
	float final_error_r = computeReprojectionErrors(objectPoints_r, imagePoints_r, rvecs_r, tvecs_r, cameraMatrix_[1], distCoeffs_[1], per_view_error);
	float final_error_p = computeReprojectionErrors(objectPoints_projector, imagePoints_projector, rvecs_projector, tvecs_projector, cameraMatrix_[2], distCoeffs_[2], per_view_error);

	cout << "cameraMatrix left" << endl;
	cout << cameraMatrix_[0] << endl;
	cout << "distCoeffs left" << endl;
	cout << distCoeffs_[0] << endl;
	cout << "cameraMatrix right" << endl;
	cout << cameraMatrix_[1] << endl;
	cout << "distCoeffs right" << endl;
	cout << distCoeffs_[1] << endl;
	cout << "cameraMatrix projector" << endl;
	cout << cameraMatrix_[2] << endl;
	cout << "distCoeffs projector" << endl;
	cout << distCoeffs_[2] << endl;
	cout << endl;
	cout << final_error_p << "::  Reprojection Error left : " << final_error_l << " of " << objectPoints_l.size() << " images   _   Reprojection Error right : " << final_error_r << " of " << objectPoints_l.size() << " images" <<
		endl;

	//	int good_l_index, good_r_index;
	//	for (int good_index = 0; good_index < image_number; good_index++)
	//	{
	//		int l_idx = -1, r_idx = -1;
	//		for (int l_index = 0; l_index < calibrate_image_index_l.size(); l_index++)
	//		{				
	//			if (calibrate_image_index_l[l_index] == good_index)
	//			{
	//				for (int r_index = 0; r_index < calibrate_image_index_r.size(); r_index++)
	//				{
	//					if (calibrate_image_index_r[r_index] == good_index)
	//					{
	//						good_l_index = l_index;
	//						good_r_index = r_index;
	//						goto go_to_number_1;
	//					}
	//				}
	//			}
	//		}
	//	}
	//
	//go_to_number_1:
	//	cout << "good left index = " << good_l_index << "   good right index = " << good_r_index << endl;


	cv::Mat Rot_g_l(3, 3, cv::DataType<double>::type);
	Rodrigues(rvecs_l[0], Rot_g_l);
	cv::Mat Rot_g_p(3, 3, cv::DataType<double>::type);
	Rodrigues(rvecs_projector[0], Rot_g_p);
	cv::Mat R_p_l(3, 3, cv::DataType<double>::type);
	cv::Mat T_p_l(3, 1, cv::DataType<double>::type);
	R_p_l = Rot_g_l*Rot_g_p.inv();
	T_p_l = tvecs_l[0] - R_p_l*tvecs_projector[0];

	cv::Mat Rot_g_r(3, 3, cv::DataType<double>::type);
	Rodrigues(rvecs_r[0], Rot_g_r);
	cv::Mat R_p_r(3, 3, cv::DataType<double>::type);
	cv::Mat T_p_r(3, 1, cv::DataType<double>::type);
	R_p_r = Rot_g_r*Rot_g_p.inv();
	T_p_r = tvecs_r[0] - R_p_r*tvecs_projector[0];

	cv::Mat R_l_r(3, 3, cv::DataType<double>::type);
	cv::Mat T_l_r(3, 1, cv::DataType<double>::type);
	R_l_r = Rot_g_r*Rot_g_l.inv();
	T_l_r = tvecs_r[0] - R_l_r*tvecs_l[0];

	cv::Mat R_r_l(3, 3, cv::DataType<double>::type);
	cv::Mat T_r_l(3, 1, cv::DataType<double>::type);
	R_r_l = Rot_g_l*Rot_g_r.inv();
	T_r_l = tvecs_l[0] - R_r_l*tvecs_r[0];

	//vector<double> points_show;
	//double x_error = 0, y_error = 0, z_error = 0;
	//for (int image_index = 0; image_index < image_number; image_index++)
	//{
	//	cv::Mat Rot_g_l2(3, 3, cv::DataType<double>::type);
	//	Rodrigues(rvecs_l[image_index], Rot_g_l2);
	//	cv::Mat Rot_g_r2(3, 3, cv::DataType<double>::type);
	//	Rodrigues(rvecs_r[image_index], Rot_g_r2);
	//	cv::Mat t_g_l2 = tvecs_l[image_index];
	//	cv::Mat t_g_r2 = tvecs_r[image_index];

	//	if (objectPoints_l[image_index].size()>0 && objectPoints_r[image_index].size()>0)
	//	{
	//		vector<cv::Point2f> p1(99, cv::Point2f(0, 0)), p2(99, cv::Point2f(0, 0));
	//		vector<cv::Point3f> object1(99);
	//		for (int i = 0; i < CCPoints_index_l[image_index].size(); i++)
	//		{
	//			p1[CCPoints_index_l[image_index][i]] = imagePoints_l[image_index][i];
	//			object1[CCPoints_index_l[image_index][i]] = objectPoints_l[image_index][i];
	//		}
	//		for (int i = 0; i < CCPoints_index_r[image_index].size(); i++)
	//		{
	//			p2[CCPoints_index_r[image_index][i]] = imagePoints_r[image_index][i];
	//		}

	//		cv::undistortPoints(p1, p1, cameraMatrix_[0], distCoeffs_[0], cv::noArray(), cameraMatrix_[0]);
	//		cv::undistortPoints(p2, p2, cameraMatrix_[1], distCoeffs_[1], cv::noArray(), cameraMatrix_[1]);


	//		for (int i = 0; i < 99; i++)
	//		{

	//			if (p1[i].x>0 && p2[i].x>0)
	//			{


	//				Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
	//				Eigen::VectorXd B_ = Eigen::VectorXd(4);
	//				Eigen::VectorXd X_ = Eigen::VectorXd(3);

	//				double u1 = p1[i].x; double v1 = p1[i].y;
	//				double u2 = p2[i].x; double v2 = p2[i].y;
	//				double r1 = Rot_g_l2.at<double>(0, 0), r2 = Rot_g_l2.at<double>(0, 1), r3 = Rot_g_l2.at<double>(0, 2),
	//					r4 = Rot_g_l2.at<double>(1, 0), r5 = Rot_g_l2.at<double>(1, 1), r6 = Rot_g_l2.at<double>(1, 2),
	//					r7 = Rot_g_l2.at<double>(2, 0), r8 = Rot_g_l2.at<double>(2, 1), r9 = Rot_g_l2.at<double>(2, 2);
	//				double t1 = t_g_l2.at<double>(0), t2 = t_g_l2.at<double>(1), t3 = t_g_l2.at<double>(2);
	//				double fx1 = cameraMatrix_[0].at<double>(0, 0), fy1 = cameraMatrix_[0].at<double>(1, 1), cx1 = cameraMatrix_[0].at<double>(0, 2), cy1 = cameraMatrix_[0].at<double>(1, 2);
	//				double r_1 = Rot_g_r2.at<double>(0, 0), r_2 = Rot_g_r2.at<double>(0, 1), r_3 = Rot_g_r2.at<double>(0, 2),
	//					r_4 = Rot_g_r2.at<double>(1, 0), r_5 = Rot_g_r2.at<double>(1, 1), r_6 = Rot_g_r2.at<double>(1, 2),
	//					r_7 = Rot_g_r2.at<double>(2, 0), r_8 = Rot_g_r2.at<double>(2, 1), r_9 = Rot_g_r2.at<double>(2, 2);
	//				double t_1 = t_g_r2.at<double>(0), t_2 = t_g_r2.at<double>(1), t_3 = t_g_r2.at<double>(2);
	//				double fx2 = cameraMatrix_[1].at<double>(0, 0), fy2 = cameraMatrix_[1].at<double>(1, 1), cx2 = cameraMatrix_[1].at<double>(0, 2), cy2 = cameraMatrix_[1].at<double>(1, 2);

	//				A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
	//				A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

	//				A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
	//				A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;

	//				X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);

	//				double x = X_(0), y = X_(1), z = X_(2);

	//				x_error += abs(x - object1[i].x);
	//				y_error += abs(y - object1[i].y);
	//				z_error += abs(z - object1[i].z);

	//				points_show.push_back(x);
	//				points_show.push_back(y);
	//				points_show.push_back(z);

	//				cout << "u = " << p1[i].x << " v = " << p1[i].y << " x = " << x << " y = " << y << " z = " << z << " x^ = " << object1[i].x << " y^ = " << object1[i].y << " z^ = " << object1[i].z << endl;
	//			}
	//		}

	//	}
	//}
	//x_error /= (points_show.size() / 3);
	//y_error /= (points_show.size() / 3);
	//z_error /= (points_show.size() / 3);
	//cout << " ex = " << x_error << " ey = " << y_error << " ez = " << z_error << endl;

	//stringstream ss1, ss2, ss3;
	//ss1 << x_error; ss2 << y_error; ss3 << z_error;
	//MessageBoxA(NULL, ("x = " + ss1.str() + ", y = " + ss2.str() + ", z = " + ss3.str() + ";").c_str(), "误差分析", 1);

	cv::Mat C(3, 3, cv::DataType<double>::type);
	C.at<double>(0, 0) = 0; C.at<double>(0, 1) = -T_l_r.at<double>(2, 0); C.at<double>(0, 2) = T_l_r.at<double>(1, 0);
	C.at<double>(1, 0) = T_l_r.at<double>(2, 0); C.at<double>(1, 1) = 0; C.at<double>(1, 2) = -T_l_r.at<double>(0, 0);
	C.at<double>(2, 0) = -T_l_r.at<double>(1, 0); C.at<double>(2, 1) = T_l_r.at<double>(0, 0); C.at<double>(2, 2) = 0;

	cv::Mat C2(3, 3, cv::DataType<double>::type);
	C2.at<double>(0, 0) = 0; C2.at<double>(0, 1) = -T_r_l.at<double>(2, 0); C2.at<double>(0, 2) = T_r_l.at<double>(1, 0);
	C2.at<double>(1, 0) = T_r_l.at<double>(2, 0); C2.at<double>(1, 1) = 0; C2.at<double>(1, 2) = -T_r_l.at<double>(0, 0);
	C2.at<double>(2, 0) = -T_r_l.at<double>(1, 0); C2.at<double>(2, 1) = T_r_l.at<double>(0, 0); C2.at<double>(2, 2) = 0;

	cv::Mat E(3, 3, cv::DataType<double>::type);
	E = C*R_l_r;
	cv::Mat F_r_l(3, 3, cv::DataType<double>::type);
	F_r_l = cameraMatrix_[1].t().inv()*E*cameraMatrix_[0].inv();

	Mat E2(3, 3, cv::DataType<double>::type);
	E2 = C2*R_r_l;
	Mat F_l_r(3, 3, cv::DataType<double>::type);
	F_l_r = cameraMatrix_[0].t().inv()*E2*cameraMatrix_[1].inv();

	vector<double> global_l_points;
	vector<double> global_l_phase;
	vector<double> global_l_period;
	for (int group_index = 0; group_index < objectPoints_l.size(); group_index++)
	{
		cv::Mat Rot_l(3, 3, cv::DataType<double>::type);
		Rodrigues(rvecs_l[group_index], Rot_l);
		for (int point_index = 0; point_index < objectPoints_l[group_index].size(); point_index++)
		{
			double x = objectPoints_l[group_index][point_index].x*Rot_l.at<double>(0, 0) + objectPoints_l[group_index][point_index].y*Rot_l.at<double>(0, 1) + tvecs_l[group_index].at<double>(0, 0);
			double y = objectPoints_l[group_index][point_index].x*Rot_l.at<double>(1, 0) + objectPoints_l[group_index][point_index].y*Rot_l.at<double>(1, 1) + tvecs_l[group_index].at<double>(1, 0);
			double z = objectPoints_l[group_index][point_index].x*Rot_l.at<double>(2, 0) + objectPoints_l[group_index][point_index].y*Rot_l.at<double>(2, 1) + tvecs_l[group_index].at<double>(2, 0);
			global_l_points.push_back(x);
			global_l_points.push_back(y);
			global_l_points.push_back(z);

			global_l_phase.push_back(projector_phase_whole_l[group_index][point_index]);
			global_l_period.push_back(projector_period_whole_l[group_index][point_index]);
		}
	}

	vector<double> global_r_points;
	vector<double> global_r_phase;
	vector<double> global_r_period;
	for (int group_index = 0; group_index < objectPoints_r.size(); group_index++)
	{
		cv::Mat Rot_r(3, 3, cv::DataType<double>::type);
		Rodrigues(rvecs_r[group_index], Rot_r);
		for (int point_index = 0; point_index < objectPoints_r[group_index].size(); point_index++)
		{
			double x = objectPoints_r[group_index][point_index].x*Rot_r.at<double>(0, 0) + objectPoints_r[group_index][point_index].y*Rot_r.at<double>(0, 1) + tvecs_r[group_index].at<double>(0, 0);
			double y = objectPoints_r[group_index][point_index].x*Rot_r.at<double>(1, 0) + objectPoints_r[group_index][point_index].y*Rot_r.at<double>(1, 1) + tvecs_r[group_index].at<double>(1, 0);
			double z = objectPoints_r[group_index][point_index].x*Rot_r.at<double>(2, 0) + objectPoints_r[group_index][point_index].y*Rot_r.at<double>(2, 1) + tvecs_r[group_index].at<double>(2, 0);
			global_r_points.push_back(x);
			global_r_points.push_back(y);
			global_r_points.push_back(z);

			global_r_phase.push_back(projector_phase_whole_r[group_index][point_index]);
			global_r_period.push_back(projector_period_whole_r[group_index][point_index]);
		}
	}

	//Eigen::VectorXd A_l;
	CalculateProjectorCameraParameter(global_l_points, global_l_period, global_l_phase, c_p_system_l);
	CalculateProjectorCameraParameter(global_r_points, global_r_period, global_r_phase, c_p_system_r);

	cv::FileStorage fs(out_put_name, cv::FileStorage::WRITE);

	//cv::FileStorage fs_g("extern_paramater.xml", cv::FileStorage::WRITE);
	//fs_g << "R" << R;
	//fs_g << "T" << T;
	//fs_g << "F" << F;
	//fs_g << "F2" << F2;
	//fs_g << "Rot_l" << Rot_l;
	//fs_g << "tvec_l" << tvec_l;
	//fs_g << "Rot_r" << Rot_r;
	//fs_g << "tvec_r" << tvec_r;
	//fs_g.release();

	fs << "cameraMatrix_l" << cameraMatrix_[0];
	fs << "distCoeffs_l" << distCoeffs_[0];
	fs << "cameraMatrix_r" << cameraMatrix_[1];
	fs << "distCoeffs_r" << distCoeffs_[1];
	fs << "projectorMatrix_r" << cameraMatrix_[2];
	fs << "projectordistCoeffs_r" << distCoeffs_[2];

	fs << "R" << R_l_r;
	fs << "T" << T_l_r;
	fs << "F" << F_r_l;
	fs << "F2" << F_l_r;
	fs << "Rot_l" << Rot_g_l;
	fs << "tvec_l" << tvecs_l[0];
	fs << "Rot_r" << Rot_g_r;
	fs << "tvec_r" << tvecs_r[0];
	fs << "Rot_p" << Rot_g_p;
	fs << "tvec_p" << tvecs_projector[0];
	fs << "c_p_system_l" << c_p_system_l;
	fs << "c_p_system_r" << c_p_system_r;
	fs.release();

	InitRasterScan(out_put_name);

	cv::destroyWindow("Calibration left camera Images");
	cv::destroyWindow("Calibration right camera Images");
	cv::destroyWindow("Calibration fringe horizontal Images");
	cv::destroyWindow("Calibration projector circle center Images");

	//Mat C(3, 3, cv::DataType<double>::type);
	//C.at<double>(0, 0) = 0; C.at<double>(0, 1) = -T_p_l.at<double>(2, 0); C.at<double>(0, 2) = T_p_l.at<double>(1, 0);
	//C.at<double>(1, 0) = T_p_l.at<double>(2, 0); C.at<double>(1, 1) = 0; C.at<double>(1, 2) = -T_p_l.at<double>(0, 0);
	//C.at<double>(2, 0) = -T_p_l.at<double>(1, 0); C.at<double>(2, 1) = T_p_l.at<double>(0, 0); C.at<double>(2, 2) = 0;

	//Mat C2(3, 3, cv::DataType<double>::type);
	//C2.at<double>(0, 0) = 0; C2.at<double>(0, 1) = -T_p_r.at<double>(2, 0); C2.at<double>(0, 2) = T_p_r.at<double>(1, 0);
	//C2.at<double>(1, 0) = T_p_r.at<double>(2, 0); C2.at<double>(1, 1) = 0; C2.at<double>(1, 2) = -T_p_r.at<double>(0, 0);
	//C2.at<double>(2, 0) = -T_p_r.at<double>(1, 0); C2.at<double>(2, 1) = T_p_r.at<double>(0, 0); C2.at<double>(2, 2) = 0;

	//Mat E(3, 3, cv::DataType<double>::type);
	//E = C*R_p_l;
	//Mat F(3, 3, cv::DataType<double>::type);
	//F = cameraMatrix[0].t().inv()*E*cameraMatrix[2].inv();
	////F : e = (point l.t() * F).t()

	//Mat E2(3, 3, cv::DataType<double>::type);
	//E2 = C2*R_p_r;
	//Mat F2(3, 3, cv::DataType<double>::type);
	//F2 = cameraMatrix[1].t().inv()*E2*cameraMatrix[2].inv();
	////F2 : e = (point r.t() * F2).t()



}

void RasterScan::PlaneRTCalculate(vector<cv::Mat> &left_circle_images, vector<cv::Mat> &right_circle_images, const string RT_file_name, vector<double> &totle_mask_point)
{
	vector<double> points_end;
	vector<double> points_cloud_globle;
	vector<double> points_mask2;
	vector<Eigen::VectorXd> currect_mask_points2;

	double minCircularity = 0.3;
	double maxCircularity = 1.85;
	double minContourArea = 1000;
	double maxContourArea = 15000;
	double detaThred = 20;
	//rs.XY_TsdfVolume(150, 765, 1.5, imgr.cols, imgr.rows);

	cv::FileStorage fs(RT_file_name, cv::FileStorage::WRITE);

	for (int image_index = 0; image_index < left_circle_images.size(); image_index++)
	{
		stringstream ss;
		string index_;
		ss << image_index;
		ss >> index_;

		Mat imgl_mask = left_circle_images[image_index];
		//imgl_mask = Distortion(imgl_mask, 0);
		//imgl_mask = imgl_mask*0.9;
		//cv::imwrite("./images/intr_left/g5.bmp", imgl_mask);
		Mat imgr_mask = right_circle_images[image_index];
		//imgr_mask = Distortion(imgr_mask, 1);
		//imgr_mask = imgr_mask*0.9;
		//cv::imwrite("./images/intr_right/g5.bmp", imgr_mask);
		Mat imgl_mask2;
		Mat imgr_mask2;
		imgl_mask.copyTo(imgl_mask2);
		imgr_mask.copyTo(imgr_mask2);
		cv::cvtColor(imgl_mask2, imgl_mask2, CV_GRAY2BGR);
		cv::cvtColor(imgr_mask2, imgr_mask2, CV_GRAY2BGR);

		std::vector<cv::Point2d> MaskPoints_left;
		std::vector<cv::Point2d> MaskPoints_right;
		CircleCenterCalculate2(imgl_mask, MaskPoints_left);
		for (int point_index = 0; point_index < MaskPoints_left.size(); point_index++)
		{
			double x, y;
			UndistortionPoint(MaskPoints_left[point_index].x, MaskPoints_left[point_index].y, intr1, distCoeffs[0], x, y);
			MaskPoints_left[point_index].x = x;
			MaskPoints_left[point_index].y = y;
		}
		CircleCenterCalculate2(imgr_mask, MaskPoints_right);
		for (int point_index = 0; point_index < MaskPoints_right.size(); point_index++)
		{
			double x, y;
			UndistortionPoint(MaskPoints_right[point_index].x, MaskPoints_right[point_index].y, intr2, distCoeffs[1], x, y);
			MaskPoints_right[point_index].x = x;
			MaskPoints_right[point_index].y = y;
		}
		//blobRecognition(imgl_mask, MaskPoints_left, minCircularity, maxCircularity, minContourArea, maxContourArea, detaThred);
		//blobRecognition(imgr_mask, MaskPoints_right, minCircularity, maxCircularity, minContourArea, maxContourArea, detaThred);
		for (uint i = 0; i < MaskPoints_left.size(); i++)
		{
			stringstream ss;
			ss << i;
			string index_;
			ss >> index_;
			cv::RNG& rng = cv::theRNG();
			cv::Scalar color2 = cv::Scalar(rng(256), rng(256), rng(256));
			circle(imgl_mask2, MaskPoints_left[i], 0.5, color2);
			putText(imgl_mask2, index_, MaskPoints_left[i], cv::FONT_HERSHEY_SIMPLEX, 1, color2, 4);
			circle(imgr_mask2, MaskPoints_right[i], 0.5, color2);
			putText(imgr_mask2, index_, MaskPoints_right[i], cv::FONT_HERSHEY_SIMPLEX, 1, color2, 4);
		}
		cout << "left " << MaskPoints_left.size() << ", right " << MaskPoints_right.size() << endl;
		//calculate mask points 0
		vector<Eigen::VectorXd> currect_mask_points;
		vector<double> points_mask;
		EpiLinesFindPoint(imgl_mask2, imgr_mask2, MaskPoints_left, MaskPoints_right);
		cout << "matched mask point :" << MaskPoints_left.size() << endl;
		Calculate3DMaskPoints(MaskPoints_left, MaskPoints_right, points_mask, currect_mask_points);

		//calculate rotation and transformation matrix
		Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
		cv::Mat rt_mat(4, 4, CV_64FC1);
		bool matched = EdgeSearch(currect_mask_points, Rt);
		if (!matched)
		{
			fs << "rt_mat" + index_ << rt_mat;
			continue;
		}
		Eigen::Matrix4d Rt_trans = Rt.transpose();
		memcpy(rt_mat.data, Rt_trans.data(), 16 * sizeof(double));
		cout << Rt << endl;
		cout << rt_mat << endl;
		fs << "rt_mat" + index_ << rt_mat;

		for (int mask_index = 0; mask_index < currect_mask_points.size(); mask_index++)
		{
			Eigen::Vector4d point1(currect_mask_points[mask_index][0], currect_mask_points[mask_index][1], currect_mask_points[mask_index][2], 1),
				point2;
			point2 = Rt*point1;
			totle_mask_point.push_back(point2[0]);
			totle_mask_point.push_back(point2[1]);
			totle_mask_point.push_back(point2[2]);
		}



	}
	fs.release();
}

void RasterScan::CalculateProjectorCameraParameter(const vector<double> &Points, const vector<double> &k, const vector<double> &t, cv::Mat &X)
{
	int good_theta_number = 0;
	vector<double> p2, k2, t2;
	for (int point_index = 0; point_index < k.size(); point_index++)
	{

		if (k[point_index] == -1 || t[point_index] == -1 || t[point_index] >= 1 || t[point_index] <= 0)
		{
			//cout << "wrong!!" << endl;
			continue;
		}
		p2.push_back(Points[point_index * 3 + 0]); p2.push_back(Points[point_index * 3 + 1]); p2.push_back(Points[point_index * 3 + 2]);
		t2.push_back(t[point_index]); k2.push_back(k[point_index]);
		good_theta_number++;
	}

	//A = Eigen::VectorXd::Zero(7);
	//Eigen::MatrixXd A_plane = Eigen::MatrixXd(good_theta_number, 7);
	//Eigen::VectorXd B_plane = Eigen::VectorXd(good_theta_number);
	//Eigen::VectorXd X_plane = Eigen::VectorXd(7);

	X = cv::Mat::zeros(7, 1, CV_64FC1);
	cv::Mat A = cv::Mat::zeros(good_theta_number, 7, CV_64FC1);
	cv::Mat B = cv::Mat::zeros(good_theta_number, 1, CV_64FC1);

	for (int point_index = 0; point_index < good_theta_number; point_index++)
	{

		double x = p2[point_index * 3 + 0];
		double y = p2[point_index * 3 + 1];
		double z = p2[point_index * 3 + 2];
		double theta = k2[point_index] * 64 + t2[point_index];
		//cout << "No."<<point_index<<" x = " << x << " y = " << y << " z = " << z << " k = " << k[point_index] << " t = " << t[point_index] << endl;
		A.at<double>(point_index, 0) = x;
		A.at<double>(point_index, 1) = y;
		A.at<double>(point_index, 2) = z;
		A.at<double>(point_index, 3) = 1;
		A.at<double>(point_index, 4) = -1 * theta*x;
		A.at<double>(point_index, 5) = -1 * theta*y;
		A.at<double>(point_index, 6) = -1 * theta*z;
		B.at<double>(point_index, 0) = theta;
	}

	//X_plane = A_plane.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_plane);
	////cout << "Aplane : " << endl << A_plane << endl;
	////cout << "Xplane : " << endl << X_plane << endl;
	//A = X_plane;

	cv::solve(A, B, X, CV_SVD);
}

void RasterScan::OneCameraPointCloudCalculate(cv::Mat &periodic_, cv::Mat &phase_, vector<double> &point_cloud, const bool camera_flag)
{
	//vector<Mat> left_images;
	//calculate periodic 0 & phase 0

	Mat cameraMatrix;
	Mat Rot_inv, T_inv;
	//Eigen::VectorXd A_;
	Mat X_;
	if (!camera_flag)
	{
		cameraMatrix = intr1;
		//A_ = A_l;
		X_ = c_p_system_l;
		Rot_inv = Rot_l.inv();
		T_inv = -1 * Rot_inv * tvec_l;
	}
	else
	{
		cameraMatrix = intr2;
		//A_ = A_r;
		X_ = c_p_system_r;
		Rot_inv = Rot_r.inv();
		T_inv = -1 * Rot_inv * tvec_r;
	}
	//ReadScanData("./new_scan/r" + index_ + "/", left_images, cameraMatrix[0], distCoeffs[0], 0);
	//vc::UnwrapComp(images_in, phase_left1, periodic_left1);

	Mat depth_map = Mat::zeros(phase_.size(), CV_64FC1);
	for (int m = 0; m < phase_.cols; m++)
	{
		for (int n = 0; n < phase_.rows; n++)
		{
			double k = periodic_.at<double>(n, m);
			double t = phase_.at<double>(n, m);
			if (k == -1 || t == -1 || t<0 || k <= 0)
			{
				continue;
			}
			double theta = k * 64 + t;
			double q1 = (m - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
			double q2 = (n - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);
			double z = (theta - X_.at<double>(3)) / (X_.at<double>(0)*q1 + X_.at<double>(1)*q2 + X_.at<double>(2) - theta*(X_.at<double>(4)*q1 + X_.at<double>(5)*q2 + X_.at<double>(6)));
			double x = q1*z;
			double y = q2*z;
			double dis = sqrt(x*x + y*y + z*z);
			if (dis > 1000 || z < 0)
			{
				continue;
			}
			depth_map.at<double>(n, m) = z / 1000.0;

			Eigen::Vector4d point(x, y, z, 1);
			//Eigen::Vector4d point_ = Rt*point;


			double x2 = Rot_inv.at<double>(0, 0)*point(0) + Rot_inv.at<double>(0, 1)*point(1) + Rot_inv.at<double>(0, 2)*point(2) + T_inv.at<double>(0, 0);
			double y2 = Rot_inv.at<double>(1, 0)*point(0) + Rot_inv.at<double>(1, 1)*point(1) + Rot_inv.at<double>(1, 2)*point(2) + T_inv.at<double>(1, 0);
			double z2 = Rot_inv.at<double>(2, 0)*point(0) + Rot_inv.at<double>(2, 1)*point(1) + Rot_inv.at<double>(2, 2)*point(2) + T_inv.at<double>(2, 0);

			point_cloud.push_back(x2);
			point_cloud.push_back(y2);
			point_cloud.push_back(z2);
		}
	}
}

void RasterScan::CompareCameraAndProjectorPointCloud(cv::Mat &periodic_, cv::Mat &phase_, cv::Mat &depth_map, vector<cv::Point2d> &points1, vector<cv::Point2d> &points2, vector<double> &point_cloud, double threshold_value)
{
	//vector<Mat> left_images;
	//calculate periodic 0 & phase 0
	depth_map = cv::Mat::zeros(phase_.size(), CV_64FC1);
	Mat cameraMatrix = intr2;
	Mat Rot_inv = Rot_r.inv();
	Mat T_inv = -1 * Rot_inv * tvec_r;
	//Eigen::VectorXd A_;
	Mat x_ = c_p_system_r;

	//vector<double> points;
	for (int point_index = 0; point_index < points1.size(); point_index++)
	{
		//cout << point_index << " : " << points2[point_index].x << " ," << points2[point_index].y << endl;
		Eigen::MatrixXd A_ = Eigen::MatrixXd(4, 3);
		Eigen::VectorXd B_ = Eigen::VectorXd(4);
		Eigen::VectorXd X_ = Eigen::VectorXd(3);

		double u1 = points1[point_index].x; double v1 = points1[point_index].y;
		double u2 = points2[point_index].x; double v2 = points2[point_index].y;
		double r1 = Rot_l.at<double>(0, 0), r2 = Rot_l.at<double>(0, 1), r3 = Rot_l.at<double>(0, 2),
			r4 = Rot_l.at<double>(1, 0), r5 = Rot_l.at<double>(1, 1), r6 = Rot_l.at<double>(1, 2),
			r7 = Rot_l.at<double>(2, 0), r8 = Rot_l.at<double>(2, 1), r9 = Rot_l.at<double>(2, 2);
		double t1 = tvec_l.at<double>(0), t2 = tvec_l.at<double>(1), t3 = tvec_l.at<double>(2);
		double fx1 = intr1.at<double>(0, 0), fy1 = intr1.at<double>(1, 1), cx1 = intr1.at<double>(0, 2), cy1 = intr1.at<double>(1, 2);
		double r_1 = Rot_r.at<double>(0, 0), r_2 = Rot_r.at<double>(0, 1), r_3 = Rot_r.at<double>(0, 2),
			r_4 = Rot_r.at<double>(1, 0), r_5 = Rot_r.at<double>(1, 1), r_6 = Rot_r.at<double>(1, 2),
			r_7 = Rot_r.at<double>(2, 0), r_8 = Rot_r.at<double>(2, 1), r_9 = Rot_r.at<double>(2, 2);
		double t_1 = tvec_r.at<double>(0), t_2 = tvec_r.at<double>(1), t_3 = tvec_r.at<double>(2);
		double fx2 = intr2.at<double>(0, 0), fy2 = intr2.at<double>(1, 1), cx2 = intr2.at<double>(0, 2), cy2 = intr2.at<double>(1, 2);

		A_(0, 0) = u1*r7 - cx1*r7 - fx1*r1; A_(0, 1) = u1*r8 - cx1*r8 - fx1*r2; A_(0, 2) = u1*r9 - cx1*r9 - fx1*r3; B_(0) = cx1*t3 - u1*t3 + fx1*t1;
		A_(1, 0) = v1*r7 - cy1*r7 - fy1*r4; A_(1, 1) = v1*r8 - cy1*r8 - fy1*r5; A_(1, 2) = v1*r9 - cy1*r9 - fy1*r6; B_(1) = cy1*t3 - v1*t3 + fy1*t2;

		A_(2, 0) = u2*r_7 - cx2*r_7 - fx2*r_1; A_(2, 1) = u2*r_8 - cx2*r_8 - fx2*r_2; A_(2, 2) = u2*r_9 - cx2*r_9 - fx2*r_3; B_(2) = cx2*t_3 - u2*t_3 + fx2*t_1;
		A_(3, 0) = v2*r_7 - cy2*r_7 - fy2*r_4; A_(3, 1) = v2*r_8 - cy2*r_8 - fy2*r_5; A_(3, 2) = v2*r_9 - cy2*r_9 - fy2*r_6; B_(3) = cy2*t_3 - v2*t_3 + fy2*t_2;


		//cout << "The least-squares solution is:\n"
		//	<< A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_)<<endl;
		X_ = A_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_);

		double x = X_(0), y = X_(1), z = X_(2);

		//if (u2==122&&v2==468)
		//{
		//	cout << "A :" << endl << A_ << endl;
		//	cout << "B_ :" << endl << B_ << endl;
		//	cout << "X_ :" << endl << X_ << endl;
		//}


		double k = periodic_.at<double>(points2[point_index].y, points2[point_index].x);
		double t = phase_.at<double>(points2[point_index].y, points2[point_index].x);
		if (k == -1 || t == -1 || t<0 || k <= 0)
		{
			continue;
		}
		double theta = t;
		double q1 = (points2[point_index].x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
		double q2 = (points2[point_index].y - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);
		double z3 = (theta - x_.at<double>(3)) / (x_.at<double>(0)*q1 + x_.at<double>(1)*q2 + x_.at<double>(2) - theta*(x_.at<double>(4)*q1 + x_.at<double>(5)*q2 + x_.at<double>(6)));
		double x3 = q1*z3;
		double y3 = q2*z3;
		double dis = sqrt(x3*x3 + y3*y3 + z3*z3);
		if (dis > 1000 || z3 < 0)
		{
			continue;
		}

		double x2 = Rot_inv.at<double>(0, 0)*x3 + Rot_inv.at<double>(0, 1)*y3 + Rot_inv.at<double>(0, 2)*z3 + T_inv.at<double>(0, 0);
		double y2 = Rot_inv.at<double>(1, 0)*x3 + Rot_inv.at<double>(1, 1)*y3 + Rot_inv.at<double>(1, 2)*z3 + T_inv.at<double>(1, 0);
		double z2 = Rot_inv.at<double>(2, 0)*x3 + Rot_inv.at<double>(2, 1)*y3 + Rot_inv.at<double>(2, 2)*z3 + T_inv.at<double>(2, 0);

		if (Distance(x, y, z, x2, y2, z2)<threshold_value)
		{
			point_cloud.push_back(x);
			point_cloud.push_back(y);
			point_cloud.push_back(z);
			depth_map.at<double>(points2[point_index].y, points2[point_index].x) = z3;
		}
	}

}



float GammaRight2(float x)
{
	float gamma_coef[8]
		/*{
		3.14479411e-01, 3.44367743e+00, -1.91460133e+01,
		7.03408966e+01, -1.51314743e+02, 1.85934860e+02, -1.20647781e+02,
		3.20093307e+01
		};*/ // pg cam big
	{
		1.36380166e-01, 5.73008633e+00, -3.23815804e+01,
		1.12941849e+02, -2.30589554e+02, 2.71262177e+02, -1.70173294e+02,
		4.40663795e+01
	};
	return gamma_coef[0]
		+ gamma_coef[1] * pow(x, 1)
		+ gamma_coef[2] * pow(x, 2)
		+ gamma_coef[3] * pow(x, 3)
		+ gamma_coef[4] * pow(x, 4)
		+ gamma_coef[5] * pow(x, 5)
		+ gamma_coef[6] * pow(x, 6)
		+ gamma_coef[7] * pow(x, 7);
}

void PropagateRow(Mat &img)
{
	for (int hi = 1; hi < img.rows; ++hi)
		img.row(0).copyTo(img.row(hi));
}

Mat RasterScan::PatternPlain(int n, float f, int N, float init_phase)
{
	const float Ap = 0.5;
	const float Bp = 0.4;
	//const float f = 1;

	Mat pimg(kPatternSize, CV_32FC1);
	for (int wi = 0; wi < pimg.cols; ++wi)
	{
		float xp = wi * 1.f / pimg.cols;
		float intensity = Ap + Bp * cos(2 * CV_PI * f * xp - 2 * CV_PI * n / N + init_phase);
		//intensity /= 255.f;
		pimg.at<float>(0, wi) = GammaRight2(intensity);
	}

	PropagateRow(pimg);

	return pimg;
}

void UnwrappingPlain(const vector<Mat> &cimg, Mat &phi_out, int N, int base_idx)
{
	const cv::Size img_size = cv::Size(cimg[0].cols, cimg[0].rows);
	Mat Ac(img_size, CV_64FC1);
	for (int pi = 0; pi < Ac.total(); ++pi)
	{
		double intensity_sum = 0;
		for (int ii = 0; ii < N; ++ii)
			intensity_sum += cimg[base_idx + ii].at<unsigned char>(pi) / 255.0;
		// avg
		Ac.at<double>(pi) = intensity_sum / N;
	}
	Mat Bc(img_size, CV_64FC1);
	Mat phi = Mat::ones(img_size, CV_64FC1);
	phi *= -2;
	const double thresh_bc = 0.02;
	for (int pi = 0; pi < Ac.total(); ++pi)
	{
		double p1_sum = 0, p2_sum = 0;
		for (int ii = 0; ii < N; ++ii)
		{
			p1_sum += cimg[base_idx + ii].at<unsigned char>(pi) * sin(2 * CV_PI * ii / N);
			p2_sum += cimg[base_idx + ii].at<unsigned char>(pi) * cos(2 * CV_PI * ii / N);
		}
		Bc.at<double>(pi) = 2.0 / N * sqrt(pow(p1_sum, 2) + pow(p2_sum, 2)) / 255.0;

		if (Bc.at<double>(pi) > thresh_bc)
			phi.at<double>(pi) = atan2(p1_sum, p2_sum) / CV_PI;
	}
	phi_out = phi;
}

void RasterScan::PatternsComp(vector<Mat> &pimg)
{
	pimg.clear();

	for (int pi = 0; pi < kOrder - 1; ++pi)
		for (int ni = 0; ni < kN_comp; ++ni)
			pimg.push_back(PatternPlain(ni, pow(kF_comp, pi), kN_comp, 0.f));

	for (int ni = 0; ni < kN_comp_last; ++ni)
		pimg.push_back(PatternPlain(ni, pow(kF_comp, kOrder - 1), kN_comp_last, 0.f));

	for (int ni = 0; ni < kN_comp_last; ++ni)
		pimg.push_back(PatternPlain(ni, pow(kF_comp, kOrder - 1), kN_comp_last, CV_PI / kN_comp_last));
}

void RasterScan::CompensatePhase(const Mat &phi1, const Mat &phi2, Mat &phi_out)
{
	const int N = kN_comp_last;

	Mat phi1_ = phi1.clone();
	Mat phi2_ = phi2.clone();
	const cv::Size img_size = cv::Size(phi1_.cols, phi1_.rows);

	Mat valid = (phi1_ != -2) & (phi2_ != -2);

	Mat temp = phi1_ < 0;
	temp.convertTo(temp, CV_64FC1, 2.f / 255);
	phi1_ = (phi1_ + temp) / 2;

	temp = phi2_ < 0;
	temp.convertTo(temp, CV_64FC1, 2.f / 255);
	phi2_ = (phi2_ + temp) / 2;

	Mat phi = Mat::ones(img_size, CV_64FC1);
	phi *= -1;
	for (int pi = 0; pi < img_size.area(); ++pi)
		if (valid.at<unsigned char>(pi) != 0)
		{
			if (phi1_.at<double>(pi) < phi2_.at<double>(pi))
				phi.at<double>(pi) = (phi1_.at<double>(pi) + phi2_.at<double>(pi) - 1.0 / (2 * N)) / 2.0;
			else
				phi.at<double>(pi) = (1.0 + (phi1_.at<double>(pi) + phi2_.at<double>(pi) - 1.0 / (2 * N))) / 2.0;
		}

	/*const double error_threshold = 0.5 / (2 * CV_PI);
	for (int pi = 0; pi < img_size.area(); ++pi)
	if (valid.at<unsigned char>(pi) != 0)
	{
	double error_value = 0;
	if (phi1_.at<double>(pi) < phi2_.at<double>(pi))
	error_value = phi2_.at<double>(pi) - phi1_.at<double>(pi) - 1.0 / (2 * N);
	else
	error_value = 1.0 + (phi2_.at<double>(pi) - phi1_.at<double>(pi) - 1.0 / (2 * N));

	if (abs(error_value) > error_threshold)
	phi.at<double>(pi) = -1;
	}*/
	phi_out = phi;
}

void RasterScan::UnwrapComp(const vector<Mat> &cimg, Mat &phiO, Mat &periodO)
{
	vector<Mat> phi_mats;
	Mat phi, phi1, phi2;
	for (int pi = 0; pi < kOrder - 1; ++pi)
	{
		UnwrappingPlain(cimg, phi, kN_comp, pi * kN_comp);
		Mat temp = (phi < 0) & (phi != -2);
		temp.convertTo(temp, CV_64FC1, 2.f / 255);
		phi = (phi + temp) / 2;
		phi_mats.push_back(phi);
	}
	UnwrappingPlain(cimg, phi1, kN_comp_last, (kOrder - 1) * kN_comp);
	UnwrappingPlain(cimg, phi2, kN_comp_last, (kOrder - 1) * kN_comp + kN_comp_last);
	CompensatePhase(phi1, phi2, phi);
	phi_mats.push_back(phi);

	Mat valid = Mat::ones(cimg[0].rows, cimg[0].cols, CV_8UC1) * 255;
	for (auto iter = phi_mats.begin(); iter != phi_mats.end(); ++iter)
		valid &= ((*iter) != -1);

	vector<Mat> period_mats;
	for (int oi = 0; oi < kOrder - 1; ++oi)
	{
		Mat p = kF_comp * phi_mats[oi] - phi_mats[oi + 1];
		p.convertTo(p, CV_8SC1);
		period_mats.push_back(p);
	}

	Mat period = Mat::ones(phi1.rows, phi1.cols, CV_64FC1);
	period *= -pow(kF_comp, kOrder - 1);
	for (int pi = 0; pi < phi1.total(); ++pi)
		if (valid.at<unsigned char>(pi) != 0)
		{
			double value = period_mats[0].at<char>(pi);
			for (int i = 1; i < period_mats.size(); ++i)
				value = value * kF_comp + period_mats[i].at<char>(pi);
			period.at<double>(pi) = value;
		}
	period /= pow(kF_comp, kOrder - 1);

	for (int pi = 0; pi < phi1.total(); ++pi)
		if (valid.at<unsigned char>(pi) == 0 && phi.at<double>(pi) != -1)
			phi.at<double>(pi) = -1;

	phiO = phi;
	periodO = period;
	Mat sssq(period.size(), CV_8UC1);
	period.convertTo(sssq, CV_8UC1, 250);
	Mat sssqw(phi.size(), CV_8UC1);
	phi.convertTo(sssqw, CV_8UC1, 180);
}

void RasterScan::UndistortionPoint(double u, double v, cv::Mat &intr, cv::Mat &dist, double &ut, double &vt)
{
	double fx = intr.at<double>(0, 0), fy = intr.at<double>(1, 1), cx = intr.at<double>(0, 2), cy = intr.at<double>(1, 2);
	double k1 = dist.at<double>(0), k2 = dist.at<double>(1), p1 = dist.at<double>(2), p2 = dist.at<double>(3), k3 = dist.at<double>(4);

	double x, y, x0, y0;
	x0 = x = (u - cx) / fx;
	y0 = y = (v - cy) / fy;
	for (int i = 0; i < 5; i++)
	{


		//double r = x*x + y*y;

		//double xt = x*(1 + k1*r + k2*r*r + k3*r*r*r) + 2 * p1*x*y + p2*(r*r + 2 * x*x);
		//double yt = y*(1 + k1*r + k2*r*r + k3*r*r*r) + p1*(r*r + 2 * y*y) + 2 * p2*x*y;

		//ut = xt*fx + cx;
		//vt = yt*fy + cy;

		double r2 = x*x + y*y;
		double icdist = 1 / (1 + ((k3 * r2 + k2)*r2 + k1)*r2);
		double deltaX = 2 * p1 * x*y + p2 * (r2 + 2 * x*x);
		double deltaY = p1 * (r2 + 2 * y*y) + 2 * p2 * x*y;

		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}


	ut = x*fx + cx;
	vt = y*fy + cy;
}

