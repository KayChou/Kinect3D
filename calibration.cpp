#include "calibration.h"


bool Calibration::performCalibration(framePacket *packet){
	cv::Mat registered(packet->height_d, packet->width_d, CV_8UC3);

	for(int i=0; i<packet->height_d; i++){
		for(int j=0; j<packet->width_d; j++){
			registered.at<cv::Vec3b>(i, j)[0] = packet->vertices[i*packet->width_d + j].B;
			registered.at<cv::Vec3b>(i, j)[1] = packet->vertices[i*packet->width_d + j].G;
			registered.at<cv::Vec3b>(i, j)[2] = packet->vertices[i*packet->width_d + j].R;
		}
	}
	// if current camera is not calibrated, then try to get marker
	if(!calibrated){
		GetMarker(registered);
		std::vector<Point3f> featurePoints;
		Point3f temp;
		if(marker.id > 0){
			for(int i=0; i<5; i++){
				temp.X = packet->vertices[(int)(marker.corners[i].Y * packet->width_d + marker.corners[i].X)].X;
				temp.Y = packet->vertices[(int)(marker.corners[i].Y * packet->width_d + marker.corners[i].X)].Y;
				temp.Z = packet->vertices[(int)(marker.corners[i].Y * packet->width_d + marker.corners[i].X)].Z;
				featurePoints.push_back(temp);
			}
			Procrusters(marker, featurePoints, T, R);
			calibratedNum++;
		}
		else{
			calibratedNum = 0;
		}
		calibrated = (calibratedNum > 30) ? true : false;
	}

	cv::putText(registered, calibrated ? "calibrated=true" : "calibrated=false", cv::Point(0, 30), 2, 1, cv::Scalar(0, 255, 0));
	Point3f tempPoint;
	for(int i=0; i<packet->height_d; i++){
		for(int j=0; j<packet->width_d; j++){
			packet->vertices[i*packet->width_d + j].B = registered.at<cv::Vec3b>(i, j)[0];
			packet->vertices[i*packet->width_d + j].G = registered.at<cv::Vec3b>(i, j)[1];
			packet->vertices[i*packet->width_d + j].R = registered.at<cv::Vec3b>(i, j)[2];

			tempPoint.X = packet->vertices[i*packet->width_d + j].X;
			tempPoint.Y = packet->vertices[i*packet->width_d + j].Y;
			tempPoint.Z = packet->vertices[i*packet->width_d + j].Z;

			RotatePoint(tempPoint, R, T);
			packet->vertices[i*packet->width_d + j].X = tempPoint.X;
			packet->vertices[i*packet->width_d + j].Y = tempPoint.Y;
			packet->vertices[i*packet->width_d + j].Y = tempPoint.Z;
		}
	}

	return true;
}


bool Calibration::GetMarker(cv::Mat &img){
    vector<MarkerInfo> markers;
	cv::Mat img2, img3;
	cv::cvtColor(img, img2, CV_BGR2GRAY);
	cv::threshold(img2, img2, nThreshold, 255, CV_THRESH_BINARY);

	img2.copyTo(img3);

	vector<vector<cv::Point>> contours;	
	cv::findContours(img3, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	for (unsigned int i = 0; i < contours.size(); i++){
		vector<cv::Point> corners;
		double area = cv::contourArea(contours[i]);

		if (area < nMinSize || area > nMaxSize)
			continue;

		cv::approxPolyDP(contours[i], corners, sqrt(area)*dApproxPolyCoef, true);

		vector<cv::Point2f> cornersFloat;
		for (unsigned int j = 0; j < corners.size(); j++){
			cornersFloat.push_back(cv::Point2f((float)corners[j].x, (float)corners[j].y));
		}

		if (!cv::isContourConvex(corners) && corners.size() == nMarkerCorners && OrderCorners(cornersFloat)){	
			bool order = true;
			int code = GetCode(img2, vPts, cornersFloat);

			if (code < 0){
				reverse(cornersFloat.begin() + 1, cornersFloat.end());
				code = GetCode(img2, vPts, cornersFloat);

				if (code < 0) { continue; }

				order = false;
			}

			//I have commented this out as it crashed for some people, if you want additional accuracy in calibration try to uncomment it.
			//CornersSubPix(cornersFloat, contours[i], order);

			vector<Point2f> cornersFloat2(nMarkerCorners);
			vector<Point3f> points3D;			

			for (int i = 0; i < nMarkerCorners; i++){
				cornersFloat2[i] = Point2f(cornersFloat[i].x, cornersFloat[i].y);
			}

			GetMarkerPoints(points3D);

			markers.push_back(MarkerInfo(code, cornersFloat2, points3D));

			if (bDraw){
				for (unsigned int j = 0; j < corners.size(); j++){
					cv::circle(img, cornersFloat[j], 2, cv::Scalar(0, 50 * j, 0), 1);
					cv::line(img, cornersFloat[j], cornersFloat[(j + 1) % cornersFloat.size()], cv::Scalar(0, 0, 255), 2);
				}
			}
		}
	}

	if (markers.size() > 0)
	{
		double maxArea = 0;
		int maxInd = 0;

		for (unsigned int i = 0; i < markers.size(); i++){
			if (GetMarkerArea(markers[i]) > maxArea){
				maxInd = i;
				maxArea = GetMarkerArea(markers[i]);
			}
		}

		marker = markers[maxInd];
		if (bDraw){
			for (int j = 0; j < nMarkerCorners; j++){
				cv::Point2f pt1 = cv::Point2f(marker.corners[j].X, marker.corners[j].Y);
				cv::Point2f pt2 = cv::Point2f(marker.corners[(j + 1) % nMarkerCorners].X, marker.corners[(j + 1) % nMarkerCorners].Y);
				cv::line(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			}
		}

		return true;
	}
	else
		return false;
}


void Calibration::GetMarkerPoints(vector<Point3f> &pts)
{
	pts.push_back(Point3f(0.0f, -1.0f, 0.0f));
	pts.push_back(Point3f(-1.0f, -1.6667f, 0.0f));
	pts.push_back(Point3f(-1.0f, 1.0f, 0.0f));
	pts.push_back(Point3f(1.0f, 1.0f, 0.0f));
	pts.push_back(Point3f(1.0f, -1.6667f, 0.0f));
}


void Calibration::Procrusters(MarkerInfo &marker, vector<Point3f> &markerInWorld, vector<float> &worldToMarkerT, vector<vector<float>> &worldToMarkerR){
    int nVertices = marker.points.size();

	Point3f markerCenterInWorld;
	Point3f markerCenter;
	for (int i = 0; i < nVertices; i++)
	{
		markerCenterInWorld.X += markerInWorld[i].X / nVertices;
		markerCenterInWorld.Y += markerInWorld[i].Y / nVertices;
		markerCenterInWorld.Z += markerInWorld[i].Z / nVertices;

		markerCenter.X += marker.points[i].X / nVertices;
		markerCenter.Y += marker.points[i].Y / nVertices;
		markerCenter.Z += marker.points[i].Z / nVertices;
	}

	worldToMarkerT.resize(3);
	worldToMarkerT[0] = -markerCenterInWorld.X;
	worldToMarkerT[1] = -markerCenterInWorld.Y;
	worldToMarkerT[2] = -markerCenterInWorld.Z;

	vector<Point3f> markerInWorldTranslated(nVertices);
	vector<Point3f> markerTranslated(nVertices);
	for (int i = 0; i < nVertices; i++)
	{
		markerInWorldTranslated[i].X = markerInWorld[i].X + worldToMarkerT[0];
		markerInWorldTranslated[i].Y = markerInWorld[i].Y + worldToMarkerT[1];
		markerInWorldTranslated[i].Z = markerInWorld[i].Z + worldToMarkerT[2];

		markerTranslated[i].X = marker.points[i].X - markerCenter.X;
		markerTranslated[i].Y = marker.points[i].Y - markerCenter.Y;
		markerTranslated[i].Z = marker.points[i].Z - markerCenter.Z;
	}

	cv::Mat A(nVertices, 3, CV_64F);
	cv::Mat B(nVertices, 3, CV_64F);

	for (int i = 0; i < nVertices; i++)
	{
		A.at<double>(i, 0) = markerTranslated[i].X;
		A.at<double>(i, 1) = markerTranslated[i].Y;
		A.at<double>(i, 2) = markerTranslated[i].Z;

		B.at<double>(i, 0) = markerInWorldTranslated[i].X;
		B.at<double>(i, 1) = markerInWorldTranslated[i].Y;
		B.at<double>(i, 2) = markerInWorldTranslated[i].Z;
	}

	cv::Mat M = A.t() * B;

	cv::SVD svd;
	svd(M);
	cv::Mat R = svd.u * svd.vt;

	double det = cv::determinant(R);

	if (det < 0)
	{
		cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
		temp.at<double>(2, 2) = -1;
		R = svd.u * temp * svd.vt;
	}

	worldToMarkerR.resize(3);

	for (int i = 0; i < 3; i++)
	{
		worldToMarkerR[i].resize(3);
		for (int j = 0; j < 3; j++)
		{
			worldToMarkerR[i][j] = static_cast<float>(R.at<double>(i, j));
		}
	}
}


double Calibration::GetMarkerArea(MarkerInfo &marker)
{
	cv::Mat hull;

	vector<cv::Point2f> cvCorners(nMarkerCorners);
	for (int i = 0; i < nMarkerCorners; i++)
	{
		cvCorners[i] = cv::Point2f(marker.corners[i].X, marker.corners[i].Y);
	}

	cv::convexHull(cvCorners, hull);
	return cv::contourArea(hull);
}


void Calibration::GetMarkerPointsForWarp(vector<cv::Point2f> &pts)
{
	pts.push_back(cv::Point2f(0, 1));
	pts.push_back(cv::Point2f(-1, 1.6667f));
	pts.push_back(cv::Point2f(-1, -1));
	pts.push_back(cv::Point2f(1, -1));
	pts.push_back(cv::Point2f(1, 1.6667f));
}


int Calibration::GetCode(cv::Mat &img, vector<cv::Point2f> points, vector<cv::Point2f> corners)
{
	cv::Mat H, img2;
	
	int minX = 0, minY = 0;

	double markerInterior = 2 - 2 * dMarkerFrame;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		points[i].x = static_cast<float>((points[i].x - dMarkerFrame + 1) * 50);
		points[i].y = static_cast<float>((points[i].y - dMarkerFrame + 1) * 50);
	}

	H = cv::findHomography(corners, points);
	cv::warpPerspective(img, img2, H, cv::Size((int)(50 * markerInterior), (int)(50 * markerInterior)));

	int xdiff = img2.cols / 3;
	int ydiff = img2.rows / 3;
	int tot = xdiff * ydiff;
	int vals[9];

	cv::Mat integral;
	cv::integral(img2, integral);
	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int temp;
			temp = integral.at<int>((i + 1) * xdiff, (j + 1) * ydiff);
			temp += integral.at<int>(i * xdiff, j * ydiff);
			temp -= integral.at<int>((i + 1) * xdiff, j * ydiff);
			temp -= integral.at<int>(i * xdiff, (j + 1) * ydiff);

			temp = temp / tot;

			if (temp < 128)
				vals[j + i * 3] = 0;
			else if (temp >= 128)
				vals[j + i * 3] = 1;
		}
	}

	int ones = 0;
	int code = 0;
	for (int i = 0; i < 4; i++)
	{	
		if (vals[i] == vals[i + 4])
			return -1;
		else if (vals[i] == 1)
		{
			code += static_cast<int>(pow(2, (double)(3 - i)));
			ones++;
		}
	}
	
	if (ones / 2 == (float)ones / 2.0)
	{
		if (vals[8] == 0)
			return -1;
	}

	if (ones / 2 != ones / 2.0)
	{
		if (vals[8] == 1)
			return -1;
	}
	
	return code;
}


bool Calibration::OrderCorners(vector<cv::Point2f> &corners)
{
	vector<int> hull;

	cv::convexHull(corners, hull);

	if (hull.size() != corners.size() - 1)
		return false;

	int index = -1;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		bool found = false;
		for (unsigned int j = 0; j < hull.size(); j++)
		{
			if (hull[j] == i)
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			index = i;
			break;
		}
	}

	vector<cv::Point2f> corners2;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		corners2.push_back(corners[(index + i)%corners.size()]);
	}

	corners = corners2;
	return true;
}


void Calibration::RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T)
{
	std::vector<float> res(3);
	point.X += T[0];
	point.Y += T[1];
	point.Z += T[2];

	res[0] = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res[1] = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res[2] = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	point.X = res[0];
	point.Y = res[1];
	point.Z = res[2];
}


Calibration::Calibration(){
	nMinSize = 100;
	nMaxSize = 1000000000;
	nThreshold = 120;
	dApproxPolyCoef = 0.12;

	dMarkerFrame = 0.4;
	nMarkerCorners = 5;

	bDraw = true;
	save2Local = false;
	calibratedNum = 0;
	GetMarkerPointsForWarp(vPts);
}


Calibration::~Calibration(){

}
