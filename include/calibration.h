#pragma once
#include "common.h"
#include "ICP.h"
using namespace std;


class Calibration{
public:
    Calibration();
    ~Calibration();

public:
    bool calibrated;
    std::vector<float> T;
	std::vector<std::vector<float>> R;
    MarkerInfo marker;

    int nMarkerCorners;
	vector<cv::Point2f> vPts;
    int nMinSize;
	int nMaxSize;
	int nThreshold;
    double dApproxPolyCoef;
	double dMarkerFrame;
	bool bDraw;
    bool save2Local;

    int calibratedNum;

public:
    bool performCalibration(framePacket *packet);
    bool GetMarker(cv::Mat &img);
    void Procrusters(MarkerInfo &marker, 
                     vector<Point3f> &markerInWorld, 
                     vector<float> &worldToMarkerT, 
                     vector<vector<float>> &worldToMarkerR);
    void GetMarkerPoints(vector<Point3f> &pts);
    double GetMarkerArea(MarkerInfo &marker);
    void GetMarkerPointsForWarp(vector<cv::Point2f> &pts);
    int GetCode(cv::Mat &img, vector<cv::Point2f> points, vector<cv::Point2f> corners);
    bool OrderCorners(vector<cv::Point2f> &corners);
    void RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);
};