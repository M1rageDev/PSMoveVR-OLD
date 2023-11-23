#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <stdio.h>
#include <vector>

std::vector<cv::Point3f> CAM_CALIB_OBJ_POINT;
std::vector<std::vector<cv::Point3f>> CAM_CALIB_OBJ_POINTS;
std::vector<std::vector<cv::Point2f>> CAM_CALIB_IMG_POINTS;

void startCalibratingCamera(float squareSize) {
	CAM_CALIB_OBJ_POINT.clear();
	CAM_CALIB_OBJ_POINTS.clear();
	CAM_CALIB_IMG_POINTS.clear();

	for (int y = 0; y < 7; y++) {
		for (int x = 0; x < 7; x++) {
			CAM_CALIB_OBJ_POINT.push_back(cv::Point3f((float)(x * squareSize), (float)(y * squareSize), 0.f));
		}
	}
}

std::tuple<bool, cv::Mat, cv::Mat> calibrateCamera(cv::Mat image, bool finish) {
	if (!finish) {
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

		cv::Mat gray;
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

		cv::Mat corners;
		bool ret = cv::findChessboardCorners(gray, cv::Size(7, 7), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (ret) {
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			CAM_CALIB_IMG_POINTS.push_back(corners);
			CAM_CALIB_OBJ_POINTS.push_back(CAM_CALIB_OBJ_POINT);
			std::cout << "Added one" << std::endl;
		}

		return { false, cv::Mat(), cv::Mat() };
	}

	// calibrate
	if (CAM_CALIB_OBJ_POINTS.size() > 0) {
		cv::Mat camMatrix, camDist;
		std::vector<cv::Mat> rvecs, tvecs;
		cv::Size imgSize = cv::Size(image.cols, image.rows);

		double ret = cv::calibrateCamera(CAM_CALIB_OBJ_POINTS, CAM_CALIB_IMG_POINTS, imgSize, camMatrix, camDist, rvecs, tvecs);
		std::cout << "Successfully calibrated camera." << std::endl;
		return { true, camMatrix, camDist };
	}
	else {
		std::cout << "Could not calibrate camera: no points captured.";
		return { true, cv::Mat(), cv::Mat() };
	}
}
