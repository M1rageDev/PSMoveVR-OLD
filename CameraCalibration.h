#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/gtx/string_cast.hpp"

#include <stdio.h>
#include <vector>

#ifndef VRCAMCALIB_H
#define VRCAMCALIB_H
#define GLM_ENABLE_EXPERIMENTAL

// A4 paper is (10.5, 14.85)
inline std::vector<cv::Point3f> CALIBRATION_MAT_OBJECT;

inline std::vector<cv::Point3f> CAM_CALIB_OBJ_POINT;
inline std::vector<std::vector<cv::Point3f>> CAM_CALIB_OBJ_POINTS;
inline std::vector<std::vector<cv::Point2f>> CAM_CALIB_IMG_POINTS;

inline void startCalibratingCamera(float squareSize) {
	CAM_CALIB_OBJ_POINT.clear();
	CAM_CALIB_OBJ_POINTS.clear();
	CAM_CALIB_IMG_POINTS.clear();

	for (int y = 0; y < 7; y++) {
		for (int x = 0; x < 7; x++) {
			CAM_CALIB_OBJ_POINT.push_back(cv::Point3f((float)(x * squareSize), (float)(y * squareSize), 0.f));
		}
	}
}

inline std::tuple<bool, cv::Mat, cv::Mat> calibrateCamera(cv::Mat image, bool finish) {
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

inline std::tuple<bool, glm::mat4, cv::Mat, cv::Mat> calibrateWorldMatrix(std::vector<cv::Point2f> samples, cv::Mat cameraMatrix, cv::Mat cameraDistortion) {
	/*
	CALIBRATION_MAT_OBJECT = {
		cv::Point3f(-10.5f, 0.f, -14.85f),
		cv::Point3f(10.5f, 0.f, 14.85f),
		cv::Point3f(-10.5f, 0.f, 14.85f),
		cv::Point3f(10.5f, 0.f, -14.85f)
	};*/
	CALIBRATION_MAT_OBJECT = {
		cv::Point3f(-23.75f, 0.f, -26.7f),
		cv::Point3f(23.75f, 0.f, 26.7f),
		cv::Point3f(-23.75f, 0.f, 26.7f),
		cv::Point3f(23.75f, 0.f, -26.7f)
	};

	cv::Mat rvecs, tvecs;
	bool ret = cv::solvePnP(CALIBRATION_MAT_OBJECT, samples, cameraMatrix, cameraDistortion, rvecs, tvecs);

	if (ret) {
		std::cout << "Computing inverse camera pose..." << std::endl;
		cv::Mat R;
		cv::Rodrigues(rvecs, R);

		float rotMat[9];
		for (int i = 0; i < 9; i++)
		{
			rotMat[i] = static_cast<float>(R.at<double>(i));
		}

		cv::Mat R_inv = R.t();
		cv::Mat tvecInv = -R_inv * tvecs; // translation of the inverse R|t transform
		float tv[3];
		for (int i = 0; i < 3; i++)
		{
			tv[i] = static_cast<float>(tvecInv.at<double>(i));
		}

		float RTMat[] = {
			rotMat[0], rotMat[1], rotMat[2], 0.0f,
			rotMat[3], rotMat[4], rotMat[5], 0.0f,
			rotMat[6], rotMat[7], rotMat[8], 0.0f,
			tv[0], tv[1], tv[2], 1.0f };

		glm::mat4 mtx = glm::make_mat4(RTMat);
		std::cout << glm::to_string(mtx) << std::endl;
		return { true, mtx, rvecs, tvecs };
	}
	else {
		return { false, glm::mat4(), cv::Mat(), cv::Mat() };
	}
}
#endif
