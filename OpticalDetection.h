#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <stdio.h>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/glm.hpp>

#include "PS3EYEDriver/src/ps3eye.h"

#include "VRMath.h"
#include "VRConfig.h"

using namespace ps3eye;

const float REAL_BALL_RADIUS = 2.25f;	
std::vector<cv::Point3d> OBJECT_POINTS = {
	cv::Point3d(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS, 0.f),
	cv::Point3d(REAL_BALL_RADIUS, REAL_BALL_RADIUS,  0.f),
	cv::Point3d(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS,  0.f),
	cv::Point3d(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS,  0.f) };

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

std::tuple<cv::Mat, cv::Scalar, cv::Scalar> calibrateColor(cv::Mat frame, float hCenter, float hRange, float sCenter, float sRange, float vCenter, float vRange) {
	cv::Mat hsvFrame, detected;
	cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

	float hMin = hCenter - hRange;
	float hMax = hCenter + hRange;
	float sMin = vrmath::clamp(sCenter - sRange, 0.f, 255.f);
	float sMax = vrmath::clamp(sCenter + sRange, 0.f, 255.f);
	float vMin = vrmath::clamp(vCenter - vRange, 0.f, 255.f);
	float vMax = vrmath::clamp(vCenter + vRange, 0.f, 255.f);

	cv::Scalar low = cv::Scalar(hMin, sMin, vMin);
	cv::Scalar high = cv::Scalar(hMax, sMax, vMax);
	cv::inRange(hsvFrame, low, high, detected);

	return { detected, low, high };
}

glm::vec3 detectBall(cv::Mat frame, cv::Scalar low, cv::Scalar high) {
	cv::Mat hsvFrame;
	cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

	cv::Point minus1Point = cv::Point(-1, -1);
	cv::Mat mask;
	cv::inRange(hsvFrame, low, high, mask);
	cv::erode(mask, mask, cv::Mat(), minus1Point, 2);
	cv::dilate(mask, mask, cv::Mat(), minus1Point, 2);
	
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

	if (!contours.empty()) {
		float maxArea = 0;
		int maxIndex = 0;
		for (int i = 0; i < contours.size(); ++i) {
			float area = cv::contourArea(contours.at(i));
			if (area > maxArea) {
				maxArea = area;
				maxIndex = i;
			}
		}

		cv::Point2f center;
		float radius;
		cv::minEnclosingCircle(contours[maxIndex], center, radius);
		
		return glm::vec3(center.x, center.y, radius);
	}
}

std::tuple<bool, cv::Mat, cv::Mat> estimate3D(glm::vec3 ball, cv::Mat matrix, cv::Mat distortion) {
	std::vector<cv::Point2d> imagePoints = {
		cv::Point2d(ball.x - ball.z, ball.y - ball.z),
		cv::Point2d(ball.x + ball.z, ball.y + ball.z),
		cv::Point2d(ball.x - ball.z, ball.y + ball.z),
		cv::Point2d(ball.x + ball.z, ball.y - ball.z)
	};

	cv::Mat tvec;
	cv::Mat rvec;
	bool ret = cv::solvePnP(OBJECT_POINTS, imagePoints, matrix, distortion, rvec, tvec, false);
	return {ret, rvec, tvec};
}

glm::vec3 estimate3D_coneFitting(glm::vec3 ball, float f_px, int imgW, int imgH) {
	float X_px = ball.x;
	float Y_px = ball.y;
	float A_px = ball.z;

	X_px -= imgW / 2;
	Y_px = imgH / 2 - Y_px;

	float L_px = sqrtf(X_px * X_px + Y_px * Y_px);

	float k = L_px / f_px;
	float j = (L_px + A_px) / f_px;
	float l = (j - k) / (1 + j * k);

	float D_cm = REAL_BALL_RADIUS * sqrtf(1 + l * l) / l;

	float fl = f_px / L_px;
	float Z_cm = D_cm * fl / sqrtf(1 + fl * fl);

	float L_cm = Z_cm * k;

	float X_cm = L_cm * X_px / L_px;
	float Y_cm = L_cm * Y_px / L_px;

	return glm::vec3(X_cm, Y_cm, Z_cm);
}

namespace opticalMethods {
	cv::Scalar COLOR_LEFT_LOW;
	cv::Scalar COLOR_LEFT_HIGH;
	cv::Scalar COLOR_RIGHT_LOW;
	cv::Scalar COLOR_RIGHT_HIGH;

	cv::Mat CAMERA_MAT;
	cv::Mat CAMERA_DIST;

	ControllerHandler* moves;
	PS3EYECam::PS3EYERef cam;

	glm::vec3 right3D;
	glm::vec3 left3D;
	glm::vec3 lastRight3D;
	glm::vec3 lastLeft3D;

	std::tuple<bool, glm::vec3> processController(cv::Mat frame, bool left) {
		cv::Scalar colorLow = left ? COLOR_LEFT_LOW : COLOR_RIGHT_LOW;
		cv::Scalar colorHigh = left ? COLOR_LEFT_HIGH : COLOR_RIGHT_HIGH;

		glm::vec3 ball = detectBall(frame, colorLow, colorHigh);
		if (ball.z <= 0) return { false, glm::vec3() };
		cv::circle(frame, cv::Point(ball.x, ball.y), (int)ball.z, cv::Scalar(255, 255, 255), 3);

		glm::vec3 cameraSpace = estimate3D_coneFitting(ball, CAMERA_MAT.at<double>(0, 0), frame.cols, frame.rows);
		return { true, cameraSpace };
	}

	void init(ControllerHandler* controllers, PS3EYECam::PS3EYERef eye) {
		moves = controllers;
		cam = eye;

		// camera
		if (fileExists("config/camera.yml")) {
			beginRead("config/camera.yml");
			CAMERA_MAT = readNode("matrix");
			CAMERA_DIST = readNode("distortion");
			endRead();
			std::cout << "Loaded camera intristic and distortion matrix." << std::endl;
		}

		if (fileExists("config/color.yml")) {
			beginRead("config/color.yml");
			COLOR_LEFT_LOW = readNodeScalar("leftLow");
			COLOR_LEFT_HIGH = readNodeScalar("leftHigh");
			COLOR_RIGHT_LOW = readNodeScalar("rightLow");
			COLOR_RIGHT_HIGH = readNodeScalar("rightHigh");
			endRead();
			std::cout << "Loaded color calibration data." << std::endl;
		}
	}

	void saveColor() {
		beginSave("config/color.yml");
		addNode("leftLow", COLOR_LEFT_LOW);
		addNode("leftHigh", COLOR_LEFT_HIGH);
		addNode("rightLow", COLOR_RIGHT_LOW);
		addNode("rightHigh", COLOR_RIGHT_HIGH);
		endSave();
		std::cout << "Saved color calibration data." << std::endl;
	}

	void saveCamera() {
		beginSave("config/camera.yml");
		addNode("matrix", CAMERA_MAT);
		addNode("distortion", CAMERA_DIST);
		endSave();
		std::cout << "Saved camera intristic and distortion matrix." << std::endl;
	}

	void loop(cv::Mat frame) {
		auto [_, right3D] = processController(frame, false);  // right
		auto [__, left3D] = processController(frame, true);  // left
		opticalMethods::right3D = vrmath::posFilter(lastRight3D, right3D, 0.f, 0.f);
		opticalMethods::left3D = vrmath::posFilter(lastLeft3D, left3D, 0.f, 0.f);
		lastRight3D = right3D;
		lastLeft3D = left3D;
	}
}
