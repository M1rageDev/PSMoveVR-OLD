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

#include "VRMath.h"
#include "VRConfig.h"

const float REAL_BALL_RADIUS = 2.25f;	
const cv::Mat OBJECT_POINTS = cv::Mat(1, 4, CV_32F, 
	new cv::Point2f[4] { 
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS),
		cv::Point2f(REAL_BALL_RADIUS, REAL_BALL_RADIUS),
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS),
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS)
	}
);

std::tuple<cv::Mat, cv::Mat> calibrateCamera(const char* filePath) {

}

std::tuple<cv::Mat, cv::Scalar, cv::Scalar> calibrateColor(cv::Mat frame, float hCenter, float hRange, float sCenter, float sRange, float vCenter, float vRange, bool finished) {
	cv::Mat hsvFrame, detected;
	cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

	float hMin = hCenter - hRange;
	float hMax = hCenter + hRange;
	float sMin = (sCenter - sRange, 0.f, 255.f);
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
	cv::Mat eroded;
	cv::Mat inRange;
	cv::inRange(hsvFrame, low, high, inRange);
	cv::erode(inRange, eroded, cv::Mat(), minus1Point, 2);
	cv::dilate(eroded, mask, cv::Mat(), minus1Point, 2);
	
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

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
	cv::Mat imagePoints = cv::Mat(1, 4, CV_64F,
		new cv::Point2f[4]{ 
			cv::Point2f(ball.x - ball.z, ball.y - ball.z),
			cv::Point2f(ball.x + ball.z, ball.y + ball.z),
			cv::Point2f(ball.x - ball.z, ball.y + ball.z),
			cv::Point2f(ball.x + ball.z, ball.y - ball.z)
		}
	);

	cv::Mat tvec;
	cv::Mat rvec;
	bool ret = cv::solvePnP(OBJECT_POINTS, imagePoints, matrix, distortion, rvec, tvec);
	return {ret, rvec, tvec};
}

namespace opticalMethods {
	cv::Scalar COLOR_LEFT_LOW;
	cv::Scalar COLOR_LEFT_HIGH;
	cv::Scalar COLOR_RIGHT_LOW;
	cv::Scalar COLOR_RIGHT_HIGH;

	cv::Mat CAMERA_MAT;
	cv::Mat CAMERA_DIST;

	ControllerHandler* moves;

	glm::vec3 right3D;
	glm::vec3 left3D;
	glm::vec3 lastRight3D;
	glm::vec3 lastLeft3D;

	std::tuple<bool, glm::vec3> processController(cv::Mat frame, bool left) {
		cv::Scalar colorLow = left ? COLOR_LEFT_LOW : COLOR_RIGHT_LOW;
		cv::Scalar colorHigh = left ? COLOR_LEFT_HIGH : COLOR_RIGHT_HIGH;

		glm::vec3 ball = detectBall(frame, colorLow, colorHigh);
		if (!(ball.z > 0)) return { false, glm::vec3() };

		auto [ret, rvec, tvec] = estimate3D(ball, CAMERA_MAT, CAMERA_DIST);
		if (ret) {
			glm::vec3 cameraSpace = glm::vec3(tvec.at<float>(cv::Point(0, 0)), tvec.at<float>(cv::Point(1, 0)), tvec.at<float>(cv::Point(2, 0)));
			std::cout << cameraSpace.x << std::endl;
			return { true, cameraSpace };
		}
		else {
			return { false, glm::vec3() };
		}
	}

	void calibrate(ControllerHandler* controllers) {
		moves = controllers;

		// color
		if (!fileExists("config/color.yml")) {

		}
		else {
			beginRead("config/color.yml");
			COLOR_LEFT_LOW = readNodeScalar("leftLow");
			COLOR_LEFT_HIGH = readNodeScalar("leftHigh");
			COLOR_RIGHT_LOW = readNodeScalar("rightLow");
			COLOR_RIGHT_HIGH = readNodeScalar("rightHigh");
			endRead();
		}

		// camera
		if (!fileExists("config/camera.yml")) {

		}
		else {
			beginRead("config/camera.yml");
			CAMERA_MAT = readNodeScalar("matrix");
			CAMERA_DIST = readNodeScalar("distortion");
			endRead();
		}
	}

	void loop(cv::Mat frame) {
		processController(frame, false);
		processController(frame, true);
	}
}
