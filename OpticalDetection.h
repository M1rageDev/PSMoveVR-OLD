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
	bool COLOR_CALIBRATED = false;
	cv::Scalar COLOR_LEFT_LOW;
	cv::Scalar COLOR_LEFT_HIGH;
	cv::Scalar COLOR_RIGHT_LOW;
	cv::Scalar COLOR_RIGHT_HIGH;

	bool CAMERA_CALIBRATED = false;
	cv::Mat CAMERA_MAT;
	cv::Mat CAMERA_DIST;

	bool CAMERA_POSE_CALIBRATED = false;
	glm::mat4 CAMERA_POSE;

	glm::vec4 HEAD_POSITION;

	ControllerHandler* moves;
	PS3EYECam::PS3EYERef cam;
	
	glm::vec3 right2D;
	glm::vec3 left2D;

	glm::vec4 right3D;
	glm::vec4 left3D;
	glm::vec4 lastRight3D;
	glm::vec4 lastLeft3D;

	float lastRightRadius = 0.f;
	float lastLeftRadius = 0.f;

	glm::vec4 transformSpace(glm::vec3 camCoord) {
		glm::vec4 tvec4 = glm::vec4(camCoord, 1.f);
		glm::vec4 worldCoord = CAMERA_POSE * tvec4;
		return worldCoord;
	}

	std::tuple<bool, glm::vec4> processController(cv::Mat frame, float dt, bool left) {
		cv::Scalar colorLow = left ? COLOR_LEFT_LOW : COLOR_RIGHT_LOW;
		cv::Scalar colorHigh = left ? COLOR_LEFT_HIGH : COLOR_RIGHT_HIGH;

		glm::vec3 ball = detectBall(frame, colorLow, colorHigh);
		if (ball.z <= 0) return { false, glm::vec4() };

		ball.z = vrmath::lerp(left ? lastLeftRadius : lastRightRadius, ball.z, dt * 4.f);

		if (left) {
			lastLeftRadius = ball.z;
			left2D = ball;
		}
		else {
			lastRightRadius = ball.z;
			right2D = ball;
		}

		glm::vec3 cameraSpace = estimate3D_coneFitting(ball, CAMERA_MAT.at<double>(0, 0), frame.cols, frame.rows);
		glm::vec4 worldSpace = transformSpace(cameraSpace);
		return { true, worldSpace };
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
			CAMERA_CALIBRATED = true;
			std::cout << "Loaded camera intristic and distortion matrix." << std::endl;
		}

		if (fileExists("config/color.yml")) {
			beginRead("config/color.yml");
			COLOR_LEFT_LOW = readNodeScalar("leftLow");
			COLOR_LEFT_HIGH = readNodeScalar("leftHigh");
			COLOR_RIGHT_LOW = readNodeScalar("rightLow");
			COLOR_RIGHT_HIGH = readNodeScalar("rightHigh");
			endRead();
			COLOR_CALIBRATED = true;
			std::cout << "Loaded color calibration data." << std::endl;
		}

		if (fileExists("config/cameraPose.yml")) {
			beginRead("config/cameraPose.yml");
			CAMERA_POSE = glm::transpose(readNodeMat4("matrix"));  // TODO: FIX THIS
			endRead();
			CAMERA_POSE_CALIBRATED = true;
			std::cout << "Loaded camera-to-world matrix." << std::endl;
		}
	}

	void saveColor() {
		beginSave("config/color.yml");
		addNode("leftLow", COLOR_LEFT_LOW);
		addNode("leftHigh", COLOR_LEFT_HIGH);
		addNode("rightLow", COLOR_RIGHT_LOW);
		addNode("rightHigh", COLOR_RIGHT_HIGH);
		endSave();
		COLOR_CALIBRATED = true;
		std::cout << "Saved color calibration data." << std::endl;
	}

	void saveCamera() {
		beginSave("config/camera.yml");
		addNode("matrix", CAMERA_MAT);
		addNode("distortion", CAMERA_DIST);
		endSave();
		CAMERA_CALIBRATED = true;
		std::cout << "Saved camera intristic and distortion matrix." << std::endl;
	}

	void saveCameraPose() {
		beginSave("config/cameraPose.yml");
		addNode("matrix", CAMERA_POSE);
		endSave();
		CAMERA_POSE_CALIBRATED = true;
		std::cout << "Saved camera-to-world matrix." << std::endl;
	}

	void loop(cv::Mat frame, float dt) {
		auto [_, right3D] = processController(frame, dt, false);  // right
		auto [__, left3D] = processController(frame, dt, true);  // left
		opticalMethods::right3D = vrmath::posFilter(lastRight3D, right3D, 0.f, 0.f) / 100.f - HEAD_POSITION;
		opticalMethods::left3D = vrmath::posFilter(lastLeft3D, left3D, 0.f, 0.f) / 100.f - HEAD_POSITION;
		lastRight3D = right3D;
		lastLeft3D = left3D;
	}
}
