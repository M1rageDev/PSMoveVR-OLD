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

const float REAL_BALL_RADIUS = 2.25f;	
const cv::Mat OBJECT_POINTS = cv::Mat(1, 4, CV_64F, 
	new cv::Point2f[4] { 
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS),
		cv::Point2f(REAL_BALL_RADIUS, REAL_BALL_RADIUS),
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS),
		cv::Point2f(-REAL_BALL_RADIUS, -REAL_BALL_RADIUS)
	}
);

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
	cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (!contours.empty()) {
		float maxArea = 0;
		int maxIndex = 0;
		for (int i = 0; i < contours.size(); ++i) {
			float area = cv::contourArea(contours[i]);
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

void drawController(cv::Mat frame, glm::vec3 ball, cv::Scalar color) {
	cv::circle(frame, cv::Point2f(ball.x, ball.y), ball.z, color, 2);
}
