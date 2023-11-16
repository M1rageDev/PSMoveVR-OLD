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

glm::vec3 detectBall(cv::Mat frame, cv::Mat colorL, cv::Mat colorH) {
	cv::Mat hsvFrame;
	cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

	cv::Point minus1Point = cv::Point(-1, -1);
	cv::Mat mask;
	cv::Mat eroded;
	cv::Mat inRange;
	cv::inRange(hsvFrame, colorL, colorH, inRange);
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

std::tuple<cv::Mat, cv::Mat, cv::Mat> calibrateColor(cv::Mat frame, float hCenter, float hRange, float sCenter, float sRange, float vCenter, float vRange, bool finished) {
	cv::Mat hsvFrame, detected;
	cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

	float hMin = hCenter - hRange;
	float hMax = hCenter + hRange;
	float sMin = (sCenter - sRange, 0.f, 255.f);
	float sMax = vrmath::clamp(sCenter + sRange, 0.f, 255.f);
	float vMin = vrmath::clamp(vCenter - vRange, 0.f, 255.f);
	float vMax = vrmath::clamp(vCenter + vRange, 0.f, 255.f);

	cv::Mat low(hMin, sMin, vMin);
	cv::Mat high(hMax, sMax, vMax);
	cv::inRange(hsvFrame, low, high, detected);

	return std::tuple<cv::Mat, cv::Mat, cv::Mat>(low, high, detected);
}
