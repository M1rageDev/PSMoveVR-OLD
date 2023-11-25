#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/gtx/string_cast.hpp"

#ifndef VRCONFIG_H
#define VRCONFIG_H
// private
namespace VRCONF_ {
	const char* FILENAME;
	cv::FileStorage FS;
}

// utils
bool fileExists(const char* name) {
	std::ifstream f(name);
	bool good = f.good();
	f.close();
	return good;
}

// saving (CV)
void beginSave(const char* path) {
	VRCONF_::FILENAME = path;
	VRCONF_::FS = cv::FileStorage(path, cv::FileStorage::WRITE);
}

void addNode(const char* name, cv::Mat mat) {
	VRCONF_::FS << name << mat;
}

void addNode(const char* name, cv::Scalar mat) {
	VRCONF_::FS << name << mat;
}

void addNode(const char* name, glm::mat4 mat) {
	cv::Mat cvMat(4, 4, CV_32F);
	memcpy(cvMat.data, glm::value_ptr(mat), 16 * sizeof(float));
	VRCONF_::FS << name << cvMat;
}

void endSave() {
	VRCONF_::FILENAME = "";
	VRCONF_::FS.release();
}

// reading (CV)
void beginRead(const char* path) {
	VRCONF_::FILENAME = path;
	VRCONF_::FS = cv::FileStorage(path, cv::FileStorage::READ);
}

cv::Mat readNode(const char* name) {
	cv::Mat node;
	VRCONF_::FS[name] >> node;
	return node;
}

cv::Scalar readNodeScalar(const char* name) {
	cv::Scalar node;
	VRCONF_::FS[name] >> node;
	return node;
}

glm::mat4 readNodeMat4(const char* name) {
	glm::mat4 node;
	cv::Mat cvNode(4, 4, CV_32F);
	VRCONF_::FS[name] >> cvNode;
	memcpy(glm::value_ptr(node), cvNode.data, 16 * sizeof(float));

	return node;
}

void endRead() {
	VRCONF_::FILENAME = "";
	VRCONF_::FS.release();
}

#endif
