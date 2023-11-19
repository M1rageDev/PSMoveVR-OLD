#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>

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

// saving
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

void endSave() {
	VRCONF_::FILENAME = "";
	VRCONF_::FS.release();
}

// reading
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

void endRead() {
	VRCONF_::FILENAME = "";
	VRCONF_::FS.release();
}
#endif
