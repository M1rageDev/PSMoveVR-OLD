#pragma once
#include <stdio.h>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include "MagwickAHRS.h"
#include "psmoveapi/psmoveapi.h"

struct ControllerHandler : public psmoveapi::Handler {
	ControllerHandler() : color{ 0.f, 0.f, 0.f }, rumble(0.f), accel{ 0.f, 0.f, 0.f }, gyro{ 0.f, 0.f, 0.f }, ahrs(glm::quat(0.7071069f, 0.7071067f, 0.f, 0.f)), timestep(0), lastTime(), orientation(), madgwickBeta(0.035f), gyroOffsets{ 0.f, 0.f, 0.f } {}
	ControllerHandler(glm::quat initialRot, float madgwickBeta_ = 0.035f) : color{ 0.f, 0.f, 0.f }, rumble(0.f), accel{ 0.f, 0.f, 0.f }, gyro{ 0.f, 0.f, 0.f }, ahrs(initialRot), timestep(0), lastTime(), orientation(), madgwickBeta(madgwickBeta_), gyroOffsets{ 0.f, 0.f, 0.f } {}

	virtual void connect(Controller* controller) {
		printf("Controller connected: %s\n", controller->serial);
	}

	virtual void update(Controller* controller) {
		float curTime = std::clock();
		timestep = (curTime - lastTime) / 1000;
		accel = glm::vec3(controller->accelerometer.x, controller->accelerometer.y, controller->accelerometer.z);
		gyro = glm::vec3(controller->gyroscope.x, controller->gyroscope.y, controller->gyroscope.z);
		ahrs.update(gyro - gyroOffsets, accel, madgwickBeta, timestep);
		orientation = ahrs.q;

		controller->color = color;
		controller->rumble = rumble;

		lastTime = curTime;
	}

	virtual void disconnect(Controller* controller) {
		printf("Controller disconnected: %s\n", controller->serial);
	}

	glm::vec3 getGyro() {
		return gyro;
	}

	float madgwickBeta;
	MadgwickAHRS ahrs;
	float timestep;
	float lastTime;

	RGB color;
	float rumble;

	glm::vec3 accel;
	glm::vec3 gyro;
	glm::quat orientation;

	glm::vec3 gyroOffsets;
};
