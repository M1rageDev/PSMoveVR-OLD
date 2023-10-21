#pragma once
#include <stdio.h>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include "MagwickAHRS.h"
#include "psmoveapi/psmoveapi.h"

struct ControllerHandler : public psmoveapi::Handler {
	ControllerHandler() : color{ 0.f, 0.f, 0.f }, rumble(0.f), accel{ 0.f, 0.f, 0.f }, gyro{ 0.f, 0.f, 0.f }, ahrs(glm::quat(0.7071069f, 0.7071067f, 0.f, 0.f)), timestep(0), lastTime(0), orientation(), madgwickBeta(0.035f) {}
	ControllerHandler(glm::quat initialRot, float madgwickBeta_=0.035f) : color{ 0.f, 0.f, 0.f }, rumble(0.f), accel{0.f, 0.f, 0.f}, gyro{ 0.f, 0.f, 0.f }, ahrs(initialRot), timestep(0), lastTime(0), orientation(), madgwickBeta(madgwickBeta_) {}

	virtual void connect(Controller* controller) {
		printf("Controller connected: %s\n", controller->serial);
	}

	virtual void update(Controller* controller) {
		timestep = std::time(nullptr) - lastTime;
		accel = glm::vec3(controller->accelerometer.x, controller->accelerometer.y, controller->accelerometer.z);
		gyro = glm::vec3(controller->gyroscope.x, controller->gyroscope.y, controller->gyroscope.z);
		ahrs.update(gyro, accel, madgwickBeta, timestep);

		controller->color = color;
		controller->rumble = rumble;

		lastTime = std::time(nullptr);
	}

	virtual void disconnect(Controller* controller) {
		printf("Controller disconnected: %s\n", controller->serial);
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
};
