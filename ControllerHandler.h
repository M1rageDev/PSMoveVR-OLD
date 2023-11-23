#pragma once
#include <stdio.h>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include "MagwickAHRS.h"
#include "psmoveapi/psmoveapi.h"

const glm::quat MOVE_Q90 = glm::quat(0.7071069f, -0.7071067f, 0.f, 0.f);

struct ButtonStructure {
	float trigger = 0.f;
	bool square, triangle, cross, circle, move, select, start, playstation;
};

struct ControllerStructure {
	MadgwickAHRS ahrs = MadgwickAHRS(glm::quat(0.7071069f, 0.7071067f, 0.f, 0.f));
	RGB color = { 0.f, 0.f, 0.f };
	float rumble = 0.f;

	ButtonStructure buttons = ButtonStructure();	

	glm::vec3 accel = { 0.f, 0.f, 0.f };
	glm::vec3 gyro = { 0.f, 0.f, 0.f };
	glm::quat orientation = glm::quat();

	glm::vec3 gyroOffsets = { 0.f, 0.f, 0.f };

	float timestep = 0.f;
	float lastTime = 0.f;

	void update(Controller* controller, float mBeta) {
		float curTime = clock();
		timestep = (curTime - lastTime) / 1000.f;
		lastTime = curTime;

		accel = glm::vec3(controller->accelerometer.x, controller->accelerometer.y, controller->accelerometer.z);
		gyro = glm::vec3(controller->gyroscope.x, controller->gyroscope.y, controller->gyroscope.z);
		ahrs.update(gyro - gyroOffsets, accel, mBeta, timestep);
		orientation = ahrs.q * MOVE_Q90;

		controller->color = color;
		controller->rumble = rumble;

		buttons.square = (controller->buttons & Btn_SQUARE) != 0;
		buttons.triangle = (controller->buttons & Btn_TRIANGLE) != 0;
		buttons.cross = (controller->buttons & Btn_CROSS) != 0;
		buttons.circle = (controller->buttons & Btn_CIRCLE) != 0;
		buttons.move = (controller->buttons & Btn_MOVE) != 0;
		buttons.select = (controller->buttons & Btn_SELECT) != 0;
		buttons.start = (controller->buttons & Btn_START) != 0;

		buttons.trigger = controller->trigger;
	}
};

struct ControllerHandler : public psmoveapi::Handler {
	ControllerHandler(const char* leftSerialNumber, const char* rightSerialNumber) {
		left = ControllerStructure();
		leftSerial = leftSerialNumber;

		right = ControllerStructure();
		rightSerial = rightSerialNumber;
	}
	ControllerHandler(glm::quat initialRot, const char* leftSerialNumber, const char* rightSerialNumber, float madgwickBeta_ = 0.035f) {
		madgwickBeta = madgwickBeta_;

		left = ControllerStructure();
		left.ahrs = MadgwickAHRS(initialRot);
		leftSerial = leftSerialNumber;

		right = ControllerStructure();
		right.ahrs = MadgwickAHRS(initialRot);
		rightSerial = rightSerialNumber;
	}

	virtual void connect(Controller* controller) {
		const char* handeness;
		if (strcmp(controller->serial, rightSerial)) {
			rightConnected = true;
			handeness = "Right";
		}
		else {
			leftConnected = true;
			handeness = "Left";
		}
		printf("%s controller connected: %s\n", handeness, controller->serial);
	}

	virtual void update(Controller* controller) {
		if (strcmp(controller->serial, leftSerial)) {
			left.update(controller, madgwickBeta);
		}
		else {
			right.update(controller, madgwickBeta);
		}
		
	}

	virtual void disconnect(Controller* controller) {
		const char* handeness;
		if (strcmp(controller->serial, rightSerial)) {
			rightConnected = false;
			handeness = "Right";
		}
		else {
			leftConnected = false;
			handeness = "Left";
		}
		printf("%s controller disconnected: %s\n", handeness, controller->serial);
	}

	glm::vec3 getGyro(bool leftFunction) {
		if (leftFunction) {
			return left.gyro;
		}
		else {
			return right.gyro;
		}
	}

	glm::vec3 getAccel(bool leftFunction) {
		if (leftFunction) {
			return left.accel;
		}
		else {
			return right.accel;
		}
	}

	float madgwickBeta = 0.035f;

	ControllerStructure left;
	const char* leftSerial;
	bool leftConnected;
	ControllerStructure right;
	const char* rightSerial;
	bool rightConnected;
};
