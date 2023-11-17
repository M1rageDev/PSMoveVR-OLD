#pragma once
#include <vector>
#include <iostream>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

glm::vec3 calibrateGyroscope(unsigned int samples, ControllerStructure* getGyro, psmoveapi::PSMoveAPI* api) {
	std::cout << "Starting gyro calibration, make sure the controller is standing COMPLETELY still." << std::endl;
	std::vector<glm::vec3> imuArray;
	glm::vec3 gyroOffsets{ 0.0, 0.0, 0.0 };
	while (imuArray.size() < samples + 1) {
		api->update();
		auto gyro_data = getGyro->gyro;
		imuArray.push_back(gyro_data);
	}
	int imuArraySize = imuArray.size();

	glm::vec3 sum{ 0.0, 0.0, 0.0 };
	for (int i = 0; i < imuArraySize; i++) {
		sum += imuArray[i];
	}
	gyroOffsets = sum / (float)imuArraySize;
	std::cout << "Gyro calibration complete: " << gyroOffsets.x << " " << gyroOffsets.y << " " << gyroOffsets.z << std::endl;
	return gyroOffsets;
}
