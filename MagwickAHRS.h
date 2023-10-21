#pragma once
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

class MadgwickAHRS {
public:
	glm::quat q;
	MadgwickAHRS(glm::quat initialPose);
	~MadgwickAHRS();

	void update(glm::vec3 g, glm::vec3 a, float b, float timestep);
};
