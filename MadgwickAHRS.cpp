#include "MagwickAHRS.h"

MadgwickAHRS::MadgwickAHRS(glm::quat initialPose) {
	this->q = glm::quat(initialPose);
}

MadgwickAHRS::~MadgwickAHRS() {}

void MadgwickAHRS::update(glm::vec3 g, glm::vec3 a, float b, float timestep) {
	glm::quat* Quat = &q;
	float q1 = Quat->w;
	float q2 = Quat->x;
	float q3 = Quat->y;
	float q4 = Quat->z;

	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _4q1 = 4.0 * q1;
	float _4q2 = 4.0 * q2;
	float _4q3 = 4.0 * q3;
	float _8q2 = 8.0 * q2;
	float _8q3 = 8.0 * q3;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	float norm_ = glm::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	if (norm_ == 0.f) return;
	norm_ = 1.f / norm_;
	a.x *= norm_;
	a.y *= norm_;
	a.z *= norm_;

	// Gradient decent algorithm corrective step
	float s1 = _4q1 * q3q3 + _2q3 * a.x + _4q1 * q2q2 - _2q2 * a.y;
	float s2 = _4q2 * q4q4 - _2q4 * a.x + 4.0 * q1q1 * q2 - _2q1 * a.y - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * a.z;
	float s3 = 4.0 * q1q1 * q3 + _2q1 * a.x + _4q3 * q4q4 - _2q4 * a.y - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * a.z;
	float s4 = 4.0 * q2q2 * q4 - _2q2 * a.x + 4.0 * q3q3 * q4 - _2q3 * a.y;
	norm_ = 1.0 / glm::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);  // normalise step magnitude
	s1 *= norm_;
	s2 *= norm_;
	s3 *= norm_;
	s4 *= norm_;

	float qDot1 = 0.5 * (-q2 * g.x - q3 * g.y - q4 * g.z) - b * s1;
	float qDot2 = 0.5 * (q1 * g.x + q3 * g.z - q4 * g.y) - b * s2;
	float qDot3 = 0.5 * (q1 * g.y - q2 * g.z + q4 * g.x) - b * s3;
	float qDot4 = 0.5 * (q1 * g.z + q2 * g.y - q3 * g.x) - b * s4;

	q1 += qDot1 * timestep;
	q2 += qDot2 * timestep;
	q3 += qDot3 * timestep;
	q4 += qDot4 * timestep;
	norm_ = 1.0 / glm::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // normalise quaternion
	Quat->w = q1 * norm_;
	Quat->x = q2 * norm_;
	Quat->y = q3 * norm_;
	Quat->z = q4 * norm_;
}
