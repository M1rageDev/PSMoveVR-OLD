#pragma once
#include <iostream>
#include <glm/vec3.hpp>

#ifndef VRMATH_H
#define VRMATH_H
namespace vrmath {

	float clamp(float x, float l, float h) {
		return std::min(std::max(x, l), h);
	}

	float clamp01(float x) {
		return clamp(x, 0.f, 1.f);
	}

	float lerp(float a, float b, float t) {
		return a * (1.f - t) + b * t;
	}

	glm::vec3 lerp3(glm::vec3 a, glm::vec3 b, float t) {
		return glm::vec3(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t));
	}

	glm::vec3 posFilter(glm::vec3 l, glm::vec3 c, float xy_delta, float z_delta) {
		float mag = glm::length(l - c);
		float XYweight = clamp01(lerp(0.4f, 0.75f, mag / 5.f + xy_delta));
		float Zweight = clamp01(lerp(0.05f, 0.1f, mag / 5.f + z_delta));

		return glm::vec3(lerp(l.x, c.x, XYweight), lerp(l.y, c.y, XYweight), lerp(l.z, c.z, Zweight));
	}

}
#endif
