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
		return a * (1.f - t) + (b * t);
	}

	glm::vec4 posFilter(glm::vec4 l, glm::vec4 c, float xy_delta, float z_delta) {
		float mag = glm::length(l - c);
		float XYweight = clamp01(lerp(0.4f, 0.75f, mag / 5.f + xy_delta));
		float Zweight = clamp01(lerp(0.05f, 0.1f, mag / 5.f + z_delta));

		return glm::vec4(lerp(l.x, c.x, XYweight), lerp(l.y, c.y, XYweight), lerp(l.z, c.z, Zweight), c.w);
	}

	bool valInThresh(float val, float center, float thresh) {
		return (val < center + thresh) && (val > center - thresh);
	}

}
#endif
