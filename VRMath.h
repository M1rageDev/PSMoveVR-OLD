#pragma once

#include <iostream>

namespace vrmath {

	float clamp(float x, float l, float h) {
		return std::min(std::max(x, l), h);
	}

	float clamp01(float x) {
		return clamp(x, 0.f, 1.f);
	}

}
