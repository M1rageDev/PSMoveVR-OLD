#pragma once
#include <glad/glad.h>
#include <stdio.h>
#include <filesystem>
#include <iostream> 

void GlClearError() {
	while (glGetError() != GL_NO_ERROR);
}

bool GlLogCall(const char* function, const char* file, int line) {
	while (GLenum error = glGetError()) {
		std::cout << "[Opengl Error] (" << error << ") " << function <<
			" " << file << ":" << line << std::endl;
		return false;
	}
	return true;
}

#define ASSERT(x) if(!(x)) __debugbreak();
#define GlCall(x) GlClearError();\
                  x;\
                  ASSERT(GlLogCall(#x, __FILE__, __LINE__))
