#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "stb_image.h"
#include <glad/glad.h>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Texture
{
public:
	GLuint handle;
	int w, h, c;

	Texture();
	Texture(const char* texturePath);
	~Texture();

	void Use(GLenum unit);
	void Reload(const char* texturePath);
private:

};

