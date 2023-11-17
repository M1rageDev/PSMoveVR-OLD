#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glad/glad.h>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader
{
public:
	GLuint handle;

	Shader();
	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();

	void Use();

	void SetVector4(const char* name, glm::vec4 value);
	void SetVector3(const char* name, glm::vec3 value);
	void SetMatrix4(const char* name, glm::mat4 value);
	void SetInt(const char* name, int value);
private:
	
};
