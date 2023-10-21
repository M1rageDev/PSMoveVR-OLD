#pragma once
#include "Shader.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <glad/glad.h>
class RenderObject
{
public:
	RenderObject();
	RenderObject(Shader* shader_);
	~RenderObject();
	void Draw();
	void LoadModel(std::string path);
private:
	GLuint VBO, VAO, EBO;
	Shader* shader;
	float* vertices;
	unsigned int vertLength;
	unsigned int* triangles;
	unsigned int triLength;
};
