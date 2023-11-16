#pragma once
#include "Shader.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <glad/glad.h>

class RenderGrid
{
public:
	RenderGrid();
	RenderGrid(Shader* shader_);
	~RenderGrid();
	void Draw();
	void LoadModel(std::string path);
private:
	GLuint VBO, VAO, EBO;
	Shader* shader;
	unsigned int triLength;
};
