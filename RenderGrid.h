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
	int width, height;

	RenderGrid();
	RenderGrid(Shader* shader_, int w, int h);
	~RenderGrid();
	void Draw();
private:
	GLuint VBO, VAO, EBO;
	Shader* shader;
	unsigned int elementLength;
};
