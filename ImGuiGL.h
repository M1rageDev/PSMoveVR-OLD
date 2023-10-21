#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glad/glad.h>

class ImGuiGL
{
public:
	GLuint tex, rbo;
	unsigned int w, h;
	ImGuiGL() {}
	ImGuiGL(int w, int h);
	~ImGuiGL();
	void Use();
	void Deuse();
	void UseTexture();
private:
	GLuint fbo;
};
