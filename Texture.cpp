#include "Texture.h"

Texture::Texture() {

}

Texture::Texture(const char* texturePath) {
	glGenTextures(1, &handle);
	glBindTexture(GL_TEXTURE_2D, handle);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	stbi_set_flip_vertically_on_load(1);
	unsigned char* data = stbi_load(texturePath, &w, &h, &c, 0);
	if (data)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	}
	else
	{
		std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);
}

Texture::~Texture() {
	glDeleteTextures(1, &handle);
}

void Texture::Use(GLenum unit) {
	glActiveTexture(unit);
	glBindTexture(GL_TEXTURE_2D, handle);
}

void Texture::Reload(const char* texturePath) {
	Use(GL_TEXTURE0);
	stbi_set_flip_vertically_on_load(1);
	unsigned char* data = stbi_load(texturePath, &w, &h, &c, 0);
	if (data)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	}
	else
	{
		std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);
}
