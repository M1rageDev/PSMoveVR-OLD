#include "ImGuiGL.h"

ImGuiGL::ImGuiGL(int w_, int h_) {
	w = w_;
	h = h_;

	// fbo
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	// color
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	
	// depth/stencil
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);

	// tex
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);

	// rbo
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "Framebuffer is not complete!" << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ImGuiGL::Use() {
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glViewport(0, 0, w, h);
}

void ImGuiGL::Deuse() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ImGuiGL::UseTexture() {
	glBindTexture(GL_TEXTURE_2D, tex);
}

ImGuiGL::~ImGuiGL() {
	glDeleteFramebuffers(1, &fbo);
}
