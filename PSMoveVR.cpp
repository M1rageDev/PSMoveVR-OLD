#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "PS3EYEDriver/src/ps3eye.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"	
#include <iostream> 

using namespace ps3eye;

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

GLFWwindow* window;
GLuint camTexture;

int main(int argc, char** argv)
{
	if (!glfwInit())
		return -1;
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window = glfwCreateWindow(800, 600, "Baller", nullptr, nullptr);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

	printf("%s\n", glGetString(GL_VERSION));
	std::vector<ps3eye::PS3EYECam::PS3EYERef> devices = ps3eye::PS3EYECam::getDevices();
	std::cout << "found " << devices.size() << " devices." << std::endl;

	PS3EYECam::PS3EYERef eye = devices.at(0);
	bool res = eye->init(640, 480, 60);
	eye->setAutogain(true);
	std::cout << "init eye result " << res << std::endl;
	eye->start();

	unsigned char* videoPixels = new unsigned char[eye->getWidth() * eye->getHeight() * 3];
	cv::Size videoSize(eye->getWidth(), eye->getHeight());
	eye->getFrame(videoPixels);
	cv::Mat camImg(videoSize, CV_8UC3, videoPixels);

	glGenTextures(1, &camTexture);
	glBindTexture(GL_TEXTURE_2D, camTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GlCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, videoPixels));

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	while (!glfwWindowShouldClose(window))
	{
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		eye->getFrame(videoPixels);
		camImg = cv::Mat(videoSize, CV_8UC3, videoPixels);
		cv::Mat colored;
		cv::cvtColor(camImg, colored, 4);
		glBindTexture(GL_TEXTURE_2D, camTexture);	
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, colored.ptr());

		ImGui::Begin("n");
		ImGui::Text("Running at %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Image((void*)(intptr_t)camTexture, ImVec2(640, 480));
		ImGui::End();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	eye->stop();
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	
	return 0;
}
