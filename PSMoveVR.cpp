#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "PS3EYEDriver/src/ps3eye.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <psmoveapi/psmove.h>
#include "ControllerHandler.h"
#include "RenderObject.h"
#include "ImGuiGL.h"
#include "Macros.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"	

#include <stdio.h>
#include <filesystem>
#include <iostream> 

using namespace ps3eye;

GLFWwindow* window;
GLuint camTexture;

unsigned char* videoPixels;
const int width = 640;
const int height = 480;

int main(int argc, char** argv)
{
	// GL, GLAD and GLFW
	if (!glfwInit())
		return -1;
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window = glfwCreateWindow(800, 600, "PSMoveVR Core", nullptr, nullptr);
	if (!window) {
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

	// PS3 EYE
	std::vector<ps3eye::PS3EYECam::PS3EYERef> devices = ps3eye::PS3EYECam::getDevices();
	std::cout << "found " << devices.size() << " PS3 eyes" << std::endl;
	PS3EYECam::PS3EYERef eye = devices.at(0);
	eye->init(width, height, 60);
	eye->setExposure(20);
	eye->setGain(0);
	eye->setFlip(true);
	eye->start();

	// PS Move
	ControllerHandler moveR;
	psmoveapi::PSMoveAPI moveRAPI(&moveR);

	// realtime video
	videoPixels = new unsigned char[eye->getWidth() * eye->getHeight() * 3];
	cv::Size videoSize(eye->getWidth(), eye->getHeight());
	ImVec2 videoSizeIV2(width, height);
	eye->getFrame(videoPixels);
	cv::Mat camImg(videoSize, CV_8UC3, videoPixels);

	// realtime visualization
	Shader controllerShader = Shader("shaders/test.vert", "shaders/test.frag");
	RenderObject controllerGL = RenderObject(&controllerShader);
	controllerGL.LoadModel("models/psmove.obj");
	ImGuiGL controllerWindow = ImGuiGL(640, 480);

	glm::mat4 trans = glm::mat4(1.f);
	trans = glm::translate(trans, glm::vec3(0.f, 0.f, 0.f));
	glm::mat4 proj = glm::perspective(glm::radians(70.f), (float)640 / (float)480, 0.01f, 100.0f);

	// realtime video texture
	glGenTextures(1, &camTexture);
	glBindTexture(GL_TEXTURE_2D, camTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GlCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, videoPixels));

	// imgui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// clear and init
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		moveRAPI.update();

		// image processing
		eye->getFrame(videoPixels);
		camImg.data = videoPixels;

		// controller
		glm::vec3 accel = moveR.accel;
		glm::vec3 gyro = moveR.gyro;
		moveR.color = { 0.f, 1.f, 1.f };

		// camera output
		ImGui::Begin("Camera output");
		if (!ImGui::IsWindowCollapsed()) {
			// image display
			glBindTexture(GL_TEXTURE_2D, camTexture);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, camImg.ptr());
			ImGui::Image((void*)(intptr_t)camTexture, videoSizeIV2);
		}
		ImGui::End();

		// realtime vis
		ImGui::Begin("Visualization");

		GlCall(controllerWindow.Use());
		glClearColor(1.f, 1.f, 1.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(controllerShader.handle);
		controllerShader.SetMatrix4("transform", trans);
		controllerShader.SetMatrix4("projection", proj);
		GlCall(controllerGL.Draw());

		controllerWindow.Deuse();
		glClearColor(0.f, 0.7f, 1.f, 1.f);
		controllerWindow.UseTexture();
		ImGui::Image((void*)(intptr_t)controllerWindow.tex, videoSizeIV2);
		ImGui::End();

		// diagnostics
		ImGui::Begin("Diagnostics");
		ImGui::Text("Running at %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Text("Ax->%.3f", accel.x);
		ImGui::Text("Ay->%.3f", accel.y);
		ImGui::Text("Az->%.3f", accel.z);
		ImGui::End();

		//render
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	eye->stop();
	glDeleteTextures(1, &camTexture);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	
	return 0;
}
