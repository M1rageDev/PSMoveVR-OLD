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
#include "IMUCalibration.h"
#include "RenderObject.h"
#include "ImGuiGL.h"
#include "Texture.h"
#include "Macros.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"	

#include <functional>
#include <stdio.h>
#include <filesystem>
#include <iostream> 

using namespace ps3eye;

GLFWwindow* window;
GLuint camTexture;

unsigned char* videoPixels;
const int width = 640;
const int height = 480;

ControllerHandler moveR;
psmoveapi::PSMoveAPI moveRAPI(&moveR);
bool controllerLoopRunning = true;

void controllerLoopTask() {
	while (controllerLoopRunning) {
		moveRAPI.update();
	}
}

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
	glfwWindowHint(GLFW_MAXIMIZED, true);
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
	moveR.color = { 0.f, 1.f, 1.f };

	// controller transformations
	glm::vec3 forwardDir = glm::vec3(0, 0, 1);
	glm::vec3 upDir = glm::vec3(0, 1, 0);
	glm::vec3 rightDir = glm::vec3(1, 0, 0);
	glm::mat4 controllerTransform = glm::mat4(rightDir.x, upDir.x, forwardDir.x, 0.f,
											  rightDir.y, upDir.y, forwardDir.y, 0.f,
											  rightDir.z, upDir.z, forwardDir.z, 0.f,
											  0.f,        0.f,     0.f,          1.f);
	glm::quat q90 = glm::quat(0.7071069f, -0.7071067f, 0.f, 0.f);

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
	Texture controllerTexture = Texture("textures/blue/psmove.png");
	Texture controllerDiffuse = Texture("textures/blue/psmove_diff.png");
	Texture controllerSpecular = Texture("textures/blue/psmove_spec.png");
	glm::mat4 proj = glm::perspective(glm::radians(60.f), 640.f / 480.f, 0.1f, 100.f);

	// realtime video texture
	glGenTextures(1, &camTexture);
	glBindTexture(GL_TEXTURE_2D, camTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GlCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, videoPixels));

	// controller settings window
	const char* controllerColors[] = { "Blue", "Magenta", "Amber", "Disabled" };
	int lastControllerColor = 0;
	int currentColour = 0;

	// calibrate IMU
	moveR.gyroOffsets = calibrateGyroscope(5000, &moveR, &moveRAPI);

	// imgui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	// tasks
	std::thread controllerTask(controllerLoopTask);

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// clear and init
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// image processing
		eye->getFrame(videoPixels);
		camImg.data = videoPixels;

		// controller
		glm::vec3 accel = moveR.accel;
		glm::quat sensorQuat = moveR.orientation * q90;
		glm::quat glSpaceQuat = glm::quat(sensorQuat.w, sensorQuat.x, sensorQuat.z, -sensorQuat.y);

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
		glEnable(GL_DEPTH_TEST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		controllerShader.Use();
		controllerTexture.Use(GL_TEXTURE1);
		controllerSpecular.Use(GL_TEXTURE2);
		controllerDiffuse.Use(GL_TEXTURE3);
		glm::mat4 transformMatrix = controllerTransform * glm::mat4(glSpaceQuat);
		controllerShader.SetMatrix4("transform", transformMatrix);
		controllerShader.SetMatrix4("projection", proj);
		controllerShader.SetInt("texture0", 1);
		controllerShader.SetInt("textureSpec", 2);
		controllerShader.SetInt("textureDiff", 3);
		GlCall(controllerGL.Draw());

		controllerWindow.Deuse();
		glDisable(GL_DEPTH_TEST);
		glClearColor(0.f, 0.7f, 1.f, 1.f);
		controllerWindow.UseTexture();
		ImGui::Image((void*)(intptr_t)controllerWindow.tex, videoSizeIV2, ImVec2(0, 1), ImVec2(1, 0));
		ImGui::End();
		
		// controller settings
		ImGui::Begin("Controller");
		ImGui::Combo("Controller color", &currentColour, controllerColors, IM_ARRAYSIZE(controllerColors));
		if (currentColour != lastControllerColor) {
			switch (currentColour)
			{
			case 0:
				controllerTexture.Reload("textures/blue/psmove.png");
				controllerDiffuse.Reload("textures/blue/psmove_diff.png");
				controllerSpecular.Reload("textures/blue/psmove_spec.png");
				moveR.color = { 0.f, 1.f, 1.f };
				break;
			case 1:
				controllerTexture.Reload("textures/magenta/psmove.png");
				controllerDiffuse.Reload("textures/magenta/psmove_diff.png");
				controllerSpecular.Reload("textures/magenta/psmove_spec.png");
				moveR.color = { 1.f, 0.f, 1.f };
				break;
			case 2:
				controllerTexture.Reload("textures/amber/psmove.png");
				controllerDiffuse.Reload("textures/amber/psmove_diff.png");
				controllerSpecular.Reload("textures/amber/psmove_spec.png");
				moveR.color = { 1.f, 1.f, 0.f };
				break;
			case 3:
				controllerTexture.Reload("textures/disabled/psmove.png");
				controllerDiffuse.Reload("textures/disabled/psmove_diff.png");
				controllerSpecular.Reload("textures/disabled/psmove_spec.png");
				moveR.color = { 0.f, 0.f, 0.f };
				break;
			default:
				break;
			}
		}
		ImGui::Text("Ax->%.3f", accel.x);
		ImGui::Text("Ay->%.3f", accel.y);
		ImGui::Text("Az->%.3f", accel.z);
		ImGui::Text("Gx->%.3f", moveR.gyro.x - moveR.gyroOffsets.x);
		ImGui::Text("Gy->%.3f", moveR.gyro.y - moveR.gyroOffsets.y);
		ImGui::Text("Gz->%.3f", moveR.gyro.z - moveR.gyroOffsets.z);
		lastControllerColor = currentColour;
		ImGui::End();

		// diagnostics
		ImGui::Begin("Diagnostics");
		ImGui::Text("Running at %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();

		//render
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	controllerLoopRunning = false;
	controllerTask.join();
	eye->stop();
	glDeleteTextures(1, &camTexture);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	
	return 0;
}
