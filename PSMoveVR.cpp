#include <WinSock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <sys/types.h> 

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
#include <opencv2/core/utils/logger.hpp>

#include <psmoveapi/psmove.h>
#include "ControllerHandler.h"
#include "IMUCalibration.h"
#include "RenderGrid.h"
#include "RenderObject.h"
#include "CameraCalibration.h"
#include "OpticalDetection.h"
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
#include <chrono>
#include <format>
#include <algorithm>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define UDP_SEND_PORT 49152

using namespace ps3eye;

GLFWwindow* window;
GLuint camTexture;

unsigned char* videoPixels;
const int width = 640;
const int height = 480;
ImVec2 videoSize(width, height);
cv::Size videoSizeCV(width, height);

ControllerHandler moves("00:06:f7:c9:a1:fb", "00:13:8a:9c:31:42");
psmoveapi::PSMoveAPI moveAPI(&moves);
bool controllerLoopRunning = true;
bool lastPressedTri = false;
float recenterTimer = 0.f;

const std::string UDP_BUF = "{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|0|0|0|0|0|0|0|0|{}|{}|0|0|0|0|0|0|0|0";

float opticalTimestep = 0.f;
float opticalLastTick = 0.f;

int currentApplicationStage = 0;

float centerH = 0.f, centerS = 0.f, centerV = 0.f, rangeH = 0.f, rangeS = 0.f, rangeV = 0.f;

int capturedFramesCamCalib = 0;
float camCalibSquareSize = 2.5f;

int camPoseCalibStage = 0;
std::vector<cv::Point2f> camPoseCalibSamples;
glm::mat4 camPoseCalibMatrix;
cv::Mat camPoseCalibRvec, camPoseCalibTvec;

void captureCameraImage(PS3EYECam::PS3EYERef* eyes, cv::Mat img) {
	eyes[0]->getFrame(videoPixels);
}

void controllerLoopTask() {
	while (controllerLoopRunning) {
		moveAPI.update();
	}
}

void opticalTask(PS3EYECam::PS3EYERef* eyes, cv::Mat img) {
	while (controllerLoopRunning) {
		float curTime = clock();
		opticalTimestep = curTime - opticalLastTick;
		opticalLastTick = curTime;

		if (currentApplicationStage == 0 || currentApplicationStage == 4) {
			captureCameraImage(eyes, img);
			opticalMethods::loop(img, opticalTimestep / 1000.f);
		}
	}
}

void commsTask() {
	Sleep(2000);

	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		exit(1);
		return;
	}

	SOCKET sock = INVALID_SOCKET;
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
	{
		std::cerr << "Failed the socket with error: " << WSAGetLastError() << '\n';
		exit(1);
		return;
	}

	sockaddr_in remoteAddr;
	ZeroMemory(&remoteAddr, sizeof(remoteAddr));
	remoteAddr.sin_family = AF_INET;
	remoteAddr.sin_port = htons(UDP_SEND_PORT);
	inet_pton(AF_INET, "127.0.0.1", &remoteAddr.sin_addr);

	while (controllerLoopRunning) {
		if (currentApplicationStage == 0) {
			// actual sending
			float rxcL = opticalMethods::left3D.x;
			float rycL = opticalMethods::left3D.y;
			float rzcL = opticalMethods::left3D.z;
			float qwL = moves.left.orientation.w;
			float qxL = moves.left.orientation.x;
			float qyL = moves.left.orientation.z;
			float qzL = -moves.left.orientation.y;
			float trigL = moves.left.buttons.trigger;

			float rxcR = opticalMethods::right3D.x;
			float rycR = opticalMethods::right3D.y;
			float rzcR = opticalMethods::right3D.z;
			float qwR = moves.right.orientation.w;
			float qxR = moves.right.orientation.x;
			float qyR = moves.right.orientation.z;
			float qzR = -moves.right.orientation.y;
			float trigR = moves.right.buttons.trigger;
			
			std::string strBuf = std::vformat(UDP_BUF, std::make_format_args(rxcL, rycL, rzcL, qwL, qxL, qyL, qzL, rxcR, rycR, rzcR, qwR, qxR, qyR, qzR, trigL, trigR));
			const char* sendBuf = strBuf.c_str();

			int sent = sendto(sock, sendBuf, strlen(sendBuf), 0, (sockaddr*)&remoteAddr, sizeof(remoteAddr));
			if (sent == SOCKET_ERROR)
			{
				std::cerr << "Failed the sendto with error: " << WSAGetLastError() << '\n';
				exit(1);
				return;
			}
		}
	}

	closesocket(sock);
	WSACleanup();
}

void showCameraWindow(cv::Mat img, const char* name) {
	ImGui::Begin(name);
	if (!ImGui::IsWindowCollapsed()) {
		// image display
		glBindTexture(GL_TEXTURE_2D, camTexture);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, img.ptr());
		ImGui::Image((void*)(intptr_t)camTexture, videoSize);
	}
	ImGui::End();
}

std::tuple<bool, cv::Scalar, cv::Scalar> colorCalibrationStage(PS3EYECam::PS3EYERef* eyes, cv::Mat img, const char* controller) {
	captureCameraImage(eyes, img);

	// info & controls
	ImGui::Begin("Color calibration");
	ImGui::Text("This is the color calibration stage for the %s controller. Choose the center and range color values below so that only the controller is shown on the mask window.", controller);

	ImGui::SliderFloat("Hue center", &centerH, 0.f, 179.f);
	ImGui::SliderFloat("Hue range", &rangeH, 0.f, 179.f);
	ImGui::Separator();
	ImGui::SliderFloat("Saturation center", &centerS, 0.f, 255.f);
	ImGui::SliderFloat("Saturation range", &rangeS, 0.f, 255.f);
	ImGui::Separator();
	ImGui::SliderFloat("Value center", &centerV, 0.f, 255.f);
	ImGui::SliderFloat("Value range", &rangeV, 0.f, 255.f);
	ImGui::Separator();
	bool finished = ImGui::Button("Finish", ImVec2(150, 25));
	ImGui::End();

	// mask output
	auto [masked, low, high] = calibrateColor(img, centerH, rangeH, centerS, rangeS, centerV, rangeV);
	cv::Mat bgrMask;
	cv::cvtColor(masked, bgrMask, cv::COLOR_GRAY2BGR);
	showCameraWindow(bgrMask, "Mask window");

	return { finished, low, high };
}

int main(int argc, char** argv)
{
	// CV
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

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
	std::vector<PS3EYECam::PS3EYERef> devices = PS3EYECam::getDevices();
	std::cout << "Found " << devices.size() << " PS3 eyes." << std::endl;
	PS3EYECam::PS3EYERef* eyes = new PS3EYECam::PS3EYERef[devices.size()];
	for (int i = 0; i < devices.size(); i++) {
		PS3EYECam::PS3EYERef eye = devices.at(i);
		eye->init(width, height, 75);
		eye->setExposure(20);
		eye->setGain(0);
		eye->setFlip(true);
		eye->start();
		eyes[i] = eye;
	}

	// PS Move
	moves.left.color = { 0.f, 1.f, 1.f };
	moves.right.color = { 1.f, 0.f, 1.f };

	// realtime video
	videoPixels = new unsigned char[width * height * 3];
	eyes[0]->getFrame(videoPixels);
	cv::Mat camImg(videoSizeCV, CV_8UC3, videoPixels);

	// realtime visualization
	Shader gridShader = Shader("shaders/grid.vert", "shaders/grid.frag");
	RenderGrid gridGL = RenderGrid(&gridShader, 5, 5);

	Shader controllerShader = Shader("shaders/controller.vert", "shaders/controller.frag");
	RenderObject controllerGL = RenderObject(&controllerShader);
	controllerGL.LoadModel("models/psmove_small.obj");
	Texture controllerTexture = Texture("textures/psmove.png");
	Texture controllerDiffuse = Texture("textures/psmove_diff.png");
	Texture controllerSpecular = Texture("textures/psmove_spec.png");

	ImGuiGL controllerWindow = ImGuiGL(640, 480);
	glm::mat4 proj = glm::perspective(glm::radians(80.f), 640.f / 480.f, 0.01f, 1000.f);

	// realtime video texture
	glGenTextures(1, &camTexture);
	glBindTexture(GL_TEXTURE_2D, camTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GlCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, videoPixels));

	// calibrate IMU
	moves.right.gyroOffsets = calibrateGyroscope(5000, &moves.right, &moveAPI);
	moves.left.gyroOffsets = calibrateGyroscope(5000, &moves.left, &moveAPI);

	// imgui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();
	glDisable(GL_DEPTH_TEST);
	glClearColor(0.647f, 0.765f, 0.804f, 1.f);

	// optical calibration
	opticalMethods::init(&moves, eyes[0]);

	// tasks
	std::thread controllerTaskThread(controllerLoopTask);
	std::thread opticalTaskThread(opticalTask, eyes, camImg);
	std::thread commsTaskThread(commsTask);

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// clear and init
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		switch (currentApplicationStage) {
		case 0: // main stage
		{
			// recenter
			if (moves.right.buttons.triangle && !(lastPressedTri)) {
				opticalMethods::HEAD_POSITION = opticalMethods::right3D;
			}
			lastPressedTri = moves.right.buttons.triangle;

			// dynamic undrift
			/*
			if (vrmath::valInThresh(moves.right.accel.y, 1.f, 0.15f) && vrmath::valInThresh(moves.left.accel.y, 1.f, 0.15f)) {
				recenterTimer += 1000.0f / ImGui::GetIO().Framerate;
			}
			else {
				recenterTimer = 0.f;
			}

			if (recenterTimer >= 2.f) {
				moves.right.recenter = glm::inverse(glm::quat(moves.right.orientation)) * MOVE_Q90;
				moves.left.recenter = glm::inverse(glm::quat(moves.left.orientation)) * MOVE_Q90;
				recenterTimer = 0.f;
			}*/

			// controllers
			glm::quat sensorQuatL = moves.left.orientation;
			glm::quat glSpaceQuatL = glm::quat(sensorQuatL.w, sensorQuatL.x, sensorQuatL.z, -sensorQuatL.y);

			glm::quat sensorQuatR = moves.right.orientation;
			glm::quat glSpaceQuatR = glm::quat(sensorQuatR.w, sensorQuatR.x, sensorQuatR.z, -sensorQuatR.y);

			// camera output
			showCameraWindow(camImg, "Camera output");

			// realtime vis
			ImGui::Begin("Visualization");

			GlCall(controllerWindow.Use());
			glClearColor(0.447f, 0.565f, 0.604f, 1.f);
			glEnable(GL_DEPTH_TEST);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			controllerShader.Use();
			controllerTexture.Use(GL_TEXTURE1);
			controllerSpecular.Use(GL_TEXTURE2);
			controllerDiffuse.Use(GL_TEXTURE3);
			controllerShader.SetMatrix4("projection", proj);
			controllerShader.SetInt("texture0", 1);
			controllerShader.SetInt("textureSpec", 2);
			controllerShader.SetInt("textureDiff", 3);

			// left pass
			glm::mat4 transformMatrix = glm::mat4(glSpaceQuatL);
			controllerShader.SetMatrix4("transform", transformMatrix);
			controllerShader.SetVector4("translate", opticalMethods::left3D + glm::vec4(0.f, 0.f, 2.f, 0.f));
			controllerShader.SetVector3("bulbColor", glm::vec3(moves.left.color.r, moves.left.color.g, moves.left.color.b));
			GlCall(controllerGL.Draw());

			// right pass
			transformMatrix = glm::mat4(glSpaceQuatR);
			controllerShader.SetMatrix4("transform", transformMatrix);
			controllerShader.SetVector4("translate", opticalMethods::right3D + glm::vec4(0.f, 0.f, 2.f, 0.f));
			controllerShader.SetVector3("bulbColor", glm::vec3(moves.right.color.r, moves.right.color.g, moves.right.color.b));
			GlCall(controllerGL.Draw());

			//grid (broken)
			//gridShader.Use();
			//gridShader.SetMatrix4("projection", proj);
			//gridShader.SetVector3("color", glm::vec3(1.f, 1.f, 1.f));
			//GlCall(gridGL.Draw());

			controllerWindow.Deuse();
			glDisable(GL_DEPTH_TEST);
			glClearColor(0.647f, 0.765f, 0.804f, 1.f);
			controllerWindow.UseTexture();
			ImGui::Image((void*)(intptr_t)controllerWindow.tex, videoSize, ImVec2(0, 1), ImVec2(1, 0));
			ImGui::End();

			// diagnostics
			ImGui::Begin("Diagnostics");
			ImGui::Text("%.2f", recenterTimer);
			ImGui::Text("Main thread running at %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Text("Optical thread running at %.3f ms/frame (%.1f FPS)", opticalTimestep, 1000.f / opticalTimestep);
			if (moves.leftConnected) {
				ImGui::SeparatorText("Left");
				ImGui::Text("X %.1f", opticalMethods::left3D.x);
				ImGui::Text("Y %.1f", opticalMethods::left3D.y);
				ImGui::Text("Z %.1f", opticalMethods::left3D.z);
				ImGui::Text("AHRS thread running at % .3f ms / frame(% .1f FPS)", moves.left.timestep * 1000.f, 1.f / moves.left.timestep);
			}
			if (moves.rightConnected) {
				ImGui::SeparatorText("Right");
				ImGui::Text("X %.1f", opticalMethods::right3D.x);
				ImGui::Text("Y %.1f", opticalMethods::right3D.y);
				ImGui::Text("Z %.1f", opticalMethods::right3D.z);
				ImGui::Text("AHRS thread running at %.3f ms/frame (%.1f FPS)", moves.right.timestep * 1000.f, 1.f / moves.right.timestep);
			}
			ImGui::End();

			// stage switching (recalibration etc.)
			ImGui::Begin("Calibration");
			if (!opticalMethods::CAMERA_POSE_CALIBRATED) ImGui::Text("Warning: Camera pose not calibrated.");
			if (ImGui::Button("Calibrate left color", ImVec2(150, 25))) {
				currentApplicationStage = 1;
				centerH = 0.f, centerS = 0.f, centerV = 0.f, rangeH = 0.f, rangeS = 0.f, rangeV = 0.f;
			}
			if (ImGui::Button("Calibrate right color", ImVec2(150, 25))) {
				currentApplicationStage = 2;
				centerH = 0.f, centerS = 0.f, centerV = 0.f, rangeH = 0.f, rangeS = 0.f, rangeV = 0.f;
			}
			if (ImGui::Button("Calibrate camera", ImVec2(150, 25))) {
				eyes[0]->setExposure(150);
				eyes[0]->setGain(100);
				capturedFramesCamCalib = 0;
				currentApplicationStage = 3;
				startCalibratingCamera(camCalibSquareSize);
			}
			if (opticalMethods::COLOR_CALIBRATED && opticalMethods::CAMERA_CALIBRATED) {
				if (ImGui::Button("Calibrate camera pose", ImVec2(150, 25))) {
					eyes[0]->setExposure(150);
					eyes[0]->setGain(100);
					camPoseCalibStage = 0;
					camPoseCalibSamples.clear();
					currentApplicationStage = 4;
				}
			}
			ImGui::End();

			break;
		}
		case 1: // color calibration LEFT
		{
			auto [finished, low, high] = colorCalibrationStage(eyes, camImg, "LEFT");
			if (finished) {
				opticalMethods::COLOR_LEFT_LOW = low;
				opticalMethods::COLOR_LEFT_HIGH = high;
				currentApplicationStage = 0;
				opticalMethods::saveColor();
			}

			break;
		}
		case 2: // color calibration RIGHT
		{
			auto [finished, low, high] = colorCalibrationStage(eyes, camImg, "RIGHT");
			if (finished) {
				opticalMethods::COLOR_RIGHT_LOW = low;
				opticalMethods::COLOR_RIGHT_HIGH = high;
				currentApplicationStage = 0;
				opticalMethods::saveColor();
			}

			break;
		}
		case 3: // camera calibration
		{
			captureCameraImage(eyes, camImg);
			showCameraWindow(camImg, "Camera output");

			ImGui::Begin("Camera calibration");
			ImGui::Text("This is the camera calibration stage. Please put a chessboard (doesn't matter if it's printed) in front of the camera so that it's seen and press the Capture button. Make sure to have different angles in each capture. Try to take at least 15 shots.");
			ImGui::Text("Captured frames: %i", capturedFramesCamCalib);
			if (ImGui::Button("Capture")) {
				calibrateCamera(cv::Mat(videoSizeCV, CV_8UC3, videoPixels), false);
				capturedFramesCamCalib++;
			}
			if (ImGui::Button("Finish")) {
				eyes[0]->setExposure(20);
				eyes[0]->setGain(0);
				auto [ret, mat, dist] = calibrateCamera(cv::Mat(videoSizeCV, CV_8UC3, videoPixels), true);
				opticalMethods::CAMERA_MAT = mat;
				opticalMethods::CAMERA_DIST = dist;
				opticalMethods::saveCamera();
				currentApplicationStage = 0;
			}
			ImGui::End();

			break;
		}
		case 4: // camera pose calibration
		{
			captureCameraImage(eyes, camImg);

			ImGui::Begin("Camera pose calibration");
			switch (camPoseCalibStage) {
			case 0:
				ImGui::Text("Place an A4-sized paper sheet anywhere on the playspace floor. Make sure it can be clearly seen by the camera and press 'Next'");
				break;
			case 1:
				ImGui::Text("Place the RIGHT controller in corner 1 and click the 'Next' button");
				break;
			case 2:
				ImGui::Text("Place the RIGHT controller in corner 2 and click the 'Next' button");
				break;
			case 3:
				ImGui::Text("Place the RIGHT controller in corner 3 and click the 'Next' button");
				break;
			case 4:
				ImGui::Text("Place the RIGHT controller in corner 4 and click the 'Next' button");
				break;
			case 5:
			{
				// calibrate 
				auto [ret, camPoseCalibMatrix_, camPoseCalibRvec_, camPoseCalibTvec_] = calibrateWorldMatrix(camPoseCalibSamples, opticalMethods::CAMERA_MAT, opticalMethods::CAMERA_DIST);
				camPoseCalibMatrix = camPoseCalibMatrix_;
				camPoseCalibRvec = camPoseCalibRvec_;
				camPoseCalibTvec = camPoseCalibTvec_;
				eyes[0]->setExposure(150);
				eyes[0]->setGain(100);
				camPoseCalibStage++;
				break;
			}
			case 6:
			{
				// vis
				cv::line(camImg, camPoseCalibSamples[0], camPoseCalibSamples[3], cv::Scalar(255, 255, 255), 3);
				cv::line(camImg, camPoseCalibSamples[3], camPoseCalibSamples[1], cv::Scalar(255, 255, 255), 3);
				cv::line(camImg, camPoseCalibSamples[1], camPoseCalibSamples[2], cv::Scalar(255, 255, 255), 3);
				cv::line(camImg, camPoseCalibSamples[2], camPoseCalibSamples[0], cv::Scalar(255, 255, 255), 3);
				cv::drawFrameAxes(camImg, opticalMethods::CAMERA_MAT, opticalMethods::CAMERA_DIST, camPoseCalibRvec, camPoseCalibTvec, 10.f);
				break;
			}
			case 7:
			{
				// end
				eyes[0]->setExposure(20);
				eyes[0]->setGain(0);
				opticalMethods::CAMERA_POSE = glm::transpose(camPoseCalibMatrix);
				opticalMethods::saveCameraPose();
				currentApplicationStage = 0;
				break;
			}
			}

			if (ImGui::Button("Next")) {
				if (camPoseCalibStage == 0) {
					eyes[0]->setExposure(20);
					eyes[0]->setGain(0);
				}
				else {
					if (camPoseCalibStage < 5) camPoseCalibSamples.push_back(cv::Point2f(opticalMethods::right2D.x, opticalMethods::right2D.y));
				}
				camPoseCalibStage++;
			}
			ImGui::End();

			showCameraWindow(camImg, "Camera output");

			break;
		}
		}

		//render
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	controllerLoopRunning = false;
	controllerTaskThread.join();
	opticalTaskThread.join();
	commsTaskThread.join();
	for (int i = 0; i < sizeof(eyes) / sizeof(eyes[0]); i++)
		eyes[i]->stop();
	glDeleteTextures(1, &camTexture);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	
	return 0;
}
