#include <vector>
#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include "GLFW/glfw3.h"
#define _USE_MATH_DEFINES
#include <math.h>
#define GLFW_NO_GLU

#include "graphics.h"

using namespace std;

#pragma warning(disable:4996)

/*
** �V�F�[�_�I�u�W�F�N�g
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;
GLFWwindow  *window;
//���_�I�u�W�F�N�g
GLuint vao;
GLuint vbo;
//OpenGL���ł̕ϐ�
const int max_num = 150;
int logsize = 150;//750fps�̏ꍇ�ŉ��t���[�����o�͂��邩
vector<float> pointlogs(max_num * 3 * logsize, 0);
vector<float> pos(max_num * 3, 0);
glm::mat4 mvp;
glm::mat4 View;
glm::mat4 Projection;
GLuint Matrix;
glm::vec3 campos;//�J�����ʒu���W�x�N�g��
vector<cv::Mat> gl_img_Logs;//OpenGL�o�͐}�̕ۑ�Vector
//Imgui�ł�View�s��Čv�Z���̃p�����[�^
float h_angle = 0;
float v_angle = M_PI / 6.0f;
float fov = 45.0f;
float H_robot = 470;
const float H_camera = 600;
float diffX = 0;
float diffY = 0;
float diffZ = 0;
double currentTime =0.0;

/*
** �V�F�[�_�[�̃\�[�X�v���O�������������ɓǂݍ���
*/
int readShaderSource(GLuint shader, const char *file)
{
	FILE *fp;
	const GLchar *source;
	GLsizei length;
	int ret;

	/* �t�@�C�����J�� */
	fp = fopen(file, "rb");
	if (fp == NULL) {
		perror(file);
		return -1;
	}

	/* �t�@�C���̖����Ɉړ������݈ʒu (�܂�t�@�C���T�C�Y) �𓾂� */
	fseek(fp, 0L, SEEK_END);
	length = ftell(fp);

	/* �t�@�C���T�C�Y�̃��������m�� */
	source = (GLchar *)malloc(length);
	if (source == NULL) {
		fprintf(stderr, "Could not allocate read buffer.\n");
		return -1;
	}

	/* �t�@�C����擪����ǂݍ��� */
	fseek(fp, 0L, SEEK_SET);
	ret = fread((void *)source, 1, length, fp) != (size_t)length;
	fclose(fp);

	/* �V�F�[�_�̃\�[�X�v���O�����̃V�F�[�_�I�u�W�F�N�g�ւ̓ǂݍ��� */
	if (ret)
		fprintf(stderr, "Could not read file: %s.\n", file);
	else
		glShaderSource(shader, 1, &source, &length);

	/* �m�ۂ����������̊J�� */
	free((void *)source);

	return ret;
}

//OpenGL�œ_�Q�`�悷��֐�
int writepointcloud(Capture *cap, bool *flg) {
	if (glfwInit() == GL_FALSE)
	{
		std::cerr << "Can't initilize GLFW" << std::endl;
		return 1;
	}
	//�J��OpenGL�̃o�[�W�����w��
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(1024, 768, "SAMPLE", NULL, NULL);
	if (window == nullptr)
	{
		std::cerr << "Can't create GLFW window." << std::endl;
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);

	//GLEW�̏�����
	//MakeCOntextcurrent�̌�ɍs��Ȃ��Ǝ��s����炵��
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Can't initilize GLEW" << std::endl;
		return 1;
	}

	glClearColor(0.2f, 0.2f, 0.2f, 0.2f);

	//Shader�I�u�W�F�N�g�쐬
	vertShader = glCreateShader(GL_VERTEX_SHADER);
	fragShader = glCreateShader(GL_FRAGMENT_SHADER);

	//�\�[�X�v���O�����ǂݍ���
	if (readShaderSource(vertShader, "3dpoint.vert")) exit(1);
	if (readShaderSource(fragShader, "3dpoint.frag")) exit(1);

	//Shader�R���p�C��
	glCompileShader(vertShader);
	glCompileShader(fragShader);

	//�v���O�����I�u�W�F�N�g�쐬
	gl2Program = glCreateProgram();
	glAttachShader(gl2Program, vertShader);
	glDeleteShader(vertShader);
	glAttachShader(gl2Program, fragShader);
	glDeleteShader(fragShader);

	//�v���O�����I�u�W�F�N�g�̃����N
	glBindAttribLocation(gl2Program, 0, "position");
	glBindFragDataLocation(gl2Program, 0, "gl_FragColor");
	glLinkProgram(gl2Program);

	//���_�z��I�u�W�F�N�g

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//���_�o�b�t�@�I�u�W�F�N�g

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, pointlogs.size() * 4, nullptr, GL_DYNAMIC_DRAW);

	//Vertexshader�̎Q��
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//���_�o�b�t�@�I�u�W�F�N�g�̌�������
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	Matrix = glGetUniformLocation(gl2Program, "MVP");

	//�ۑ��pMat�t�@�C��
	cv::Mat gl_img(768, 1024, CV_8UC3);

	//OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
	bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
	bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
	bool err = gladLoadGL() == 0;
#else
	bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif
	if (err)
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!\n");
		return 1;
	}

	//Imgui�̏�����
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	while (glfwWindowShouldClose(window) == GL_FALSE && *flg)
	{
		vector<cv::Mat> worlds = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1];
		glClear(GL_COLOR_BUFFER_BIT);

		//Shader�v���O�����g�p�J�n
		glUseProgram(gl2Program);

		//���_�s��̍X�V
		computeMatrices();
		mvp = Projection * View;
		glUniformMatrix4fv(Matrix, 1, GL_FALSE, &mvp[0][0]);


		//�`�悷��_�Q
		for (auto j = 0; j < worlds.size(); ++j)
		{
			pos[j * 3 + 0] = worlds[j].at<double>(0, 0);
			pos[j * 3 + 1] = worlds[j].at<double>(0, 1);
			pos[j * 3 + 2] = worlds[j].at<double>(0, 2);
		}

		pointlogs.insert(pointlogs.end(), pos.begin(), pos.end());
		pointlogs.erase(pointlogs.begin(), pointlogs.begin() + 450);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		//glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), &position[0][0]);//�X�V
		/*�����ɕ`��*/
		glBufferSubData(GL_ARRAY_BUFFER, 0, pointlogs.size() * 4, &pointlogs[0]);
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, max_num*logsize);
		glBindVertexArray(0);

		glfwPollEvents();//�}�E�X�̓����ۑ�

		//Imgui�J�n
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		//Imgui�̃E�C���h�E�\��
		ImGui::SetNextWindowSize(ImVec2(200, 300));
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::Begin("View controller");
		{
			ImGui::Text("Time: %.3f [s]", (float)currentTime);

			ImGui::Spacing();
			ImGui::Text("View direction angle");
			ImGui::SliderFloat("v angle", &v_angle, -3.14f, 3.14f, "%.3f");
			ImGui::SliderFloat("h angle", &h_angle, -3.14f, 3.14f, "%.3f");
			
			ImGui::Spacing();
			ImGui::Text("Camera position");
			ImGui::SliderFloat("X", &diffX, -100.0f, 100.0f, "%.3f");
			ImGui::SliderFloat("Y", &diffY, -100.0f, 100.0f, "%.3f");
			ImGui::SliderFloat("Z", &diffZ, -100.0f, 100.0f, "%.3f");

			if (ImGui::Button("Reset")){
				v_angle = M_PI / 6.0f;
				h_angle = 0;
				diffX = 0;
				diffY = 0;
				diffZ = 0;
			}

			if (ImGui::Button("Quit")) { break;}
		}
		ImGui::End();

		//Imgui�����_�����O
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		//�t�����g�o�b�t�@�ƃo�b�N�o�b�t�@�̓���ς�
		glfwSwapBuffers(window);

		//OpenGL�̃o�b�t�@��cv::mat�z��ɕۑ�
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, gl_img.data);
		cv::flip(gl_img, gl_img, 0);
		/*cv::imshow("gl_img",gl_img);
		cv::waitKey(1);*/
		gl_img_Logs.push_back(gl_img.clone());//�X�V���Ԃ͌��ؒf�@�ƈ�v���Ă��Ȃ�
	}
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);

	//Imgui�̏I��
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();
}

void computeMatrices() {
	static double lastTime = glfwGetTime();//�͂��߂���
	currentTime = glfwGetTime();//����
	float deltaT = float(currentTime - lastTime);

	//�J�����ʒu�x�N�g���X�V
	campos = glm::vec3(0+diffX, -400+diffY, H_robot - H_camera+diffZ);

	//�����x�N�g���X�V
	glm::vec3 direction(
		cos(v_angle)*sin(h_angle),
		sin(v_angle),
		cos(v_angle)*cos(h_angle)
	);
	//�E�x�N�g���X�V
	glm::vec3 right = glm::vec3(
		sin(h_angle - 3.14f / 2.0f),
		0,
		cos(h_angle - 3.14f / 2.0f)
	);
	//��x�N�g��
	glm::vec3 up = glm::cross(right, direction);

	//�ˉe�s��̍X�V
	Projection = glm::perspective(glm::radians(fov), 4.0f / 3.0f, 0.1f, 1000.0f);

	//�J�����s��̍X�V
	View = glm::lookAt(
		campos,
		campos + direction,
		up
	);

	lastTime = currentTime;//���ԍX�V
}