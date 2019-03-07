#include "graphics.h"
#include <vector>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#define GLFW_NO_GLU

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
//View�s��Čv�Z���̃p�����[�^
float h_angle = 0;
float v_angle = M_PI / 6.0f;
float fov = 45.0f;
float speed = 3.0f;
float mousespeed = 0.005f;
float H_robot = 470;
const float H_camera = 600;

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

	glfwTerminate();
}

void computeMatrices() {
	static double lastTime = glfwGetTime();//�͂��߂���
	double currentTime = glfwGetTime();//����
	float deltaT = float(currentTime - lastTime);

	//Mouse�ʒu���猩�Ă��������ύX
	double xp, yp;
	glfwGetCursorPos(window, &xp, &yp);
	//glfwSetCursorPos(window, 1024/2, 768/2);//�}�E�X�ʒu���Z�b�g
	//printf("xp: %f, yp: %f \n",xp,yp);

	//�J�����ʒu�x�N�g���X�V
	campos = glm::vec3(0, -400, H_robot - H_camera);

	//�����x�N�g���X�V
	/*h_angle += mousespeed * float(1024 / 2 - xp);
	v_angle += mousespeed * float(768 / 2 - yp);*/
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


	//���L�[�������J�����ʒu�ύX
	// Move forward
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		campos += direction * deltaT * speed;
	}
	// Move backward
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		campos -= direction * deltaT * speed;
	}
	// Strafe right
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		campos += right * deltaT * speed;
	}
	// Strafe left
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		campos -= right * deltaT * speed;
	}

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