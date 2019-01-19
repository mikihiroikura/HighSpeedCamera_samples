#include "mesh.h"

GLFWwindow  *window;
//���_�I�u�W�F�N�g
GLuint vao;
GLuint vbo;

const int max_num = 150;
GLfloat position[max_num][3];//num�͒萔�łȂ��Ƃ����Ȃ��̂Œ���
glm::mat4 mvp;
GLuint Matrix;

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

//OpenGL�S�̂̏�����
int initmesh() {
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

	window = glfwCreateWindow(640, 480, "SAMPLE", NULL, NULL);
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

	//�_�Q������
	for (auto j = 0; j < max_num; ++j)
	{
		position[j][0] = 0.0f;
		position[j][1] = 0.0f;
		position[j][2] = 0.0f;
	}

	//���_�z��I�u�W�F�N�g

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//���_�o�b�t�@�I�u�W�F�N�g

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(position), nullptr, GL_DYNAMIC_DRAW);

	//Vertexshader�̎Q��
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//���_�o�b�t�@�I�u�W�F�N�g�̌�������
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// �J�����s��
	glm::mat4 View = glm::lookAt(
		glm::vec3(4, 4, 4), // ���[���h��ԂŃJ������(4,3,3)�ɂ���܂��B
		glm::vec3(0, 0, 0), // ���_�����Ă��܂��B
		glm::vec3(0, 1, 0)  // ���������(0,-1,0�ɃZ�b�g����Ə㉺�t�]���܂��B)
	);
	glm::mat4 E = glm::mat4(1.0f);
	glm::mat4 Projection = glm::perspective(glm::radians(60.0f), 4.0f / 3.0f, 0.1f, 100.0f);
	mvp = Projection * View;

	Matrix = glGetUniformLocation(gl2Program, "MVP");
}


//3����mesh�̕`��
void makemesh(Capture *cap,bool *flg)
{
	vector<cv::Mat> worlds = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1];
	while (glfwWindowShouldClose(window) == GL_FALSE && worlds.size()>0 && *flg)
	{
		glClear(GL_COLOR_BUFFER_BIT);
		
		//Shader�v���O�����g�p�J�n
		glUseProgram(gl2Program);

		glUniformMatrix4fv(Matrix, 1, GL_FALSE, &mvp[0][0]);

		//�`�悷��_�Q
		for (auto j = 0; j < worlds.size(); ++j)
		{
			position[j][0] = worlds[j].at<double>(0, 0);
			position[j][1] = worlds[j].at<double>(0, 1);
			position[j][2] = worlds[j].at<double>(0, 2);
		}

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), position);//�X�V
		/*�����ɕ`��*/
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, max_num);
		glBindVertexArray(0);

		glfwSwapBuffers(window);
	}
}

void endmesh() {
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);

	glfwTerminate();
}
