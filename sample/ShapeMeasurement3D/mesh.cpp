#include "mesh.h"

GLFWwindow  *window;
//頂点オブジェクト
GLuint vao;
GLuint vbo;

const int max_num = 150;
GLfloat position[max_num][3];//numは定数でないといけないので注意
glm::mat4 mvp;
GLuint Matrix;

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
int readShaderSource(GLuint shader, const char *file)
{
	FILE *fp;
	const GLchar *source;
	GLsizei length;
	int ret;

	/* ファイルを開く */
	fp = fopen(file, "rb");
	if (fp == NULL) {
		perror(file);
		return -1;
	}

	/* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
	fseek(fp, 0L, SEEK_END);
	length = ftell(fp);

	/* ファイルサイズのメモリを確保 */
	source = (GLchar *)malloc(length);
	if (source == NULL) {
		fprintf(stderr, "Could not allocate read buffer.\n");
		return -1;
	}

	/* ファイルを先頭から読み込む */
	fseek(fp, 0L, SEEK_SET);
	ret = fread((void *)source, 1, length, fp) != (size_t)length;
	fclose(fp);

	/* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
	if (ret)
		fprintf(stderr, "Could not read file: %s.\n", file);
	else
		glShaderSource(shader, 1, &source, &length);

	/* 確保したメモリの開放 */
	free((void *)source);

	return ret;
}

//OpenGL全体の初期化
int initmesh() {
	if (glfwInit() == GL_FALSE)
	{
		std::cerr << "Can't initilize GLFW" << std::endl;
		return 1;
	}
	//開くOpenGLのバージョン指定
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

	//GLEWの初期化
	//MakeCOntextcurrentの後に行わないと失敗するらしい
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Can't initilize GLEW" << std::endl;
		return 1;
	}

	glClearColor(0.2f, 0.2f, 0.2f, 0.2f);

	//Shaderオブジェクト作成
	vertShader = glCreateShader(GL_VERTEX_SHADER);
	fragShader = glCreateShader(GL_FRAGMENT_SHADER);

	//ソースプログラム読み込み
	if (readShaderSource(vertShader, "3dpoint.vert")) exit(1);
	if (readShaderSource(fragShader, "3dpoint.frag")) exit(1);

	//Shaderコンパイル
	glCompileShader(vertShader);
	glCompileShader(fragShader);

	//プログラムオブジェクト作成
	gl2Program = glCreateProgram();
	glAttachShader(gl2Program, vertShader);
	glDeleteShader(vertShader);
	glAttachShader(gl2Program, fragShader);
	glDeleteShader(fragShader);

	//プログラムオブジェクトのリンク
	glBindAttribLocation(gl2Program, 0, "position");
	glBindFragDataLocation(gl2Program, 0, "gl_FragColor");
	glLinkProgram(gl2Program);

	//点群初期化
	for (auto j = 0; j < max_num; ++j)
	{
		position[j][0] = 0.0f;
		position[j][1] = 0.0f;
		position[j][2] = 0.0f;
	}

	//頂点配列オブジェクト

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//頂点バッファオブジェクト

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(position), nullptr, GL_DYNAMIC_DRAW);

	//Vertexshaderの参照
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//頂点バッファオブジェクトの結合解除
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// カメラ行列
	glm::mat4 View = glm::lookAt(
		glm::vec3(4, 4, 4), // ワールド空間でカメラは(4,3,3)にあります。
		glm::vec3(0, 0, 0), // 原点を見ています。
		glm::vec3(0, 1, 0)  // 頭が上方向(0,-1,0にセットすると上下逆転します。)
	);
	glm::mat4 E = glm::mat4(1.0f);
	glm::mat4 Projection = glm::perspective(glm::radians(60.0f), 4.0f / 3.0f, 0.1f, 100.0f);
	mvp = Projection * View;

	Matrix = glGetUniformLocation(gl2Program, "MVP");
}


//3次元meshの描画
void makemesh(Capture *cap,bool *flg)
{
	vector<cv::Mat> worlds = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1];
	while (glfwWindowShouldClose(window) == GL_FALSE && worlds.size()>0 && *flg)
	{
		glClear(GL_COLOR_BUFFER_BIT);
		
		//Shaderプログラム使用開始
		glUseProgram(gl2Program);

		glUniformMatrix4fv(Matrix, 1, GL_FALSE, &mvp[0][0]);

		//描画する点群
		for (auto j = 0; j < worlds.size(); ++j)
		{
			position[j][0] = worlds[j].at<double>(0, 0);
			position[j][1] = worlds[j].at<double>(0, 1);
			position[j][2] = worlds[j].at<double>(0, 2);
		}

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), position);//更新
		/*ここに描画*/
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
