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
** シェーダオブジェクト
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;
GLFWwindow  *window;
//頂点オブジェクト
GLuint vao;
GLuint vbo;
//OpenGL内での変数
const int max_num = 150;
int logsize = 150;//750fpsの場合で何フレーム分出力するか
vector<float> pointlogs(max_num * 3 * logsize, 0);
vector<float> pos(max_num * 3, 0);
glm::mat4 mvp;
glm::mat4 View;
glm::mat4 Projection;
GLuint Matrix;
glm::vec3 campos;//カメラ位置座標ベクトル
vector<cv::Mat> gl_img_Logs;//OpenGL出力図の保存Vector
//ImguiでのView行列再計算時のパラメータ
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

//OpenGLで点群描画する関数
int writepointcloud(Capture *cap, bool *flg) {
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

	window = glfwCreateWindow(1024, 768, "SAMPLE", NULL, NULL);
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

	//頂点配列オブジェクト

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//頂点バッファオブジェクト

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, pointlogs.size() * 4, nullptr, GL_DYNAMIC_DRAW);

	//Vertexshaderの参照
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//頂点バッファオブジェクトの結合解除
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	Matrix = glGetUniformLocation(gl2Program, "MVP");

	//保存用Matファイル
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

	//Imguiの初期化
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	while (glfwWindowShouldClose(window) == GL_FALSE && *flg)
	{
		vector<cv::Mat> worlds = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1];
		glClear(GL_COLOR_BUFFER_BIT);

		//Shaderプログラム使用開始
		glUseProgram(gl2Program);

		//視点行列の更新
		computeMatrices();
		mvp = Projection * View;
		glUniformMatrix4fv(Matrix, 1, GL_FALSE, &mvp[0][0]);


		//描画する点群
		for (auto j = 0; j < worlds.size(); ++j)
		{
			pos[j * 3 + 0] = worlds[j].at<double>(0, 0);
			pos[j * 3 + 1] = worlds[j].at<double>(0, 1);
			pos[j * 3 + 2] = worlds[j].at<double>(0, 2);
		}

		pointlogs.insert(pointlogs.end(), pos.begin(), pos.end());
		pointlogs.erase(pointlogs.begin(), pointlogs.begin() + 450);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		//glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), &position[0][0]);//更新
		/*ここに描画*/
		glBufferSubData(GL_ARRAY_BUFFER, 0, pointlogs.size() * 4, &pointlogs[0]);
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, max_num*logsize);
		glBindVertexArray(0);

		glfwPollEvents();//マウスの動き保存

		//Imgui開始
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		//Imguiのウインドウ表示
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

		//Imguiレンダリング
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		//フロントバッファとバックバッファの入れ変え
		glfwSwapBuffers(window);

		//OpenGLのバッファをcv::mat配列に保存
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, gl_img.data);
		cv::flip(gl_img, gl_img, 0);
		/*cv::imshow("gl_img",gl_img);
		cv::waitKey(1);*/
		gl_img_Logs.push_back(gl_img.clone());//更新時間は光切断法と一致していない
	}
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);

	//Imguiの終了
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();
}

void computeMatrices() {
	static double lastTime = glfwGetTime();//はじめだけ
	currentTime = glfwGetTime();//毎回
	float deltaT = float(currentTime - lastTime);

	//カメラ位置ベクトル更新
	campos = glm::vec3(0+diffX, -400+diffY, H_robot - H_camera+diffZ);

	//方向ベクトル更新
	glm::vec3 direction(
		cos(v_angle)*sin(h_angle),
		sin(v_angle),
		cos(v_angle)*cos(h_angle)
	);
	//右ベクトル更新
	glm::vec3 right = glm::vec3(
		sin(h_angle - 3.14f / 2.0f),
		0,
		cos(h_angle - 3.14f / 2.0f)
	);
	//上ベクトル
	glm::vec3 up = glm::cross(right, direction);

	//射影行列の更新
	Projection = glm::perspective(glm::radians(fov), 4.0f / 3.0f, 0.1f, 1000.0f);

	//カメラ行列の更新
	View = glm::lookAt(
		campos,
		campos + direction,
		up
	);

	lastTime = currentTime;//時間更新
}