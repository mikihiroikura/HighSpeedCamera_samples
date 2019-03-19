#pragma once
#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <GL/glew.h>
#include <glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <gtc/matrix_transform.hpp> 
#include <gtx/transform.hpp>
#include "parameters.h"

//�֐��̐錾
extern int readShaderSource(GLuint shader, const char *file);
extern int writepointcloud(Capture *cap, bool *flg);
extern void computeMatrices();

#endif // !GRAPHICS_H
