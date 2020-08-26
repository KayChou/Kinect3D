#ifndef RealTimeRender_H_
#define RealTimeRender_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "shader.h"
#include "common.h"

bool render(std::vector<Point3f> verts, std::vector<RGB> colors);

void processInput(GLFWwindow *window);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

#endif