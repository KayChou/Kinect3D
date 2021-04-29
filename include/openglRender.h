#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/string_cast.hpp"

#include "shader.h"
#include "common.h"
#include "utils.h"

bool render(std::vector<Point3f> verts, std::vector<RGB> colors);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

void start_PLY_FIFO_Process();


class openglRender
{
private:
    unsigned int WIN_WIDTH;
    unsigned int WIN_HEIGHT;
    unsigned int VBO;
    unsigned int VAO;
    unsigned int EBO;

    GLFWwindow* window;
    glm::vec3 cameraPos;
    glm::vec3 cameraTar;
    glm::vec3 cameraUp;

    float fov;
    float deltaTime;
    float lastFrame;

    Shader *shaderModel;

    FIFO<frameMesh>** input;
    Context *context;

    float *vertices;
    int *faces;

    int filter_anti_shake_cnt;

public:
    void init(FIFO<frameMesh>** input, Context *context);
    void loop();

    void processInput();
    void glInit();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);

};