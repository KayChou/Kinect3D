#include "openglRender.h"


void openglRender::init(FIFO<frameMesh>** input, Context *context)
{
    this->WIN_WIDTH = 800;
    this->WIN_HEIGHT = 600;

    this->input = input;
    this->context = context;

    this->cameraTar = glm::vec3(0.000000, -0.300000, -0.300000);
    this->cameraPos = glm::vec3(-0.772169, -0.946579, -0.873113);
    this->cameraUp = glm::vec3(0.542452, 0.092309, -0.835000);
    this->fov =  45.0f;

    // timing
    this->deltaTime = 0.0f;	// time between current frame and last frame
    this->lastFrame = 0.0f;

    this->vertices = new float[Width_depth_HR * Height_depth_HR * 6 * 5];
    this->faces = new int[Width_depth_HR * Height_depth_HR * 3 * 5];
}


void openglRender::loop()
{
    std::printf("Thread opengl live render started\n"); fflush(stdout);

    RGB tempColor;
    Point3f temp;
    int Vn = 0;
    int Fn = 0;
    int cnt = 0;
    std::string filename;

    timeval t_start, t_end;
    float t_delay;

    glInit();

    while(!glfwWindowShouldClose(window)) {
        gettimeofday(&t_start, NULL);
        Vn = 0;
        Fn = 0;
        for(int i=0; i<numKinects; i++) {
            frameMesh *packet = input[i]->get();

            for(int i=0; i<packet->Fn; i++) {
                faces[3 * Fn + 0] = packet->triangles[i].v1 + Vn;
                faces[3 * Fn + 1] = packet->triangles[i].v2 + Vn;
                faces[3 * Fn + 2] = packet->triangles[i].v3 + Vn;
                if(faces[3 * Fn + 0] && faces[3 * Fn + 1] && faces[3 * Fn + 2]) {
                    Fn++;
                }
            }

            for(int i=0; i<packet->Vn; i++) {
                vertices[6 * Vn + 0] = packet->vertices[i].X;
                vertices[6 * Vn + 1] = packet->vertices[i].Y;
                vertices[6 * Vn + 2] = packet->vertices[i].Z;
                vertices[6 * Vn + 3] = (float)packet->vertices[i].R / 255.0f;
                vertices[6 * Vn + 4] = (float)packet->vertices[i].G / 255.0f;
                vertices[6 * Vn + 5] = (float)packet->vertices[i].B / 255.0f;
                Vn++;
            } 
            packet->destroy();
        }

        if(context->b_save2Local) { // save pointcloud to local
            std::printf("Begin to save one frame\n"); fflush(stdout);
            filename =  "../datas/mesh_" + std::to_string(cnt++) + ".ply";
            savePlyFile(filename.c_str(), vertices, faces, Vn, Fn);
            std::printf("Finish save one frame\n"); fflush(stdout);
            
            if(context->b_cfg_saved == false) {
                saveKRT(this->context);
                context->b_cfg_saved = true;
            }
        }

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput();
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_DEPTH_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, 6 * 4 * Vn, vertices, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * 4 * Fn, faces, GL_DYNAMIC_DRAW);

        // configure vertex attribute: position and color
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        shaderModel->use();
        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f);
        shaderModel->setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = glm::lookAt(cameraPos, cameraTar, cameraUp);
        shaderModel->setMat4("view", view);
        shaderModel->setMat4("model", glm::mat4(1.0f));

        glBindVertexArray(VAO);
        glPointSize(2.0);
        glDrawArrays(GL_POINTS, 0, Vn);

        // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        // glDrawElements(GL_TRIANGLES, 3 * Fn, GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();

        gettimeofday(&t_end, NULL);
        t_delay = get_time_diff_ms(t_start, t_end);
#ifdef LOG
        std::printf("opengl render one frame: %f\n", t_delay); fflush(stdout);
#endif

        usleep(20000);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    delete [] this->vertices;
    delete [] this->faces;

    // glfw: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
}


void openglRender::glInit()
{
    //init glfw and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //glfw: create window
    this->window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Kinect 3D", NULL, NULL);
    if(window == NULL){
        std::cout << "Failed to create GLFW window" << std::endl; fflush(stdout);
        glfwTerminate();
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all opengl function pointers
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cout << "Failed to initialize GLAD" << std::endl; fflush(stdout);
    }

    #ifdef CMAKE
        shaderModel = new Shader("../include/opengl/vertexShader.vert", "../include/opengl/fragShader.frag");
    #else
        shaderModel = new Shader("include/opengl/vertexShader.vert", "include/opengl/fragShader.frag");
    #endif

    // create VAO and VBO, bind VAO first, then bind and set vertex buffer
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
}

// ================================================================================
// key board input
// ================================================================================
void openglRender::processInput()
{
    float translationSpeed = 0.1;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        if(glm::length(cameraPos - cameraTar) > cameraSpeed){
            cameraPos += cameraSpeed * glm::normalize(cameraTar - cameraPos);
        }
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        cameraPos -= cameraSpeed * glm::normalize(cameraTar - cameraPos);
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS | glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        glm::vec3 temp = cameraPos + cameraSpeed * glm::normalize(glm::cross(cameraUp, cameraTar-cameraPos));
        cameraPos = cameraTar + glm::length(cameraPos - cameraTar) * glm::normalize(temp - cameraTar);
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS | glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        glm::vec3 temp = cameraPos - cameraSpeed * glm::normalize(glm::cross(cameraUp, cameraTar-cameraPos));
        cameraPos = cameraTar + glm::length(cameraPos - cameraTar) * glm::normalize(temp - cameraTar);
    }
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){
        glm::vec3 right = glm::cross(cameraUp, cameraTar - cameraPos);
        glm::vec3 temp = (cameraPos + cameraSpeed * cameraUp);
        cameraPos = cameraTar + glm::length(cameraTar - cameraPos) * glm::normalize(temp - cameraTar);
        cameraUp = glm::normalize(glm::cross(cameraTar - cameraPos, right));
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS){
        glm::vec3 right = glm::cross(cameraUp, cameraTar - cameraPos);
        glm::vec3 temp = (cameraPos - cameraSpeed * cameraUp);
        cameraPos = cameraTar + glm::length(cameraTar - cameraPos) * glm::normalize(temp - cameraTar);
        cameraUp = glm::normalize(glm::cross(cameraTar - cameraPos, right));
    }
    if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS){ // translate x +
        cameraPos += glm::vec3(translationSpeed, 0.0f, 0.0f);
        cameraTar += glm::vec3(translationSpeed, 0.0f, 0.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){ // translate y +
        cameraPos += glm::vec3(0.0f, translationSpeed, 0.0f);
        cameraTar += glm::vec3(0.0f, translationSpeed, 0.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS){ // translate z +
        cameraPos += glm::vec3(0.0f, 0.0f, translationSpeed);
        cameraTar += glm::vec3(0.0f, 0.0f, translationSpeed);
    }
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS){ // translate x -
        cameraPos -= glm::vec3(translationSpeed, 0.0f, 0.0f);
        cameraTar -= glm::vec3(translationSpeed, 0.0f, 0.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){ // translate y -
        cameraPos -= glm::vec3(0.0f, translationSpeed, 0.0f);
        cameraTar -= glm::vec3(0.0f, translationSpeed, 0.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS){ // translate z -
        cameraPos -= glm::vec3(0.0f, 0.0f, translationSpeed);
        cameraTar -= glm::vec3(0.0f, 0.0f, translationSpeed);
    }
    // std::cout << "Target: " << glm::to_string(cameraTar) << std::endl;
    // std::cout << "Position: " << glm::to_string(cameraPos) << std::endl;
    // std::cout << "Up: " << glm::to_string(cameraUp) << std::endl;
}


// ================================================================================
// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ================================================================================
void openglRender::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}
