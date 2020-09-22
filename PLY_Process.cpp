#include "PLY_Process.h"
#include "3DRealTimeRender.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
glm::vec3 cameraPos   = glm::vec3(0.0f, 0.0f, 1.0f);
glm::vec3 cameraTar = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);

float fov =  45.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


// ================================================================================
// init opengl window
// ================================================================================
void start_PLY_FIFO_Process(FIFO<framePacket>** input, Context *context){
    std::printf("Thread opengl live render started\n"); fflush(stdout);
    std::vector<Point3f> verts;
	std::vector<RGB> colors;
	RGB tempColor;
    Point3f temp;

    int cnt = 0;

    //init glfw and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //glfw: create window
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Kinect 3D", NULL, NULL);
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

    //glEnable(GL_DEPTH_TEST);
#ifdef CMAKE
    Shader shader("../include/opengl/vertexShader.vert", "../include/opengl/fragShader.frag");
#else
    Shader shader("include/opengl/vertexShader.vert", "include/opengl/fragShader.frag");
#endif

    // create VAO and VBO, bind VAO first, then bind and set vertex buffer
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(glVertices), glVertices, GL_STATIC_DRAW);

    // configure vertex attribute: position and color
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    while(!glfwWindowShouldClose(window)){
        for(int i=0; i<numKinects; i++){
            framePacket *packet = input[i]->get();
            if( packet == NULL ) { continue; }

            for(int i=0; i<packet->height_d *  packet->width_d; i++){
                temp.X = packet->vertices[i].X;
                temp.Y = packet->vertices[i].Y;
                temp.Z = packet->vertices[i].Z;
                tempColor.B = packet->vertices[i].B;
                tempColor.G = packet->vertices[i].G;
                tempColor.R = packet->vertices[i].R;
                if(temp.Z > z_bbox_min && temp.Z < z_bbox_max && 
                   temp.X > x_bbox_min && temp.X < x_bbox_max && 
                   temp.Y > y_bbox_min && temp.Y < y_bbox_max){
                    verts.push_back(temp);
                    colors.push_back(tempColor);
                }
            }
            packet->destroy();            
        }

        float *vertices = new float[6 * verts.size()];
        for(int i=0; i<verts.size(); i++){
            vertices[6*i] = verts[i].X;
            vertices[6*i + 1] = verts[i].Y;
            vertices[6*i + 2] = verts[i].Z;
            if(colors.size() == verts.size()){
                vertices[6*i + 3] = (float)(colors[i].R) / 255.0f;
                vertices[6*i + 4] = (float)(colors[i].G) / 255.0f;
                vertices[6*i + 5] = (float)(colors[i].B) / 255.0f;
            }
            else{
                vertices[6*i + 3] = 1.0;
                vertices[6*i + 4] = 0.0;
                vertices[6*i + 5] = 0.0;
            }
        }
        if(context->b_save2Local){
            std::string filename = "./datas/pointcloud_" + std::to_string(cnt++) + ".ply";
            savePlyFile(filename.c_str(), verts, false, colors);
            std::printf("Save one frame pointcloud\n");fflush(stdout);
        }

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_DEPTH_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT);

        glBufferData(GL_ARRAY_BUFFER, 6*4*verts.size(), vertices, GL_DYNAMIC_DRAW);
        shader.use();
        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        shader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = glm::lookAt(cameraPos, cameraTar, cameraUp);
        shader.setMat4("view", view);
        shader.setMat4("model", glm::mat4(1.0f));

        glBindVertexArray(VAO);
        glPointSize(1.0);
        glDrawArrays(GL_POINTS, 0, verts.size());

        glfwSwapBuffers(window);
        glfwPollEvents();
        delete vertices;
        verts.clear();
        colors.clear();

        usleep(20000);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
}


// ================================================================================
// key board input
// ================================================================================
void processInput(GLFWwindow *window)
{
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
}


// ================================================================================
// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ================================================================================
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


void destroy_PLY_FIFO_Process(){

}