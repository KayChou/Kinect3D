#version 460 core
out vec4 FragColor;

in vec3 RGB;

void main()
{
	//FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
	FragColor = vec4(RGB, 1.0f);
}