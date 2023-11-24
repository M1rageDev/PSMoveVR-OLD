#version 330 core
layout (location = 0) in vec3 aPosition;

out vec3 FragPos;

uniform mat4 projection;

void main()
{
    gl_Position = projection * (vec4(aPosition, 1f) + vec4(0.0, 0.0, 5.0. 0.0));
    FragPos = aPosition;
}