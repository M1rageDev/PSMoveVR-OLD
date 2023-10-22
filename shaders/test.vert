#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec2 aTexCoord;

out vec2 TexCoord;
out vec3 FragPos;
out vec3 ModelPos;

uniform mat4 projection;
uniform mat4 transform;

void main()
{
    gl_Position = vec4(aPosition, 1f) * transform;
    FragPos = (vec4(aPosition, 1f) * transform).xyz;
    ModelPos = aPosition;
    TexCoord = aTexCoord;
}