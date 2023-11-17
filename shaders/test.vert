#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec3 aNormal;

out vec2 TexCoord;
out vec3 FragPos;
out vec3 ModelPos;
out vec3 Normal;

uniform mat4 projection;
uniform mat4 transform;
uniform vec4 translate;

void main()
{
    gl_Position = projection * (transform * vec4(aPosition, 1f) + vec4(0.0, 0.0, -3.0, 0.0) + translate);
    FragPos = (vec4(aPosition, 1f) * transform + translate).xyz;
	Normal = (transform * vec4(aNormal, 1.0)).xyz;
    ModelPos = aPosition;
    TexCoord = aTexCoord;
}