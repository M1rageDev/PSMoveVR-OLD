#version 330 core

const vec3 dir = vec3(0.0, 1.0, 0.0);

in vec3 FragPos;
in vec2 TexCoord;
in vec3 Normal;
out vec4 FragColor;

uniform sampler2D texture0;
uniform sampler2D textureDiff;
uniform sampler2D textureSpec;
uniform vec3 bulbColor;
uniform vec3 viewPos;

void main()
{
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(dir); 
    vec3 diff = max(dot(norm, lightDir), 0.0) * vec3(1.0);
	
	vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
	vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
	
	float diffTex = texture2D(textureDiff, TexCoord).r;
	
	vec3 ambient = texture2D(texture0, TexCoord).xyz * vec3(0.5);
	vec3 diffuse = texture2D(texture0, TexCoord).xyz * (diffTex * diff);
	if (diffTex == 0) {
		diffuse = bulbColor;
	}
	vec3 specular = texture2D(textureSpec, TexCoord).xyz * spec;

    vec3 lightAmount = ambient + diffuse + specular;
    FragColor = vec4(lightAmount, 1f);
}
