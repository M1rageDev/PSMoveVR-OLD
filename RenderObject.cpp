#include "RenderObject.h"

size_t split(const std::string& txt, std::vector<std::string>& strs, char ch)
{
	size_t pos = txt.find(ch);
	size_t initialPos = 0;
	strs.clear();

	while (pos != std::string::npos) {
		strs.push_back(txt.substr(initialPos, pos - initialPos));
		initialPos = pos + 1;

		pos = txt.find(ch, initialPos);
	}

	strs.push_back(txt.substr(initialPos, std::min(pos, txt.size()) - initialPos + 1));

	return strs.size();
}

RenderObject::RenderObject() {

}

RenderObject::RenderObject(Shader* shader_) {
	shader = shader_;
}

RenderObject::~RenderObject() {
}

void RenderObject::Draw() {
	glBindVertexArray(VAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glDrawElements(GL_TRIANGLES, triLength, GL_UNSIGNED_INT, 0);
}

void RenderObject::LoadModel(std::string path) {
	std::ifstream reader(path);

	std::vector<glm::vec3> positions;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	std::vector<float> verts;
	std::vector<unsigned int> tris;

	if (reader.is_open()) {
		std::string line;
		while (std::getline(reader, line)) {
			if (line.empty()) continue;

			std::vector<std::string> parts;
			split(line, parts, ' ');
			if (parts[0] == "v") {
				float x = std::stof(parts[1]);
				float y = std::stof(parts[2]);
				float z = std::stof(parts[3]);
				positions.emplace_back(x, y, z);
			}
			else if (parts[0] == "vt") {
				float u = std::stof(parts[1]);
				float v = std::stof(parts[2]);
				uvs.emplace_back(u, v);
			}
			else if (parts[0] == "vn") {
				float x = std::stof(parts[1]);
				float y = std::stof(parts[2]);
				float z = std::stof(parts[3]);
				normals.emplace_back(x, y, z);
			}
			else if (parts[0] == "f") {
				for (int i = 1; i <= 3; i++) {
					std::vector<std::string> indices;
					split(parts[i], indices, '/');
					int posIndex = std::stoi(indices[0]) - 1;
					int uvIndex = std::stoi(indices[1]) - 1;
					int normalIndex = std::stoi(indices[2]) - 1;
					verts.push_back(positions[posIndex].x);
					verts.push_back(positions[posIndex].y);
					verts.push_back(positions[posIndex].z);
					verts.push_back(uvs[uvIndex].x);
					verts.push_back(uvs[uvIndex].y);
					verts.push_back(normals[normalIndex].x);
					verts.push_back(normals[normalIndex].y);
					verts.push_back(normals[normalIndex].z);
					tris.push_back(verts.size() / 8 - 1);
				}
			}
		}
		reader.close();
	}

	unsigned int vertLength = verts.size();
	triLength = tris.size();

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(verts[0]) * verts.size(), verts.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(tris[0]) * tris.size(), tris.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));
	glEnableVertexAttribArray(2);

	return;
}
