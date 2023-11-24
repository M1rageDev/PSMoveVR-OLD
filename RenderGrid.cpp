#include "RenderGrid.h"

RenderGrid::RenderGrid() {

}

RenderGrid::RenderGrid(Shader* shader_, int w, int h) {
	shader = shader_;

	std::vector<glm::vec3> verts;
	std::vector<unsigned int> indices;

	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			float x = (float)i;
			float y = -1.f;
			float z = (float)j;
			verts.push_back(glm::vec3(x, y, z));
		}
	}

	for (int i = 0; i < h; i++) {
		if (i != w - 1) {
			indices.push_back(i);
			indices.push_back(i + w);
		}
		if (i != h - 1) {
			indices.push_back(i);
			indices.push_back(i + 1);
		}
	}

	elementLength = indices.size() * 4;

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(verts[0]) * verts.size(), verts.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
}

RenderGrid::~RenderGrid() {
}

void RenderGrid::Draw() {
	glBindVertexArray(VAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glDrawElements(GL_LINES, elementLength, GL_UNSIGNED_INT, 0);
}
