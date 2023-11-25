#include "RenderGrid.h"

RenderGrid::RenderGrid() {

}

RenderGrid::RenderGrid(Shader* shader_, int w, int h) {
	shader = shader_;

	std::vector<glm::vec3> verts;
	std::vector<glm::uvec4> indices;

	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			float x = (float)i/(float)w;
			float y = -1.f;
			float z = (float)j / (float)h;
			verts.push_back(glm::vec3(x, y, z));
		}
	}

	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {

			int row1 = j * (h + 1);
			int row2 = (j + 1) * (w + 1);

			indices.push_back(glm::uvec4(row1 + i, row1 + i + 1, row1 + i + 1, row2 + i + 1));
			indices.push_back(glm::uvec4(row2 + i + 1, row2 + i, row2 + i, row1 + i));

		}
	}

	elementLength = indices.size() * 4;

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(verts[0]) * verts.size(), glm::value_ptr(verts[0]), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), glm::value_ptr(indices[0]), GL_STATIC_DRAW);

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
