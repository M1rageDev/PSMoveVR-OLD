#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>

bool fileExists(const char* name) {
    std::ifstream f(name);
    return f.good();
}
