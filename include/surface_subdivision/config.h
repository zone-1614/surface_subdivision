#pragma once
#include <string>
#include <filesystem>

namespace zone {
static std::string current_path = std::filesystem::current_path().string();

}
