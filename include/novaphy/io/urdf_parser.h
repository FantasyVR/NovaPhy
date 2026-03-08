#pragma once

#include <filesystem>
#include <string>

#include "novaphy/io/scene_types.h"

namespace novaphy {

class UrdfParser {
public:
    UrdfModelData parse_file(const std::filesystem::path& urdf_path) const;
    std::string write_string(const UrdfModelData& model) const;
    void write_file(const UrdfModelData& model, const std::filesystem::path& urdf_path) const;
};

}  // namespace novaphy
