#pragma once

#include <filesystem>

#include "novaphy/io/scene_types.h"

namespace novaphy {

class OpenUsdImporter {
public:
    explicit OpenUsdImporter(float min_supported_version = 21.08f);
    UsdStageData import_file(const std::filesystem::path& usd_path) const;
    float min_supported_version() const;

private:
    float min_supported_version_ = 21.08f;
};

}  // namespace novaphy
