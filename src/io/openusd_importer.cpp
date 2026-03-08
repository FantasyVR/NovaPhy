#include "novaphy/io/openusd_importer.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace novaphy {
namespace {

std::string trim(const std::string& s) {
    const size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    const size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

Vec3f parse_vec3_tuple(const std::string& value, const Vec3f& fallback) {
    std::regex tuple_re(R"(\(([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\))");
    std::smatch m;
    if (!std::regex_search(value, m, tuple_re)) return fallback;
    return Vec3f(std::stof(m[1].str()), std::stof(m[2].str()), std::stof(m[3].str()));
}

Vec4f parse_vec4_tuple(const std::string& value, const Vec4f& fallback) {
    std::regex tuple_re(R"(\(([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\))");
    std::smatch m;
    if (!std::regex_search(value, m, tuple_re)) return fallback;
    return Vec4f(std::stof(m[1].str()), std::stof(m[2].str()), std::stof(m[3].str()), std::stof(m[4].str()));
}

std::string parse_quoted(const std::string& value) {
    const size_t first = value.find('"');
    if (first == std::string::npos) return "";
    const size_t second = value.find('"', first + 1);
    if (second == std::string::npos) return "";
    return value.substr(first + 1, second - first - 1);
}

std::string join_path(const std::string& parent, const std::string& child) {
    if (parent.empty()) return "/" + child;
    if (parent == "/") return "/" + child;
    return parent + "/" + child;
}

int find_prim_by_path(const std::vector<UsdPrim>& prims, const std::string& path) {
    for (int i = 0; i < static_cast<int>(prims.size()); ++i) {
        if (prims[i].path == path) return i;
    }
    return -1;
}

}  // namespace

OpenUsdImporter::OpenUsdImporter(float min_supported_version)
    : min_supported_version_(min_supported_version) {}

UsdStageData OpenUsdImporter::import_file(const std::filesystem::path& usd_path) const {
    std::ifstream in(usd_path);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open USD file: " + usd_path.string());
    }

    std::vector<std::string> scope_paths;
    scope_paths.push_back("");
    UsdStageData stage;
    std::string line;
    std::regex version_re(R"(#usda\s+([0-9]+)\.([0-9]+))");
    std::regex prim_re(R"PRIM((def|over)\s+([A-Za-z0-9_]+)\s+"([^"]+)")PRIM");
    std::regex timesample_key_re(R"(([-+0-9eE\.]+)\s*:\s*(.*))");

    int active_prim_index = -1;
    bool in_time_samples = false;
    std::string active_time_sample_property;

    while (std::getline(in, line)) {
        const std::string t = trim(line);
        const std::string_view text = t;
        if (text.empty()) continue;

        std::smatch vm;
        if (std::regex_search(t, vm, version_re)) {
            float version = std::stof(vm[1].str() + "." + vm[2].str());
            if (version < min_supported_version_) {
                throw std::runtime_error("USD version too old: " + std::to_string(version));
            }
            continue;
        }

        if (text == "}") {
            if (in_time_samples) {
                in_time_samples = false;
                active_time_sample_property.clear();
            } else if (scope_paths.size() > 1) {
                scope_paths.pop_back();
                active_prim_index = find_prim_by_path(stage.prims, scope_paths.back());
            }
            continue;
        }

        if (active_prim_index >= 0 && in_time_samples) {
            std::smatch tsm;
            if (std::regex_search(t, tsm, timesample_key_re)) {
                float time = std::stof(tsm[1].str());
                const std::string rhs = trim(tsm[2].str());
                UsdPrim& prim = stage.prims[active_prim_index];
                auto it = std::find_if(
                    prim.tracks.begin(), prim.tracks.end(),
                    [&](const UsdAnimationTrack& track) { return track.property_name == active_time_sample_property; });
                if (it == prim.tracks.end()) {
                    prim.tracks.push_back(UsdAnimationTrack{active_time_sample_property});
                    it = prim.tracks.end() - 1;
                }
                if (rhs.find('(') != std::string::npos) {
                    if (rhs.find(',') != std::string::npos && std::count(rhs.begin(), rhs.end(), ',') == 3) {
                        const Vec4f q = parse_vec4_tuple(rhs, Vec4f(1.0f, 0.0f, 0.0f, 0.0f));
                        it->vec4_samples.emplace_back(time, q);
                    } else {
                        const Vec3f v = parse_vec3_tuple(rhs, Vec3f::Zero());
                        it->vec3_samples.emplace_back(time, v);
                    }
                }
            }
            continue;
        }

        std::smatch pm;
        if (std::regex_search(t, pm, prim_re)) {
            UsdPrim prim;
            prim.type_name = pm[2].str();
            prim.name = pm[3].str();
            prim.parent_path = scope_paths.back();
            prim.path = join_path(scope_paths.back(), prim.name);
            stage.prims.push_back(prim);
            scope_paths.push_back(prim.path);
            active_prim_index = static_cast<int>(stage.prims.size()) - 1;
            continue;
        }

        if (active_prim_index < 0) {
            if (text.starts_with("defaultPrim")) {
                stage.default_prim = parse_quoted(t);
            } else if (text.starts_with("upAxis")) {
                stage.up_axis = parse_quoted(t);
            } else if (text.starts_with("metersPerUnit")) {
                const size_t eq = t.find('=');
                if (eq != std::string::npos) stage.meters_per_unit = std::stof(trim(t.substr(eq + 1)));
            }
            continue;
        }

        UsdPrim& prim = stage.prims[active_prim_index];
        if (text.starts_with("float physics:mass")) {
            prim.mass = std::stof(trim(t.substr(t.find('=') + 1)));
        } else if (text.starts_with("float physics:density")) {
            prim.density = std::stof(trim(t.substr(t.find('=') + 1)));
        } else if (text.starts_with("rel material:binding")) {
            prim.material_binding = parse_quoted(t);
        } else if (text.starts_with("float3 novaphy:boxHalfExtents")) {
            prim.box_half_extents = parse_vec3_tuple(t, Vec3f::Zero());
        } else if (text.starts_with("float novaphy:sphereRadius")) {
            prim.sphere_radius = std::stof(trim(t.substr(t.find('=') + 1)));
        } else if (text.starts_with("double3 xformOp:translate")
                   || text.starts_with("float3 xformOp:translate")) {
            prim.local_transform.position = parse_vec3_tuple(t, Vec3f::Zero());
        } else if (text.starts_with("quatf xformOp:orient")
                   || text.starts_with("quatd xformOp:orient")) {
            const Vec4f q_wxyz = parse_vec4_tuple(t, Vec4f(1.0f, 0.0f, 0.0f, 0.0f));
            prim.local_transform.rotation = Quatf(q_wxyz.x(), q_wxyz.y(), q_wxyz.z(), q_wxyz.w()).normalized();
        } else if (t.find("timeSamples") != std::string::npos) {
            const size_t key_end = t.find("timeSamples");
            const std::string lhs = trim(t.substr(0, key_end));
            const size_t space = lhs.rfind(' ');
            active_time_sample_property = space == std::string::npos ? lhs : trim(lhs.substr(space + 1));
            in_time_samples = true;
        }
    }

    return stage;
}

float OpenUsdImporter::min_supported_version() const {
    return min_supported_version_;
}

}  // namespace novaphy
