#include "slope/containers/string.hpp"
#include "slope/containers/vector.hpp"
#include "slope/debug/log.hpp"
#include "slope/math/math.hpp"
#include "nlohmann/json.hpp"
#include "cxxopts/include/cxxopts.hpp"
#include <iostream>
#include <fstream>

using namespace slope;

struct MeshData {
    struct Vertex {
        Vec3 position;
        Vec3 normal;
        Vec2 tex_coords;
    };

    struct Face {
        uint32_t start_index = 0;
        uint32_t size = 0;
    };

    Vector<Vertex>      m_vertices;
    Vector<uint32_t>    m_indices;
    Vector<Face>        m_faces;
};

bool convert(const String& path) {
    std::ifstream src(path);
    if (!src.is_open()) {
        log::error("Can't open {}");
        return false;
    }

    String line;
    while (std::getline(src,line)) {
        std::istringstream ss(line);
        // TODO: conversion
    }
    src.close();

    return true;
}

int main(int argc, char** argv) {
    cxxopts::Options options("asset_importer");

    options.add_options()
            ("path", "source-path", cxxopts::value<std::string>())
            ("h,help", "Print usage");

    options.positional_help("source-path");
    options.parse_positional("path");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    if (result.count("path")) {
        bool ok = convert(result["path"].as<std::string>());
        return ok ? 0 : 1;
    }

    std::cout << "Use -h, --help for usage details" << std::endl;
    return 0;
}