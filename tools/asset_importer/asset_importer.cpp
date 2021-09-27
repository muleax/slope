#include "nlohmann/json.hpp"
#include "cxxopts/include/cxxopts.hpp"
#include <iostream>
#include <fstream>

bool convert(const std::string& path) {
    std::ifstream src(path);
    if (!src.is_open()) {
        std::cout << "Error: can't open " << path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(src,line)) {
        std::cout << line << '\n';
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