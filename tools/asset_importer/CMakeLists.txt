cmake_minimum_required(VERSION 3.20)

slope_executable(asset_importer ${SLOPE_TOOLS_OUTPUT_DIR})

slope_internal_deps(asset_importer PRIVATE slope)
slope_external_deps(asset_importer PRIVATE nlohmann_json cxxopts)
