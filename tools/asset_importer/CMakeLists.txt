cmake_minimum_required(VERSION 3.20)

slope_executable(asset_importer ${SLOPE_TOOLS_OUTPUT_DIR})

target_link_libraries(asset_importer
        PUBLIC slope nlohmann_json
        PRIVATE cxxopts)
