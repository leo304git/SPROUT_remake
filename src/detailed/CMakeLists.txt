# Collect file information
file(GLOB detailed_SRC "*.cpp")
file(GLOB detailed_HEADER "*.h")

add_library(detailed ${detailed_SRC} ${detailed_HEADER})
target_include_directories(detailed PUBLIC ${PROJECT_SOURCE_DIR}/src/detailed)
target_link_libraries(detailed PRIVATE base)

