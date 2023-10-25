# Collect file information
file(GLOB base_SRC "*.cpp")
file(GLOB base_HEADER "*.h")

add_library(base ${base_SRC} ${base_HEADER})
target_include_directories(base PUBLIC ${PROJECT_SOURCE_DIR}/src/base)

# add_subdirectory(global)
# add_subdirectory(detailed)
