# Collect file information
file(GLOB detailed_SRC "*.cpp")
file(GLOB detailed_HEADER "*.h")

add_library(detailed ${detailed_SRC} ${detailed_HEADER})
target_include_directories(detailed PUBLIC ${PROJECT_SOURCE_DIR}/src/detailed)
target_link_libraries(detailed PRIVATE base)

# Add Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(detailed PUBLIC Eigen3::Eigen)