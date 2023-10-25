add_executable(sprout main.cpp)
target_include_directories(sprout PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(sprout PRIVATE base detailed)

# Add Gurobi
# find_package(GUROBI REQUIRED)
# target_link_libraries(pd PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
# target_link_libraries(pd PUBLIC ${GUROBI_LIBRARY})
# target_include_directories(pd PRIVATE "${GUROBI_INCLUDE_DIRS}")

# Add clipp library
#target_link_libraries(pd PRIVATE clipp)
#target_include_directories(pd PRIVATE "${clipp_SOURCE_DIR}"