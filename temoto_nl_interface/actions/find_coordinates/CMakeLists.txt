cmake_minimum_required(VERSION 3.5)
project(example_navigate)

add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(temoto_action_engine REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} temoto_action_engine)

class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_package()