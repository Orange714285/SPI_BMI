cmake_minimum_required (VERSION 3.10)

project(gpio_demo)

aux_source_directory (${CMAKE_SOURCE_DIR}/src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_include_directories (${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE pigpio gpiod)
