set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_include_directories(Dart PUBLIC ${PROJECT_SOURCE_DIR}/lib/include)
target_link_libraries(${PROJECT_NAME} PRIVATE
	dart
)


