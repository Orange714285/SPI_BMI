cmake_minimum_required(VERSION 3.10)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)
add_library(dart STATIC
	${SRC_LIST}
)

target_include_directories(dart PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dart PUBLIC
	pigpio 
	gpiod
)


		

