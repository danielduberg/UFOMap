cmake_minimum_required(VERSION 3.9)

project(ufomap 
	VERSION 1.0 
	DESCRIPTION "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown"
	LANGUAGES CXX
)

set(MODERN_CMAKE_BUILD_TESTING ON)

include(CTest)

add_subdirectory(cmake)
add_subdirectory(docs)
add_subdirectory(src)
add_subdirectory(tests)