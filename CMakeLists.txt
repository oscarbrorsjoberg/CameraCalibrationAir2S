cmake_minimum_required(VERSION 3.11)
project(CameraCalibration VERSION 1.0.0)


find_package(OpenCV 4.5 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	program_options)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)



macro(Depender depender)
	if(NOT ${depender}_FOUND)
		message(FATAL_ERROR "Camera calibration requires ${depender}")
	else()
		message("${depender} VERSION " ${${depender}_VERSION})
	endif()
endmacro()


Depender(OpenCV)
Depender(Boost)
Depender(yaml-cpp)
Depender(GTest)


set(build_flags #APPEND
 "-std=c++17"
 "-lstdc++fs"
 "-Wall"
 "-Wextra"
 "-pedantic"
)

# Test binary

add_executable(camtests
	src/tests/main.cpp
	src/camera.cpp
)

target_compile_options(camtests
	PUBLIC
	${build_flags}
)

target_include_directories(camtests
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/src/
)

target_link_libraries(camtests
	PUBLIC
	${OpenCV_LIBS}
	GTest::gtest_main
	yaml-cpp
)


# cli binary

add_executable(${PROJECT_NAME}
	src/main.cpp
	src/utils.cpp
	src/camera.cpp
)


target_compile_options(${PROJECT_NAME}
	PUBLIC
	${build_flags}
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/src/
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	${OpenCV_LIBS}
	Boost::program_options
	yaml-cpp
)
