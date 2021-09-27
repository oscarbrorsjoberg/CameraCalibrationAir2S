cmake_minimum_required(VERSION 3.11)
project(CameraCalibration VERSION 1.0.0)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	program_options
)

find_package(yaml-cpp REQUIRED)


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(build_flags #APPEND
 "-std=c++17"
 "-lstdc++fs"
 "-Wall"
 "-Wextra"
 "-pedantic"
)


message("OpenCV VERSION " ${OpenCV_VERSION})
message("Boost VERSION " ${Boost_VERSION})
message("YAML VERSION " ${yaml-cpp_VERSION})

add_executable(${PROJECT_NAME}
	src/main.cpp
)

target_compile_options(${PROJECT_NAME}
	PUBLIC
	${build_flags}
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	${OpenCV_LIBS}
	Boost::program_options
	yaml-cpp
	)
