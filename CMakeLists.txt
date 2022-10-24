cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
project(azure-kinect-multiviewer)

# Main executables
add_executable(main main.cpp)
set_property(TARGET main PROPERTY CXX_STANDARD 17)
set_property(TARGET main PROPERTY CXX_STANDARD_REQUIRED ON)

# OpenGL
find_package(OpenGL REQUIRED)
target_link_libraries(main OpenGL::GL)
message(STATUS "OpenGL library status:")
message(STATUS "    libraries: ${OPENGL_LIBRARIES}")

# Azure Kinect DK SDK
set(k4a_DIR "C:/Program Files/Azure Kinect SDK v1.4.1")
set(k4a_LIB_DIR "${k4a_DIR}/sdk/windows-desktop/amd64/release/lib")
set(k4a_LIBS "${k4a_LIB_DIR}/k4a.lib" "${k4a_LIB_DIR}/k4arecord.lib")
include_directories("${k4a_DIR}/sdk/include")
target_link_libraries(main ${k4a_LIBS})

# turbojpeg
# set(libjpeg-turbo_DIR "C:/libjpeg-turbo-gcc64/lib/cmake/libjpeg-turbo") # GCC
set(libjpeg-turbo_DIR "C:/libjpeg-turbo64/lib/cmake/libjpeg-turbo") # MSVC
find_package(libjpeg-turbo REQUIRED)
target_link_libraries(main libjpeg-turbo::turbojpeg-static)

# OpenGL Loader - GL3W
set(gl3w_dir ${CMAKE_CURRENT_SOURCE_DIR}/gl3w)
set(gl3w_inc ${gl3w_dir}/include)
set(gl3w_src ${gl3w_dir}/src/gl3w.c)
if (EXISTS ${gl3w_src})
	message(STATUS "Found gl3w source files")
else()
	execute_process(COMMAND python3 gl3w_gen.py WORKING_DIRECTORY ${gl3w_dir}) # use python or python3
	if (NOT EXISTS ${gl3w_src})
		message(FATAL_ERROR "Could not generate gl3w source files")
	endif()
endif()
add_library(gl3w STATIC ${gl3w_src})
target_include_directories(gl3w PUBLIC ${gl3w_inc})
target_link_libraries(main gl3w)

# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/glfw)
target_link_libraries(main glfw)

# Dear ImGui
set(imgui_dir ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
file(GLOB imgui_src ${imgui_dir}/*.cpp)
set(imgui_backend ${imgui_dir}/backends/imgui_impl_glfw.cpp ${imgui_dir}/backends/imgui_impl_opengl3.cpp)
add_library(imgui STATIC ${imgui_src} ${imgui_backend})
target_include_directories(imgui PUBLIC ${imgui_dir} ${imgui_dir}/backends)
target_link_libraries(imgui glfw)
target_link_libraries(main imgui)

# Native File Dialog Extended
add_subdirectory("./nfd")
target_link_libraries(main nfd)
