cmake_minimum_required (VERSION 3.5)

# set c++ standard
message(STATUS "set c++ standard")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set binary output paths
message(STATUS "set binary output path")
set(PROJECT_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# set module path
message(STATUS "set module path")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# set pmp-library options
set(PMP_BUILD_EXAMPLES OFF)
set(PMP_BUILD_TESTS OFF)
set(PMP_BUILD_DOCS OFF)

project ("surface_subdivision" LANGUAGES CXX)

# set windows specific definition :)
if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES -DNOMINMAX)
endif()

# set up emscripten for generate html file
if (EMSCRIPTEN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -s ALLOW_MEMORY_GROWTH=1 -s USE_GLFW=3 -s WASM=0 -s USE_WEBGL2=1")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

message(STATUS "add submodule pmp-library")
add_subdirectory(external/pmp-library)

# configuration
message(STATUS "configure file")
# set(model_path "${PROJECT_SOURCE_DIR}/model/")
configure_file(config/config.h.in "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h")

# find_package(spdlog CONFIG REQUIRED)

file(GLOB srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)

message(STATUS "add executable and include directories, link libraries")
set(executable_name "surface_subdivision")
add_executable (${executable_name} ${srcs})
target_include_directories(${executable_name} PRIVATE include)
target_include_directories(${executable_name} PRIVATE ${PROJECT_SOURCE_DIR}/external/pmp-library/src)
target_include_directories(${executable_name} PRIVATE ${PROJECT_SOURCE_DIR}/external/pmp-library/external/imgui)
target_include_directories(${executable_name} PRIVATE ${PROJECT_SOURCE_DIR}/external/pmp-library/external/eigen)
target_include_directories(${executable_name} PRIVATE ${PROJECT_SOURCE_DIR}/external/pmp-library/external/glfw/include)
target_include_directories(${executable_name} PRIVATE ${PROJECT_SOURCE_DIR}/external/pmp-library/external/glew/include)

target_link_libraries(${executable_name} PRIVATE pmp_vis)
# target_link_libraries(${executable_name} PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

if (EMSCRIPTEN)
    set_target_properties(${executable_name} PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/external/pmp-library/src/apps/data/shell.html --preload-file ${PROJECT_SOURCE_DIR}/external/pmp-library/external/pmp-data/off/bunny.off@input.off")
endif()