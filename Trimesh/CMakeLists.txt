cmake_minimum_required (VERSION 2.6)
project (Trimesh)

include_directories(/home/joeshepley/EigenTest/source_dir/)
include_directories(/home/joeshepley/ode-0.13.1/include/ode)
include_directories(/home/joeshepley/ode-0.13.1/include/drawstuff)



SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -w")
SET(GCC_COVERAGE_LINK_FLAGS    "-lode -ldrawstuff -lGL -lGLU -lglut -lX11 -lpthread")

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

add_executable(Trimesh sceneValidator.cpp list.cpp objLoader.cpp obj_parser.cpp string_extra.cpp )
target_link_libraries(Trimesh ode drawstuff GL GLU glut X11 pthread)
