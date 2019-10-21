# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)
SET(CMAKE_BUILD_TYPE Debug)#modified
# directories
set (Viso2_INCLUDE_DIRS src)

# include directory
include_directories("${Viso2_INCLUDE_DIRS}")

# use sse3 instruction set and an optimized build; note that at least on modern versions of GCC
# (5.4+) the -O3 flag is actually very important and can lead to 3-4x slowdown if not set!
set(CMAKE_CXX_FLAGS "-march=native -fPIC -O3 -std=c++11")

#Added
find_package(Eigen3 3.3 REQUIRED)

# sources
set(LIBVISO2_SRC_FILES
    src/filter.cpp
    src/matcher.cpp
    src/matrix.cpp
    src/reconstruction.cpp
    src/triangle.cpp
    src/viso.cpp
    src/viso_mono.cpp
    src/viso_stereo.cpp
)
include_directories(${EIGEN3_INCLUDE_DIR}) #Added

add_library(viso2 ${LIBVISO2_SRC_FILES})
set(Viso2_LIBS viso2 CACHE INTERNAL "")

# demo program (unused; requires png++: sudo apt install libpng++-dev)
add_executable(viso2-demo src/demo.cpp)
target_link_libraries(viso2-demo viso2 png)
