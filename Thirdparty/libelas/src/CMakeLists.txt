# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# include directory
include_directories(${LIBELAS_SRC_DIR}
			${OPENCV_INCLUDE_DIR})

# build demo program
add_library(libelas
descriptor.cpp
elas.cpp
filter.cpp
matrix.cpp
triangle.cpp
)

target_link_libraries(libelas
${OpenCV_LIBS}
)

