cmake_minimum_required(VERSION 3.17.0)
project(Stag)

find_package(OpenCV REQUIRED)

file(GLOB SRC_FILE1 "./src/*.c*")
file(GLOB SRC_FILE2 "./src/ED/*.c*")

include_directories(
    common/include
    ${OpenCV_INCLUDE_DIRS}
)
add_library(
    ${PROJECT_NAME} SHARED
    ${SRC_FILE1}
    ${SRC_FILE2}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)
