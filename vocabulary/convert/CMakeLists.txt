cmake_minimum_required(VERSION 3.0.2)
project(convert_txt2bin)

find_package(OpenCV REQUIRED)
find_package(DBoW2)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
)

add_executable(convert_txt2bin convert_txt2bin.cc)

target_link_libraries(convert_txt2bin
  ${OpenCV_LIBRARIES}
  ${DBoW2_LIBRARIES}
)
