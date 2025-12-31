cmake_minimum_required(VERSION 3.22)

project(vision_vru VERSION 1.0 LANGUAGES CXX C)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Suppress OpenCV deprecation warnings about enum arithmetic
add_compile_options(-Wno-deprecated-enum-enum-conversion)

find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Add link directory for OpenCV dependencies like TBB
link_directories(${OpenCV_INSTALL_PATH}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

#
# Detector
#
add_executable(${PROJECT_NAME} src/vision_vru.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY configs/ DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/${PROJECT_NAME}")

#
# Calibrator
#
add_executable(vision_calibrator src/calibrator.cpp)
target_include_directories(vision_calibrator PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_calibrator PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

install(TARGETS vision_calibrator RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

#
# Webcam test
#
add_executable(simple_webcam_test src/simple_webcam_test.cpp)
target_include_directories(simple_webcam_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(simple_webcam_test PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

install(TARGETS simple_webcam_test RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
