cmake_minimum_required(VERSION 3.16)
project(ControlNew4 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui aruco calib3d)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

set(RTIMULIB_INCLUDE_DIR /home/deathstar/libraries/RTIMULib2/RTIMULib)
set(RTIMULIB_LIBRARY_DIR /home/deathstar/libraries/RTIMULib2/RTIMULib/build)
set(ROBOTLIB_INCLUDE_DIR /home/deathstar/deathstar/lib)
set(ROBOTLIB_LIBRARY_DIR /home/deathstar/deathstar/lib)

add_library(sensor_ctrl STATIC
    constellation_sensor.cpp
    controller.cpp
)

target_include_directories(sensor_ctrl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
    ${RTIMULIB_INCLUDE_DIR}
    ${ROBOTLIB_INCLUDE_DIR}
)

target_link_directories(sensor_ctrl PUBLIC
    ${RTIMULIB_LIBRARY_DIR}
    ${ROBOTLIB_LIBRARY_DIR}
)

target_link_libraries(sensor_ctrl PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${LIBCAMERA_LIBRARIES}
    RTIMULib
    robot
    gpiod
    pthread
    m
)

target_compile_options(sensor_ctrl PRIVATE ${LIBCAMERA_CFLAGS_OTHER})

add_executable(main main.cpp)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RTIMULIB_INCLUDE_DIR}
    ${ROBOTLIB_INCLUDE_DIR}
)

target_link_directories(main PRIVATE
    ${RTIMULIB_LIBRARY_DIR}
    ${ROBOTLIB_LIBRARY_DIR}
)

target_link_libraries(main PRIVATE sensor_ctrl)

set_target_properties(main PROPERTIES
    BUILD_RPATH "${RTIMULIB_LIBRARY_DIR};${ROBOTLIB_LIBRARY_DIR}"
)

target_compile_options(main PRIVATE -Wall -O2)
