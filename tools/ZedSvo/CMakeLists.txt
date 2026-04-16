cmake_minimum_required(VERSION 3.10)
project(ZedSvo)

# RTAB-Map headers – cmake config was stripped from image, pass path directly.
# Default matches the install layout from Dockerfile (rtabmap 0.21).
IF(NOT RTABMAP_INCLUDE_DIRS)
    SET(RTABMAP_INCLUDE_DIRS /usr/local/include/rtabmap-0.21)
ENDIF()
INCLUDE_DIRECTORIES(${RTABMAP_INCLUDE_DIRS})

# PCL + Eigen + OpenCV (required transitively by rtabmap headers)
INCLUDE_DIRECTORIES(/usr/include/pcl-1.12 /usr/include/eigen3 /usr/include/opencv4)

IF(NOT ZED_INCLUDE_DIRS)
    SET(ZED_INCLUDE_DIRS /usr/local/zed/include)
ENDIF()
INCLUDE_DIRECTORIES(${ZED_INCLUDE_DIRS})

IF(NOT ZED_LIBRARIES)
    SET(ZED_LIBRARIES /usr/local/zed/lib/libsl_zed.so)
ENDIF()

# sl/Camera.hpp includes CUDA headers (cuda.h, cuda_runtime.h)
FIND_PACKAGE(CUDAToolkit QUIET)
IF(CUDAToolkit_FOUND)
    INCLUDE_DIRECTORIES(${CUDAToolkit_INCLUDE_DIRS})
ELSE()
    FIND_PACKAGE(CUDA QUIET)
    IF(CUDA_FOUND)
        INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
    ENDIF()
ENDIF()

ADD_EXECUTABLE(zed_svo main.cpp)

# L4T BSP libraries (libcuda.so.1, libnvbufsurface, libnvargus etc.) live on
# the Jetson host and are injected at runtime by the NVIDIA container runtime.
# Allow unresolved shared-library symbols at link time.
TARGET_LINK_OPTIONS(zed_svo PRIVATE -Wl,--allow-shlib-undefined)

TARGET_LINK_LIBRARIES(zed_svo
    rtabmap_core
    rtabmap_utilite
    ${ZED_LIBRARIES}
    opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d opencv_features2d
)

SET_TARGET_PROPERTIES(zed_svo
    PROPERTIES OUTPUT_NAME zed_svo)

INSTALL(TARGETS zed_svo
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
    BUNDLE  DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
