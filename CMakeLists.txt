cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# SET(ROS_BUILD_TYPE Release)
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm") 
add_definitions("-DENABLE_SSE")
MESSAGE("SSE flags: " ${SSE_FLAGS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--no-as-needed -pthread -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
ELSE()
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wl,--no-as-needed -pthread -Wall  -O3 -march=native ")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -Wall  -O3 -march=native")
ENDIF()

MESSAGE(">>>>>  Compiler flags are: " ${CMAKE_CXX_FLAGS})

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# customize opencv dir
if ($ENV{ROS_DISTRO} STREQUAL "indigo")
    set( OpenCV_DIR /opt/opencv2/share/OpenCV )
    find_package(OpenCV 2.4.13 REQUIRED core ocl)
elseif ($ENV{ROS_DISTRO} STREQUAL "kinetic")
    # set( OpenCV_DIR /opt/opencv-3.4.1/share/OpenCV )
    set( OpenCV_DIR /opt/opencv3/share/OpenCV )
    find_package(OpenCV 3.4.1 REQUIRED)
endif()
MESSAGE("OpenCV include dir: " ${OpenCV_INCLUDE_DIRS})

# customize eigen dir
set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
MESSAGE("EIGEN include dir: " ${EIGEN3_INCLUDE_DIR})

find_package(Cholmod REQUIRED)
MESSAGE("Cholmod libs: " ${CHOLMOD_LIBRARIES})

# set(OpenBLAS_DIR /opt/OpenBLAS/lib/cmake/openblas)
# find_package(OpenBLAS REQUIRED)  # all capitalize. See cmake-2.8/Modules/FindBLAS.cmake
set(OpenBLAS_LIBRARIES /opt/OpenBLAS/lib/libopenblas.so)
MESSAGE("OpenBLAS libs:" ${OpenBLAS_LIBRARIES})

#
# set(Armadillo_DIR /opt/armadillo/share/Armadillo/CMake)
# find_package(Armadillo REQUIRED)
set(ARMADILLO_INCLUDE_DIR /opt/armadillo/include)
set(ARMADILLO_LIBRARIES /opt/armadillo/lib/libarmadillo.so)
MESSAGE("ARMADILLO libs: " ${ARMADILLO_LIBRARIES})

#set(GTSAM_INCLUDE_DIR /opt/gtsam/include)
#set(GTSAM_LIBRARIES /opt/gtsam/lib/libgtsam.so)
#MESSAGE("GTSAM libs: " ${GTSAM_LIBRARIES})

set(GTest_INCLUDE_DIR /usr/src/gtest/src)
set(GTest_LIBRARIES /usr/src/gtest/libgtest.a)
# set(GTest_LIBRARIES /usr/src/gtest/build/libgtest.a)
MESSAGE("gtest libs: " ${GTest_LIBRARIES})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
    ${ARMADILLO_INCLUDE_DIR}
    # ${GTSAM_INCLUDE_DIR}
    ${GTest_INCLUDE_DIR}
    ./Thirdparty/sse2neon/
    ./include/
    )

set(SRCS
    src/Observability.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FramePublisher.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapPublisher.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    )

set(LIBS
    ${CHOLMOD_LIBRARIES}
#    ${LAPACK_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    #    ${GTSAM_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
    )

# main execution file
rosbuild_add_executable(${PROJECT_NAME}
    ${SRCS}
    src/main.cc
    )
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)
target_link_libraries(${PROJECT_NAME} ${LIBS})


# tools for binary ORB Voc
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
./tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${LIBS})


# test cases
#FIND_PACKAGE( Boost REQUIRED system thread )
#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#MESSAGE("Boost libs: " ${Boost_LIBRARIES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_executable(testObs
    ${SRCS}
    #   ./test/test_Initial.cpp
    ./test/test_Kine_1.cpp
    ./test/test_Kine_2.cpp
    ./test/test_Jacobian.cpp
    ./test/test_Greedy.cpp
    ./test/test_RANSAC.cpp
    #    ./test/test_LazierBound.cpp
    # ./test/test_BoxLOG.cpp
    # ./test/test_GoodMap.cpp
    ./test/main.cpp
    )
rosbuild_link_boost(testObs thread)
target_link_libraries(testObs
    ${LIBS}
    ${GTest_LIBRARIES}
    GL
    glut
    GLU
    )
