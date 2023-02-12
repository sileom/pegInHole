cmake_minimum_required(VERSION 3.0)

project(libfranka-unibas_insert_bh CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)



#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wall")


find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

find_package(OpenCV 4.2.0 REQUIRED )
find_package(realsense2 REQUIRED)
#find_package(VISP REQUIRED)
find_package(PCL 1.8 REQUIRED )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ------ MATLAB ------
#SET(MATLAB_ROOT_DIR "/home/labarea-franka/MATLAB/R2018a")
#SET(MATLAB_INCLUDE_DIRS "/home/labarea-franka/MATLAB/R2018a/extern/include")
#SET(MATLAB_MEX_LIBRARY "/home/labarea-franka/MATLAB/R2018a/extern/lib/glnxa64")
#find_package(Matlab REQUIRED)
#include_directories(${MATLAB_INCLUDE_DIRS})


add_library(common_i STATIC
  common_i.cpp
  dynModel_i.cpp
)

#add_library(movep2p STATIC
#  movep2p.cpp
#)

#include_directories(${VISP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(/home/labarea-franka/libfranka/include/imbs/)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(common_i PUBLIC Franka::Franka Eigen3::Eigen3)
#target_link_libraries(movep2p PUBLIC Franka::Franka Eigen3::Eigen3)

set(UNIBAS_INSERT_BH
  scan3d
  big_head_insertion
  big_head_insertion_yolo
  #hand_guidance
)

set(OpenCV_LIBS /usr/local/lib/libopencv_dnn.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_core.so.4.2 /usr/local/lib/libopencv_imgproc.so.4.2 opencv_core opencv_highgui)

foreach(unibas_insert_bh ${UNIBAS_INSERT_BH})
  add_executable(${unibas_insert_bh} ${unibas_insert_bh}.cpp utils_cam.hpp utils.hpp utils_detector_yolo.hpp)
  target_link_libraries(${unibas_insert_bh} Franka::Franka common_i Eigen3::Eigen3)
  target_link_libraries(${unibas_insert_bh} ${OpenCV_LIBS} )
  target_link_libraries(${unibas_insert_bh} ${realsense2_LIBRARY})
  #target_link_libraries(${unibas_insert_bh} ${VISP_LIBRARIES})
  target_link_libraries(${unibas_insert_bh} ${PCL_LIBRARIES} )
  target_link_libraries(${unibas_insert_bh} Threads::Threads)
  #target_link_libraries(${unibas_insert_bh} ${MATLAB_ROOT_DIR}/extern/bin/glnxa64/libMatlabEngine.so)
  #target_link_libraries(${unibas_insert_bh} ${MATLAB_ROOT_DIR}/extern/bin/glnxa64/libMatlabDataArray.so)

  #include_directories("/usr/local/include/opencv4")
  #include_directories("/installation/OpenCV-/include/opencv4")
endforeach()



##target_link_libraries(joint_impedance_control Threads::Threads)
##target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${UNIBAS_INSERT_BH}

  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

