cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(edge_uvc src/edge_uvc.cpp)
rosbuild_add_executable(edge_drone src/edge_drone.cpp)
rosbuild_add_executable(vert_drone src/vertical_drone.cpp)
rosbuild_add_executable(dense_flow src/dense_flow.cpp)
rosbuild_add_executable(landmarks src/Landmarks.cpp)
rosbuild_add_executable(risc1_landmarks src/risc1_Landmarks.cpp)
rosbuild_add_executable(risc2_landmarks src/risc2_Landmarks.cpp)
rosbuild_add_executable(risc3_landmarks src/risc3_Landmarks.cpp)
rosbuild_add_executable(risc4_landmarks src/risc4_Landmarks.cpp)
rosbuild_add_executable(risc_landmarks src/risc_landmarks1-4.cpp)
rosbuild_add_executable(mask_tuner src/mask_tuner.cpp)
rosbuild_add_executable(quad_tracker src/Quad_tracker.cpp)
rosbuild_add_executable(quad_tracker_test src/quad_track_modi.cpp)
rosbuild_add_executable(rviz_test src/risc_Rviz.cpp)
rosbuild_add_executable(pong src/pong.cpp)
rosbuild_add_executable(draw_cortex src/draw_cortex.cpp)
rosbuild_add_executable(draw_risc_maap src/draw_risc_maap.cpp)
rosbuild_add_executable(draw_trajectories src/draw_trajectories.cpp)
rosbuild_add_executable(draw_angles src/draw_angles.cpp)
rosbuild_add_executable(draw_waypoints src/draw_waypoints.cpp)
rosbuild_add_executable(quad_track_cortex src/quad_track_withcortex.cpp)
rosbuild_add_executable(detect_objects src/detect_objects.cpp)
#target_link_libraries(example ${PROJECT_NAME})
