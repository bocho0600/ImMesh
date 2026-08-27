cmake_minimum_required(VERSION 3.16)
project(immesh_ros2)

# ---------------------------------------------------------------------------
# ImMesh, minus its odometry.
#
# The upstream build is one catkin target containing the LiDAR-inertial odometry, the
# mesher and an OpenGL/ImGui viewer. This builds the mesher alone: five translation units
# that between them have no ROS calls at all, compiled with IMMESH_STANDALONE so the one
# file that reaches for the odometry header takes the lighter includes instead.
#
# Two things the 2023 sources need from a modern toolchain:
#
#   - CGAL 5. The intersection API returns a boost::variant there and a std::variant from
#     CGAL 6, and tools_graphics.hpp reads it with boost::get. Pinning the version the code
#     was written against is a great deal safer than rewriting the geometry that decides
#     triangle topology.
#   - Force-included standard headers. GCC 15 no longer pulls <memory>, <mutex>, <sstream>
#     or <cstdint> in transitively, and these sources relied on that. SHELL: keeps CMake
#     from collapsing the repeated -include flags into one.
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(IMMESH_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Several of the packages below hand back /usr/include among their include directories.
# That is harmless on a distro whose headers match the compiler, and fatal here, where the
# toolchain ships its own glibc: mixing the two makes pthread_cond_t resolve to the wrong
# type and <mutex> fails to compile, with an error that points at libstdc++ and says
# nothing about the real cause.
#
# Listing it as an implicit directory is the CMake-native way to say "the compiler already
# knows about this one" -- CMake then strips it from every target, whichever dependency
# introduced it, rather than us chasing each one.
list(APPEND CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES /usr/include)
list(APPEND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES /usr/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
# `io` is needed for save_to_ply_file (pcl::io::savePLYFileBinary) and brings VTK with it.
# That is fine inside a Nix build, where VTK's own dependencies come along in the closure;
# it does not link in an ad-hoc dev shell, where libva/libnl/libtiff are not on the path.
find_package(PCL REQUIRED COMPONENTS common io kdtree filters)
# Only the modules image_frame.cpp and pointcloud_rgbd.cpp actually call. Linking the full
# ${OpenCV_LIBS} drags in videoio and the hardware-accel backends, and with them libva,
# libnl and libtiff, none of which this node has any use for and all of which then have to
# resolve against the host system.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(CGAL REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

add_executable(immesh_node
  src/immesh_node.cpp
  ${IMMESH_ROOT}/src/ImMesh_mesh_reconstruction.cpp
  ${IMMESH_ROOT}/src/meshing/mesh_rec_geometry.cpp
  ${IMMESH_ROOT}/src/meshing/r3live/triangle.cpp
  ${IMMESH_ROOT}/src/meshing/r3live/pointcloud_rgbd.cpp
  ${IMMESH_ROOT}/src/meshing/r3live/image_frame.cpp
  ${IMMESH_ROOT}/include/ikd-Tree/ikd_Tree.cpp
)

target_compile_definitions(immesh_node PRIVATE IMMESH_STANDALONE)
target_compile_options(immesh_node PRIVATE
  -w                      # upstream is warning-noisy and none of it is actionable here
  "SHELL:-include memory" "SHELL:-include mutex" "SHELL:-include sstream" "SHELL:-include cstdint")

# ${IMMESH_ROOT}/include is deliberately NOT on the path: it holds a types.h that shadows a
# system header once the directory is searched. Only the ikd-Tree subdirectory is needed.
target_include_directories(immesh_node PRIVATE
  ${IMMESH_ROOT}/src
  ${IMMESH_ROOT}/src/meshing
  ${IMMESH_ROOT}/src/meshing/r3live
  ${IMMESH_ROOT}/src/tools
  ${IMMESH_ROOT}/include/ikd-Tree
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(immesh_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs std_srvs pcl_conversions)
target_link_libraries(immesh_node
  ${PCL_LIBRARIES} opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui CGAL::CGAL Threads::Threads
  $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

install(TARGETS immesh_node DESTINATION lib/${PROJECT_NAME})

ament_package()
