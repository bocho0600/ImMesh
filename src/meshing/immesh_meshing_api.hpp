#ifndef IMMESH_MESHING_API_HPP_
#define IMMESH_MESHING_API_HPP_

/*** The meshing core's public surface, for a consumer that brings its own odometry.
 *
 * ImMesh is a LiDAR-inertial odometry system with an incremental mesher bolted to it, and
 * the two are only loosely joined: the mesher is handed a world-frame cloud and the pose it
 * was observed from, and knows nothing about how that pose was arrived at. Everything here
 * is that joint, declared so the meshing half can be linked on its own (see
 * IMMESH_STANDALONE in ImMesh_mesh_reconstruction.cpp).
 *
 * The globals below are the mesher's state. They are defined by whoever links it -- ImMesh's
 * own node does it in ImMesh_node.cpp -- because their initial values are configuration.
 ***/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

#include "meshing/mesh_rec_geometry.hpp"

// --- mesher state, defined by the consumer -------------------------------------------
extern Global_map                   g_map_rgb_pts_mesh;     // the point store triangles index into
extern Triangle_manager             g_triangles_manager;    // the triangles themselves
extern LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;        // per-frame points + pose
extern double                       g_meshing_voxel_size;   // voxel side the mesher works in
extern double                       minimum_pts;            // minimum spacing kept in the map
extern int                          appending_pts_frame;    // points appended per frame
extern int                          g_current_frame;
extern int                          g_enable_mesh_rec;
extern bool                         g_flag_pause;
extern int                          g_frame_idx;

// --- entry points ---------------------------------------------------------------------

// Starts the background meshing thread and sizes the per-frame store. Idempotent; the
// append functions below call it, so it only needs calling directly to set the thread cap.
void start_mesh_threads( int maximum_threads );

// Queue one world-frame cloud for meshing, observed from (pose_q, pose_t). The pose is not
// used to move the points -- they are already in the world frame -- but to orient the
// triangles: a surface is drawn facing the place it was seen from.
//
// `minimum_pts_distance` is the voxel size the cloud is downsampled to before it is
// queued, which sets the finest detail the mesh can carry.
void append_frame_for_meshing( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts,
                               const Eigen::Quaterniond& pose_q, const Eigen::Vector3d& pose_t,
                               double minimum_pts_distance );

// The same, with the viewpoint at the origin. ImMesh's original offline entry point.
void reconstruct_mesh_from_pointcloud( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts,
                                       double minimum_pts_distance );

// Smooths every point and writes the mesh out as a PLY.
void save_to_ply_file( std::string ply_file, double smooth_factor, double knn );

#endif  // IMMESH_MESHING_API_HPP_
