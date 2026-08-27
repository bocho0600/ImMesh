/*** ROS 2 wrapper around ImMesh's incremental mesher, without its LiDAR odometry.
 *
 * ImMesh ships as odometry plus meshing. This node links only the meshing half (see
 * IMMESH_STANDALONE) and takes the pose from somewhere else, which is what you want when a
 * LIO is already running: the rover localises, and the mesh is built wherever there is CPU
 * to spare -- in practice the base station, off the end of the compressed point cloud link.
 *
 * Input is an accumulated map cloud, already in the world frame. That is not the shape the
 * mesher was written for -- it wants one scan at a time with the viewpoint it came from --
 * so two things bridge the gap:
 *
 *   1. Only points that have not been meshed before are forwarded. The map arrives whole
 *      every time, so without this every update would re-queue the entire thing and the
 *      cost would grow with the map rather than with what changed.
 *   2. The viewpoint comes from the odometry topic, not from the cloud. An accumulated map
 *      has no single vantage point, and the mesher uses one to decide which way a triangle
 *      faces. The robot's current pose is the closest honest answer; with no odometry yet,
 *      the centroid of the new points stands in.
 ***/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_set>
#include <visualization_msgs/msg/marker.hpp>

#include "meshing/immesh_meshing_api.hpp"

// --- the mesher's state, which the library expects its host to define ------------------
Global_map                   g_map_rgb_pts_mesh( 0 );
Triangle_manager             g_triangles_manager;
LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;
double                       threshold_scale = 1.0;
double                       minimum_pts = 0.1;
double                       g_meshing_voxel_size = 0.4;
int                          appending_pts_frame = ( int ) 5e3;
int                          g_current_frame = -1;
int                          g_enable_mesh_rec = 1;
bool                         g_flag_pause = false;
double                       g_LiDAR_frame_start_time = 0;
// g_fp_cost_time and g_fp_lio_state are deliberately absent: mesh_rec_geometry.cpp defines
// them itself, unlike everything above, which it only declares extern.
// Referenced from the RGB-projection paths this node never exercises, but they still have
// to resolve at link time. The values are ImMesh's own defaults.
Common_tools::Timer          g_cost_time_logger;
double                       g_maximum_pe_error = 40;
double                       g_initial_camera_exp_tim = 1.0;

namespace {

// Quantises a point to a grid so "have I meshed this already" is a hash lookup. The map
// arrives whole on every message; this is what turns that back into an incremental feed.
struct VoxelKey
{
    int64_t x, y, z;
    bool    operator==( const VoxelKey &o ) const { return x == o.x && y == o.y && z == o.z; }
};
struct VoxelKeyHash
{
    size_t operator()( const VoxelKey &k ) const
    {
        // Three odd primes: the usual spatial hash, cheap and well spread for grid indices.
        return ( size_t ) ( ( k.x * 73856093 ) ^ ( k.y * 19349663 ) ^ ( k.z * 83492791 ) );
    }
};

}  // namespace

class ImMeshNode : public rclcpp::Node
{
  public:
    ImMeshNode() : Node( "immesh" )
    {
        cloud_topic_ = declare_parameter< std::string >( "cloud_topic", "/Laser_map/downsampled/decompressed" );
        odom_topic_ = declare_parameter< std::string >( "odom_topic", "/Odometry" );
        mesh_topic_ = declare_parameter< std::string >( "mesh_topic", "mesh" );
        map_frame_ = declare_parameter< std::string >( "map_frame", "odom" );
        ply_path_ = declare_parameter< std::string >( "ply_path", "" );
        // The mesher's own voxel size, and the spacing it keeps points at inside one.
        g_meshing_voxel_size = declare_parameter< double >( "meshing_voxel_size", 0.4 );
        minimum_pts = declare_parameter< double >( "minimum_point_distance", 0.1 );
        // The grid the "seen this already" test runs on. Coarser than the point spacing on
        // purpose: the map is rebuilt from a voxel-image every publish, so identical
        // surfaces come back at slightly different coordinates each time and an exact test
        // would call every point new.
        new_point_resolution_ = declare_parameter< double >( "new_point_resolution", 0.1 );
        // Below this, a message is not worth waking the mesher for.
        min_new_points_ = declare_parameter< int >( "min_new_points", 100 );
        max_threads_ = declare_parameter< int >( "max_threads", 4 );
        const double publish_interval = declare_parameter< double >( "mesh_publish_interval_s", 5.0 );

        start_mesh_threads( max_threads_ );

        cloud_sub_ = create_subscription< sensor_msgs::msg::PointCloud2 >(
            cloud_topic_, rclcpp::QoS( rclcpp::KeepLast( 2 ) ),
            std::bind( &ImMeshNode::onCloud, this, std::placeholders::_1 ) );
        odom_sub_ = create_subscription< nav_msgs::msg::Odometry >(
            odom_topic_, rclcpp::QoS( rclcpp::KeepLast( 10 ) ),
            std::bind( &ImMeshNode::onOdom, this, std::placeholders::_1 ) );
        // The mesh is large and slow-moving, and a viewer that connects later should not
        // have to wait for the next one, so it is latched.
        mesh_pub_ = create_publisher< visualization_msgs::msg::Marker >(
            mesh_topic_, rclcpp::QoS( rclcpp::KeepLast( 1 ) ).transient_local() );
        mesh_timer_ = create_wall_timer( std::chrono::duration< double >( publish_interval ),
                                         std::bind( &ImMeshNode::publishMesh, this ) );
        save_srv_ = create_service< std_srvs::srv::Trigger >(
            "mesh_save", std::bind( &ImMeshNode::onSave, this, std::placeholders::_1, std::placeholders::_2 ) );

        RCLCPP_INFO( get_logger(), "Meshing '%s' into '%s' (frame %s), viewpoint from '%s'.",
                     cloud_topic_.c_str(), mesh_topic_.c_str(), map_frame_.c_str(), odom_topic_.c_str() );
    }

  private:
    void onOdom( const nav_msgs::msg::Odometry::SharedPtr msg )
    {
        std::lock_guard< std::mutex > lock( pose_mutex_ );
        pose_q_ = Eigen::Quaterniond( msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z );
        pose_t_ = Eigen::Vector3d( msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z );
        have_pose_ = true;
    }

    void onCloud( const sensor_msgs::msg::PointCloud2::SharedPtr msg )
    {
        // Converted through PointXYZ on purpose: a map cloud carries geometry and nothing
        // else, and asking PCL for PointXYZI when there is no intensity field logs a warning
        // per message and leaves the channel uninitialised, which the mesher then reads.
        pcl::PointCloud< pcl::PointXYZ > xyz;
        pcl::fromROSMsg( *msg, xyz );
        pcl::PointCloud< pcl::PointXYZI >::Ptr cloud( new pcl::PointCloud< pcl::PointXYZI > );
        cloud->points.resize( xyz.points.size() );
        for ( size_t i = 0; i < xyz.points.size(); ++i )
        {
            cloud->points[ i ].x = xyz.points[ i ].x;
            cloud->points[ i ].y = xyz.points[ i ].y;
            cloud->points[ i ].z = xyz.points[ i ].z;
            cloud->points[ i ].intensity = 0.f;
        }
        if ( cloud->points.empty() )
        {
            return;
        }

        pcl::PointCloud< pcl::PointXYZI >::Ptr fresh( new pcl::PointCloud< pcl::PointXYZI > );
        fresh->points.reserve( cloud->points.size() );
        const double inv_res = 1.0 / new_point_resolution_;
        for ( const auto &p : cloud->points )
        {
            if ( !std::isfinite( p.x ) || !std::isfinite( p.y ) || !std::isfinite( p.z ) )
            {
                continue;
            }
            const VoxelKey key{ ( int64_t ) std::floor( p.x * inv_res ), ( int64_t ) std::floor( p.y * inv_res ),
                                ( int64_t ) std::floor( p.z * inv_res ) };
            if ( seen_.insert( key ).second )
            {
                fresh->points.push_back( p );
            }
        }
        fresh->width = fresh->points.size();
        fresh->height = 1;

        if ( ( int ) fresh->points.size() < min_new_points_ )
        {
            RCLCPP_DEBUG( get_logger(), "%zu new points, below min_new_points; skipping.", fresh->points.size() );
            return;
        }

        Eigen::Quaterniond q;
        Eigen::Vector3d    t;
        {
            std::lock_guard< std::mutex > lock( pose_mutex_ );
            q = pose_q_;
            t = pose_t_;
            if ( !have_pose_ )
            {
                // No odometry yet. The centroid is at least inside the scene rather than at
                // an arbitrary origin, which keeps the triangle orientation broadly sane.
                t = Eigen::Vector3d::Zero();
                for ( const auto &p : fresh->points )
                {
                    t += Eigen::Vector3d( p.x, p.y, p.z );
                }
                t /= ( double ) fresh->points.size();
                RCLCPP_WARN_ONCE( get_logger(),
                                  "No odometry on '%s' yet; using the cloud centroid as the viewpoint. "
                                  "Triangle orientation will improve once it arrives.",
                                  odom_topic_.c_str() );
            }
        }

        RCLCPP_INFO( get_logger(), "Meshing %zu new points of %zu (%zu seen so far).", fresh->points.size(),
                     cloud->points.size(), seen_.size() );
        append_frame_for_meshing( fresh, q, t, minimum_pts );
    }

    void publishMesh()
    {
        /*** Both locks, for the whole read.
         *
         * get_all_triangle_list walks the per-region hash map, and the meshing thread
         * inserts into it; its own mutex argument only guards each inner copy, not the
         * walk, so a rehash mid-iteration is a crash. Reading vertex positions then indexes
         * m_rgb_pts_vec, which the same thread appends to. Held together rather than in
         * turn so the triangles and the points they index cannot disagree.
         ***/
        std::vector< Triangle_set > triangle_sets;
        {
            std::lock_guard< std::mutex > mesh_lock( g_mutex_reconstruct_mesh );
            g_triangles_manager.get_all_triangle_list( triangle_sets, nullptr, 0 );
        }
        std::lock_guard< std::mutex > map_lock( g_mutex_append_map );
        size_t n_triangles = 0;
        for ( const auto &set : triangle_sets )
        {
            n_triangles += set.size();
        }
        if ( n_triangles == 0 )
        {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now();
        marker.header.frame_id = map_frame_;
        marker.ns = "immesh";
        marker.id = 0;
        // TRIANGLE_LIST rather than a mesh resource: the geometry is generated here and the
        // viewer is on this machine, so there is nothing to be gained by routing it through
        // a file for RViz to parse back.
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.color.r = 0.75f;
        marker.color.g = 0.75f;
        marker.color.b = 0.78f;
        marker.color.a = 1.0f;
        marker.points.reserve( n_triangles * 3 );

        const size_t n_pts = g_map_rgb_pts_mesh.m_rgb_pts_vec.size();
        size_t       skipped = 0;
        for ( const auto &set : triangle_sets )
        {
        for ( const auto &tri : set )
        {
            // A triangle indexes into the point store, and the mesher may have dropped a
            // point since the set was copied, so the bound is checked rather than assumed.
            if ( tri == nullptr || ( size_t ) tri->m_tri_pts_id[ 0 ] >= n_pts ||
                 ( size_t ) tri->m_tri_pts_id[ 1 ] >= n_pts || ( size_t ) tri->m_tri_pts_id[ 2 ] >= n_pts )
            {
                ++skipped;
                continue;
            }
            for ( int i = 0; i < 3; ++i )
            {
                const vec_3                 v = g_map_rgb_pts_mesh.m_rgb_pts_vec[ tri->m_tri_pts_id[ i ] ]->get_pos();
                geometry_msgs::msg::Point   p;
                p.x = v( 0 );
                p.y = v( 1 );
                p.z = v( 2 );
                marker.points.push_back( p );
            }
        }
        }
        if ( marker.points.empty() )
        {
            return;
        }
        mesh_pub_->publish( marker );
        RCLCPP_INFO( get_logger(), "Published %zu triangles (%zu skipped, %.1f MB).", marker.points.size() / 3,
                     skipped, marker.points.size() * sizeof( geometry_msgs::msg::Point ) / 1e6 );
    }

    void onSave( const std::shared_ptr< std_srvs::srv::Trigger::Request >,
                 std::shared_ptr< std_srvs::srv::Trigger::Response > response )
    {
        if ( ply_path_.empty() )
        {
            response->success = false;
            response->message = "No output path: set the ply_path parameter.";
            return;
        }
        save_to_ply_file( ply_path_, 0.1, 20 );
        response->success = true;
        response->message = "Wrote the mesh to '" + ply_path_ + "'.";
        RCLCPP_INFO( get_logger(), "%s", response->message.c_str() );
    }

    std::string cloud_topic_, odom_topic_, mesh_topic_, map_frame_, ply_path_;
    double      new_point_resolution_ = 0.1;
    int         min_new_points_ = 100;
    int         max_threads_ = 4;

    std::unordered_set< VoxelKey, VoxelKeyHash > seen_;
    std::mutex                                   pose_mutex_;
    Eigen::Quaterniond                           pose_q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d                              pose_t_ = Eigen::Vector3d::Zero();
    bool                                         have_pose_ = false;

    rclcpp::Subscription< sensor_msgs::msg::PointCloud2 >::SharedPtr cloud_sub_;
    rclcpp::Subscription< nav_msgs::msg::Odometry >::SharedPtr       odom_sub_;
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr  mesh_pub_;
    rclcpp::TimerBase::SharedPtr                                     mesh_timer_;
    rclcpp::Service< std_srvs::srv::Trigger >::SharedPtr             save_srv_;
};

int main( int argc, char **argv )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared< ImMeshNode >() );
    rclcpp::shutdown();
    return 0;
}
