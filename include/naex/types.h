
#ifndef NAEX_TYPES_H
#define NAEX_TYPES_H

#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <memory>
//#include <naex/buffer.h>

//#define K_NEIGHBORS 32

namespace naex
{
// Basic floating-point and index types
typedef float Elem;
typedef Elem Value;
//typedef uint32_t Index;
typedef int Index;

// Arrays and matrices
typedef Eigen::Matrix<Value, 4, 1, Eigen::DontAlign> Vec4;
typedef Eigen::Map<Vec4> Vec4Map;
typedef Eigen::Map<const Vec4> ConstVec4Map;

typedef Eigen::Matrix<Value, 3, 1, Eigen::DontAlign> Vec3;
typedef Eigen::Map<Vec3> Vec3Map;
typedef Eigen::Map<const Vec3> ConstVec3Map;

typedef Eigen::Matrix<Value, 2, 1, Eigen::DontAlign> Vec2;
typedef Eigen::Map<Vec2> Vec2Map;
typedef Eigen::Map<const Vec2> ConstVec2Map;

typedef Eigen::Matrix<Value, 3, 3, Eigen::DontAlign> Mat3;
typedef Eigen::Matrix<Value, 4, 4, Eigen::DontAlign> Mat4;
typedef Eigen::Matrix<Value, 3, Eigen::Dynamic, Eigen::DontAlign> Mat3X;
typedef Eigen::Map<Mat3X> MatXMap;
typedef Eigen::Map<const Mat3X> ConstMatXMap;

typedef Eigen::Quaternion<Value, Eigen::DontAlign> Quat;
typedef Eigen::Map<Quat> QuatMap;
typedef Eigen::Map<const Quat> ConstQuatMap;

typedef std::vector<Index> Indices;

// Vertex and edge indices
// TODO: Rename both to Index (to be used elsewhere too).
typedef Index Vertex;
typedef Index Edge;
// Edge cost or length
typedef Elem Cost;

typedef flann::Matrix<Value> FlannMat;
typedef FlannMat FMat;
typedef flann::Index<flann::L2_3D<Value>> FlannIndex;
typedef std::shared_ptr<FlannIndex> FlannIndexPtr;
typedef std::shared_ptr<const FlannIndex> ConstFlannIndexPtr;

//    namespace IndexState
//    {
//enum IndexState
//{
//    TO_UPDATE = 0,
//    UP_TO_DATE = 1
//};
//    }

//    namespace Label
//    {
//enum Label
//{
//    UNKNOWN = 0,
//    EMPTY = 1,
//    TRAVERSABLE = 2,
//    EDGE = 3,
//    ACTOR = 4,
//    OBSTACLE = 5
//};

enum Flags
{
    // Point was updated including its neighborhood. Otherwise it's queued for
    // updated.
    UPDATED     = 1 << 0,
    // A static point, not dynamic or empty, necessary for being traversable.
    STATIC      = 1 << 1,
    // Approximately horizontal orientation based on normal direction,
    // necessary condition for being traversable.
    HORIZONTAL  = 1 << 2,
    // Near another actor.
    ACTOR       = 1 << 3,
    // A point at the edge, i.e. a frontier.
    EDGE        = 1 << 4,
    // Traversable based on terrain roughness and obstacles in neighborhood.
    TRAVERSABLE = 1 << 5
};

//    }
//typedef Buffer<uint8_t> Labels;

const Index INVALID_INDEX = std::numeric_limits<Index>::max();
const Vertex INVALID_VERTEX = std::numeric_limits<Vertex>::max();

class Point
{
public:
    Point():
            position_{std::numeric_limits<Value>::quiet_NaN(),
                      std::numeric_limits<Value>::quiet_NaN(),
                      std::numeric_limits<Value>::quiet_NaN()},
            normal_{std::numeric_limits<Value>::quiet_NaN(),
                    std::numeric_limits<Value>::quiet_NaN(),
                    std::numeric_limits<Value>::quiet_NaN()},
            normal_support_(0),
            ground_diff_std_(std::numeric_limits<Value>::quiet_NaN()),
            min_ground_diff_(std::numeric_limits<Value>::quiet_NaN()),
            max_ground_diff_(std::numeric_limits<Value>::quiet_NaN()),
            mean_abs_ground_diff_(std::numeric_limits<Value>::quiet_NaN()),
            viewpoint_{std::numeric_limits<Value>::quiet_NaN(),
                       std::numeric_limits<Value>::quiet_NaN(),
                       std::numeric_limits<Value>::quiet_NaN()},
            dist_to_actor_(std::numeric_limits<Value>::quiet_NaN()),
            actor_last_visit_(std::numeric_limits<Value>::quiet_NaN()),
            dist_to_other_actors_(std::numeric_limits<Value>::quiet_NaN()),
            other_actors_last_visit_(std::numeric_limits<Value>::quiet_NaN()),
            dist_to_obstacle_(std::numeric_limits<Value>::quiet_NaN()),
            flags_(0),
            num_empty_(0),
            num_occupied_(0),
            dist_to_plane_(std::numeric_limits<Value>::quiet_NaN()),
            num_obstacle_pts_(0),
            num_edge_neighbors_(0),
            path_cost_(std::numeric_limits<Value>::quiet_NaN()),
            reward_(std::numeric_limits<Value>::quiet_NaN()),
            relative_cost_(std::numeric_limits<Value>::quiet_NaN())
    {}
//
    Value position_[3];
    // Geometric features
    // TODO: More compact normal representation? Maybe just for sharing,
    // impacts on memory is small compared to neighbors.
    // TODO: Switch to compact (u)int8 types where possible.
    Value normal_[3];
//    int8 normal_[3];
    // Normal scale is common to all points.
    // Number of points used in normal computation.
    uint8_t normal_support_;
    // Roughness features (in neighborhood radius).
    // from ball neighborhood
    Value ground_diff_std_;
    // circle in ground plane
    Value min_ground_diff_;
    Value max_ground_diff_;
    Value mean_abs_ground_diff_;
    // Viewpoint (for occupancy assessment and measurement distance)
    Value viewpoint_[3];
    // Distance (Euclidean + time) to this actor and other actors.
    Value dist_to_actor_;
    Value actor_last_visit_;
    Value dist_to_other_actors_;
    Value other_actors_last_visit_;
    // Distance to nearest obstacle (non horizontal point).
    Value dist_to_obstacle_;
    // Point flags accoring to Flags.
    uint8_t flags_;
    // Number of occurences of empty/occupied state.
    uint8_t num_empty_;
    uint8_t num_occupied_;
    Value dist_to_plane_;
    // Number of obstacles nearby.
    uint8_t num_obstacle_pts_;
    // Edge flag.
//    uint8_t edge_;
    // Number of edge points nearby.
    uint8_t num_edge_neighbors_;
    // Label based on normal direction (normal already needs graph).
//    uint8_t normal_label_;
    // Label based on terrain roughness.
//    uint8_t functional_label_;
    // Planning costs and rewards
    Value path_cost_;
    Value reward_;
    Value relative_cost_;
};

class Neighborhood
{
public:
    Neighborhood():
            position_{},
//            neighbor_count_(0),
            neighbors_{},
            distances_{}
    {}

    // TODO: Make K_NEIGHBORS a parameter.
    static const Index K_NEIGHBORS = 48;
    Value position_[3];
    // NN Graph
    // Number of valid entries in neighbors_ and distances_.
    Index neighbor_count_;
    Index neighbors_[K_NEIGHBORS];
    Value distances_[K_NEIGHBORS];
    // Index state from IndexState enum.
    // uint8_t index_state_;
};

}  // namespace naex

#endif  // NAEX_TYPES_H
