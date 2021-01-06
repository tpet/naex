
#ifndef NAEX_TYPES_H
#define NAEX_TYPES_H

#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <memory>
//#include <naex/buffer.h>

namespace naex
{
// Basic floating-point and index types
typedef float Elem;
typedef Elem Value;
typedef uint32_t Index;

// Arrays and matrices
typedef Eigen::Matrix<Value, 3, 1, Eigen::DontAlign> Vec3;
typedef Eigen::Map<Vec3> Vec3Map;
typedef Eigen::Map<const Vec3> ConstVec3Map;
typedef Eigen::Matrix<Value, 2, 1, Eigen::DontAlign> Vec2;
typedef Eigen::Map<Vec2> Vec2Map;
typedef Eigen::Map<const Vec3> ConstVec2Map;
typedef Eigen::Matrix<Value, 3, 3, Eigen::DontAlign> Mat3;
typedef Eigen::Quaternion<Value, Eigen::DontAlign> Quat;
typedef Eigen::Matrix<Value, 3, Eigen::Dynamic, Eigen::DontAlign> MatX;
typedef Eigen::Map<MatX> MatXMap;
typedef Eigen::Map<const MatX> ConstMatXMap;

typedef std::vector<Index> Indices;

// Vertex and edge indices
// TODO: Rename both to Index (to be used elsewhere too).
typedef Index Vertex;
typedef Index Edge;
// Edge cost or length
typedef Elem Cost;

typedef flann::Matrix<Value> FlannMat;
typedef flann::Index<flann::L2_3D<Value>> FlannIndex;
typedef std::shared_ptr<FlannIndex> FlannIndexPtr;
typedef std::shared_ptr<const FlannIndex> ConstFlannIndexPtr;

//    namespace IndexState
//    {
enum IndexState
{
    TO_UPDATE = 0,
    UP_TO_DATE = 1
};
//    }

//    namespace Label
//    {
enum Label
{
    UNKNOWN = 0,
    EMPTY = 1,
    TRAVERSABLE = 2,
    EDGE = 3,
    ACTOR = 4,
    OBSTACLE = 5
};
//    }
//typedef Buffer<uint8_t> Labels;

const Vertex INVALID_VERTEX = std::numeric_limits<Vertex>::max();

class Point
{
public:
    static const Index K_NEIGHBORS = 32;
    Value position_[3];
    // Geometric features
    // TODO: More compact normal representation? Maybe just for sharing,
    // impacts on memory is small compared to neighbors.
    Value normal_[3];
    Index normal_support_;  // Normal scale is common to all points
    // Roughness features.
    // from ball neighborhood
    Value ground_diff_std_;
    // circle in ground plane
    Value ground_diff_min_;
    Value ground_diff_max_;
    Value ground_abs_diff_mean_;
    // Viewpoint (for occupancy assessment and measurement distance)
    Value viewpoint_[3];
    // Occupancy
    uint8_t empty_;
    uint8_t occupied_;
    uint8_t num_obstacle_pts_;
    uint8_t num_edge_neighbors_;
    uint8_t normal_label_;
    uint8_t functional_label_;
    // Planning costs and rewards
    Value path_cost_;
    Value reward_;
    Value relative_cost_;
    // NN Graph
    Index neighbor_count_;
    Index neighbors_[K_NEIGHBORS];
    Value distances_[K_NEIGHBORS];
    // Index state:
    // unindexed = 0
    // dirty / to update = 1
    // indexed = 2
    uint8_t index_state_;
};

}  // namespace naex

#endif  // NAEX_TYPES_H
