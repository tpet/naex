
#ifndef NAEX_TYPES_H
#define NAEX_TYPES_H

#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <naex/buffer.h>

namespace naex
{
    // Point cloud position and normal element type
    typedef float Elem;
    typedef float Value;
    typedef Eigen::Matrix<Elem, 3, 1, Eigen::DontAlign> Vec3;
    typedef Eigen::Map<Vec3> Vec3Map;
    typedef Eigen::Map<const Vec3> ConstVec3Map;
    typedef Eigen::Matrix<Elem, 2, 1, Eigen::DontAlign> Vec2;
    typedef Eigen::Map<Vec2> Vec2Map;
    typedef Eigen::Map<const Vec3> ConstVec2Map;
    typedef Eigen::Matrix<Elem, 3, 3, Eigen::DontAlign> Mat3;
    typedef Eigen::Quaternion<Elem, Eigen::DontAlign> Quat;
    typedef Eigen::Matrix<Elem, 3, Eigen::Dynamic, Eigen::DontAlign> MatX;
    typedef Eigen::Map<MatX> MatXMap;
    typedef Eigen::Map<const MatX> ConstMatXMap;

    // Vertex and edge indices
    // TODO: Rename both to Index (to be used elsewhere too).
    typedef uint32_t Vertex;
    typedef uint32_t Edge;
    typedef uint32_t Index;
    // Edge cost or length
    typedef Elem Cost;

    typedef flann::Matrix<Elem> FlannMat;
    typedef flann::Index<flann::L2_3D<Elem>> FlannIndex;
    typedef std::shared_ptr<FlannIndex> FlannIndexPtr;
    typedef std::shared_ptr<const FlannIndex> ConstFlannIndexPtr;

    enum Label
    {
        TRAVERSABLE = 0,
        EMPTY = 1,
        UNKNOWN = 2,
        EDGE = 3,
        ACTOR = 4,
        OBSTACLE = 5
    };
    typedef Buffer<uint8_t> Labels;

    const Vertex INVALID_VERTEX = std::numeric_limits<Vertex>::max();

}  // namespace naex

#endif //NAEX_TYPES_H
