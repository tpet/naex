#ifndef NAEX_GEOM_H
#define NAEX_GEOM_H

#include <Eigen/Dense>
#include <cstddef>
#include <cmath>
#include <mutex>
#include <naex/buffer.h>
#include <naex/clouds.h>
#include <naex/iterators.h>
#include <naex/nearest_neighbors.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <set>
#include <unordered_set>

namespace naex
{

/**
 * Angle from z vector [0, 0, 1].
 * @param x Unit vector.
 * @return Angle from up.
 */
Value inline angle_from_up(const Vec3& x)
{
    return std::acos(x(2));
}

/**
 * Inclination w.r.t. x-y plane.
 * @param x Unit vector.
 * @return Inclination w.r.t. x-y plane.
 */
Value inline inclination(const Vec3& x)
{
    return std::atan2(x(2), std::hypot(x(0), x(1)));
//    return Value(M_PI_2) - angle_from_up_normalized(x);
}

/**
 * Plane from three points (in non-degenerate configuration).
 * @tparam Derived
 * @param p0
 * @param p1
 * @param p2
 * @return
 */
//template<typename T>
//Vec4 plane_from_points(T& p0, T& p1, T& p2)
template <typename Derived>
Vec4 plane_from_points(const Eigen::MatrixBase<Derived>& p0,
                       const Eigen::MatrixBase<Derived>& p1,
                       const Eigen::MatrixBase<Derived>& p2)
{
    Vec3 n = (p1 - p0).cross(p2 - p0).normalized();
    Value d = -n.dot(p0);
//    return Vec4(n, d);
    return Vec4(n(0), n(1), n(2), d);
}

Vec4 e2p(Vec3& v)
{
//    return Vec4(v, 1.);
    return Vec4(v(0), v(1), v(2), 1.);
}

}  // namespace naex

#endif  // NAEX_GEOM_H
