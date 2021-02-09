#ifndef NAEX_MAP_H
#define NAEX_MAP_H

#include <cstddef>
#include <cmath>
#include <mutex>
#include <naex/buffer.h>
#include <naex/clouds.h>
#include <naex/geom.h>
#include <naex/iterators.h>
#include <naex/nearest_neighbors.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <set>
#include <unordered_set>

namespace naex
{
class Map
{
public:
    typedef std::recursive_mutex Mutex;
    typedef std::lock_guard<Mutex> Lock;

    static const size_t DEFAULT_CAPACITY = 10000000;

    Map():
            index_(),
            dirty_indices_(),
            // Parameter defaults
            points_min_dist_(0.2f),
            min_empty_cos_(0.259),
            // Occupancy
            min_num_empty_(2),
            min_empty_ratio_(1.0f),
            max_occ_counter_(7),
            neighborhood_radius_(0.6f),
            edge_min_centroid_offset_(0.5f),
            // Traversability
        //        max_nn_height_diff_(.15),
            min_ground_diff_(-0.1),
            max_ground_diff_(0.3),
            clearance_low_(0.15),
            clearance_high_(0.8),
            clearance_radius_(neighborhood_radius_),
            min_points_obstacle_(3),
            max_ground_diff_std_(0.1),
            max_mean_abs_ground_diff_(0.1),
            min_dist_to_obstacle_(clearance_radius_),
            max_pitch_(30. / 180. * M_PI),
            max_roll_(30. / 180. * M_PI),
            inclination_penalty_(1.)
    {
//        dirty_indices_.reser
        updated_indices_.reserve(10000);
        dirty_indices_.reserve(10000);
        cloud_.reserve(DEFAULT_CAPACITY);
        graph_.reserve(DEFAULT_CAPACITY);
    }

    FlannMat position_matrix(Index start = 0, Index end = 0)
    {
        if (cloud_.empty()) {
            return FlannMat();
        }
        Index size = (start < end) ? end - start : Index(cloud_.size()) - start;
        auto res = FlannMat(cloud_[start].position_, static_cast<size_t>(size), 3, sizeof(Point));
//        ROS_INFO("Position matrix %lu-by-%lu. First point: [%.2f, %.2f, %.2f].",
//                 res.rows, res.cols, res[0][0], res[0][1], res[0][2]);
        return res;
    }

    flann::Matrix<int> neighbor_matrix()
    {
        if (cloud_.empty()) {
            return flann::Matrix<int>();
        }
        // TODO: Inconsistency between boost graph (Index = uint32) and flann (int).
//        return flann::Matrix<int>(reinterpret_cast<int*>(cloud_[0].neighbors_),
//                                  graph_.size(),
//                                  3,
//                                  sizeof(Point));
        return flann::Matrix<int>(reinterpret_cast<int*>(graph_[0].neighbors_),
                                  graph_.size(),
                                  Neighborhood::K_NEIGHBORS,
                                  sizeof(Neighborhood));
    }

    Value* position(size_t i)
    {
        return &cloud_[i].position_[0];
    }

    const Value* position(size_t i) const
    {
        return &cloud_[i].position_[0];
    }

    Value* normal(size_t i)
    {
        return &cloud_[i].normal_[0];
    }

    const Value* normal(size_t i) const
    {
        return &cloud_[i].normal_[0];
    }

//    template<T>
//    bool flag(T value, T flag)
//    {
//        return flag & value;
//    }

//    bool point_empty(Index i)
//    {
//        return cloud_[i].num_empty_ >= min_num_empty_
//                && Value(cloud_[i].num_empty_) / cloud_[i].num_occupied_ >= min_empty_ratio_;
//    }

    bool point_empty(const Point& p)
    {
        return p.num_empty_ >= min_num_empty_
            && Value(p.num_empty_) / p.num_occupied_ >= min_empty_ratio_;
    }

    bool point_near(Index i, const std::vector<Value>& points, Value radius)
    {
        for (Index j = 0; j + 2 < points.size(); j += 3)
        {
            if ((ConstVec3Map(cloud_[i].position_) - ConstVec3Map(&points[j])).norm() <= radius)
            {
                return true;
            }
        }
        return false;
    }

    inline Cost edge_cost(const Edge& e) const
    {
        const auto v0 = source(e);
        const auto v1_index = target_index(e);
        const auto v1 = target(e);
//            if (labels_[v0] != TRAVERSABLE || labels_[v1] != TRAVERSABLE)
//        if (cloud_[v1].functional_label_ != TRAVERSABLE)
        if (!(cloud_[v1].flags_ & TRAVERSABLE) || (cloud_[v1].flags_ & EDGE))
        {
            return std::numeric_limits<Cost>::infinity();
        }
//        Cost d = std::sqrt(cloud_[v0].distances_[v1_index]);
        Cost c = graph_[v0].distances_[v1_index];
//        if (c > clearance_radius_)
        if (c > 2 * points_min_dist_)
        {
            return std::numeric_limits<Cost>::infinity();
        }

        // Check pitch and roll limits.
        const Vec3 forward = ConstVec3Map(cloud_[v1].position_) - ConstVec3Map(cloud_[v0].position_);
        const Vec3 left = ConstVec3Map(cloud_[v1].normal_).cross(forward);
        Value pitch = inclination(forward);
        Value roll = inclination(left);
        if (std::abs(pitch) > max_pitch_ || std::abs(roll) > max_roll_)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        // Initialize with distance computed in NN search.
        // Multiple with relative pitch and roll.
        c *= 1 + inclination_penalty_ * std::abs(pitch) / max_pitch_;
        c *= 1 + inclination_penalty_ * std::abs(roll) / max_roll_;
        // Penalize distance to obstacles and other actors.
        if (std::isfinite(cloud_[v1].dist_to_obstacle_))
        {
            c *= 1 + std::max(Value(0),
                              1 - cloud_[v1].dist_to_obstacle_ / (2 * clearance_radius_));
        }
        // Penalize by edge and obstacles points nearby.
        c *= 1 + Value(cloud_[v1].num_edge_neighbors_) / Neighborhood::K_NEIGHBORS;
        c *= 1 + Value(cloud_[v1].num_obstacle_neighbors_) / Neighborhood::K_NEIGHBORS;
        // TODO: Add soft margins around other actors.
        // Would need current position of other actors.
//        c *= std::isfinite(cloud_[v1].dist_to_other_actors_)
//                ? std::max(0.f, 1. - cloud_[v1].dist_to_other_actors_ / (2. * clearance_radius_))
//                : 1.f;
        // TODO: Make distance correspond to expected travel time.
        return c;
    }

    void update_index()
    {
        Timer t;
        // TODO: Update only dirty points.
        if (!empty())
        {
            Lock cloud_lock(cloud_mutex_);
            Lock index_lock(index_mutex_);
//                index_ = std::make_shared<flann::Index<flann::L2_3D<Elem>>>(points_, flann::KDTreeSingleIndexParams());
            index_ = std::make_shared<flann::Index<flann::L2_3D<Value>>>(
                    position_matrix(),
                    flann::KDTreeSingleIndexParams());
            // TODO: Is this necessary?
            index_->buildIndex();
            ROS_DEBUG("Index updated for %lu points (%.3f s).",
                      index_->size(), t.seconds_elapsed());
        }
        else
        {
            ROS_INFO("Index not updated due to empty map.");
        }
    }

    std::vector<Index> collect_points_to_update()
    {
        std::vector<Index> indices;
//        indices.reserve(cloud_.size());
        for (Index i = 0; i < cloud_.size(); ++i)
        {
            if (!(cloud_[i].flags_ & UPDATED))
            {
                indices.push_back(i);
            }
        }
        return indices;
    }

    template<typename It>
    void update_neighborhood(It begin, It end)
    {
        // NB: It should be stable iteration order.
        Timer t;

        std::vector<Neighborhood> dirty_cloud;

        for (It it = begin; it != end; ++it)
        {
            dirty_cloud.push_back(graph_[*it]);
        }

        if (dirty_cloud.empty())
        {
            ROS_DEBUG("Graph up to date, no points to update (%.3f s).",
                     t.seconds_elapsed());
            return;
        }

        flann::Matrix<Value> positions(dirty_cloud[0].position_,
                                       dirty_cloud.size(),
                                       3,
                                       sizeof(Neighborhood));
        flann::Matrix<Index> neighbors(dirty_cloud[0].neighbors_,
                                     dirty_cloud.size(),
                                     Neighborhood::K_NEIGHBORS,
                                     sizeof(Neighborhood));
        flann::Matrix<Value> distances(dirty_cloud[0].distances_,
                                       dirty_cloud.size(),
                                       Neighborhood::K_NEIGHBORS,
                                       sizeof(Neighborhood));

        t.reset();
        flann::SearchParams params;
        params.checks = 64;
        params.cores = 0;
        params.max_neighbors = Neighborhood::K_NEIGHBORS;
        params.sorted = true;
        {
            Lock cloud_lock(cloud_mutex_);
            Lock index_lock(index_mutex_);
            index_->knnSearch(positions, neighbors, distances,
                              Neighborhood::K_NEIGHBORS, params);
            // FIXME: Radius search with max K seems to result in errors.
//            index_->radiusSearch(positions, neighbors, distances,
//                                 neighborhood_radius_, params);
        }
//        ROS_INFO("Search complete (%.3f s).", t.seconds_elapsed());
        // Propagate NN info into the main graph.
        auto point_it = dirty_cloud.begin();
        for (It it = begin; it != end; ++it, ++point_it)
        {
            std::copy((*point_it).neighbors_,
                      (*point_it).neighbors_ + Neighborhood::K_NEIGHBORS,
                      graph_[*it].neighbors_);
//            std::copy((*point_it).distances_,
//                      (*point_it).distances_ + Neighborhood::K_NEIGHBORS,
//                      graph_[*it].distances_);
            std::transform((*point_it).distances_,
                           (*point_it).distances_ + Neighborhood::K_NEIGHBORS,
                           graph_[*it].distances_,
                           [](const Value x) -> Value { return std::sqrt(x); });
        }

        ROS_DEBUG("Neighborhood updated at %lu / %lu pts (%.3f s).",
                  dirty_cloud.size(), cloud_.size(), t.seconds_elapsed());
        // TODO: Update features and labels.
    }

    inline Vertex num_vertices() const
    {
        return size();
    }
    inline Edge num_edges() const
    {
        // TODO: Compute true number based on valid neighbors.
        return num_vertices() * Neighborhood::K_NEIGHBORS;
    }
    inline std::pair<VertexIter, VertexIter> vertices() const
    {
        return { 0, size() };
    }
    inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
    {
        // TODO: Limit to valid edges here or just by costs?
        // Skip the first neighbor - the vertex itself.
        return { u * Neighborhood::K_NEIGHBORS + 1, (u + 1) * Neighborhood::K_NEIGHBORS };
    }
    inline Edge out_degree(const Vertex& u) const
    {
        // TODO: Compute true number based on valid neighbors.
//        return num_edges();
        return Neighborhood::K_NEIGHBORS - 1;
    }
    inline Vertex source(const Edge& e) const
    {
//        return e / num_edges();
        return e / Neighborhood::K_NEIGHBORS;
    }
    inline Vertex target_index(const Edge& e) const
    {
//        return e % num_edges();
        return e % Neighborhood::K_NEIGHBORS;
    }
    inline Vertex target(const Edge& e) const
    {
        return graph_[source(e)].neighbors_[target_index(e)];
    }

    void update_dirty()
    {
        Lock lock(dirty_mutex_);
        Timer t;

        Timer t_part;
        update_neighborhood(dirty_indices_.begin(), dirty_indices_.end());
//        ROS_DEBUG("Graph updated at %lu points (%.6f s).",
//                  dirty_indices_.size(), t_part.seconds_elapsed());

        t_part.reset();
        compute_features(dirty_indices_.begin(), dirty_indices_.end());
//        ROS_DEBUG("Features updated at %lu points (%.6f s).",
//                  dirty_indices_.size(), t_part.seconds_elapsed());

        t_part.reset();
        compute_labels(dirty_indices_.begin(), dirty_indices_.end());
//        ROS_DEBUG("Labels updated at %lu points (%.6f s).",
//                  dirty_indices_.size(), t_part.seconds_elapsed());

        ROS_DEBUG("%lu points updated (%.3f s).", dirty_indices_.size(), t.seconds_elapsed());
    }

    void clear_updated()
    {
        Lock lock(updated_mutex_);
        const auto n = updated_indices_.size();
        updated_indices_.clear();
        ROS_DEBUG("%lu updated indices cleared.", n);
    }

    void clear_dirty()
    {
        Lock lock(dirty_mutex_);
        const auto n = dirty_indices_.size();
        dirty_indices_.clear();
        ROS_DEBUG("%lu dirty indices cleared.", n);
    }

    void estimate_plane(Index i)
    {

    }

    template<typename It>
    void compute_features(It begin, It end)
    {
        Timer t;
        const auto semicircle_centroid_offset = Value(4.0 * neighborhood_radius_ / (3.0 * M_PI));
        Index n = 0;
        Index n_edge = 0;

        for (It it = begin; it != end; ++it, ++n)
        {
            const auto v0 = *it;

            // Disregard empty / dynamic points.
            if (!(cloud_[v0].flags_ & STATIC))
            {
                continue;
            }

            // Estimate normals (local surface orientation).
            Vec3 mean = Vec3::Zero();
            Mat3 cov = Mat3::Zero();
            cloud_[v0].normal_support_ = 0;
            // First neighbor is the point itself.
            for (Index j = 0; j < Neighborhood::K_NEIGHBORS; ++j)
            {
                if (!valid_neighbor(graph_[v0].neighbors_[j], graph_[v0].distances_[j]))
                {
                    continue;
                }

                Index v1 = graph_[v0].neighbors_[j];

                // Disregard empty points.
                if (!(cloud_[v1].flags_ & STATIC))
                {
                    continue;
                }
                if (graph_[v0].distances_[j] <= clearance_radius_) {
                    mean += ConstVec3Map(cloud_[v1].position_);
//                    Vec3 pc = (ConstVec3Map(cloud_[v1].position_) - mean);
                    Vec3 pc = (ConstVec3Map(cloud_[v1].position_) - ConstVec3Map(cloud_[v0].position_));
                    cov += pc * pc.transpose();
                    ++cloud_[v0].normal_support_;
                }
            }
            mean /= cloud_[v0].normal_support_;
            cov /= cloud_[v0].normal_support_;
            Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
            Vec3Map normal(cloud_[v0].normal_);
            normal = solver.eigenvectors().col(0);

            // Compute other properties dependent on normal.
            cloud_[v0].ground_diff_std_ = std::sqrt(solver.eigenvalues()(0));

            // Inject support label here where we have mean at hand.
            const auto centroid_offset = (mean - ConstVec3Map(cloud_[v0].position_)).norm();
            if (centroid_offset > edge_min_centroid_offset_ * semicircle_centroid_offset)
            {
                cloud_[v0].flags_ |= EDGE;
                n_edge;
            }
            else
            {
                cloud_[v0].flags_ &= ~EDGE;
            }
        }
        ROS_DEBUG("%lu / %lu edge points (%.4f s).",
                  size_t(n_edge), size_t(n), t.seconds_elapsed());
    }

    template<typename It>
    void compute_labels(It begin, It end)
    {
        Timer t;
        Index n = 0;
        Index n_horizontal = 0;
        Index n_traverable = 0;
        Index n_empty = 0;
        Index n_actor = 0;
        Index n_obstacle = 0;

        const auto max_slope = std::max(max_pitch_, max_roll_);
        const auto min_z = std::cos(max_slope);

        for (It it = begin; it != end; ++it, ++n)
        {
            const auto v0 = *it;

            // Disregard empty points. Don't recompute anything for these.
            if (!(cloud_[v0].flags_ & STATIC))
            {
                ++n_empty;
                continue;
            }

            // Clear flags we may set later.
            cloud_[v0].flags_ &= ~(HORIZONTAL | TRAVERSABLE);

            // Actor flag is temporary and orthogonal to others.
            // TODO: Update distances to actors in planning.
            if (cloud_[v0].flags_ & ACTOR)
            {
                ++n_actor;
            }

            // TODO: Check sign once normals keep consistent orientation.
            if (std::abs(cloud_[v0].normal_[2]) >= min_z)
            {
                // Approx. horizontal based on normal.
                cloud_[v0].flags_ |= HORIZONTAL;
                ++n_horizontal;
            }
//            else  // if (std::abs(normals_[i][2]) < min_z)
//            {
//                // Approx. vertical based on normal.
//                cloud_[v0].flags_ &= ~(HORIZONTAL | TRAVERSABLE);
//            }

            // Count edge neighbors (must run in the second pass).
            cloud_[v0].num_edge_neighbors_ = 0;
            cloud_[v0].num_obstacle_neighbors_ = 0;
            // Compute ground features (need second loop through neighbors).
            cloud_[v0].min_ground_diff_ = std::numeric_limits<Elem>::infinity();
            cloud_[v0].max_ground_diff_ = -std::numeric_limits<Elem>::infinity();
            cloud_[v0].mean_abs_ground_diff_ = 0.;
            // Compute clearance.
            cloud_[v0].dist_to_obstacle_ = std::numeric_limits<Value>::infinity();
            cloud_[v0].num_obstacle_pts_ = 0;
            Index n_cylinder_pts = 0;

            for (Vertex j = 0; j < Neighborhood::K_NEIGHBORS; ++j)
            {
                const auto v1 = graph_[v0].neighbors_[j];
                // Disregard invalid neighbors.
                if (!valid_neighbor(v1, graph_[v0].distances_[j]))
                {
                    continue;
                }
                // Disregard distant neighbors.
                if (graph_[v0].distances_[j] > neighborhood_radius_)
                {
                    continue;
                }

                // Disregard empty points.
                if (!(cloud_[v1].flags_ & STATIC))
                {
                    continue;
                }

                if (cloud_[v1].flags_ & EDGE)
                {
                    ++cloud_[v0].num_edge_neighbors_;
                }

                if (!(cloud_[v1].flags_ & HORIZONTAL))
                {
                    ++cloud_[v0].num_obstacle_neighbors_;
                }

                // Avoid driving near obstacles.
                // TODO: Use clearance as hard constraint and distance to obstacles as costs.
                // Hard constraint can be removed with min_dist_to_obstacle_.
                if (!(cloud_[v1].flags_ & HORIZONTAL))
                {
                    if (graph_[v0].distances_[j] <= min_dist_to_obstacle_)
                    {
                        cloud_[v0].flags_ &= ~TRAVERSABLE;
                    }
                    cloud_[v0].dist_to_obstacle_ = std::min(graph_[v0].distances_[j], cloud_[v0].dist_to_obstacle_);
                }

                Vec3Map p0(cloud_[v0].position_);
                Vec3Map p1(cloud_[v1].position_);
                Vec3Map n0(cloud_[v0].normal_);

                Value height_diff = n0.dot(p1 - p0);
                Vec3 ground_pt = p1 - height_diff * n0;
                Value ground_dist = (ground_pt - p0).norm();

                if (ground_dist <= clearance_radius_)
                {
                    cloud_[v0].min_ground_diff_ = std::min(height_diff, cloud_[v0].min_ground_diff_);
                    cloud_[v0].max_ground_diff_ = std::max(height_diff, cloud_[v0].max_ground_diff_);
                    cloud_[v0].mean_abs_ground_diff_ += std::abs(height_diff);
                    ++n_cylinder_pts;
                    if (height_diff >= clearance_low_ && height_diff <= clearance_high_)
                    {
                        ++cloud_[v0].num_obstacle_pts_;
                    }
                }
            }
            cloud_[v0].mean_abs_ground_diff_  /= n_cylinder_pts;
            if ((cloud_[v0].flags_ & HORIZONTAL)
                    && cloud_[v0].num_obstacle_pts_ < min_points_obstacle_
                    && cloud_[v0].min_ground_diff_ >= min_ground_diff_
                    && cloud_[v0].max_ground_diff_ <= max_ground_diff_
                    && cloud_[v0].ground_diff_std_ <= max_ground_diff_std_
                    && cloud_[v0].mean_abs_ground_diff_ <= max_mean_abs_ground_diff_)
            {
                cloud_[v0].flags_ |= TRAVERSABLE;
                ++n_traverable;
            }
        }
        ROS_DEBUG("%lu labels updated: %lu horizontal, %lu traversable, "
                  "%lu empty, %lu actor, (%.3f s).",
                  size_t(n), size_t(n_horizontal), size_t(n_traverable),
                  size_t(n_empty), size_t(n_actor), t.seconds_elapsed());
    }

    /** Resize point buffers if necessary, update wrappers and index. */
    void reserve(size_t n)
    {
        Timer t;
        if (n < capacity()) {
            return;
        }
        cloud_.reserve(n);
        graph_.reserve(n);
        update_index();
        ROS_INFO("Capacity increased to %lu points: %.3f s.", n, t.seconds_elapsed());
    }

//    std::vector<Index> nearby_indices(const flann::Matrix<Value>& origin, Value radius)
    std::vector<Index> nearby_indices(Value* origin, Value radius)
    {
        Lock cloud_lock(cloud_mutex_);
        Lock index_lock(index_mutex_);
        RadiusQuery<Value> q(*index_, flann::Matrix<Value>(origin, 1, 3), radius);
        // TODO: Is this enough to move it?
        return q.nn_[0];
    }

//    template<typename C>  // C = Camera
    void update_occupancy_organized(const sensor_msgs::PointCloud2& cloud,
                                    const geometry_msgs::Transform& cloud_to_map_tf)
//                                    const C* camera)
    {
        Timer t;
        Timer t_part;

        if (empty())
        {
            ROS_INFO("No map points for updating occupancy.");
            return;
        }

        assert(cloud.height > 1);
        size_t n_pts = cloud.height * cloud.width;
        if (n_pts == 0)
        {
            ROS_INFO("No input points for updating occupancy.");
            return;
        }

        // TODO: Improve efficiency with camera model and cloud structure.
        Eigen::Isometry3f cloud_to_map(tf2::transformToEigen(cloud_to_map_tf));
        Eigen::Isometry3f map_to_cloud = cloud_to_map.inverse(Eigen::Isometry);

        // Create sensor ray directions with an index.
        t_part.reset();
        sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
//        flann::Matrix<float> cloud_mat(&cloud_begin[0], 1, 3, cloud.point_step);
        std::vector<Value> dirs(3 * n_pts);
        Value* dir_ptr = dirs.data();
        for (auto x_it = x_begin; x_it != x_it.end(); ++x_it, dir_ptr += 3)
        {
            Vec3Map dir(dir_ptr);
            dir = ConstVec3Map(&x_it[0]).normalized();
        }
        FlannIndex dir_index(FlannMat(dirs.data(), n_pts, 3), flann::KDTreeSingleIndexParams());
        dir_index.buildIndex();
        ROS_INFO("%lu sensor rays and index created (%.6f s).", n_pts, t_part.seconds_elapsed());

        // Find closest map points to sensor origin.
        Lock cloud_lock(cloud_mutex_);
        Lock index_lock(index_mutex_);
        Lock dirty_lock(dirty_mutex_);
        t_part.reset();
        Vec3 origin = cloud_to_map.translation();
        RadiusQuery<Value> q_map(*index_, FlannMat(origin.data(), 1, 3), 10.f);
        ROS_INFO("%lu closest map points found (%.6f s).", q_map.nn_[0].size(), t_part.seconds_elapsed());

        // Test each map point, whether we can see through.
        t_part.reset();
        std::vector<Index> nn_buf(1, INVALID_VERTEX);
        std::vector<Value> dist_buf(1, std::numeric_limits<Value>::quiet_NaN());
        flann::Matrix<Index> nn(nn_buf.data(), 1, 1);
        flann::Matrix<Value> dist(dist_buf.data(), 1, 1);
        flann::SearchParams params;
        params.checks = 64;
        params.cores = 0;
        params.sorted = true;
//        std::vector<Index> fan_nn(4, INVALID_VERTEX);
//        std::vector<Value> fan_dist(4, std::numeric_limits<Value>::quiet_NaN());
        typedef std::pair<Index, Value> IVP;
        std::vector<IVP> fan(4, {INVALID_VERTEX, std::numeric_limits<Value>::quiet_NaN()});
        // Comparator for descending order of second member.
        const auto comp = [](const IVP& a, const IVP& b) { return a.second > b.second; };
        Index n_occupied = 0;
        Index n_empty = 0;
        Index n_above = 0;
        Index n_modified = 0;
        for (const auto i: q_map.nn_[0])
        {
//            ConstVec3Map p(cloud_[i].position_);
            Vec3 p = map_to_cloud * ConstVec3Map(cloud_[i].position_);
//            Vec3 dir = (map_to_cloud * p).normalized();
            Vec3 dir = p.normalized();
            dir_index.knnSearch(FlannMat(dir.data(), 1, 3), nn, dist, 1, params);
            Index j = nn_buf[0];
            Index r = j / cloud.width;
            Index c = j % cloud.width;
            // Seek containing triangle among incident ones.
            // Test occlusion of the containing triangle.
//            fan_nn[0] = (r > 0) ? ((r - 1) * cloud.width + c) : INVALID_VERTEX;
//            fan_dist[0] = (fan_nn[0] != INVALID_VERTEX) ? : std::numeric_limits<Value>::infinity();
            Index k = 0;
            fan[k].first = r > 0
                           ? (r - 1) * cloud.width + c
                           : INVALID_VERTEX;
            fan[k].second = fan[k].first != INVALID_VERTEX
                            ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                            : -std::numeric_limits<Value>::infinity();

            ++k;
            fan[k].first = c > 0
                           ? r * cloud.width + c - 1
                           : INVALID_VERTEX;
            fan[k].second = fan[k].first != INVALID_VERTEX
                            ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                            : -std::numeric_limits<Value>::infinity();

            ++k;
            fan[k].first = c < cloud.width - 1
                           ? r * cloud.width + c + 1
                           : INVALID_VERTEX;
            fan[k].second = fan[k].first != INVALID_VERTEX
                            ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                            : -std::numeric_limits<Value>::infinity();

            ++k;
            fan[k].first = r < cloud.height - 1
                           ? (r + 1) * cloud.width + c
                           : INVALID_VERTEX;
            fan[k].second = fan[k].first != INVALID_VERTEX
                            ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                            : -std::numeric_limits<Value>::infinity();

            // Sort nearby indices by cosine distance.
            std::sort(fan.begin(), fan.end(), comp);
            // Construct plane from three points of nearest directions.
            // Test signed distance from the plane to assess occlusion.
//            ConstVec3Map p0(&(x_begin + i)[0]);
//            ConstVec3Map p1(&(x_begin + fan[0].first)[0]);
//            ConstVec3Map p2(&(x_begin + fan[1].first)[0]);
            // TODO: Orient normal.
//            Vec3 n = (p1 - p0).cross(p2 - p0).normalized();
//            Value d = -n.dot(p0);
            Vec4 plane = plane_from_points(ConstVec3Map(&(x_begin + i)[0]),
                                           ConstVec3Map(&(x_begin + fan[0].first)[0]),
                                           ConstVec3Map(&(x_begin + fan[1].first)[0]));

            // Make sure positive distance is towards sensor at [0, 0, 0],
            // i.e., outside from surface.
//            if (d < 0.)
//            {
//                n = -n;
//                d = -d;
//            }
            if (plane(3) < 0.)
            {
                plane = -plane;
            }
//            Value signed_dist = n.dot(p) + d;
            Value signed_dist = plane.dot(e2p(p));
            Value eps = points_min_dist_ / Value(2.);
            if (signed_dist > eps)
            {
                // New point above surface.
                ++n_above;
            }
            else if (signed_dist >= -eps)
            {
                // Known surface measured again.
                if (cloud_[i].num_occupied_ == std::numeric_limits<decltype(cloud_[i].num_occupied_)>::max())
                {
                    cloud_[i].num_occupied_ /= 2;
                    cloud_[i].num_empty_ /= 2;
                }
                ++cloud_[i].num_occupied_;
                ++n_occupied;
            }
            else
            {
                // Known surface seen through,
                // indicating it may be noise or it moved somewhere else.
                if (cloud_[i].num_empty_ == std::numeric_limits<decltype(cloud_[i].num_empty_)>::max())
                {
                    cloud_[i].num_occupied_ /= 2;
                    cloud_[i].num_empty_ /= 2;
                }
                ++cloud_[i].num_empty_;
                ++n_empty;
            }
            // TODO: Change point status and add to dirty indices if needed.
            if (point_empty(cloud_[i]))
            {
                if (cloud_[i].flags_ & STATIC)
                {
                    cloud_[i].flags_ &= ~STATIC;
                    dirty_indices_.insert(i);
                    ++n_modified;
                }
            }
            else
            {
                if (!(cloud_[i].flags_ & STATIC))
                {
                    cloud_[i].flags_ |= STATIC;
                    dirty_indices_.insert(i);
                    ++n_modified;
                }
            }
        }
        ROS_INFO("Occupancy of %lu points updated, %lu modified state, %lu above, %lu occupied, %lu empty (%.6f s).",
                 q_map.nn_[0].size(), size_t(n_modified), size_t(n_above), size_t(n_occupied), size_t(n_empty),
                 t_part.seconds_elapsed());
        ROS_INFO("Occupancy updated (%.3f s).", t.seconds_elapsed());
    }

    void update_occupancy_projection(const sensor_msgs::PointCloud2& cloud,
                                     const geometry_msgs::Transform& cloud_to_map_tf)
    {
        Timer t;
        Timer t_part;

        if (empty())
        {
            ROS_INFO("No map points for updating occupancy.");
            return;
        }

        assert(cloud.height > 1);
        size_t n_pts = cloud.height * cloud.width;
        if (n_pts == 0)
        {
            ROS_INFO("No input points for updating occupancy.");
            return;
        }

        t_part.reset();
        SphericalProjection model;
        if (!model.fit(cloud))
        {
            ROS_WARN("Could not fit cloud model (%.6f s).", t_part.seconds_elapsed());
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
        Eigen::Isometry3f cloud_to_map(tf2::transformToEigen(cloud_to_map_tf));
        Eigen::Isometry3f map_to_cloud = cloud_to_map.inverse(Eigen::Isometry);

        // Find closest map points to sensor origin.
        Lock cloud_lock(cloud_mutex_);
        Lock index_lock(index_mutex_);
        t_part.reset();
        Vec3 origin = cloud_to_map.translation();
        RadiusQuery<Value> q_map(*index_, FlannMat(origin.data(), 1, 3), 5.f);
        ROS_DEBUG("%lu closest map points found (%.6f s).", q_map.nn_[0].size(), t_part.seconds_elapsed());

        Lock removed_lock(updated_mutex_);
        Lock dirty_lock(dirty_mutex_);
        // Test each map point nearby, whether we can see through.
        Index n_occupied = 0;
        Index n_empty = 0;
        Index n_occluded = 0;
        Index n_modified = 0;
        const Value eps = points_min_dist_ / 2;
        for (const auto i: q_map.nn_[0])
        {
            cloud_[i].dist_to_plane_ = std::numeric_limits<Value>::quiet_NaN();
            Vec3 p = map_to_cloud * ConstVec3Map(cloud_[i].position_);
            Value r, c;
            model.project(p(0), p(1), p(2), r, c);

            if (r < 1 || r > cloud.height - 2)
                continue;
            int r0 = int(std::round(r));

            if (c < 1 || c > cloud.width - 2)
                continue;
            int c0 = int(std::round(c));

            Index i0 = r0 * cloud.width + c0;

            Value rint;
            Index i1 = std::modf(r, &rint) >= Value(0)
                       ? (r0 + 1) * cloud.width + c0
                       : (r0 - 1) * cloud.width + c0;

            Value cint;
            Index i2 = std::modf(c, &cint) >= Value(0)
                       ? r0 * cloud.width + c0 + 1
                       : r0 * cloud.width + c0 - 1;

            ConstVec3Map p0(&(x_begin + i0)[0]);
            if (!std::isfinite(p0(0)) || !std::isfinite(p0(1)) || !std::isfinite(p0(2)))
                continue;
            ConstVec3Map p1(&(x_begin + i1)[0]);
            if (!std::isfinite(p1(0)) || !std::isfinite(p1(1)) || !std::isfinite(p1(2)))
                continue;
            ConstVec3Map p2(&(x_begin + i2)[0]);
            if (!std::isfinite(p2(0)) || !std::isfinite(p2(1)) || !std::isfinite(p2(2)))
                continue;
            Vec4 plane = plane_from_points(p0, p1, p2);

            // Make sure positive distance is towards sensor at [0, 0, 0],
            // i.e., outside from surface.
            if (plane(3) < 0.)
                plane *= -1;

            Value signed_dist = plane.dot(e2p(p));
            cloud_[i].dist_to_plane_ = signed_dist;

            if (signed_dist < -eps)
            {
                // Map point is occluded. Do nothing.
                ++n_occluded;
                continue;
            }

            if (signed_dist < eps)
            {
                // Known surface measured again.
                // TODO: Param max occupancy sample size.
//                if (cloud_[i].num_occupied_ == std::numeric_limits<decltype(cloud_[i].num_occupied_)>::max())
                if (cloud_[i].num_occupied_ >= max_occ_counter_)
                {
                    cloud_[i].num_occupied_ /= 2;
                    cloud_[i].num_empty_ /= 2;
                }
                ++cloud_[i].num_occupied_;
                ++n_occupied;
            }
            else
            {
                // Known surface seen through,
                // indicating it may be noise or it moved somewhere else.

                // Check the incidence angle is not too high.
                Vec3 n = plane.head(3);
                const auto abs_cos = std::abs(p0.normalized().dot(n));
                if (abs_cos < min_empty_cos_)
                {
                    continue;
                }

                if (cloud_[i].num_empty_ >= max_occ_counter_)
                {
                    cloud_[i].num_occupied_ /= 2;
                    cloud_[i].num_empty_ /= 2;
                }
                ++cloud_[i].num_empty_;
                ++n_empty;
            }

            // TODO: Change point status and add to dirty indices if needed.
            if (point_empty(cloud_[i]))
            {
                if (cloud_[i].flags_ & STATIC)
                {
                    cloud_[i].flags_ &= ~STATIC;
                    // TODO: Change position to NaN to remove it from visualization?
//                    cloud_[i].position_[0] = std::numeric_limits<Value>::quiet_NaN();
//                    cloud_[i].position_[1] = std::numeric_limits<Value>::quiet_NaN();
//                    cloud_[i].position_[2] = std::numeric_limits<Value>::quiet_NaN();
                    // Don't update the point we remove.
                    dirty_indices_.erase(i);
                    // TODO: Add neighborhood to dirty.
                    for (const auto j: graph_[i].neighbors_)
                    {
                        // Don't add removed points.
                        if (!(cloud_[j].flags_ & STATIC))
                        {
                            continue;
                        }
                        dirty_indices_.insert(j);
                    }
                    index_->removePoint(i);
                    updated_indices_.push_back(i);
                    ++n_modified;
                }
            }
//            else
//            {
//                if (false && !(cloud_[i].flags_ & STATIC))
//                {
//                    // Don't resurrect removed points.
//                    if (!std::isfinite(cloud_[i].position_[0]))
//                    {
//                        continue;
//                    }
//                    cloud_[i].flags_ |= STATIC;
//                    dirty_indices_.insert(i);
//                    // TODO: Add neighborhood to dirty.
//                    for (const auto j: graph_[i].neighbors_)
//                    {
//                        // Don't add removed points.
//                        if (!(cloud_[j].flags_ & STATIC))
//                        {
//                            continue;
//                        }
//                        dirty_indices_.insert(j);
//                    }
//                    ++n_modified;
//                }
//            }
        }
        ROS_INFO("Occupancy of %lu points updated, %lu modified state, %lu occluded, %lu occupied, %lu empty (%.6f s).",
                 q_map.nn_[0].size(), size_t(n_modified), size_t(n_occluded), size_t(n_occupied), size_t(n_empty),
                 t_part.seconds_elapsed());
    }

    void update_occupancy_unorganized(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
        // TODO: Update static (hit) / dynamic (see through) points.
        Timer t_dyn;
        // Compute input points directions.
        ConstVec3Map x_origin(origin[0]);
        Buffer<Value> dirs_buf(points.rows * points.cols);
        FlannMat dirs(dirs_buf.begin(), points.rows, points.cols);
        for (Index i = 0; i < points.rows; ++i) {
            Vec3Map dir(dirs[i]);
            dir = ConstVec3Map(points[i]) - x_origin;
            const Elem norm = dir.norm();
            if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3) {
                ROS_INFO_THROTTLE(1.0,
                                  "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
                                  dir.x(), dir.y(), dir.z(), norm);
                dir = Vec3::Zero();
            }
            else {
                dir /= norm;
            }
        }
        // Create index of view directions.
        FlannIndex dirs_index(dirs, flann::KDTreeSingleIndexParams());
        dirs_index.buildIndex();

        // Get map points nearby to check.
        Elem nearby_dist = 25.;
        RadiusQuery<Value> q_nearby(*index_, origin, nearby_dist);
        Index n_nearby = q_nearby.nn_[0].size();
        ROS_DEBUG("Found %u points up to %.1f m from sensor (%.3f s).",
                  n_nearby, nearby_dist, t_dyn.seconds_elapsed());
//            Buffer<Index> occupied_buf(n_nearby);
//            Buffer<Index> empty_buf(n_nearby);
        Buffer<Value> nearby_dirs_buf(3 * n_nearby);
        FlannMat nearby_dirs(nearby_dirs_buf.begin(), n_nearby, 3);
        for (Index i = 0; i < n_nearby; ++i) {
            const Index v0 = q_nearby.nn_[0][i];
            Vec3Map dir(nearby_dirs[i]);
//                dir = ConstVec3Map(points_[v0]) - x_origin;
            dir = ConstVec3Map(&cloud_[v0].position_[0]) - x_origin;
            const Elem norm = dir.norm();
            if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3) {
                ROS_INFO_THROTTLE(1.0,
                                  "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
                                  dir.x(), dir.y(), dir.z(), norm);
                dir = Vec3::Zero();
            }
            else {
                dir /= norm;
            }
        }
        int k_dirs = 5;  // Number of closest rays to check.
        uint16_t max_observations = 15;
        Query<Value> q_dirs(dirs_index, nearby_dirs, k_dirs);
        for (Index i = 0; i < n_nearby; ++i) {
            const Index v_map = q_nearby.nn_[0][i];
//                ConstVec3Map x_map(points_[v_map]);
//                uint16_t empty_orig = empty_buf_[v_map];
//                ConstVec3Map x_map(&cloud_[v_map].position_[0]);
//                auto x_map = position_vector(v_map);
            ConstVec3Map x_map(position(v_map));
//            auto empty_orig = cloud_[v_map].empty_;
            for (Index j = 0; j < k_dirs; ++j) {
                const Index v_new = q_dirs.nn_[i][j];
                // TODO: Avoid segfault v_new > size?
                // Discarding small clouds seems to do the trick?
                if (v_new >= points.rows) {
                    ROS_WARN_THROTTLE(1.0,
                                      "Skipping invalid NN (%u >= %lu) during merge.",
                                      v_new,
                                      points.rows);
                    continue;
                }
                ConstVec3Map x_new(points[v_new]);
                // Match close neighbors as a single static object.
                const Elem d = (x_new - x_map).norm();
                if (d <= points_min_dist_)
                {
                    // Static / occupied.
                    if (cloud_[v_map].num_occupied_ >= max_observations)
                    {
                        cloud_[v_map].num_occupied_ /= 2;
                        cloud_[v_map].num_empty_ /= 2;
                    }
                    ++cloud_[v_map].num_occupied_;
                    // Clear empty marks from this cloud.
                    // TODO: Why would we?
//                        empty_buf_[v_map] = empty_orig;
                    break;
                }
                // TODO: Discard rays far from the map points? What?
                const Value d_new = (x_new - x_origin).norm();
                if (std::isnan(d_new) || std::isinf(d_new) || d_new < 1e-3) {
                    ROS_WARN_THROTTLE(1.0,
                                      "Discarding invalid point [%.1g, %.1g, %.1g], distance to origin %.1g.",
                                      x_new.x(),
                                      x_new.y(),
                                      x_new.z(),
                                      d_new);
                    continue;
                }
                const Elem line_dist = (x_new - x_origin).cross(x_origin - x_map).norm() / d_new;
//                if (line_dist / 2. > points_min_dist_)
                if (line_dist > points_min_dist_ / 2)
                {
                    continue;
                }
                // Does farther point see through the map surface?
                const Elem d_map = (x_map - x_origin).norm();
                // TODO: To be usable as this normal must have correct orientation.
//                    ConstVec3Map n_map(normals_[v_map]);
                ConstVec3Map n_map(normal(v_map));
//                    Elem level = n_map.dot(x_new - x_map);
//                    if (d_new > d_map && level < 0.)
                Elem cos = n_map.dot(ConstVec3Map(dirs[v_new]));
                if (d_new > d_map && std::abs(cos) > min_empty_cos_) {
                    // Dynamic / see through.
                    if (cloud_[v_map].num_empty_ >= max_observations) {
                        cloud_[v_map].num_occupied_ /= 2;
                        cloud_[v_map].num_empty_ /= 2;
                    }
                    ++cloud_[v_map].num_empty_;
                }
            }
//                ROS_INFO("Point [%.1f, %.1f, %.1f]: %u occupied, %u empty.",
//                        x_map.x(), x_map.y(), x_map.z(), occupied_buf_[v_map], empty_buf_[v_map]);
//                if (empty_buf_[v_map] / occupied_buf_[v_map] >= 4)
//            if (float(cloud_[v_map].num_empty_) / cloud_[v_map].num_occupied_ >= empty_ratio_) {
//                // TODO: Remove point?
//                // Handle this in processing NN.
//            }
            if (point_empty(cloud_[v_map]))
            {
                if (cloud_[v_map].flags_ & STATIC)
                {
                    cloud_[v_map].flags_ &= ~STATIC;
                }
            }
            else
            {
                if (!(cloud_[v_map].flags_ & STATIC))
                {
                    cloud_[v_map].flags_ |= STATIC;
                }
            }
        }
        ROS_DEBUG("Checking and updating %u static/dynamic points nearby: %.3f s.",
                  n_nearby,
                  t_dyn.seconds_elapsed());
    }

    void initialize(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
//        ROS_INFO("Point size: %lu bytes.", sizeof(Point));
//        ROS_INFO("Neighborhood size: %lu bytes.", sizeof(Neighborhood));
        Lock cloud_lock(cloud_mutex_);
        assert(cloud_.empty());
        Lock index_lock(index_mutex_);
        Lock dirty_lock(dirty_mutex_);
        Timer t;
        for (size_t i = 0; i < points.rows; ++i)
        {
            dirty_indices_.insert(static_cast<Index>(cloud_.size()));

            Point point;
            std::copy(points[i], points[i] + points.cols, point.position_);
            point.flags_ |= STATIC;
//            point.flags_ &= ~UPDATED;
//            cloud_.emplace_back(point);
            cloud_.push_back(point);

            Neighborhood neigh;
            std::copy(points[i], points[i] + points.cols, neigh.position_);
//            neigh.neighbor_count_ = 0;
//            graph_.emplace_back(neigh);
            graph_.push_back(neigh);
        }
        assert(cloud_.size() == points.rows);
        ROS_DEBUG("%lu dirty indices.", dirty_indices_.size());
//        ROS_INFO("Map initialized with %lu points.", points.rows);
        update_index();
//        update_dirty();
        ROS_INFO("Map initialized with %lu points (%.3f s).",
                 points.rows,
                 t.seconds_elapsed());
    }

//        void merge(flann::Matrix<Elem> points, flann::Matrix<Elem> origin)
    void merge(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
        Timer t;
        ROS_DEBUG("Merging cloud started. Capacity %lu points.", capacity());
        // TODO: Forbid allocation while in use or lock?

        if (empty())
        {
            initialize(points, origin);
            ROS_INFO("Map initialized with %lu points (%.3f s).",
                     points.rows, t.seconds_elapsed());
            return;
        }

        // TODO: Switch to organized occupancy updates.
        // TODO: Flag dirty points also based on occupancy updates.
        // Move it in a calling method where we have cloud structure.
//        update_occupancy_unorganized(points, origin);

        // Find NN distance within current map.
//        Lock index_lock(index_mutex_);
        Lock cloud_lock(cloud_mutex_);
        Lock index_lock(index_mutex_);
        Query<Elem> q(*index_, points, Neighborhood::K_NEIGHBORS, neighborhood_radius_);
        ROS_DEBUG("Got neighbors for %lu points (%.3f s).",
                  points.rows,
                  t.seconds_elapsed());

        // Merge points with distance to NN higher than threshold.
        // We'll assume that input points also (approx.) comply to the
        // same min. distance threshold, so each added point can be tested
        // separately.
        Lock added_lock(updated_mutex_);
        Lock dirty_lock(dirty_mutex_);
        Index start = static_cast<Index>(size());
        for (Index i = 0; i < points.rows; ++i)
        {
            // Collect dirty indices.
            bool add = true;
            for (Index j = 0; j < q.dist_.cols; ++j)
            {
                // Halt once all valid neighbors have been processed.
                // Relevant for radius search with a limit on the num. of neighbors.
                if (!valid_neighbor(q.nn_[i][j], q.dist_[i][j]))
                {
                    break;
                }
                // Neglect points which are not static (or, removed from map).
                if (!(cloud_[q.nn_[i][j]].flags_ & STATIC))
                {
                    continue;
                }
                // For ordered distances we should encounter minimum first.
                // Don't add the point if it is too close to other points.
                if (q.dist_[i][j] < points_min_dist_ * points_min_dist_)
                {
                    add = false;
                    break;
                }
                // Points beyond neighborhood radius are not affected.
                if (q.dist_[i][j] > neighborhood_radius_ * neighborhood_radius_)
                {
                    break;
                }
                // A neighbor of added point within specified distance.
                dirty_indices_.insert(q.nn_[i][j]);
            }
            if (add)
            {
                updated_indices_.push_back(cloud_.size());
                dirty_indices_.insert(cloud_.size());

                Point point;
                std::copy(points[i], points[i] + points.cols, point.position_);
                // Start with as static?
                // TODO: Or only increment occupied flag?
                point.flags_ |= STATIC;
                cloud_.push_back(point);

                Neighborhood neigh;
                std::copy(points[i], points[i] + points.cols, neigh.position_);
                graph_.push_back(neigh);
            }
        }

        // TODO: Rebuild index time to time, don't wait till it doubles in size.
        float rebuild_threshold = (index_->size() + 1000.f) / index_->size();
        index_->addPoints(position_matrix(start), rebuild_threshold);

        ROS_INFO("%lu points merged into map with %lu points (%.3f s).",
                 size_t(size() - start),
                 size_t(size()),
                 t.seconds_elapsed());
    }

    void initialize_cloud(sensor_msgs::PointCloud2& cloud)
    {
        cloud.point_step = uint32_t(offsetof(Point, position_));
        append_field<decltype(Point().position_[0])>("x", 1, cloud);
        append_field<decltype(Point().position_[0])>("y", 1, cloud);
        append_field<decltype(Point().position_[0])>("z", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, normal_));
        append_field<decltype(Point().normal_[0])>("normal_x", 1, cloud);
        append_field<decltype(Point().normal_[0])>("normal_y", 1, cloud);
        append_field<decltype(Point().normal_[0])>("normal_z", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, normal_support_));
        append_field<decltype(Point().normal_support_)>("normal_support", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, ground_diff_std_));
        append_field<decltype(Point().ground_diff_std_)>("ground_diff_std", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, min_ground_diff_));
        append_field<decltype(Point().min_ground_diff_)>("min_ground_diff", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, max_ground_diff_));
        append_field<decltype(Point().max_ground_diff_)>("max_ground_diff", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, mean_abs_ground_diff_));
        append_field<decltype(Point().mean_abs_ground_diff_)>("mean_abs_ground_diff", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, viewpoint_));
        append_field<decltype(Point().viewpoint_[0])>("viewpoint_x", 1, cloud);
        append_field<decltype(Point().viewpoint_[0])>("viewpoint_y", 1, cloud);
        append_field<decltype(Point().viewpoint_[0])>("viewpoint_z", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, dist_to_actor_));
        append_field<decltype(Point().dist_to_actor_)>("dist_to_actor", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, actor_last_visit_));
        append_field<decltype(Point().actor_last_visit_)>("actor_last_visit", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, dist_to_other_actors_));
        append_field<decltype(Point().dist_to_other_actors_)>("dist_to_other_actors", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, other_actors_last_visit_));
        append_field<decltype(Point().other_actors_last_visit_)>("other_actors_last_visit", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, dist_to_obstacle_));
        append_field<decltype(Point().dist_to_obstacle_)>("dist_to_obstacle", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, flags_));
        append_field<decltype(Point().flags_)>("flags", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, num_empty_));
        append_field<decltype(Point().num_empty_)>("num_empty", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, num_occupied_));
        append_field<decltype(Point().num_occupied_)>("num_occupied", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, dist_to_plane_));
        append_field<decltype(Point().dist_to_plane_)>("dist_to_plane", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, num_obstacle_pts_));
        append_field<decltype(Point().num_obstacle_pts_)>("num_obstacle_pts", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, num_edge_neighbors_));
        append_field<decltype(Point().num_edge_neighbors_)>("num_edge_neighbors", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, path_cost_));
        append_field<decltype(Point().path_cost_)>("path_cost", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, reward_));
        append_field<decltype(Point().reward_)>("reward", 1, cloud);

        cloud.point_step = uint32_t(offsetof(Point, relative_cost_));
        append_field<decltype(Point().relative_cost_)>("relative_cost", 1, cloud);

        cloud.point_step = uint32_t(sizeof(Point));
    }

    void create_cloud_msg(sensor_msgs::PointCloud2& cloud)
    {
        initialize_cloud(cloud);
        Lock cloud_lock(cloud_mutex_);
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.resize(cloud_.size());
//        std::copy(&cloud_.front(), &cloud_.back(), &cloud.data.front());
//        std::copy(cloud_.begin(), cloud_.end(), cloud.data.begin());
//        std::copy(cloud_.begin(), cloud_.end(), reinterpret_cast<Point*>(&cloud.data[0]));
        const auto from = reinterpret_cast<const uint8_t*>(cloud_.data());
        const auto to = reinterpret_cast<const uint8_t*>(cloud_.data() + cloud_.size());
        auto out = cloud.data.data();
        assert((to - from) == cloud.data.size());
        std::copy(from, to, out);
    }

    template<typename C>
    void create_cloud_msg(const C& indices, sensor_msgs::PointCloud2& cloud)
    {
        initialize_cloud(cloud);
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.resize(indices.size());
//        auto it = indices.begin();
        auto out = cloud.data.data();
//        for (Index i = 0; i < n; ++i, ++it, out += cloud.point_step)
//        {
//            const auto from = reinterpret_cast<uint8_t*>(&cloud_[*it]);
//            std::copy(from, from + cloud.point_step, out);
//        }
        for (auto it = indices.begin(); it != indices.end(); ++it, out += cloud.point_step)
        {
            const auto from = reinterpret_cast<uint8_t*>(&cloud_[*it]);
            std::copy(from, from + cloud.point_step, out);
        }
    }

//    template<typename It>
//    void create_debug_cloud(It begin, Index n, sensor_msgs::PointCloud2& cloud)
//    {
//        initialize_cloud(cloud);
//        sensor_msgs::PointCloud2Modifier modifier(cloud);
//        modifier.resize(n);
//        It it = begin;
//        auto out = cloud.data.data();
//        for (Index i = 0; i < n; ++i, ++it, out += cloud.point_step)
//        {
//            const auto from = reinterpret_cast<uint8_t*>(&cloud_[*it]);
//            std::copy(from, from + cloud.point_step, out);
//        }
//    }

//    void create_cloud_msg(sensor_msgs::PointCloud2& cloud)
//    {
//        Timer t;
//        create_debug_cloud(cloud);
//        ROS_DEBUG("Creating cloud with %u points: %.3f s.",
//                  cloud.height * cloud.width, t.seconds_elapsed());
//    }

//    void create_dirty_cloud(sensor_msgs::PointCloud2& cloud)
//    {
//        Lock lock(dirty_mutex_);
//        Timer t;
//        create_debug_cloud(dirty_indices_.begin(), dirty_indices_.size(), cloud);
//        ROS_DEBUG("Creating dirty cloud with %u points: %.3f s.",
//                  cloud.height * cloud.width, t.seconds_elapsed());
//    }

    size_t capacity() const
    {
        Lock cloud_lock(cloud_mutex_);
        return cloud_.capacity();
    }

    size_t size() const
    {
        // TODO: Lock? Make thread safe?
        Lock cloud_lock(cloud_mutex_);
        return cloud_.size();
    }

    bool empty() const
    {
        Lock cloud_lock(cloud_mutex_);
        return cloud_.empty();
    }

    // Lock mutexes in this sequence
    // cloud_mutex_, index_mutex_, dirty_mutex_.
    // to avoid deadlocks.

    mutable Mutex cloud_mutex_;
    std::vector<Point> cloud_;
    std::vector<Neighborhood> graph_;

    mutable Mutex index_mutex_;
    std::shared_ptr<flann::Index<flann::L2_3D<Value>>> index_;

    mutable Mutex updated_mutex_;
    std::vector<Index> updated_indices_;

    mutable Mutex dirty_mutex_;
//    std::set<Index> dirty_indices_;
    std::unordered_set<Index> dirty_indices_;

    // Map parameters
    float points_min_dist_;
    // Occupancy
    float min_empty_cos_;
    int min_num_empty_;
    float min_empty_ratio_;
    int max_occ_counter_;

    // Graph
//    int neighbor
    float neighborhood_radius_;
//    float traversable_radius_;
    // Traversability
    float edge_min_centroid_offset_;
//    float max_nn_height_diff_;
    float min_ground_diff_;
    float max_ground_diff_;
    float clearance_low_;
    float clearance_high_;
    float clearance_radius_;
    float min_points_obstacle_;
    float max_ground_diff_std_;
    float max_mean_abs_ground_diff_;
    float min_dist_to_obstacle_;
    float max_pitch_;
    float max_roll_;
    float inclination_penalty_;
};

/** https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html */
class Graph
{
public:
    Graph(const Map& map):
        map_(map)
    {}
    inline Vertex num_vertices() const
    {
        return map_.num_vertices();
    }
    /** Returns the number of edges in the graph g. */
    inline Edge num_edges() const
    {
        // TODO: Compute true number based on valid neighbors.
        return map_.num_edges();
    }
    inline std::pair<VertexIter, VertexIter> vertices() const
    {
        return map_.vertices();
    }
    inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
    {
        // TODO: Limit to valid edges here or just by costs?
        return map_.out_edges(u);
    }
    inline Edge out_degree(const Vertex& u) const
    {
        // TODO: Compute true number based on valid neighbors.
        return map_.out_degree(u);
    }
    inline Vertex source(const Edge& e) const
    {
        return map_.source(e);
    }
    inline Vertex target_index(const Edge& e) const
    {
        return map_.target_index(e);
    }
    inline Vertex target(const Edge& e) const
    {
        return map_.target(e);
    }
    const Map& map_;
};

class EdgeCosts
{
public:
    EdgeCosts(const Map& map)
        :
        map_(map)
    {
    }
    inline Cost operator[](const Edge& e) const
    {
        return map_.edge_cost(e);
    }
private:
    const Map& map_;
};

}  // namespace naex

#endif  // NAEX_MAP_H
