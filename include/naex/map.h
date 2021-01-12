#ifndef NAEX_MAP_H
#define NAEX_MAP_H

#include <cstddef>
#include <mutex>
#include <naex/buffer.h>
#include <naex/clouds.h>
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

    Map()
        :
        index_(),
        dirty_indices_(),
        // Parameter defaults
        points_min_dist_(.2f),
        min_empty_cos_(.3f),
        // Occupancy
        empty_ratio_(2.f),
        neighborhood_radius_(.6f),
        traversable_radius_(neighborhood_radius_),
        edge_min_centroid_offset_(.75f),
        // Traversability
//        max_nn_height_diff_(.15),
        min_ground_diff_(-.1),
        max_ground_diff_(.3),
        clearance_low_(.15),
        clearance_high_(.8),
        min_points_obstacle_(3),
        max_ground_diff_std_(0.1),
        max_mean_abs_ground_diff_(0.1),
        min_dist_to_obstacle_(0.7),
        max_pitch_(30. / 180. * M_PI),
        max_roll_(30. / 180. * M_PI)

    {
//        dirty_indices_.reser
        dirty_indices_.reserve(100000);
        cloud_.reserve(DEFAULT_CAPACITY);
    }

    flann::Matrix<Value> position_matrix()
    {
        if (cloud_.empty()) {
            return FlannMat();
        }
        return flann::Matrix<Value>(cloud_[0].position_, cloud_.size(), 3, sizeof(Point));
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

    bool point_empty(Index i)
    {
        return float(cloud_[i].num_empty_) / cloud_[i].num_occupied_ >= empty_ratio_;
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
        if (!(cloud_[v1].flags_ & TRAVERSABLE))
        {
            return std::numeric_limits<Cost>::infinity();
        }
//        Cost d = std::sqrt(cloud_[v0].distances_[v1_index]);
        Cost d = std::sqrt(graph_[v0].distances_[v1_index]);
        if (d > traversable_radius_)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        Elem height_diff = cloud_[v1].position_[2] - cloud_[v0].position_[2];
        Elem inclination = std::asin(std::abs(height_diff) / d);
        if (inclination > max_pitch_)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        // TODO: Check pitch and roll separately.
        /*
        Elem pitch0 = std::acos(normals_[v0][2]);
        if (pitch0 > max_pitch_)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        Elem pitch1 = std::acos(normals_[v1][2]);
        if (pitch1 > max_pitch_)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        float roll0 = 0.f;
        float roll1 = 0.f;
        */
        // Initialize with distance computed in NN search.
        // Multiple with relative pitch and roll.
//            d *= (1.f + (pitch0 + pitch1 + inclination) / 3.f / max_pitch_ + (roll0 + roll1) / 2.f / max_roll_);
        d *= (1.f + (inclination / max_pitch_));
//            std::cout << v0 << " -> " << v1 << ": " << d << std::endl;
        // TODO: Make distance correspond to expected travel time.
        return d;
    }

//    Cost edge_cost(Index e)
//    {
//        return 0.f;
//    }

    void update_index()
    {
        Timer t;
        // TODO: Update only dirty points.
        if (!empty()) {
            Lock lock(index_mutex_);
//                index_ = std::make_shared<flann::Index<flann::L2_3D<Elem>>>(points_, flann::KDTreeSingleIndexParams());
            index_ = std::make_shared<flann::Index<flann::L2_3D<Elem>>>(
                position_matrix(),
                flann::KDTreeSingleIndexParams());
            // TODO: Is this necessary?
//                index_->buildIndex();
            ROS_INFO("Updating index: %.3f s.", t.seconds_elapsed());
        }
    }

    /**
     * Assign point labels based on normal orientation.
     */
    template<typename It>
//    void compute_normal_labels(const std::vector<Index>& indices)
    void compute_normal_labels(It begin, It end)
    {
        Timer t;
        // Maximum slope allowed in some direction.
        auto max_slope = std::max(max_pitch_, max_roll_);
        auto min_z = std::cos(max_slope);
        Index n_traverable = 0;
        Index n_obstacle = 0;
//        for (Index i = 0; i < size(); ++i)
//        for (auto it = indices.begin(); it != indices.end(); ++it)
//        Index n = 0;
        for (It it = begin; it != end; ++it)
        {
            const auto i = *it;
            // TODO: Check sign once normals keep consistent orientation.
            if (std::abs(cloud_[i].normal_[2]) >= min_z)
            {
                // Approx. horizontal based on normal.
//                cloud_[i].normal_label_ = TRAVERSABLE;
                cloud_[i].flags_ |= HORIZONTAL;
                ++n_traverable;
            }
            else  // if (std::abs(normals_[i][2]) < min_z)
            {
                // Approx. vertical based on normal.
//                cloud_[i].normal_label_ = OBSTACLE;
                cloud_[i].flags_ &= ~HORIZONTAL;
                ++n_obstacle;
            }
        }
        ROS_INFO("Normal labels of %lu pts: %u traversable, %u obstacle (%.3f s).",
                 n_traverable + n_obstacle, n_traverable, n_obstacle, t.seconds_elapsed());
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
    void update_graph(It begin, It end)
    {
        // NB: It should be stable iteration order.
        Timer t;

//        class GraphPoint
//        {
//        public:
//            Value position_[3];
//            // Number of valid entries in neighbors_ and distances_.
//            Index neighbor_count_;
//            int neighbors_[Point::K_NEIGHBORS];
//            Value distances_[Point::K_NEIGHBORS];
//        };

        // Collect points to updated.
        // TODO: Append to a list on the fly for faster collection here?
//        std::vector<Index> dirty_indices;
//        std::vector<Neighborhood> dirty_cloud;
//        dirty_indices.reserve(cloud_.size());
//        dirty_cloud.reserve(cloud_.size());
//        for (Index i = 0; i < cloud_.size(); ++i) {
//            if (!(cloud_[i].flags_ & UPDATED)) {
//                dirty_indices.push_back(i);
//                Neighborhood neighborhood;
//                std::copy(cloud_[i].position_,
//                          cloud_[i].position_ + 3,
//                          neighborhood.position_);
//                dirty_cloud.emplace_back(neighborhood);
//            }
//        }

//        std::vector<Index> dirty_indices;
        std::vector<Neighborhood> dirty_cloud;
//        dirty_indices.reserve(cloud_.size());
//        dirty_cloud.reserve(end - begin);
        for (It it = begin; it != end; ++it)
        {
            Neighborhood neighborhood;
            std::copy(cloud_[*it].position_,
                      cloud_[*it].position_ + 3,
                      neighborhood.position_);
            dirty_cloud.emplace_back(neighborhood);
        }

        if (dirty_cloud.empty())
        {
            ROS_INFO("Graph up to date (no points to update): %.3f s.",
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

        flann::SearchParams params;
        params.checks = 64;
        params.cores = 0;
//            params.max_neighbors = k;
//            points_index_.radiusSearch(points_, nn_, dist_, radius, params);
        {
            Lock lock(index_mutex_);
            index_->knnSearch(positions,
                              neighbors,
                              distances,
                              Neighborhood::K_NEIGHBORS,
                              params);
        }
        // Propagate NN info into the main graph.
//        for (Index i = 0; i < dirty_indices.size(); ++i)
//        {
//            std::copy(dirty_cloud[i].neighbors_,
//                      dirty_cloud[i].neighbors_ + Neighborhood::K_NEIGHBORS,
//                      graph_[dirty_indices[i]].neighbors_);
//            std::copy(dirty_cloud[i].distances_,
//                      dirty_cloud[i].distances_ + Neighborhood::K_NEIGHBORS,
//                      graph_[dirty_indices[i]].distances_);
//        }
        auto point_it = dirty_cloud.begin();
        for (It it = begin; it != end; ++it, ++point_it)
        {
            std::copy((*point_it).neighbors_,
                      (*point_it).neighbors_ + Neighborhood::K_NEIGHBORS,
                      graph_[*it].neighbors_);
            std::copy((*point_it).distances_,
                      (*point_it).distances_ + Neighborhood::K_NEIGHBORS,
                      graph_[*it].distances_);
        }

        ROS_INFO("Updating NN graph at %lu / %lu pts: %.3f s.",
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
//        return { u * num_edges(), (u + 1) * num_edges() };
        return { u * Neighborhood::K_NEIGHBORS, (u + 1) * Neighborhood::K_NEIGHBORS };
    }
    inline Edge out_degree(const Vertex& u) const
    {
        // TODO: Compute true number based on valid neighbors.
//        return num_edges();
        return Neighborhood::K_NEIGHBORS;
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
//        return nn_[source(e)][target_index(e)];
//        return cloud_[source(e)].neighbors_[target_index(e)];
        return graph_[source(e)].neighbors_[target_index(e)];
    }

    void update_dirty()
    {
        Timer t;
        update_graph(dirty_indices_.begin(), dirty_indices_.end());
        update_features(dirty_indices_.begin(), dirty_indices_.end());
        compute_normal_labels(dirty_indices_.begin(), dirty_indices_.end());
        compute_final_labels(dirty_indices_.begin(), dirty_indices_.end());
        const auto n = dirty_indices_.size();
        dirty_indices_.clear();
        ROS_INFO("%lu points updated (%.3f s).", n, t.seconds_elapsed());
    }

    template<typename It>
    void update_features(It begin, It end)
    {
        const Value semicircle_centroid_offset =
            4.f * neighborhood_radius_ / (3.f * M_PIf32);;
        auto radius2 = neighborhood_radius_ * neighborhood_radius_;
        // Update dirty points.
        // TODO: Use a precomputed index list.
//        for (Index i = 0; i < cloud_.size(); ++i)
//        for (auto it = indicesIndex i = 0; i < cloud_.size(); ++i)
        for (It it = begin; it != end; ++it)
        {
            auto i = *it;
//            if (cloud_[i].index_state_ == UP_TO_DATE)
//            if (cloud_[i].flags_ & UPDATED)
//            {
//                continue;
//            }
//            assert(!(cloud_[i].index_state_ & UPDATED));

            //***
            // Disregard empty / dynamic points.
//            if (cloud_[i].functional_label_ == EMPTY)
            if (!(cloud_[i].flags_ & STATIC))
            {
                continue;
            }
            Vec3 mean = Vec3::Zero();
            Mat3 cov = Mat3::Zero();
//            num_normal_pts_[v0] = 0;
            cloud_[i].normal_support_ = 0;
            for (Index j = 0; j < Neighborhood::K_NEIGHBORS; ++j)
            {
                if (std::isinf(graph_[i].distances_[j]))
                {
                    continue;
                }
//            for (size_t j = 0; j < nn_.cols; ++j)
//            {
//                Index v1 = nn_[v0][j];
                Index k = graph_[i].neighbors_[j];
                // Disregard empty points.
//                if (cloud_[k].functional_label_ == EMPTY)
                if (!(cloud_[k].flags_ & STATIC))
                {
                    continue;
                }
                if (graph_[i].distances_[j] <= radius2) {
                    mean += ConstVec3Map(cloud_[k].position_);
//                    ++num_normal_pts_[v0];
//                    Vec3 pc = (ConstVec3Map(cloud_[v1].position_) - mean);
                    Vec3 pc = (ConstVec3Map(cloud_[k].position_) - ConstVec3Map(cloud_[i].position_));
                    cov += pc * pc.transpose();
                    ++cloud_[i].normal_support_;
                }
            }
//                if (n < min_normal_pts)
//                {
//                    continue;
//                }
//                mean /= n;
            mean /= cloud_[i].normal_support_;
            cov /= cloud_[i].normal_support_;
            Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
            Vec3Map normal(cloud_[i].normal_);
            normal = solver.eigenvectors().col(0);
            cloud_[i].ground_diff_std_ = std::sqrt(solver.eigenvalues()(0));
            // Inject support label here where we have computed mean.
            const auto centroid_offset = (mean - ConstVec3Map(cloud_[i].position_)).norm();
            if (centroid_offset > edge_min_centroid_offset_ * semicircle_centroid_offset)
            {
//                cloud_[i].functional_label_ = EDGE;
                cloud_[i].flags_ |= EDGE;
            }
            else
            {
                cloud_[i].flags_ &= ~EDGE;
            }
            // Index state is to be updated in update_graph.
//            cloud_[i].index_state_ = UP_TO_DATE;
        }
    }

    template<typename It>
    void compute_final_labels(It begin, It end)
    {
        Timer t;
        Index n = 0;
        Index n_traverable = 0;
        Index n_empty = 0;
        Index n_unknown = 0;
        Index n_edge = 0;
        Index n_actor = 0;
        Index n_obstacle = 0;
//        for (Vertex v0 = 0; v0 < nn_.rows; ++v0)
        for (It it = begin; it != end; ++it, ++n)
        {
            const auto v0 = *it;
//            num_edge_neighbors_[v0] = 0;
//            cloud_[v0].num
            // Disregard empty points.
//            if (point_empty(v0))
            if (!(cloud_[v0].flags_ & STATIC))
            {
                ++n_empty;
                continue;
            }
            for (Vertex j = 0; j < Neighborhood::K_NEIGHBORS; ++j)
            {
                const auto v1 = graph_[v0].neighbors_[j];
                if (cloud_[v1].flags_ & EDGE)
                {
                    ++cloud_[v0].num_edge_neighbors_;
                }
            }
            if (cloud_[v0].flags_ & EDGE)
            {
                ++n_edge;
                continue;
            }
            else if (cloud_[v0].flags_ & ACTOR)
            {
                ++n_actor;
                continue;
            }
            // Adjust only traversable points.
//                if (labels_[v0] != TRAVERSABLE)
//                {
//                    continue;
//                }
            // Compute ground features for all points.
//                Elem min_height_diff = std::numeric_limits<Elem>::infinity();
//                Elem max_height_diff = -std::numeric_limits<Elem>::infinity();
            cloud_[v0].min_ground_diff_ = std::numeric_limits<Elem>::infinity();
            cloud_[v0].max_ground_diff_ = -std::numeric_limits<Elem>::infinity();
            cloud_[v0].mean_abs_ground_diff_ = 0.;
            uint8_t n = 0;
//                uint8_t n_points_obstacle;
            cloud_[v0].num_obstacle_pts_ = 0;
            for (size_t j = 0; j < Neighborhood::K_NEIGHBORS; ++j)
            {
                const auto v1 = graph_[v0].neighbors_[j];
                // Disregard empty points.
                if (!(cloud_[v1].flags_ & STATIC))
                {
                    continue;
                }
                // Avoid driving near obstacles.
                // TODO: Use clearance as hard constraint and distance to obstacles as costs.
                // Hard constraint can be removed with min_dist_to_obstacle_.
                if (!(cloud_[v1].flags_ & HORIZONTAL)
                        && graph_[v0].distances_[j] <= min_dist_to_obstacle_)
                {
                    n_unknown += bool(cloud_[v0].flags_ &= TRAVERSABLE);
                    cloud_[v0].flags_ &= ~TRAVERSABLE;
//                    ++n_unknown;
//                        break;
                }
                Vec3Map n0(cloud_[v0].normal_);
                Vec3Map p0(cloud_[v1].position_);
                Vec3Map p1(cloud_[v1].position_);
                Value height_diff = n0.dot(p1 - p0);
                Vec3 ground_pt = p1 - height_diff * n0;
                Value ground_dist = (ground_pt - p0).norm();

                if (ground_dist <= traversable_radius_)
                {
                    cloud_[v0].min_ground_diff_ = std::min(cloud_[v0].min_ground_diff_, height_diff);
                    cloud_[v0].max_ground_diff_ = std::max(cloud_[v0].max_ground_diff_, height_diff);
                    cloud_[v0].mean_abs_ground_diff_ += std::abs(height_diff);
                    ++n;
                    if (height_diff >= clearance_low_ && height_diff <= clearance_high_)
                    {
                        ++cloud_[v0].num_obstacle_pts_;
                    }
                }
//                    ROS_INFO("height diff.: %.2f m, ground dist.: %.2f m", height_diff, ground_dist);
//                    if (ground_dist > radius_)
//                    {
//                        continue;
//                    }
//                    if (std::abs(height_diff) > max_nn_height_diff)
//                    if (max_height_diff - min_height_diff > max_nn_height_diff)
//                    {
//                        labels_[v0] = UNKNOWN;
//                        ++n_adjusted;
//                        break;
//                    }
            }
            cloud_[v0].mean_abs_ground_diff_  /= n;
            if ((cloud_[v0].flags_ & STATIC & HORIZONTAL)
                    && cloud_[v0].num_obstacle_pts_ < min_points_obstacle_
                    && cloud_[v0].min_ground_diff_ >= min_ground_diff_
                    && cloud_[v0].max_ground_diff_ <= max_ground_diff_
                    && cloud_[v0].ground_diff_std_ <= max_ground_diff_std_
                    && cloud_[v0].mean_abs_ground_diff_ <= max_mean_abs_ground_diff_)
            {
                ++n_traverable;
//                n_unknown += bool(cloud_[v0].flags_ &= TRAVERSABLE);
                cloud_[v0].flags_ |= TRAVERSABLE;
            }
            else
            {
                ++n_unknown;
                cloud_[v0].flags_ &= ~TRAVERSABLE;
            }
        }
        ROS_INFO("%lu final labels: %u traversable, %u empty, %u unknown, %u edge, %u actor, %u obstacle (%.3f s).",
                 n, n_traverable, n_empty, n_unknown, n_edge, n_actor, n_obstacle, t.seconds_elapsed());
    }

    /** Resize point buffers if necessary, update wrappers and index. */
    void reserve(size_t n)
    {
        Timer t;
        if (n < capacity()) {
            return;
        }
        cloud_.reserve(n);
        update_index();
        ROS_INFO("Capacity increased to %lu points: %.3f s.", n, t.seconds_elapsed());
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
                                  dir.x(),
                                  dir.y(),
                                  dir.z(),
                                  norm);
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
                                  dir.x(),
                                  dir.y(),
                                  dir.z(),
                                  norm);
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
                if (line_dist / 2. > points_min_dist_)
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
            if (float(cloud_[v_map].num_empty_) / cloud_[v_map].num_occupied_ >= empty_ratio_) {
                // TODO: Remove point?
                // Handle this in processing NN.
            }
            if (point_empty(v_map))
            {
                cloud_[v_map].flags_ &= ~STATIC;
            }
        }
        ROS_DEBUG("Checking and updating %u static/dynamic points nearby: %.3f s.",
                  n_nearby,
                  t_dyn.seconds_elapsed());
    }

    void initialize(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
        assert(cloud_.empty());
        Timer t;
        for (size_t i = 0; i < points.rows; ++i)
        {
            Point point;
            for (size_t j = 0; j < 3; ++j)
            {
                point.position_[j] = points[i][j];
            }
            point.flags_ &= ~UPDATED;
            dirty_indices_.insert(static_cast<Index>(cloud_.size()));
            cloud_.emplace_back(point);
        }
        assert(cloud_.size() == points.rows);
        update_index();
        update_dirty();
        ROS_INFO("Map initialized with %lu points (%.3f s).",
                 points.rows,
                 t.seconds_elapsed());
    }

//        void merge(flann::Matrix<Elem> points, flann::Matrix<Elem> origin)
    void merge(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
//            Lock lock(snapshot_mutex_);
        Lock lock(cloud_mutex_);
        Timer t;
        ROS_DEBUG("Merging cloud started. Capacity %lu points.", capacity());
        // TODO: Forbid allocation while in use or lock?
//        reserve(size() + points.rows);

        if (empty())
        {
            initialize(points, origin);
            return;
        }

        // TODO: Switch to organized occupancy updates.
        // Move it in a calling method where we have cloud structure.
//        update_occupancy_unorganized(points, origin);

        // Find NN distance within current map.
//        t.reset();
        Lock index_lock(index_mutex_);
//        Query<Elem> q(*index_, points, 1);
        Query<Elem> q(*index_, points, Neighborhood::K_NEIGHBORS, neighborhood_radius_);
        ROS_DEBUG("Got neighbors for %lu points (%.3f s).",
                  points.rows,
                  t.seconds_elapsed());

        // Merge points with distance to NN higher than threshold.
        // We'll assume that input points also (approx.) comply to the
        // same min. distance threshold, so each added point can be tested
        // separately.
        t.reset();
        Index n_added = 0;
        const auto min_dist_2 = points_min_dist_ * points_min_dist_;
        for (Index i = 0; i < points.rows; ++i)
        {
            // Skip the point if in case
//            if (invalid_index(q.dist_[i][0]) || invalid_distance(q.dist_[i][0]))
//            {
//                continue;
//            }
            // Skip points too close to solid points already in the map.
            // To check only the solid points, the SOLID flag must be updated
            // together with occupancy counters.
            // If other points are removed from the index, we don't have to
            // worry about that here.
//            if (!invalid_index(q.nn_[i][0])
//                    && !invalid_distance(q.dist_[i][0])
//                    && cloud_[q.nn_[i][0]]
//                    && q.dist_[i][0] < min_dist_2 && cloud_[])
//            {
//                continue;
//            }

            Point point;
            point.flags_ &= ~UPDATED;
            for (Index j = 0; j < 3; ++j)
            {
                point.position_[j] = points[i][j];
            }
            cloud_.emplace_back(point);
            ++n_added;

            // Collect dirty indices.
            bool added = true;
            for (Index j = 0; j < q.dist_.cols; ++j)
            {
                if (invalid_index(q.nn_[i][j]) || invalid_distance(q.dist_[i][j]))
                {
                    // All valid neighbors have been processed.
                    break;
                }
                if (!(cloud_[q.nn_[i][j]].flags_ & STATIC))
                {
                    // Neglect points which are not static.
                    continue;
                }
                // For ordered distances we encounter minimum first.
                if (q.dist_[i][j] < min_dist_2)
                {
                    added = false;
                    break;
                }
                dirty_indices_.insert(q.nn_[i][j]);
            }
            if (added)
            {
                Point point;
                std::copy(points[i], points[i] + points.cols, point.position_);
                point.flags_ &= ~UPDATED;
                cloud_.emplace_back(point);
                dirty_indices_.insert(cloud_.size() - 1);
                ++n_added;
            }
        }
//            mean /= points.rows;
//            ROS_INFO("Mean distance to map points: %.3f m.", mean);
        size_t n_map = size();
//            update_matrix_wrappers(size() + n_added);
        // TODO: flann::Index::addPoints (what indices? what with removed indices?)
//        update_index();

        ROS_INFO("%lu points merged into map with %lu points (%.3f s).",
                 n_added,
                 n_map,
                 t.seconds_elapsed());
    }

    void create_debug_cloud(sensor_msgs::PointCloud2& cloud)
    {
        cloud.point_step = uint32_t(offsetof(Point, position_));
        append_field<Value>("x", 1, cloud);
        append_field<Value>("y", 1, cloud);
        append_field<Value>("z", 1, cloud);
        cloud.point_step = uint32_t(offsetof(Point, normal_));
        append_field<Value>("normal_x", 1, cloud);
        append_field<Value>("normal_y", 1, cloud);
        append_field<Value>("normal_z", 1, cloud);
        cloud.point_step = sizeof(Point);
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.resize(cloud_.size());
//        std::copy(&cloud_.front(), &cloud_.back(), &cloud.data.front());
//        std::copy(cloud_.begin(), cloud_.end(), cloud.data.begin());
        std::copy(cloud_.begin(), cloud_.end(), reinterpret_cast<Point*>(&cloud.data[0]));
    }

    void create_cloud(sensor_msgs::PointCloud2& cloud)
    {
        create_debug_cloud(cloud);
    }

    size_t capacity() const
    {
        return cloud_.capacity();
    }

    size_t size() const
    {
        // TODO: Lock? Make thread safe?
        return cloud_.size();
    }

    bool empty() const
    {
        return cloud_.empty();
    }

    Mutex cloud_mutex_;
    std::vector<Point> cloud_;
    std::vector<Neighborhood> graph_;
    Mutex index_mutex_;
    std::shared_ptr<flann::Index<flann::L2_3D<Value>>> index_;

//    std::set<Index> dirty_indices_;
    std::unordered_set<Index> dirty_indices_;

    // Map parameters
    float points_min_dist_;
    // Occupancy
    float min_empty_cos_;
    float empty_ratio_;
    // Graph
    float neighborhood_radius_;
    float traversable_radius_;
    // Traversability
    float edge_min_centroid_offset_;
//    float max_nn_height_diff_;
    float min_ground_diff_;
    float max_ground_diff_;
    float clearance_low_;
    float clearance_high_;
    float min_points_obstacle_;
    float max_ground_diff_std_;
    float max_mean_abs_ground_diff_;
    float min_dist_to_obstacle_;
    float max_pitch_;
    float max_roll_;
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
//        return num_vertices() * Point::K_NEIGHBORS;
        return map_.num_edges();
    }
    inline std::pair<VertexIter, VertexIter> vertices() const
    {
//        return { 0, num_vertices() };
        return map_.vertices();
    }
    inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
    {
        // TODO: Limit to valid edges here or just by costs?
//        return { u * num_edges(), (u + 1) * num_edges() };
//        return { u * Point::K_NEIGHBORS, (u + 1) * Point::K_NEIGHBORS };
        return map_.out_edges(u);
    }
    inline Edge out_degree(const Vertex& u) const
    {
        // TODO: Compute true number based on valid neighbors.
//        return num_edges();
//        return Point::K_NEIGHBORS;
        return map_.out_degree(u);
    }
    inline Vertex source(const Edge& e) const
    {
//        return e / num_edges();
//        return e / Point::K_NEIGHBORS;
        return map_.source(e);
    }
    inline Vertex target_index(const Edge& e) const
    {
//        return e % num_edges();
//        return e % Point::K_NEIGHBORS;
        return map_.target_index(e);
    }
    inline Vertex target(const Edge& e) const
    {
//        return nn_[source(e)][target_index(e)];
//        return map_.cloud_[source(e)].neighbors_[target_index(e)];
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
