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
        // Parameter defaults
        points_min_dist_(.2f),
        min_empty_cos_(.3f),
        empty_ratio_(2.f),
        neighborhood_radius_(.6f),
        traversable_radius_(neighborhood_radius_),
        edge_min_centroid_offset_(.75f)
    {
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
        return flann::Matrix<int>(reinterpret_cast<int*>(cloud_[0].neighbors_),
                                  cloud_.size(),
                                  3,
                                  sizeof(Point));
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

    bool point_empty(Index i)
    {
        return float(cloud_[i].empty_) / cloud_[i].occupied_ >= empty_ratio_;
    }

    bool point_near_any(Index i, const std::vector<Value>& points, Value radius)
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
        if (cloud_[v1].functional_label_ != TRAVERSABLE)
        {
            return std::numeric_limits<Cost>::infinity();
        }
        Cost d = std::sqrt(cloud_[v0].distances_[v1_index]);
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
        for (It it = begin; it != end; ++it)
        {
            const auto i = *it;
            // TODO: Check sign once normals keep consistent orientation.
            if (std::abs(cloud_[i].normal_[i][2]) >= min_z)
            {
                // Approx. horizontal based on normal.
                cloud_[i].normal_label_ = TRAVERSABLE;
                ++n_traverable;
            }
            else  // if (std::abs(normals_[i][2]) < min_z)
            {
                // Approx. vertical based on normal.
                cloud_[i].normal_label_ = OBSTACLE;
                ++n_obstacle;
            }
        }
        ROS_INFO("Normal labels of %lu pts: %u traversable, %u obstacle (%.3f s).",
                 (end - begin), n_traverable, n_obstacle, t.seconds_elapsed());
    }

    void update_graph()
    {
        Timer t;

        class GraphPoint
        {
        public:
            Value position_[3];
            // Number of valid entries in neighbors_ and distances_.
            Index neighbor_count_;
            int neighbors_[Point::K_NEIGHBORS];
            Value distances_[Point::K_NEIGHBORS];
        };

        // Collect points to updated.
        // TODO: Append to a list on the fly for faster collection here?
        std::vector<Index> dirty_indices;
        std::vector<GraphPoint> dirty_cloud;
        dirty_indices.reserve(cloud_.size());
        dirty_cloud.reserve(cloud_.size());
        for (Index i = 0; i < cloud_.size(); ++i) {
            if (cloud_[i].index_state_ < 2) {
                dirty_indices.push_back(i);
                GraphPoint graph_point;
                std::copy(cloud_[i].position_,
                          cloud_[i].position_ + 3,
                          graph_point.position_);
                dirty_cloud.emplace_back(graph_point);
            }
        }
        if (dirty_cloud.empty()) {
            ROS_INFO("Graph up to date (no points to update): %.3f s.",
                     t.seconds_elapsed());
            return;
        }

        flann::Matrix<Value> positions(dirty_cloud[0].position_,
                                       dirty_cloud.size(),
                                       3,
                                       sizeof(GraphPoint));
        flann::Matrix<int> neighbors(dirty_cloud[0].neighbors_,
                                     dirty_cloud.size(),
                                     Point::K_NEIGHBORS,
                                     sizeof(GraphPoint));
        flann::Matrix<Value> distances(dirty_cloud[0].distances_,
                                       dirty_cloud.size(),
                                       Point::K_NEIGHBORS,
                                       sizeof(GraphPoint));

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
                              Point::K_NEIGHBORS,
                              params);
        }
        // Propagate NN info into the main graph.
        for (Index i = 0; i < dirty_indices.size(); ++i) {
            std::copy(dirty_cloud[i].neighbors_,
                      dirty_cloud[i].neighbors_ + Point::K_NEIGHBORS,
                      cloud_[dirty_indices[i]].neighbors_);
            std::copy(dirty_cloud[i].distances_,
                      dirty_cloud[i].distances_ + Point::K_NEIGHBORS,
                      cloud_[dirty_indices[i]].distances_);
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
        return num_vertices() * Point::K_NEIGHBORS;
    }
    inline std::pair<VertexIter, VertexIter> vertices() const
    {
        return { 0, size() };
    }
    inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
    {
        // TODO: Limit to valid edges here or just by costs?
//        return { u * num_edges(), (u + 1) * num_edges() };
        return { u * Point::K_NEIGHBORS, (u + 1) * Point::K_NEIGHBORS };
    }
    inline Edge out_degree(const Vertex& u) const
    {
        // TODO: Compute true number based on valid neighbors.
//        return num_edges();
        return Point::K_NEIGHBORS;
    }
    inline Vertex source(const Edge& e) const
    {
//        return e / num_edges();
        return e / Point::K_NEIGHBORS;
    }
    inline Vertex target_index(const Edge& e) const
    {
//        return e % num_edges();
        return e % Point::K_NEIGHBORS;
    }
    inline Vertex target(const Edge& e) const
    {
//        return nn_[source(e)][target_index(e)];
        return cloud_[source(e)].neighbors_[target_index(e)];
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
            if (cloud_[i].index_state_ == UP_TO_DATE) {
                continue;
            }
            assert(cloud_[i].index_state_ == TO_UPDATE);

            //***
            // Disregard empty points.
            if (cloud_[i].functional_label_ == EMPTY) {
                continue;
            }
            Vec3 mean = Vec3::Zero();
            Mat3 cov = Mat3::Zero();
//            num_normal_pts_[v0] = 0;
            cloud_[i].normal_support_ = 0;
            for (Index j = 0; j < Point::K_NEIGHBORS; ++j) {
                if (std::isinf(cloud_[i].distances_[j])) {
                    continue;
                }
//            for (size_t j = 0; j < nn_.cols; ++j)
//            {
//                Index v1 = nn_[v0][j];
                Index k = cloud_[i].neighbors_[j];
                // Disregard empty points.
                if (cloud_[k].functional_label_ == EMPTY) {
                    continue;
                }
                if (cloud_[i].distances_[j] <= radius2) {
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
            if ((mean - ConstVec3Map(cloud_[i].position_)).norm()
                > edge_min_centroid_offset_ * semicircle_centroid_offset) {
                cloud_[i].functional_label_ = EDGE;
            }

            cloud_[i].index_state_ = UP_TO_DATE;
        }
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

//        void merge(flann::Matrix<Elem> points, flann::Matrix<Elem> origin)
    void merge(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
    {
//            Lock lock(snapshot_mutex_);
        Lock lock(cloud_mutex_);
        Timer t;
        ROS_DEBUG("Merging cloud started. Capacity %lu points.", capacity());
        // TODO: Forbid allocation while in use or lock?
        reserve(size() + points.rows);

        if (empty()) {
//                auto it_points = end();
            Point point;
            for (size_t i = 0; i < points.rows; ++i)
            {
                for (size_t j = 0; j < 3; ++j) {
                    point.position_[j] = points[i][j];
                }
                cloud_.emplace_back(point);
            }
            update_index();
            ROS_INFO("Map initialized with %lu points (%.3f s).",
                     points.rows,
                     t.seconds_elapsed());
            return;
        }

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
            auto empty_orig = cloud_[v_map].empty_;
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
                if (d <= points_min_dist_) {
                    // Static / occupied.
                    if (cloud_[v_map].occupied_ >= max_observations) {
                        cloud_[v_map].occupied_ /= 2;
                        cloud_[v_map].empty_ /= 2;
                    }
                    ++cloud_[v_map].occupied_;
                    // Clear empty marks from this cloud.
                    // TODO: Why would we?
//                        empty_buf_[v_map] = empty_orig;
                    break;
                }
                // Discard rays far from the map points.
                const Elem d_new = (x_new - x_origin).norm();
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
                    if (cloud_[v_map].empty_ >= max_observations) {
                        cloud_[v_map].occupied_ /= 2;
                        cloud_[v_map].empty_ /= 2;
                    }
                    ++cloud_[v_map].empty_;
                }
            }
//                ROS_INFO("Point [%.1f, %.1f, %.1f]: %u occupied, %u empty.",
//                        x_map.x(), x_map.y(), x_map.z(), occupied_buf_[v_map], empty_buf_[v_map]);
//                if (empty_buf_[v_map] / occupied_buf_[v_map] >= 4)
            if (float(cloud_[v_map].empty_) / cloud_[v_map].occupied_ >= empty_ratio_) {
                // TODO: Remove point?
                // Handle this in processing NN.
            }
        }
        ROS_DEBUG("Checking and updating %u static/dynamic points nearby: %.3f s.",
                  n_nearby,
                  t_dyn.seconds_elapsed());

        // Find NN distance within current map.
        t.reset();
        Query<Elem> q(*index_, points, 1);
        ROS_DEBUG("Input cloud NN searched finished: %.3f s.",
                  t.seconds_elapsed());

        // Merge points with distance to NN higher than threshold.
        // We'll assume that input points also (approx.) comply to the
        // same min. distance threshold, so each added point can be tested
        // separately.
        t.reset();
        size_t n_added = 0;
        Value min_dist_2 = points_min_dist_ * points_min_dist_;
        for (size_t i = 0; i < points.rows; ++i) {
            if (q.dist_[i][0] >= min_dist_2) {
                Point point;
                for (size_t j = 0; j < 3; ++j) {
                    point.position_[j] = points[i][j];
                }
                cloud_.emplace_back(point);
                ++n_added;
            }
        }
//            mean /= points.rows;
//            ROS_INFO("Mean distance to map points: %.3f m.", mean);
        size_t n_map = size();
//            update_matrix_wrappers(size() + n_added);
        // TODO: flann::Index::addPoints (what indices? what with removed indices?)
        update_index();
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
    Mutex index_mutex_;
    std::shared_ptr<flann::Index<flann::L2_3D<Value>>> index_;

    // Parameters
    float points_min_dist_;

    float min_empty_cos_;
    float empty_ratio_;

    float neighborhood_radius_;
    float traversable_radius_;

    float edge_min_centroid_offset_;

    float max_pitch_;
    float max_roll_;
};

/** https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html */
//class Graph
//{
//public:
//    Graph(const Map& map):
//        map_(map)
//    {}
//    inline Vertex num_vertices() const
//    {
//        return map_.size();
//    }
//    /** Returns the number of edges in the graph g. */
//    inline Edge num_edges() const
//    {
//        // TODO: Compute true number based on valid neighbors.
//        return num_vertices() * Point::K_NEIGHBORS;
//    }
//    inline std::pair<VertexIter, VertexIter> vertices() const
//    {
//        return { 0, num_vertices() };
//    }
//    inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
//    {
//        // TODO: Limit to valid edges here or just by costs?
////        return { u * num_edges(), (u + 1) * num_edges() };
//        return { u * Point::K_NEIGHBORS, (u + 1) * Point::K_NEIGHBORS };
//    }
//    inline Edge out_degree(const Vertex& u) const
//    {
//        // TODO: Compute true number based on valid neighbors.
////        return num_edges();
//        return Point::K_NEIGHBORS;
//    }
//    inline Vertex source(const Edge& e) const
//    {
////        return e / num_edges();
//        return e / Point::K_NEIGHBORS;
//    }
//    inline Vertex target_index(const Edge& e) const
//    {
////        return e % num_edges();
//        return e % Point::K_NEIGHBORS;
//    }
//    inline Vertex target(const Edge& e) const
//    {
////        return nn_[source(e)][target_index(e)];
//        return map_.cloud_[source(e)].neighbors_[target_index(e)];
//    }
//    const Map& map_;
//};

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
